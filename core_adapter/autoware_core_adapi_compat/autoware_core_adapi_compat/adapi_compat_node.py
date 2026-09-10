#!/usr/bin/env python3
# Copyright 2026 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The AD API operation-mode endpoints that `autoware_core` does not implement.

WHY THIS EXISTS
==============-
concealer drives Autoware through a lifecycle:

    INITIALIZING => WAITING_FOR_ROUTE => PLANNING => WAITING_FOR_ENGAGE
                 => DRIVING => ARRIVED_GOAL

`concealer::FieldOperatorApplication` computes that state by reducing three AD API topics
(concealer/include/concealer/legacy_autoware_state.hpp):

    /api/localization/initialization_state   [autoware_core: OK, via default_adapi]
    /api/routing/state                       [autoware_core: OK, via default_adapi]
    /api/operation_mode/state                [autoware_core: MISSING]

and it calls these services (concealer/include/concealer/service.hpp waits 180 s for
availability, then throws AutowareError):

    /api/operation_mode/change_to_stop            <= called in the ctor, UNCONDITIONALLY
    /api/operation_mode/enable_autoware_control   <= before every engage
    /api/operation_mode/change_to_autonomous      <= the engage itself

`autoware_default_adapi` in autoware_core implements interface, localization and routing only ==
there is no operation_mode node == so none of those exist and nothing else publishes the topic.
Without them LegacyAutowareState is pinned at INITIALIZING forever and every scenario times out
with no other symptom.

WHAT THIS NODE IS *NOT*
======================-
It is not a general Autoware compatibility layer, and it deliberately does not grow into one.
Every endpoint here has a disposition:

    operation_mode services  => GRADUATES to autoware_core as a default_adapi OperationModeNode.
                                That upstream PR is the actual fix; this node is a placeholder
                                for it and should be measured as a burn-down, not maintained.
    RTC / MRM / emergency    => NEVER implemented. Not applicable to autoware_core (it launches
                                no module that requests cooperation and has no MRM), and
                                concealer tolerates their absence == it only subscribes.

Two endpoints that this node carried in an earlier revision are now GONE, and their absence is
the clearest evidence that the AWF interface migration paid for itself:

    /api/external/set/engage           tier4_external_api_msgs/Engage. concealer now engages via
                                       AD API /api/operation_mode/change_to_autonomous, which
                                       this node already serves.
    /api/autoware/set/velocity_limit   tier4_external_api_msgs/SetVelocityLimit. concealer now
                                       publishes autoware_internal_planning_msgs/VelocityLimit
                                       on /planning/scenario_planning/max_velocity, which
                                       velocity_smoother already subscribes to == so there is
                                       nothing left to bridge.

DESIGN NOTES
============
* Downstream service calls are FIRE-AND-FORGET and the caller is answered immediately. This is
  safe because concealer never trusts the service response for state == it polls the AD API
  triple via waitForAutowareStateToBe() == and it removes every single-threaded-executor deadlock
  risk. Downstream failures are logged loudly.
* standalone_mode=True makes this node OWN and publish OperationModeState rather than delegating
  to autoware_command_gate. Required for autoware_core releases that have no command gate, and
  the two must never both publish: concealer would see interleaved values.
* /api/operation_mode/state is published TRANSIENT_LOCAL because concealer subscribes with
  rclcpp::QoS(1).transient_local(). Without latching a late-joining concealer never receives a
  value at all, and the failure is indistinguishable from a dead publisher.
"""

import threading

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode as AdapiChangeMode
from autoware_common_msgs.msg import ResponseStatus as CommonStatus
from autoware_system_msgs.srv import ChangeOperationMode as SystemChangeMode
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def latched_qos(depth: int = 1) -> QoSProfile:
    """QoS matching concealer's `rclcpp::QoS(1).transient_local()`, and the AD API spec's own."""
    qos = QoSProfile(depth=depth)
    qos.history = HistoryPolicy.KEEP_LAST
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


MODE_NAMES = {
    OperationModeState.UNKNOWN: "UNKNOWN",
    OperationModeState.STOP: "STOP",
    OperationModeState.AUTONOMOUS: "AUTONOMOUS",
    OperationModeState.LOCAL: "LOCAL",
    OperationModeState.REMOTE: "REMOTE",
}


class AdapiOperationMode(Node):
    def __init__(self):
        super().__init__("adapi_operation_mode")

        self.standalone = bool(self.declare_parameter("standalone_mode", True).value)
        self.startup_delay = float(self.declare_parameter("startup_stop_delay_s", 2.0).value)
        self.republish_period = float(self.declare_parameter("republish_period_s", 0.5).value)

        self._lock = threading.Lock()
        self._mode = OperationModeState.STOP
        self._autoware_control = False
        # Set by the first AD API request that arrives. The startup latch below is a FALLBACK for
        # the case where nothing ever asks; it must never overrule a caller that already has.
        self._mode_requested = False

        # ==== downstream client, used only when delegating to autoware_command_gate =====
        self.cli_mode = self.create_client(
            SystemChangeMode, "/system/operation_mode/change_operation_mode"
        )

        # ==== state publishers =========================================================
        # In standalone mode this node owns both the API-facing and system-facing topics.
        # /system/operation_mode/state is read by core's mission_planner (reroute gating) and
        # velocity_smoother.
        self.pub_api_state = None
        self.pub_sys_state = None
        if self.standalone:
            self.pub_api_state = self.create_publisher(
                OperationModeState, "/api/operation_mode/state", latched_qos()
            )
            self.pub_sys_state = self.create_publisher(
                OperationModeState, "/system/operation_mode/state", latched_qos()
            )
            self.get_logger().warn(
                "standalone_mode=True: this node OWNS OperationModeState. "
                "autoware_command_gate must not be running, or concealer sees interleaved values."
            )

        # ==== the six AD API operation-mode endpoints ===================================
        self._services = [
            self._mode_service("change_to_stop", SystemChangeMode.Request.STOP),
            self._mode_service("change_to_autonomous", SystemChangeMode.Request.AUTONOMOUS),
            self._mode_service("change_to_local", SystemChangeMode.Request.LOCAL),
            self._mode_service("change_to_remote", SystemChangeMode.Request.REMOTE),
            self._mode_service("enable_autoware_control", SystemChangeMode.Request.AUTONOMOUS),
            self._mode_service("disable_autoware_control", SystemChangeMode.Request.STOP),
        ]

        # ==== system-layer service, only when this node is the state owner ===============
        # autoware_default_adapi's routing node holds a client for this and calls it on reroute.
        # In standalone mode autoware_command_gate is not running, so without this nothing serves
        # it and a reroute hangs until timeout.
        self._service_system_mode = None
        if self.standalone:
            self._service_system_mode = self.create_service(
                SystemChangeMode,
                "/system/operation_mode/change_operation_mode",
                self._on_system_mode,
            )

        # ==== latch STOP immediately, then keep it fresh ================================
        # concealer's constructor calls change_to_stop, but we must not rely on that: it polls
        # getLegacyAutowareState() and needs a value on /api/operation_mode/state before its own
        # request lands.
        if self.standalone:
            self._publish_state()
        self._startup_timer = self.create_timer(self.startup_delay, self._on_startup)
        self._republish_timer = self.create_timer(self.republish_period, self._publish_state)

        self.get_logger().info(
            f"adapi_operation_mode ready | 6 AD API endpoints | standalone={self.standalone}"
        )

    def _mode_service(self, endpoint: str, mode: int):
        return self.create_service(
            AdapiChangeMode, f"/api/operation_mode/{endpoint}", self._make_handler(mode, endpoint)
        )

    # ================================================================== state ====
    def _build_state(self) -> OperationModeState:
        message = OperationModeState()
        message.stamp = self.get_clock().now().to_msg()
        with self._lock:
            message.mode = self._mode
            message.is_autoware_control_enabled = self._autoware_control
        message.is_in_transition = False
        # All four availability flags must be True. is_autonomous_mode_available in particular
        # gates the PLANNING => WAITING_FOR_ENGAGE transition in LegacyAutowareState.
        message.is_stop_mode_available = True
        message.is_autonomous_mode_available = True
        message.is_local_mode_available = True
        message.is_remote_mode_available = True
        return message

    def _publish_state(self) -> None:
        """Also the heartbeat.

        Transient-local latching already covers late joiners; the periodic refresh keeps `stamp`
        current, which matters because LegacyAutowareState's ARRIVED_GOAL branch is time-sensitive
        (`now - route_state.stamp < 2.0`).
        """
        if not self.standalone:
            return
        message = self._build_state()
        self.pub_api_state.publish(message)
        self.pub_sys_state.publish(message)

    def _set_mode(self, mode: int) -> None:
        with self._lock:
            self._mode = mode
            self._autoware_control = mode == OperationModeState.AUTONOMOUS
        self._publish_state()

    # ================================================================== forward ==
    def _forward(self, mode: int, tag: str) -> None:
        if self.standalone:
            self._set_mode(mode)
            self.get_logger().info(f"[{tag}] mode => {MODE_NAMES.get(mode, mode)} (standalone)")
            return

        if not self.cli_mode.service_is_ready():
            self.get_logger().error(
                f"[{tag}] /system/operation_mode/change_operation_mode NOT READY. "
                "Is autoware_command_gate running? Otherwise set standalone_mode:=true."
            )
            return

        request = SystemChangeMode.Request()
        request.mode = mode
        future = self.cli_mode.call_async(request)

        def done(completed):
            try:
                status = getattr(completed.result(), "status", None)
                if status is None or status.success:
                    self.get_logger().info(f"[{tag}] mode => {MODE_NAMES.get(mode, mode)}")
                else:
                    self.get_logger().error(
                        f"[{tag}] rejected by command_gate: {status.message}"
                    )
            except Exception as error:  # noqa: BLE001
                self.get_logger().error(f"[{tag}] downstream call failed: {error}")

        future.add_done_callback(done)

    # ================================================================== handlers ==
    def _make_handler(self, mode: int, tag: str):
        def handler(_request, response):
            self.get_logger().info(f"AD API {tag}")
            with self._lock:
                self._mode_requested = True
            self._forward(mode, tag)
            # autoware_adapi_v1_msgs/ResponseStatus: concealer checks `response=>status.success`.
            response.status.success = True
            response.status.code = 0
            response.status.message = f"{tag} accepted"
            return response

        return handler

    def _on_system_mode(self, request, response):
        """Stands in for autoware_command_gate. Standalone mode only."""
        name = MODE_NAMES.get(request.mode, str(request.mode))
        if request.mode in (
            SystemChangeMode.Request.STOP,
            SystemChangeMode.Request.AUTONOMOUS,
            SystemChangeMode.Request.LOCAL,
            SystemChangeMode.Request.REMOTE,
        ):
            self._set_mode(request.mode)
            response.status.success = True
            response.status.code = 0
            response.status.message = f"Switched to {name}"
        else:
            response.status.success = False
            response.status.code = CommonStatus.PARAMETER_ERROR
            response.status.message = "Unknown operation mode requested."
        self.get_logger().info(f"[system_mode] {name} => success={response.status.success}")
        return response

    def _on_startup(self) -> None:
        """Latch STOP once, but ONLY if nothing has asked for a mode yet.

        This timer exists for the case where concealer polls /api/operation_mode/state before its
        own change_to_stop lands. It is a floor, not an override: concealer reaches
        initialize => route => engage in well under `startup_stop_delay_s` on a fast machine, and
        an unconditional forward here then puts the mode back to STOP a few hundred milliseconds
        after change_to_autonomous. The command gate answers STOP with GearCommand::PARK, so the
        ego never moves while control keeps commanding acceleration == a silent failure whose only
        symptom is traveled_distance == 0.
        """
        self._startup_timer.cancel()
        with self._lock:
            already_requested = self._mode_requested
        if already_requested:
            self.get_logger().info(
                "startup: a mode was already requested == leaving it alone"
            )
            return
        self.get_logger().info("startup: latching STOP operation mode")
        self._forward(SystemChangeMode.Request.STOP, "startup_stop")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdapiOperationMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

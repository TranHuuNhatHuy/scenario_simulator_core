#!/usr/bin/env python3
# Copyright 2026 The Autoware Contributors. Apache-2.0
"""
Acceptance test: drive autoware_core through the SSV2 lifecycle WITHOUT scenario_simulator_v2.

This replicates exactly what concealer::FieldOperatorApplication does -- the same services in
the same order, and the same LegacyAutowareState reduction
(concealer/include/concealer/legacy_autoware_state.hpp) -- and prints the state after each step.

Being able to run this without SSV2 is the whole point: if this passes, the control-plane
integration is correct, and any remaining failure is in the SSV2 data plane or the scenario.

Usage
-----
    ./run_core_only.sh                       # terminal 1
    python3 tools/handdrive.py               # terminal 2  (fake vehicle + localization)
    python3 tools/verify_lifecycle.py        # terminal 3

Exit code 0 = every step passed.
"""

import argparse
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from autoware_adapi_v1_msgs.msg import (
    LocalizationInitializationState,
    OperationModeState,
    RouteState,
)
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, InitializeLocalization, SetRoutePoints
from geometry_msgs.msg import PoseWithCovarianceStamped

GREEN, RED, YELLOW, DIM, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"


def latched():
    qos = QoSProfile(depth=1)
    qos.history = HistoryPolicy.KEEP_LAST
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


# ----------------------------------------------------------------------------------------
# Faithful port of concealer's LegacyAutowareState reduction.
# ----------------------------------------------------------------------------------------
UNDEFINED, INITIALIZING, WAITING_FOR_ROUTE, PLANNING = "UNDEFINED", "INITIALIZING", "WAITING_FOR_ROUTE", "PLANNING"
WAITING_FOR_ENGAGE, DRIVING, ARRIVED_GOAL = "WAITING_FOR_ENGAGE", "DRIVING", "ARRIVED_GOAL"


def legacy_autoware_state(loc, route, mode, now_s):
    L, R, O = LocalizationInitializationState, RouteState, OperationModeState
    if loc.state == L.UNKNOWN or route.state == R.UNKNOWN or mode.mode == O.UNKNOWN:
        return INITIALIZING
    if loc.state in (L.UNINITIALIZED, L.INITIALIZING):
        return INITIALIZING
    if loc.state != L.INITIALIZED:
        return UNDEFINED
    if route.state == R.ARRIVED:
        stamp = route.stamp.sec + route.stamp.nanosec * 1e-9
        if now_s - stamp < 2.0:
            return ARRIVED_GOAL
        return WAITING_FOR_ROUTE
    if route.state == R.UNSET:
        return WAITING_FOR_ROUTE
    if route.state in (R.SET, R.CHANGING):
        if mode.mode in (O.AUTONOMOUS, O.LOCAL, O.REMOTE) and mode.is_autoware_control_enabled:
            return DRIVING
        if mode.mode in (O.AUTONOMOUS, O.LOCAL, O.REMOTE, O.STOP):
            return WAITING_FOR_ENGAGE if mode.is_autonomous_mode_available else PLANNING
        return UNDEFINED
    return UNDEFINED


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_lifecycle")
        self.loc = LocalizationInitializationState()
        self.route = RouteState()
        self.mode = OperationModeState()
        self.create_subscription(
            LocalizationInitializationState, "/api/localization/initialization_state",
            lambda m: setattr(self, "loc", m), latched())
        self.create_subscription(
            RouteState, "/api/routing/state", lambda m: setattr(self, "route", m), latched())
        self.create_subscription(
            OperationModeState, "/api/operation_mode/state",
            lambda m: setattr(self, "mode", m), latched())

        self.cli_init = self.create_client(InitializeLocalization, "/api/localization/initialize")
        self.cli_route = self.create_client(SetRoutePoints, "/api/routing/set_route_points")
        self.cli_stop = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_stop")
        self.cli_enable = self.create_client(
            ChangeOperationMode, "/api/operation_mode/enable_autoware_control")
        # AD API engage. Upstream scenario_simulator_v2 calls tier4_external_api_msgs/Engage on
        # /api/external/set/engage here; this fork's concealer engages through the AD API
        # operation mode instead, so that is what is verified.
        self.cli_engage = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous")

    def state(self):
        now = self.get_clock().now()
        return legacy_autoware_state(
            self.loc, self.route, self.mode, now.nanoseconds * 1e-9)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_state(self, target, timeout):
        """Mirrors concealer's waitForAutowareStateToBe()."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state() == target:
                return True
        return False

    def call(self, client, request, name, timeout=20.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None, f"service {name} not available after {timeout}s"
        fut = client.call_async(request)
        end = time.monotonic() + timeout
        while time.monotonic() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            return None, f"service {name} did not respond within {timeout}s"
        return fut.result(), None


def main():
    ap = argparse.ArgumentParser()
    # defaults: spawn = lanelet 34513 s=1, goal = lanelet 34507 s=50 (tools/map_coords.py)
    ap.add_argument("--x", type=float, default=3697.15)
    ap.add_argument("--y", type=float, default=73762.76)
    ap.add_argument("--qz", type=float, default=0.2352)
    ap.add_argument("--qw", type=float, default=0.9719)
    ap.add_argument("--goal-x", type=float, default=3791.39)
    ap.add_argument("--goal-y", type=float, default=73811.02)
    ap.add_argument("--goal-qz", type=float, default=0.2347)
    ap.add_argument("--goal-qw", type=float, default=0.9721)
    ap.add_argument("--timeout", type=float, default=60.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    v = Verifier()
    results = []

    def step(name, ok, detail=""):
        results.append((name, ok, detail))
        mark = f"{GREEN}PASS{RESET}" if ok else f"{RED}FAIL{RESET}"
        print(f"  [{mark}] {name}" + (f"  {DIM}{detail}{RESET}" if detail else ""), flush=True)
        return ok

    print(f"\n{'='*72}\n AD API LIFECYCLE VERIFICATION  (autoware_core)\n{'='*72}")

    # ---- step 0: the ADAPI triple must exist -----------------------------------------
    print(f"\n{DIM}0. discovering the ADAPI triple concealer reduces...{RESET}")
    v.spin(6.0)
    step("/api/localization/initialization_state present",
         v.loc.state != 0, f"state={v.loc.state}")
    step("/api/routing/state present", v.route.state != 0, f"state={v.route.state}")
    step("/api/operation_mode/state present (THE gap this repo closes)",
         v.mode.mode != 0, f"mode={v.mode.mode}")
    step("is_autonomous_mode_available (gates WAITING_FOR_ENGAGE)",
         bool(v.mode.is_autonomous_mode_available))
    s0 = v.state()
    step("initial LegacyAutowareState == INITIALIZING", s0 == INITIALIZING, s0)

    # ---- step 1: change_to_stop (concealer ctor, unconditional) ----------------------
    print(f"\n{DIM}1. change_to_stop  (concealer calls this in its constructor){RESET}")
    res, err = v.call(v.cli_stop, ChangeOperationMode.Request(), "/api/operation_mode/change_to_stop")
    step("change_to_stop succeeded", res is not None and res.status.success, err or "")

    # ---- step 2: initialize localization --------------------------------------------
    print(f"\n{DIM}2. initialize localization -> expect WAITING_FOR_ROUTE{RESET}")
    req = InitializeLocalization.Request()
    p = PoseWithCovarianceStamped()
    p.header.frame_id = "map"
    p.header.stamp = v.get_clock().now().to_msg()
    p.pose.pose.position.x = args.x
    p.pose.pose.position.y = args.y
    p.pose.pose.orientation.z = args.qz
    p.pose.pose.orientation.w = args.qw
    req.pose = [p]
    res, err = v.call(v.cli_init, req, "/api/localization/initialize")
    step("initialize succeeded", res is not None and res.status.success,
         err or (res.status.message if res else ""))
    ok = v.wait_for_state(WAITING_FOR_ROUTE, 25.0)
    step("state -> WAITING_FOR_ROUTE", ok, v.state())

    # ---- step 3: set route ----------------------------------------------------------
    print(f"\n{DIM}3. set_route_points -> expect WAITING_FOR_ENGAGE{RESET}")
    rreq = SetRoutePoints.Request()
    rreq.header.frame_id = "map"
    rreq.header.stamp = v.get_clock().now().to_msg()
    rreq.goal.position.x = args.goal_x
    rreq.goal.position.y = args.goal_y
    rreq.goal.orientation.z = args.goal_qz
    rreq.goal.orientation.w = args.goal_qw
    res, err = v.call(v.cli_route, rreq, "/api/routing/set_route_points", timeout=30.0)
    step("set_route_points succeeded", res is not None and res.status.success,
         err or (res.status.message if res else ""))
    ok = v.wait_for_state(WAITING_FOR_ENGAGE, 25.0)
    step("state -> WAITING_FOR_ENGAGE", ok, v.state())

    # ---- step 4: engage -------------------------------------------------------------
    print(f"\n{DIM}4. enable_autoware_control + engage -> expect DRIVING{RESET}")
    res, err = v.call(v.cli_enable, ChangeOperationMode.Request(),
                      "/api/operation_mode/enable_autoware_control")
    step("enable_autoware_control succeeded", res is not None and res.status.success, err or "")
    res, err = v.call(v.cli_engage, ChangeOperationMode.Request(),
                      "/api/operation_mode/change_to_autonomous")
    step("change_to_autonomous succeeded", res is not None and res.status.success,
         err or (res.status.message if res else ""))
    ok = v.wait_for_state(DRIVING, 25.0)
    step("state -> DRIVING", ok, v.state())

    # ---- summary --------------------------------------------------------------------
    passed = sum(1 for _, ok, _ in results if ok)
    total = len(results)
    print(f"\n{'='*72}")
    if passed == total:
        print(f" {GREEN}ALL {total} CHECKS PASSED{RESET} -- the AD API control plane works on autoware_core")
    else:
        print(f" {RED}{total - passed} of {total} CHECKS FAILED{RESET}")
        for name, ok, detail in results:
            if not ok:
                print(f"   {RED}x{RESET} {name}  {DIM}{detail}{RESET}")
    print(f"{'='*72}\n")

    v.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()

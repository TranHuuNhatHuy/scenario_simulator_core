#!/usr/bin/env python3
# Copyright 2026 The Autoware Contributors. Apache-2.0
"""
Fake the vehicle interface + localization so autoware_core plans and controls with NO simulator.

This is the single most valuable debugging tool in this repo. It isolates "does autoware_core
work at all" from "does the scenario_simulator_v2 integration work". Run it against a bare
core launch; if /planning/trajectory does not appear, the problem is core (or the map, or the
spawn pose), not the integration.

Usage
-----
    # terminal 1
    ros2 launch autoware_core_scenario_launch core_scenario_simulator.launch.xml \
        map_path:=$SSC_MAP vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

    # terminal 2  (pose defaults to lanelet 34513 s=1, kashiwanoha)
    python3 tools/handdrive.py           # defaults to lanelet 34513 s=1

    # terminal 3
    ros2 service call /api/localization/initialize \
      autoware_adapi_v1_msgs/srv/InitializeLocalization \
      "{pose: [{header: {frame_id: map}, pose: {pose: {position: {x: 3697.15, y: 73762.76}, \
        orientation: {z: 0.2352, w: 0.9719}}}}]}"
    ros2 topic hz /planning/trajectory
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import AccelWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearReport,
    SteeringReport,
    TurnIndicatorsReport,
    VelocityReport,
)


def reliable(depth=1):
    qos = QoSProfile(depth=depth)
    qos.reliability = ReliabilityPolicy.RELIABLE
    return qos


class HandDrive(Node):
    def __init__(self, x, y, yaw, speed, publish_tf):
        super().__init__("handdrive")
        self.x, self.y, self.yaw, self.speed = x, y, yaw, speed
        self.publish_tf = publish_tf

        self.pub_odom = self.create_publisher(Odometry, "/localization/kinematic_state", reliable())
        self.pub_accel = self.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", reliable()
        )
        self.pub_vel = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", reliable()
        )
        self.pub_steer = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", reliable()
        )
        self.pub_mode = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", reliable()
        )
        self.pub_gear = self.create_publisher(GearReport, "/vehicle/status/gear_status", reliable())
        self.pub_turn = self.create_publisher(
            TurnIndicatorsReport, "/vehicle/status/turn_indicators_status", reliable()
        )
        self.tf = TransformBroadcaster(self) if publish_tf else None

        self.create_timer(0.02, self.tick)  # 50 Hz, matching concealer
        self.get_logger().info(
            f"handdrive at ({x:.2f}, {y:.2f}) yaw={yaw:.3f} speed={speed:.2f} tf={publish_tf}"
        )

    def tick(self):
        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

        # dead-reckon if a speed was requested
        if self.speed != 0.0:
            self.x += self.speed * 0.02 * math.cos(self.yaw)
            self.y += self.speed * 0.02 * math.sin(self.yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.speed
        self.pub_odom.publish(odom)

        accel = AccelWithCovarianceStamped()
        accel.header.stamp = now
        accel.header.frame_id = "base_link"
        self.pub_accel.publish(accel)

        vel = VelocityReport()
        vel.header.stamp = now
        vel.header.frame_id = "base_link"
        vel.longitudinal_velocity = self.speed
        self.pub_vel.publish(vel)

        steer = SteeringReport()
        steer.stamp = now
        self.pub_steer.publish(steer)

        mode = ControlModeReport()
        mode.stamp = now
        mode.mode = ControlModeReport.AUTONOMOUS
        self.pub_mode.publish(mode)

        gear = GearReport()
        gear.stamp = now
        gear.report = GearReport.DRIVE
        self.pub_gear.publish(gear)

        turn = TurnIndicatorsReport()
        turn.stamp = now
        turn.report = TurnIndicatorsReport.DISABLE
        self.pub_turn.publish(turn)

        if self.tf is not None:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "map"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf.sendTransform(tf)


def main():
    parser = argparse.ArgumentParser()
    # defaults = lanelet 34513 s=1 on kashiwanoha (see tools/map_coords.py)
    parser.add_argument("--x", type=float, default=3697.15)
    parser.add_argument("--y", type=float, default=73762.76)
    parser.add_argument("--yaw", type=float, default=0.4749)
    parser.add_argument("--speed", type=float, default=0.0, help="dead-reckon at this m/s")
    parser.add_argument("--no-tf", action="store_true", help="do not broadcast map->base_link")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = HandDrive(args.x, args.y, args.yaw, args.speed, not args.no_tf)
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

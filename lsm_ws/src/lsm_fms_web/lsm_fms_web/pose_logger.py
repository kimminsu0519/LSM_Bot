#!/usr/bin/env python3
"""
LSM FMS Pose Comparison & 1-Second Logger Node
Author: Antigravity AI
Package: lsm_fms_web

Logs continuous 1-second snapshots comparing:
1. Gazebo World / Odometry Pose (/odom)
2. Nav2 AMCL Map Pose (/amcl_pose & TF map->base_footprint)
3. FMS UI Canvas Mapped Coordinates (+2.457m X offset, +0.358m Y offset)

Saves detailed logs to CSV file (`pose_comparison_log.csv`) and prints live on stdout.
"""

import os
import sys
import math
import time
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros

# Metric Canvas Offset
OFFSET_X = 2.457
OFFSET_Y = 0.358

class PoseLoggerNode(Node):
    def __init__(self):
        super().__init__('lsm_pose_logger_node')
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            pass
        
        self.csv_path = os.path.expanduser("~/dev/LSM_repository/lsm_ws/pose_comparison_log.csv")
        self.init_csv()

        # State storage
        self.odom_pose = {"x": None, "y": None, "yaw": None}
        self.amcl_pose = {"x": None, "y": None, "yaw": None}
        self.tf_map_pose = {"x": None, "y": None, "yaw": None}

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        amcl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        odom_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_cb, odom_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, amcl_qos)

        # 1-second comparison timer
        self.timer = self.create_timer(1.0, self.timer_cb)

        self.get_logger().info("=====================================================")
        self.get_logger().info(" LSM 1-Second Real-Time Pose Comparison Logger ")
        self.get_logger().info(f" Log CSV Path: {self.csv_path}")
        self.get_logger().info("=====================================================")

    def init_csv(self):
        file_exists = os.path.exists(self.csv_path)
        with open(self.csv_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    'timestamp',
                    'gz_odom_x', 'gz_odom_y', 'gz_odom_yaw_deg',
                    'ros_map_x', 'ros_map_y', 'ros_map_yaw_deg',
                    'ui_canvas_x', 'ui_canvas_y',
                    'diff_x_gz_map', 'diff_y_gz_map'
                ])

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        self.odom_pose["x"] = round(pos.x, 3)
        self.odom_pose["y"] = round(pos.y, 3)
        self.odom_pose["yaw"] = round(yaw_deg, 1)

    def amcl_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        self.amcl_pose["x"] = round(pos.x, 3)
        self.amcl_pose["y"] = round(pos.y, 3)
        self.amcl_pose["yaw"] = round(yaw_deg, 1)

    def timer_cb(self):
        # Update TF map->base_footprint
        try:
            if self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                rx = t.transform.translation.x
                ry = t.transform.translation.y
                ori = t.transform.rotation
                siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
                cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
                yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                self.tf_map_pose["x"] = round(rx, 3)
                self.tf_map_pose["y"] = round(ry, 3)
                self.tf_map_pose["yaw"] = round(yaw_deg, 1)
        except Exception:
            pass

        # Determine effective ROS 2 Map Pose (TF map or AMCL pose or Odom fallback)
        map_x = self.tf_map_pose["x"] if self.tf_map_pose["x"] is not None else self.amcl_pose["x"]
        map_y = self.tf_map_pose["y"] if self.tf_map_pose["y"] is not None else self.amcl_pose["y"]
        map_yaw = self.tf_map_pose["yaw"] if self.tf_map_pose["yaw"] is not None else self.amcl_pose["yaw"]

        gz_x = self.odom_pose["x"]
        gz_y = self.odom_pose["y"]
        gz_yaw = self.odom_pose["yaw"]

        if map_x is not None and map_y is not None:
            canvas_x = round(map_x, 3)
            canvas_y = round(map_y, 3)
        else:
            canvas_x = None
            canvas_y = None

        diff_x = round(gz_x - map_x, 3) if (gz_x is not None and map_x is not None) else 0.0
        diff_y = round(gz_y - map_y, 3) if (gz_y is not None and map_y is not None) else 0.0

        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # Write to CSV
        with open(self.csv_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                ts,
                gz_x, gz_y, gz_yaw,
                map_x, map_y, map_yaw,
                canvas_x, canvas_y,
                diff_x, diff_y
            ])

        # Print on console
        print(f"[{ts}] GZ_Odom: ({gz_x}, {gz_y}, {gz_yaw}°) | ROS_Map: ({map_x}, {map_y}, {map_yaw}°) | UI_Canvas: ({canvas_x}, {canvas_y}) | Diff(GZ-Map): ({diff_x:+.3f}, {diff_y:+.3f})")

def main():
    if "ROS_DOMAIN_ID" not in os.environ:
        os.environ["ROS_DOMAIN_ID"] = "13"
    rclpy.init()
    node = PoseLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

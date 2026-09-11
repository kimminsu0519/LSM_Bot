#!/usr/bin/env python3
import os
import sys
import math
import argparse
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('lsm_waypoint_navigator')
        
        # Initial pose publisher for AMCL activation
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Load waypoints database
        self.yaml_path = self._find_waypoints_yaml()
        self.waypoints = self._load_waypoints(self.yaml_path)
        
        # Nav2 NavigateToPose Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = -21.0
        msg.pose.pose.position.y = -5.25
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(msg)

    def _find_waypoints_yaml(self):
        # 1. Try package share directory
        try:
            pkg_dir = get_package_share_directory('lsm_navigation')
            yaml_path = os.path.join(pkg_dir, 'config', 'waypoints_graph.yaml')
            if os.path.exists(yaml_path):
                return yaml_path
        except Exception:
            pass

        # 2. Try package source directory fallback
        fallback_path = os.path.expanduser('~/dev/LSM_repository/lsm_ws/src/lsm_navigation/config/waypoints_graph.yaml')
        if os.path.exists(fallback_path):
            return fallback_path
            
        raise FileNotFoundError("Could not locate waypoints_graph.yaml database!")

    def _load_waypoints(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data.get('waypoints', {})

    def list_waypoints(self):
        print("\n==============================================================================")
        print("📍 LSM BOT WAYPOINT DATABASE SUMMARY")
        print(f"File: {self.yaml_path}")
        print(f"Total Waypoints: {len(self.waypoints)}")
        print("==============================================================================")
        
        for wp_id, info in self.waypoints.items():
            asset_id = info.get('asset_id', wp_id)
            desc = info.get('description', '')
            upose = info.get('user_pose', {})
            pose = info.get('pose', {})
            print(f"  • [{wp_id}] ({asset_id}): User({upose.get('x')}, {upose.get('y')})m => ROS2 Pose({pose.get('x')}, {pose.get('y')})m, yaw={pose.get('yaw')} rad | {desc}")
        print("")

    def yaw_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    def send_waypoint_goal(self, wp_id, dry_run=False, set_initial_pose=False):
        if wp_id not in self.waypoints:
            self.get_logger().error(f"Waypoint ID '{wp_id}' not found in database!")
            return False

        wp_data = self.waypoints[wp_id]
        pose_info = wp_data.get('pose', {})
        upose_info = wp_data.get('user_pose', {})
        
        rx = float(pose_info.get('x', 0.0))
        ry = float(pose_info.get('y', 0.0))
        rz = float(pose_info.get('z', 0.0))
        yaw = float(pose_info.get('yaw', 0.0))
        
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)

        print("\n==============================================================================")
        print(f"🚀 TARGET WAYPOINT: {wp_id} ({wp_data.get('asset_id', wp_id)})")
        print(f"   Description: {wp_data.get('description', '')}")
        print(f"   User Pose (Bottom-Left 0,0): X={upose_info.get('x')}m, Y={upose_info.get('y')}m, Yaw={yaw} rad")
        print(f"   ROS2 Map Frame Pose: X={rx}m, Y={ry}m, Z={rz}m")
        print(f"   Quaternion Orientation: [qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}, qw={qw:.4f}]")
        print("==============================================================================")

        if dry_run:
            print("🧪 DRY-RUN MODE: Action goal packaged successfully. Skipping live Nav2 network transmission.\n")
            return True

        if set_initial_pose:
            self._publish_initial_pose()

        # Wait for Nav2 NavigateToPose action server
        self.get_logger().info("Waiting for 'navigate_to_pose' Nav2 Action Server...")
        wait_cycles = 0
        while not self._action_client.wait_for_server(timeout_sec=2.0):
            wait_cycles += 1
            if wait_cycles >= 15: # 30 seconds total
                self.get_logger().error("Nav2 'navigate_to_pose' Action Server not available after 30s!")
                self.get_logger().error("Please check if 'ros2 launch lsm_navigation navigation.launch.py' is running and Nav2 lifecycle nodes are Active.")
                return False
            self.get_logger().info(f"Still waiting for Nav2 Action Server... ({wait_cycles * 2}s / 30s)")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = rx
        goal_msg.pose.pose.position.y = ry
        goal_msg.pose.pose.position.z = rz
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Sending navigation goal for '{wp_id}' to Nav2...")
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self._feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal for '{wp_id}' was rejected by Nav2!")
            return False

        self.get_logger().info(f"Goal for '{wp_id}' accepted by Nav2. Navigating...")
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"🎉 NAV2 GOAL SUCCEEDED! Robot successfully reached waypoint '{wp_id}'!")
            return True
        else:
            self.get_logger().warn(f"Nav2 goal finished with status code: {result.status}")
            return False

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = getattr(fb, 'distance_remaining', 0.0)
        t_elapsed = getattr(fb, 'navigation_time', 0.0)
        if hasattr(t_elapsed, 'sec'):
            elapsed_sec = t_elapsed.sec + t_elapsed.nanosec * 1e-9
        else:
            try:
                elapsed_sec = float(t_elapsed)
            except Exception:
                elapsed_sec = 0.0

        self.get_logger().info(
            f"  [Nav2 Progress] Distance remaining: {dist:.2f}m | Time elapsed: {elapsed_sec:.1f}s",
            throttle_duration_sec=2.0
        )



def main():
    parser = argparse.ArgumentParser(description="LSM Bot ROS 2 Nav2 Waypoint Navigator Client")
    parser.add_argument('--wp', type=str, help="Target Waypoint ID (e.g. wp-ws-a-1, wp-ws-b-1-1, wp-charge-1)")
    parser.add_argument('--list', action='store_true', help="List all available waypoints in database")
    parser.add_argument('--dry-run', action='store_true', help="Package goal & compute poses without sending to live network")
    parser.add_argument('--set-initial-pose', action='store_true', help="Publish initial pose to /initialpose before sending goal")
    
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = WaypointNavigator()

    try:
        if args.list:
            node.list_waypoints()
        elif args.wp:
            node.send_waypoint_goal(args.wp, dry_run=args.dry_run, set_initial_pose=args.set_initial_pose)
        else:
            parser.print_help()
            print("\nAvailable Waypoints Example: --wp wp-ws-a-1, --wp wp-ws-b-1-1, --wp wp-charge-1")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

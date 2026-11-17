#!/usr/bin/env python3
"""
LSM Bot 5-Stage Logistics AMR Finite State Machine (FSM) Node
Package: lsm_navigation

Implements the 5-stage AMR logistics lifecycle using lsm_interfaces:
  0: CHARGING          (Docked at charging station)
  1: MOVING_UNLOADED   (Deadheading to pickup station or returning to charger)
  2: WORKING_LOADING   (Waiting for cargo loading completion at pickup station)
  3: MOVING_LOADED     (Transporting cargo to delivery station)
  4: WORKING_UNLOADING (Waiting for cargo unloading completion at delivery station)

Interfaces:
  - Publishes  /lsm01/robot_status      (lsm_interfaces/msg/RobotStatus) @ 10Hz
  - Publishes  /workstation/event       (lsm_interfaces/msg/WorkstationEvent)
  - Publishes  /fms/fsm_route_cmd       (std_msgs/msg/String JSON -> SequentialNavigator)
  - Subscribes /fms/route_status        (std_msgs/msg/String JSON <- SequentialNavigator)
  - Subscribes /amcl_pose, /odometry/filtered
  - Service    /workstation/trigger_mock (lsm_interfaces/srv/TriggerMockEvent)
  - Action     /fms/dispatch_task       (lsm_interfaces/action/DispatchTask)
"""

import os
import json
import math
import time
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import tf2_ros

from lsm_interfaces.msg import RobotStatus, WorkstationEvent
from lsm_interfaces.srv import TriggerMockEvent
from lsm_interfaces.action import DispatchTask


STATE_NAMES = {
    RobotStatus.STATE_CHARGING: "CHARGING",
    RobotStatus.STATE_MOVING_UNLOADED: "MOVING_UNLOADED",
    RobotStatus.STATE_WORKING_LOADING: "WORKING_LOADING",
    RobotStatus.STATE_MOVING_LOADED: "MOVING_LOADED",
    RobotStatus.STATE_WORKING_UNLOADING: "WORKING_UNLOADING",
}


class AMRStateMachineNode(Node):
    def __init__(self):
        super().__init__('amr_state_machine_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('robot_id', 'LSM01')
        self.declare_parameter('initial_battery', 98.5)
        self.declare_parameter('nav_timeout_sec', 180.0)
        self.declare_parameter('event_timeout_sec', 120.0)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.battery_level = float(self.get_parameter('initial_battery').get_parameter_value().double_value)
        self.nav_timeout_sec = float(self.get_parameter('nav_timeout_sec').get_parameter_value().double_value)
        self.event_timeout_sec = float(self.get_parameter('event_timeout_sec').get_parameter_value().double_value)

        self.cb_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()

        # FSM State variables
        self.current_state = RobotStatus.STATE_CHARGING
        self.current_task_id = ""
        self.active_station_id = "wp-charge-1"

        # Robot Pose in ROS map frame & World frame
        self.map_origin_x, self.map_origin_y = self._load_map_origin()
        self.waypoints = self._load_waypoints()

        self.pose_map_x = 2.4
        self.pose_map_y = 0.3
        self.pose_map_qz = 1.0
        self.pose_map_qw = 0.0
        self.pose_world_x = 2.4 - self.map_origin_x
        self.pose_world_y = 0.3 - self.map_origin_y
        self.linear_speed = 0.0
        self.has_pose = False

        # Synchronization events for navigation & workstation completion
        self.nav_started_event = threading.Event()
        self.nav_done_event = threading.Event()
        self.nav_success = False
        self.nav_target_wp = ""

        self.cargo_loaded_event = threading.Event()
        self.cargo_unloaded_event = threading.Event()
        self.last_event_station = ""

        # TF Buffer for live map -> base_footprint lookup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS Profiles
        amcl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        odom_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Publishers
        self.status_pub = self.create_publisher(RobotStatus, '/lsm01/robot_status', 10)
        self.ws_event_pub = self.create_publisher(WorkstationEvent, '/workstation/event', 10)
        self.route_cmd_pub = self.create_publisher(String, '/fms/fsm_route_cmd', 10)

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, amcl_qos, callback_group=self.cb_group)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, odom_qos, callback_group=self.cb_group)
        self.create_subscription(String, '/fms/route_status', self._route_status_cb, 10, callback_group=self.cb_group)
        self.create_subscription(WorkstationEvent, '/workstation/event', self._ws_event_cb, 10, callback_group=self.cb_group)

        # Direct Nav2 fallback ActionClient (used only if telemetry_bridge SequentialNavigator is offline)
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=self.cb_group)

        # Service Server: /workstation/trigger_mock
        self.mock_srv = self.create_service(
            TriggerMockEvent,
            '/workstation/trigger_mock',
            self._handle_trigger_mock,
            callback_group=self.cb_group,
        )

        # Action Server: /fms/dispatch_task
        self.action_server = ActionServer(
            self,
            DispatchTask,
            '/fms/dispatch_task',
            execute_callback=self._execute_dispatch_task,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.cb_group,
        )

        # 10Hz Status & Battery Timer
        self.status_timer = self.create_timer(0.1, self._publish_robot_status, callback_group=self.cb_group)

        self.get_logger().info(
            f"[FSM Ready] robot_id={self.robot_id}, initial_state={STATE_NAMES[self.current_state]} (0), "
            f"loaded {len(self.waypoints)} waypoints, map_origin=({self.map_origin_x}, {self.map_origin_y})"
        )

    def _load_map_origin(self):
        search_paths = [
            "/home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/maps/lsm_warehouse_map.yaml",
        ]
        try:
            search_paths.insert(0, os.path.join(get_package_share_directory('lsm_navigation'), 'maps', 'lsm_warehouse_map.yaml'))
        except Exception:
            pass
        for path in search_paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f)
                        origin = data.get('origin', [-2.4, -0.3, 0.0])
                        return float(origin[0]), float(origin[1])
                except Exception as exc:
                    self.get_logger().warn(f"Failed reading map origin from {path}: {exc}")
        return -2.4, -0.3

    def _load_waypoints(self):
        search_paths = [
            "/home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/config/waypoints_graph.yaml",
        ]
        try:
            search_paths.insert(0, os.path.join(get_package_share_directory('lsm_navigation'), 'config', 'waypoints_graph.yaml'))
        except Exception:
            pass
        for path in search_paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f)
                        return data.get('waypoints', {})
                except Exception as exc:
                    self.get_logger().warn(f"Failed reading waypoints from {path}: {exc}")
        return {}

    def resolve_station_to_waypoint(self, station_id: str) -> str:
        s = (station_id or "").strip()
        if not s:
            return "wp-charge-1"
        if s in self.waypoints:
            return s
        upper = s.upper()
        if upper in ("CHARGE-1", "CHARGE", "CHARGER", "HOME"):
            return "wp-charge-1"
        if upper.startswith("A-") and len(upper) == 3:
            cand = f"wp-ws-a-{upper[2:]}"
            if cand in self.waypoints:
                return cand
        if upper.startswith("C-") and len(upper) == 3:
            cand = f"wp-ws-c-{upper[2:]}"
            if cand in self.waypoints:
                return cand
        if upper.startswith("B-"):
            cand = f"wp-ws-{upper.lower()}"
            if cand in self.waypoints:
                return cand
        if upper.startswith("WS-"):
            cand = f"wp-{upper.lower()}"
            if cand in self.waypoints:
                return cand
        return s

    def _get_wp_world_xy(self, wp_id: str):
        wp = self.waypoints.get(wp_id)
        if not wp:
            return None
        return float(wp.get('x', 0.0)), float(wp.get('y', 0.0)), float(wp.get('yaw_deg', 0.0))

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.pose_map_x = float(msg.pose.pose.position.x)
        self.pose_map_y = float(msg.pose.pose.position.y)
        self.pose_map_qz = float(msg.pose.pose.orientation.z)
        self.pose_map_qw = float(msg.pose.pose.orientation.w)
        self.pose_world_x = self.pose_map_x - self.map_origin_x
        self.pose_world_y = self.pose_map_y - self.map_origin_y
        self.has_pose = True

    def _odom_cb(self, msg: Odometry):
        self.linear_speed = float(msg.twist.twist.linear.x)
        # Update pose via TF map->base_footprint when available
        try:
            if self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                self.pose_map_x = float(t.transform.translation.x)
                self.pose_map_y = float(t.transform.translation.y)
                self.pose_map_qz = float(t.transform.rotation.z)
                self.pose_map_qw = float(t.transform.rotation.w)
                self.pose_world_x = self.pose_map_x - self.map_origin_x
                self.pose_world_y = self.pose_map_y - self.map_origin_y
                self.has_pose = True
        except Exception:
            pass

    def _transition_state(self, new_state: int, reason: str = ""):
        with self.state_lock:
            old_state = self.current_state
            self.current_state = new_state
        old_name = STATE_NAMES.get(old_state, str(old_state))
        new_name = STATE_NAMES.get(new_state, str(new_state))
        task_info = f"task={self.current_task_id}" if self.current_task_id else "task=NONE"
        self.get_logger().info(f"[FSM TRANSITION] {old_name}({old_state}) -> {new_name}({new_state}) | {task_info} | {reason}")

    def _publish_robot_status(self):
        # Update battery smoothly depending on state
        with self.state_lock:
            state = self.current_state
            task_id = self.current_task_id

        if state == RobotStatus.STATE_CHARGING:
            self.battery_level = min(100.0, self.battery_level + 0.02)
        elif state in (RobotStatus.STATE_MOVING_UNLOADED, RobotStatus.STATE_MOVING_LOADED):
            self.battery_level = max(10.0, self.battery_level - 0.005)
        else:
            self.battery_level = max(10.0, self.battery_level - 0.002)

        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.robot_id = self.robot_id
        msg.current_state = int(state)
        msg.pose.position.x = float(self.pose_map_x)
        msg.pose.position.y = float(self.pose_map_y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = float(self.pose_map_qz)
        msg.pose.orientation.w = float(self.pose_map_qw)
        msg.battery_level = float(round(self.battery_level, 2))
        msg.current_task_id = task_id
        self.status_pub.publish(msg)

    def _route_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        event = data.get("event")
        goal_wp = data.get("goal_wp", "")
        if event == "STARTED":
            if not self.nav_target_wp or goal_wp == self.nav_target_wp:
                self.nav_started_event.set()
        elif event == "COMPLETED":
            if not self.nav_target_wp or goal_wp == self.nav_target_wp:
                self.nav_success = True
                self.nav_done_event.set()
        elif event == "FAILED":
            if not self.nav_target_wp or goal_wp == self.nav_target_wp:
                self.nav_success = False
                self.nav_done_event.set()

    def _ws_event_cb(self, msg: WorkstationEvent):
        if msg.event_type == WorkstationEvent.EVENT_CARGO_LOADED:
            self.last_event_station = msg.station_id
            self.cargo_loaded_event.set()
        elif msg.event_type == WorkstationEvent.EVENT_CARGO_UNLOADED:
            self.last_event_station = msg.station_id
            self.cargo_unloaded_event.set()

    def _handle_trigger_mock(self, request, response):
        station = request.station_id or self.active_station_id
        ev_type = int(request.event_type)

        ev = WorkstationEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.header.frame_id = "map"
        ev.station_id = station
        ev.event_type = ev_type
        ev.task_id = self.current_task_id

        if ev_type == TriggerMockEvent.Request.EVENT_CARGO_LOADED:
            self.ws_event_pub.publish(ev)
            self.cargo_loaded_event.set()
            response.success = True
            response.message = f"Mock CARGO_LOADED event triggered for station {station}"
            self.get_logger().info(f"[WorkstationEvent] CARGO_LOADED @ {station} (task={self.current_task_id})")
        elif ev_type == TriggerMockEvent.Request.EVENT_CARGO_UNLOADED:
            self.ws_event_pub.publish(ev)
            self.cargo_unloaded_event.set()
            response.success = True
            response.message = f"Mock CARGO_UNLOADED event triggered for station {station}"
            self.get_logger().info(f"[WorkstationEvent] CARGO_UNLOADED @ {station} (task={self.current_task_id})")
        elif ev_type == WorkstationEvent.EVENT_JOB_DONE:
            self.ws_event_pub.publish(ev)
            response.success = True
            response.message = f"Mock JOB_DONE event triggered for station {station}"
            self.get_logger().info(f"[WorkstationEvent] JOB_DONE @ {station}")
        else:
            response.success = False
            response.message = f"Unsupported event_type: {ev_type}"
        return response

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f"[DispatchTask] Received goal: task_id='{goal_request.task_id}', "
            f"type={goal_request.task_type}, target='{goal_request.target_station_id}'"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().warn("[DispatchTask] Cancel requested")
        return CancelResponse.ACCEPT

    def _compute_distance_to_wp(self, wp_id: str) -> float:
        coords = self._get_wp_world_xy(wp_id)
        if not coords:
            return 0.0
        wx, wy, _ = coords
        return float(math.hypot(wx - self.pose_world_x, wy - self.pose_world_y))

    def _publish_feedback(self, goal_handle, target_wp: str):
        fb = DispatchTask.Feedback()
        with self.state_lock:
            fb.current_state = int(self.current_state)
        dist = self._compute_distance_to_wp(target_wp)
        fb.distance_remaining = float(round(dist, 3))
        speed = max(abs(self.linear_speed), 0.22)
        fb.estimated_time_remaining = float(round(dist / speed, 2))
        goal_handle.publish_feedback(fb)

    def _navigate_to_waypoint(self, goal_handle, target_wp: str) -> bool:
        self.nav_started_event.clear()
        self.nav_done_event.clear()
        self.nav_success = False
        self.nav_target_wp = target_wp

        cmd_payload = {
            "command": "navigate",
            "task_id": self.current_task_id,
            "goal_wp": target_wp,
            "fsm_state": int(self.current_state),
        }
        cmd_msg = String()
        cmd_msg.data = json.dumps(cmd_payload)
        self.route_cmd_pub.publish(cmd_msg)
        self.get_logger().info(f"[FSM -> SequentialNavigator] Requested A* route to '{target_wp}'")

        # Wait briefly for SequentialNavigator ack; if offline, fall back to direct Nav2 action
        if not self.nav_started_event.wait(timeout=2.5):
            self.get_logger().warn(f"SequentialNavigator did not ack within 2.5s; falling back to direct Nav2 for '{target_wp}'")
            return self._navigate_direct_nav2(goal_handle, target_wp)

        t0 = time.monotonic()
        while rclpy.ok() and not self.nav_done_event.is_set():
            if goal_handle.is_cancel_requested:
                return False
            if time.monotonic() - t0 > self.nav_timeout_sec:
                self.get_logger().error(f"Navigation to '{target_wp}' timed out after {self.nav_timeout_sec}s")
                return False
            self._publish_feedback(goal_handle, target_wp)
            self.nav_done_event.wait(timeout=0.5)

        return self.nav_success

    def _navigate_direct_nav2(self, goal_handle, target_wp: str) -> bool:
        coords = self._get_wp_world_xy(target_wp)
        if not coords:
            self.get_logger().error(f"Waypoint '{target_wp}' not found in graph")
            return False
        wx, wy, yaw_deg = coords
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 /navigate_to_pose action server unavailable")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wx + self.map_origin_x
        goal_msg.pose.pose.position.y = wy + self.map_origin_y
        yaw_rad = math.radians(yaw_deg)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        send_future = self.nav2_client.send_goal_async(goal_msg)
        while rclpy.ok() and not send_future.done():
            time.sleep(0.05)
        nav_handle = send_future.result()
        if not nav_handle or not nav_handle.accepted:
            return False
        res_future = nav_handle.get_result_async()
        while rclpy.ok() and not res_future.done():
            if goal_handle.is_cancel_requested:
                nav_handle.cancel_goal_async()
                return False
            self._publish_feedback(goal_handle, target_wp)
            time.sleep(0.5)
        res = res_future.result()
        return bool(res and res.status == GoalStatus.STATUS_SUCCEEDED)

    def _wait_for_workstation_event(self, goal_handle, event_obj: threading.Event, target_wp: str, label: str) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and not event_obj.is_set():
            if goal_handle.is_cancel_requested:
                return False
            if time.monotonic() - t0 > self.event_timeout_sec:
                self.get_logger().error(f"Timed out waiting for {label} event at '{target_wp}'")
                return False
            self._publish_feedback(goal_handle, target_wp)
            event_obj.wait(timeout=0.5)
        return True

    def _execute_dispatch_task(self, goal_handle):
        req = goal_handle.request
        start_time = time.monotonic()
        self.current_task_id = req.task_id or f"TASK_{int(time.time())}"

        result = DispatchTask.Result()
        raw_target = (req.target_station_id or "").strip()

        try:
            # Multi-leg full 5-stage logistics mission: "PICKUP->DELIVERY" or "PICKUP->DELIVERY->CHARGER"
            if "->" in raw_target:
                parts = [self.resolve_station_to_waypoint(p) for p in raw_target.split("->") if p.strip()]
                pickup_wp = parts[0]
                delivery_wp = parts[1] if len(parts) > 1 else "wp-ws-a-5"
                return_wp = parts[2] if len(parts) > 2 else "wp-charge-1"

                self.get_logger().info(
                    f"[E2E Mission Start] {self.current_task_id}: {pickup_wp} -> {delivery_wp} -> {return_wp}"
                )

                # Leg 1: CHARGING (0) -> MOVING_UNLOADED (1) -> Pickup WP
                self.active_station_id = pickup_wp
                self._transition_state(RobotStatus.STATE_MOVING_UNLOADED, f"Dispatched to pickup {pickup_wp}")
                if not self._navigate_to_waypoint(goal_handle, pickup_wp):
                    raise RuntimeError(f"Failed navigating to pickup station {pickup_wp}")

                # Stage 2: WORKING_LOADING (2) & wait for TriggerMockEvent CARGO_LOADED (2)
                self.cargo_loaded_event.clear()
                self._transition_state(RobotStatus.STATE_WORKING_LOADING, f"Arrived at pickup {pickup_wp}; awaiting CARGO_LOADED")
                if not self._wait_for_workstation_event(goal_handle, self.cargo_loaded_event, pickup_wp, "CARGO_LOADED"):
                    raise RuntimeError("Cancelled or timed out waiting for CARGO_LOADED")

                # Leg 2: MOVING_LOADED (3) -> Delivery WP
                self.active_station_id = delivery_wp
                self._transition_state(RobotStatus.STATE_MOVING_LOADED, f"Cargo loaded; transporting to {delivery_wp}")
                if not self._navigate_to_waypoint(goal_handle, delivery_wp):
                    raise RuntimeError(f"Failed navigating to delivery station {delivery_wp}")

                # Stage 4: WORKING_UNLOADING (4) & wait for TriggerMockEvent CARGO_UNLOADED (3)
                self.cargo_unloaded_event.clear()
                self._transition_state(RobotStatus.STATE_WORKING_UNLOADING, f"Arrived at delivery {delivery_wp}; awaiting CARGO_UNLOADED")
                if not self._wait_for_workstation_event(goal_handle, self.cargo_unloaded_event, delivery_wp, "CARGO_UNLOADED"):
                    raise RuntimeError("Cancelled or timed out waiting for CARGO_UNLOADED")

                # Leg 3: Return to charger (MOVING_UNLOADED -> CHARGING)
                self.active_station_id = return_wp
                self._transition_state(RobotStatus.STATE_MOVING_UNLOADED, f"Cargo unloaded; returning to charger {return_wp}")
                if not self._navigate_to_waypoint(goal_handle, return_wp):
                    raise RuntimeError(f"Failed returning to charger {return_wp}")

                self._transition_state(RobotStatus.STATE_CHARGING, f"Docked at {return_wp}; cycle complete")
                self.current_task_id = ""
                result.success = True
                result.message = f"Full 5-stage E2E logistics mission completed ({pickup_wp} -> {delivery_wp} -> {return_wp})"
                result.total_execution_time = float(round(time.monotonic() - start_time, 2))
                goal_handle.succeed()
                return result

            # Single-stage task dispatch mode
            target_wp = self.resolve_station_to_waypoint(raw_target)
            self.active_station_id = target_wp

            if req.task_type == DispatchTask.Goal.TASK_TYPE_PICKUP:
                self._transition_state(RobotStatus.STATE_MOVING_UNLOADED, f"Pickup task to {target_wp}")
                if not self._navigate_to_waypoint(goal_handle, target_wp):
                    raise RuntimeError(f"Pickup navigation failed for {target_wp}")
                self._transition_state(RobotStatus.STATE_WORKING_LOADING, f"Arrived at pickup {target_wp}")
                result.message = f"Arrived at pickup {target_wp}; state=WORKING_LOADING"

            elif req.task_type == DispatchTask.Goal.TASK_TYPE_DELIVERY:
                self._transition_state(RobotStatus.STATE_MOVING_LOADED, f"Delivery task to {target_wp}")
                if not self._navigate_to_waypoint(goal_handle, target_wp):
                    raise RuntimeError(f"Delivery navigation failed for {target_wp}")
                self._transition_state(RobotStatus.STATE_WORKING_UNLOADING, f"Arrived at delivery {target_wp}")
                result.message = f"Arrived at delivery {target_wp}; state=WORKING_UNLOADING"

            elif req.task_type == DispatchTask.Goal.TASK_TYPE_RETURN_HOME:
                self._transition_state(RobotStatus.STATE_MOVING_UNLOADED, f"Return home task to {target_wp}")
                if not self._navigate_to_waypoint(goal_handle, target_wp):
                    raise RuntimeError(f"Return home navigation failed for {target_wp}")
                self._transition_state(RobotStatus.STATE_CHARGING, f"Docked at {target_wp}")
                self.current_task_id = ""
                result.message = f"Returned to {target_wp}; state=CHARGING"
            else:
                raise ValueError(f"Unsupported task_type: {req.task_type}")

            result.success = True
            result.total_execution_time = float(round(time.monotonic() - start_time, 2))
            goal_handle.succeed()
            return result

        except Exception as exc:
            self.get_logger().error(f"[DispatchTask Failed] {exc}")
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            result.success = False
            result.message = str(exc)
            result.total_execution_time = float(round(time.monotonic() - start_time, 2))
            self.current_task_id = ""
            return result


def main(args=None):
    if "ROS_DOMAIN_ID" not in os.environ:
        os.environ["ROS_DOMAIN_ID"] = "13"

    rclpy.init(args=args)
    node = AMRStateMachineNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

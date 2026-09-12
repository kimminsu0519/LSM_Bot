#!/usr/bin/env python3
"""
LSM FMS Telemetry & Route Dispatch WebSocket Bridge Server
Author: Antigravity AI
Package: lsm_fms_web

Provides real-time bi-directional telemetry streaming between ROS2 topics and FMS Web Dashboards
(index.html and developer.html) via zero-dependency pure Python WebSocket (Port 9090).
"""

import os
import sys
import json
import math
import time
import socket
import select
import struct
import base64
import hashlib
import threading

try:
    from lsm_fms_web.lsm_fms_route_server import TopologicalGraph
except ModuleNotFoundError:
    from lsm_fms_route_server import TopologicalGraph

HAS_ROS2 = False
HAS_NAV2_ACTION = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
    from nav_msgs.msg import Odometry
    HAS_ROS2 = True
    try:
        from nav2_msgs.action import NavigateToPose
        HAS_NAV2_ACTION = True
    except ImportError:
        HAS_NAV2_ACTION = False
except ImportError:
    HAS_ROS2 = False

WS_PORT = 9090

class PurePythonWebSocketServer:
    def __init__(self, host="0.0.0.0", port=WS_PORT, on_message_cb=None):
        self.host = host
        self.port = port
        self.on_message_cb = on_message_cb
        self.clients = set()
        self.lock = threading.Lock()
        self.running = False
        self.server_sock = None

    def start(self):
        self.running = True
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(5)
        print(f"[WebSocket Bridge] Server listening on ws://{self.host}:{self.port}")

        thread = threading.Thread(target=self._accept_loop, daemon=True)
        thread.start()

    def _accept_loop(self):
        while self.running:
            try:
                r, _, _ = select.select([self.server_sock], [], [], 0.5)
                if not r:
                    continue
                client_sock, addr = self.server_sock.accept()
                threading.Thread(target=self._handle_client, args=(client_sock, addr), daemon=True).start()
            except Exception as e:
                if self.running:
                    print(f"[WebSocket Bridge] Accept error: {e}")

    def _handle_client(self, sock, addr):
        # 1. Perform HTTP WebSocket Handshake
        try:
            req_data = sock.recv(2048).decode('utf-8', errors='ignore')
            key = None
            for line in req_data.split("\r\n"):
                if line.lower().startswith("sec-websocket-key:"):
                    key = line.split(":")[1].strip()
                    break

            if not key:
                sock.close()
                return

            GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
            sha1 = hashlib.sha1((key + GUID).encode('utf-8')).digest()
            accept_key = base64.b64encode(sha1).decode('utf-8')

            resp = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept_key}\r\n\r\n"
            )
            sock.sendall(resp.encode('utf-8'))

            with self.lock:
                self.clients.add(sock)
            print(f"[WebSocket Bridge] Client connected from {addr}")

        except Exception as e:
            sock.close()
            return

        # 2. Read Frame Loop
        buffer = bytearray()
        while self.running:
            try:
                r, _, _ = select.select([sock], [], [], 0.5)
                if not r:
                    continue
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buffer.extend(chunk)

                while len(buffer) >= 2:
                    second_byte = buffer[1]
                    masked = bool(second_byte & 0x80)
                    payload_len = second_byte & 0x7F
                    offset = 2

                    if payload_len == 126:
                        if len(buffer) < 4:
                            break
                        payload_len = struct.unpack("!H", buffer[2:4])[0]
                        offset = 4
                    elif payload_len == 127:
                        if len(buffer) < 10:
                            break
                        payload_len = struct.unpack("!Q", buffer[2:10])[0]
                        offset = 10

                    if masked:
                        if len(buffer) < offset + 4 + payload_len:
                            break
                        mask = buffer[offset:offset+4]
                        offset += 4
                        raw_payload = buffer[offset:offset+payload_len]
                        decoded = bytes([b ^ mask[i % 4] for i, b in enumerate(raw_payload)])
                        offset += payload_len
                    else:
                        if len(buffer) < offset + payload_len:
                            break
                        decoded = bytes(buffer[offset:offset+payload_len])
                        offset += payload_len

                    buffer = buffer[offset:]
                    msg_str = decoded.decode('utf-8', errors='ignore')

                    if self.on_message_cb:
                        self.on_message_cb(sock, msg_str)

            except Exception:
                break

        with self.lock:
            self.clients.discard(sock)
        sock.close()
        print(f"[WebSocket Bridge] Client disconnected from {addr}")

    def broadcast(self, message_dict):
        json_str = json.dumps(message_dict)
        frame = self._build_frame(json_str)

        with self.lock:
            dead_clients = set()
            for client in self.clients:
                try:
                    client.sendall(frame)
                except Exception:
                    dead_clients.add(client)
            self.clients.difference_update(dead_clients)

    def _build_frame(self, message_str):
        payload = message_str.encode('utf-8')
        length = len(payload)
        if length <= 125:
            header = bytes([0x81, length])
        elif length <= 65535:
            header = bytes([0x81, 126]) + struct.pack("!H", length)
        else:
            header = bytes([0x81, 127]) + struct.pack("!Q", length)
        return header + payload

class FMSBridgeTelemetryNode:
    def __init__(self, yaml_path):
        self.yaml_path = yaml_path
        self.graph = TopologicalGraph(yaml_path)
        self.ros_node = None
        self.goal_pub = None
        self.goal_pub_bot = None
        self.action_client = None
        self.action_client_bot = None
        self.initial_pose_pub = None
        self.initial_pose_pub_bot = None
        self.has_pose = False

        # Robot Telemetry State (lsm_bot_1) - Initially OFFLINE until live ROS2 pose is received
        self.robot_state = {
            "type": "telemetry",
            "robot_id": "lsm_bot_1",
            "has_pose": False,
            "x": 0.0,
            "y": 0.0,
            "yaw_deg": 0.0,
            "v": 0.0,
            "w": 0.0,
            "battery": 100,
            "status": "OFFLINE",
            "covX": 0.0,
            "covY": 0.0,
            "covYaw": 0.0
        }

        # Active Navigation Mission State
        self.active_route = []
        self.current_step = 0

        # Start WebSocket Server
        self.ws_server = PurePythonWebSocketServer(port=WS_PORT, on_message_cb=self.handle_ws_message)
        self.ws_server.start()

        # Telemetry Broadcast Timer (10Hz)
        threading.Thread(target=self._broadcast_loop, daemon=True).start()

    def handle_ws_message(self, client_sock, msg_str):
        try:
            data = json.loads(msg_str)
            msg_type = data.get("type")

            if msg_type == "request_route":
                start_wp = data.get("start_wp", "wp-charge-1")
                goal_wp = data.get("goal_wp", "wp-ws-a-1")
                path, dist = self.graph.find_shortest_path(start_wp, goal_wp)
                details = self.graph.get_route_details(path) if path else []

                resp = {
                    "type": "route_plan",
                    "start_wp": start_wp,
                    "goal_wp": goal_wp,
                    "path": path,
                    "total_dist_m": dist,
                    "details": details
                }
                self.ws_server.broadcast(resp)

            elif msg_type == "ws_call_request":
                ws_id = data.get("ws_id", "wp-a-1-1")
                # Find nearest waypoint to robot's current pose
                start_wp = data.get("start_wp")
                if not start_wp:
                    start_wp = self.find_nearest_waypoint(self.robot_state["x"], self.robot_state["y"])
                
                path, dist = self.graph.find_shortest_path(start_wp, ws_id)
                details = self.graph.get_route_details(path) if path else []
                
                self.active_route = path or []
                self.current_step = 0
                self.robot_state["status"] = "NAVIGATING"

                resp = {
                    "type": "route_plan",
                    "start_wp": start_wp,
                    "goal_wp": ws_id,
                    "path": path,
                    "total_dist_m": dist,
                    "details": details
                }
                self.ws_server.broadcast(resp)
                print(f"[TelemetryBridge] WS Call Scenario: {start_wp} -> {ws_id} ({dist:.2f}m)")
                
                # Publish ROS 2 Nav2 Goal if available
                if path and len(path) > 1:
                    target_wp_id = path[-1]
                    target_wp = self.graph.waypoints.get(target_wp_id)
                    if target_wp and self.ros_node and self.goal_pub:
                        self.publish_ros2_goal(target_wp)

            elif msg_type == "dispatch_route":
                goal_wp_id = data.get("goal_wp")
                route = data.get("route", [])
                if not goal_wp_id and route:
                    goal_wp_id = route[-1]

                if goal_wp_id:
                    target_wp = self.graph.waypoints.get(goal_wp_id)
                    if target_wp and self.ros_node and self.goal_pub:
                        self.robot_state["status"] = "NAVIGATING"
                        self.publish_ros2_goal(target_wp)
                        print(f"[TelemetryBridge] Dispatched Route Goal to ROS2 Nav2: '{goal_wp_id}' ({target_wp.get('x')}, {target_wp.get('y')})")

            elif msg_type == "reset_initial_pose":
                wp_id = data.get("wp_id", "wp-charge-1")
                self.publish_initial_pose(wp_id)

            elif msg_type == "save_pose_snapshot":
                snap_dir = os.path.expanduser("~/dev/LSM_repository/lsm_ws/pose_snapshots")
                os.makedirs(snap_dir, exist_ok=True)
                from datetime import datetime
                ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                
                ros_x = float(data.get("ros_x", self.robot_state.get("ros_x", 0.0)))
                ros_y = float(data.get("ros_y", self.robot_state.get("ros_y", 0.0)))
                canvas_x = float(data.get("canvas_x", self.robot_state.get("x", 0.0)))
                canvas_y = float(data.get("canvas_y", self.robot_state.get("y", 0.0)))
                yaw_deg = float(data.get("yaw_deg", self.robot_state.get("yaw_deg", 0.0)))
                gz_x = float(data.get("gz_x", self.robot_state.get("gz_x", 0.0)))
                gz_y = float(data.get("gz_y", self.robot_state.get("gz_y", 0.0)))
                gz_yaw = float(data.get("gz_yaw", self.robot_state.get("gz_yaw", 0.0)))
                image_b64 = data.get("image_b64", "")
                note = data.get("note", "위치 및 모니터링 캡처 디버그 기록")

                # Save canvas image snapshot if provided
                img_filename = f"snapshot_{ts_str}.png"
                img_path = os.path.join(snap_dir, img_filename)
                if image_b64 and "," in image_b64:
                    try:
                        header, b64data = image_b64.split(",", 1)
                        with open(img_path, "wb") as f_img:
                            f_img.write(base64.b64decode(b64data))
                    except Exception as e:
                        print(f"[PoseSnapshot] Image save error: {e}")

                snap_data = {
                    "id": f"snap_{ts_str}",
                    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "note": note,
                    "ros_map_pose": {"x": ros_x, "y": ros_y, "yaw_deg": yaw_deg},
                    "fms_canvas_pose": {"x": canvas_x, "y": canvas_y},
                    "gazebo_odom_pose": {"x": gz_x, "y": gz_y, "yaw_deg": gz_yaw},
                    "offset_gz_ros": {"delta_x": round(gz_x - ros_x, 3), "delta_y": round(gz_y - ros_y, 3)},
                    "image_file": img_filename if image_b64 else None,
                    "image_b64": image_b64
                }

                json_filename = f"snapshot_{ts_str}.json"
                json_path = os.path.join(snap_dir, json_filename)
                with open(json_path, "w", encoding="utf-8") as f_json:
                    json_disk = {k: v for k, v in snap_data.items() if k != "image_b64"}
                    json.dump(json_disk, f_json, indent=2, ensure_ascii=False)

                print(f"[PoseSnapshot] Saved Debug Record: {json_path}")
                self.ws_server.broadcast({
                    "type": "pose_snapshot_saved",
                    "status": "success",
                    "snapshot": snap_data
                })

        except Exception as e:
            print(f"[TelemetryBridge] Error handling WS message: {e}")

    def publish_initial_pose(self, wp_id="wp-charge-1"):
        if not HAS_ROS2 or not self.ros_node:
            return
        target_wp = self.graph.waypoints.get(wp_id)
        if not target_wp:
            return
        try:
            ros_pose = target_wp.get('pose', {})
            pos = ros_pose.get('position', {}) if isinstance(ros_pose, dict) else {}
            ori = ros_pose.get('orientation', {}) if isinstance(ros_pose, dict) else {}

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.ros_node.get_clock().now().to_msg()
            msg.pose.pose.position.x = float(pos.get('x', -0.052))
            msg.pose.pose.position.y = float(pos.get('y', -0.258))
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.z = float(ori.get('qz', 0.0))
            msg.pose.pose.orientation.w = float(ori.get('qw', 1.0))
            msg.pose.covariance[0] = 0.05
            msg.pose.covariance[7] = 0.05
            msg.pose.covariance[35] = 0.05

            if self.initial_pose_pub:
                self.initial_pose_pub.publish(msg)
            if self.initial_pose_pub_bot:
                self.initial_pose_pub_bot.publish(msg)
            print(f"[ROS2 AMCL Bridge] Reset Initial Pose to {wp_id}: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})")
        except Exception as e:
            print(f"[ROS2 AMCL Bridge] Failed to set initial pose: {e}")

    def find_nearest_waypoint(self, rx, ry):
        closest_id = "wp-charge-1"
        min_dist = float('inf')
        for wp_id, wp in self.graph.waypoints.items():
            dist = math.hypot(wp['x'] - rx, wp['y'] - ry)
            if dist < min_dist:
                min_dist = dist
                closest_id = wp_id
        return closest_id

    def publish_ros2_goal(self, target_wp):
        if not HAS_ROS2 or not self.ros_node:
            return
        try:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.header.stamp = self.ros_node.get_clock().now().to_msg()

            # Prefer ROS 2 Gazebo Map pose if stored in 'pose' dict
            ros_pose = target_wp.get('pose', {})
            pos = ros_pose.get('position', {}) if isinstance(ros_pose, dict) else {}
            ori = ros_pose.get('orientation', {}) if isinstance(ros_pose, dict) else {}

            if 'x' in pos and 'y' in pos:
                goal_x = float(pos['x'])
                goal_y = float(pos['y'])
                goal_qz = float(ori.get('qz', 0.0))
                goal_qw = float(ori.get('qw', 1.0))
            else:
                goal_x = float(target_wp.get('x', 0.0))
                goal_y = float(target_wp.get('y', 0.0))
                yaw_rad = math.radians(target_wp.get('yaw_deg', 0.0))
                goal_qz = math.sin(yaw_rad / 2.0)
                goal_qw = math.cos(yaw_rad / 2.0)

            pose_msg.pose.position.x = goal_x
            pose_msg.pose.position.y = goal_y
            pose_msg.pose.position.z = 0.0

            pose_msg.pose.orientation.z = goal_qz
            pose_msg.pose.orientation.w = goal_qw

            # 1. Topic Publish (/goal_pose and /lsm_bot_1/goal_pose)
            if self.goal_pub:
                self.goal_pub.publish(pose_msg)
            if self.goal_pub_bot:
                self.goal_pub_bot.publish(pose_msg)
            print(f"[ROS2 Nav2 Bridge] Published ROS2 Map Goal Pose: x={goal_x:.3f}, y={goal_y:.3f} to /goal_pose")

            # 2. Action Client Dispatch (/navigate_to_pose)
            if HAS_NAV2_ACTION:
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = pose_msg

                def send_action(client, action_name):
                    if client and client.wait_for_server(timeout_sec=0.2):
                        client.send_goal_async(goal_msg)
                        print(f"[ROS2 Action Bridge] Sent NavigateToPose goal to {action_name}")
                        return True
                    return False

                sent = send_action(self.action_client, '/navigate_to_pose')
                if not sent:
                    send_action(self.action_client_bot, '/lsm_bot_1/navigate_to_pose')

        except Exception as e:
            print(f"[ROS2 Nav2 Bridge] Failed to publish goal pose: {e}")

    def _broadcast_loop(self):
        while True:
            # Broadcast telemetry message
            self.ws_server.broadcast(self.robot_state)
            # When offline (no ROS2 pose), broadcast 0.5Hz ping to prevent log spam
            if not self.robot_state.get("has_pose", False):
                time.sleep(2.0)
            else:
                time.sleep(0.1)

    def update_amcl_pose(self, ros_x, ros_y, yaw_deg, cov_x=0.002, cov_y=0.002, cov_yaw=0.01):
        self.has_pose = True
        self.robot_state["has_pose"] = True
        if self.robot_state["status"] == "OFFLINE":
            self.robot_state["status"] = "IDLE"
        
        # Store raw ROS2 Gazebo map coordinates
        self.robot_state["ros_x"] = round(ros_x, 3)
        self.robot_state["ros_y"] = round(ros_y, 3)

        # Map ROS2 Gazebo pose to Metric Canvas coordinates (+2.457m, +0.358m offset)
        self.robot_state["x"] = round(ros_x + 2.457, 3)
        self.robot_state["y"] = round(ros_y + 0.358, 3)
        self.robot_state["yaw_deg"] = round(yaw_deg, 1)
        self.robot_state["covX"] = round(cov_x, 4)
        self.robot_state["covY"] = round(cov_y, 4)
        self.robot_state["covYaw"] = round(cov_yaw, 4)

    def update_odom(self, v, w, gz_x=None, gz_y=None, gz_yaw=None):
        self.has_pose = True
        self.robot_state["has_pose"] = True
        if self.robot_state["status"] == "OFFLINE":
            self.robot_state["status"] = "IDLE"
        self.robot_state["v"] = round(v, 2)
        self.robot_state["w"] = round(w, 2)
        if gz_x is not None and gz_y is not None and gz_yaw is not None:
            self.robot_state["gz_x"] = round(gz_x, 3)
            self.robot_state["gz_y"] = round(gz_y, 3)
            self.robot_state["gz_yaw"] = round(gz_yaw, 1)

def main():
    if "ROS_DOMAIN_ID" not in os.environ:
        os.environ["ROS_DOMAIN_ID"] = "13"
        print("[ROS2 Environment] Set default ROS_DOMAIN_ID=13")
    else:
        print(f"[ROS2 Environment] Using ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}")

    search_paths = [
        "/home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/config/waypoints_graph2.yaml",
        "/home/kms/dev/LSM_repository/lsm_ws/src/lsm_fms_web/web/config/waypoints_graph2.yaml",
    ]
    if HAS_ROS2:
        try:
            from ament_index_python.packages import get_package_share_directory
            search_paths.insert(0, os.path.join(get_package_share_directory('lsm_navigation'), 'config', 'waypoints_graph2.yaml'))
            search_paths.insert(1, os.path.join(get_package_share_directory('lsm_fms_web'), 'web', 'config', 'waypoints_graph2.yaml'))
        except Exception:
            pass

    default_yaml = None
    for p in search_paths:
        if os.path.exists(p):
            default_yaml = p
            break

    print("=== LSM FMS Telemetry & Route Dispatch WebSocket Bridge ===")
    print(f"Graph Data: {default_yaml}")

    bridge = FMSBridgeTelemetryNode(default_yaml)

    if HAS_ROS2:
        rclpy.init()
        node = Node('lsm_fms_telemetry_bridge_node')
        try:
            node.declare_parameter('use_sim_time', True)
        except Exception:
            pass
        bridge.ros_node = node

        # QoS Profile for AMCL Pose (TRANSIENT_LOCAL is required by Nav2 AMCL)
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

        # Publishers
        bridge.goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
        bridge.goal_pub_bot = node.create_publisher(PoseStamped, '/lsm_bot_1/goal_pose', 10)
        bridge.initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        bridge.initial_pose_pub_bot = node.create_publisher(PoseWithCovarianceStamped, '/lsm_bot_1/initialpose', 10)
        
        # Action Clients
        if HAS_NAV2_ACTION:
            bridge.action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
            bridge.action_client_bot = ActionClient(node, NavigateToPose, '/lsm_bot_1/navigate_to_pose')
        
        # Subscriptions
        # Subscriptions & TF Listener for continuous real-time pose tracking
        try:
            import tf2_ros
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer, node)

            def tf_timer_cb():
                try:
                    target_frame = 'map'
                    if not tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                        target_frame = 'odom'
                    
                    if tf_buffer.can_transform(target_frame, 'base_footprint', rclpy.time.Time()):
                        t = tf_buffer.lookup_transform(target_frame, 'base_footprint', rclpy.time.Time())
                        rx = t.transform.translation.x
                        ry = t.transform.translation.y
                        ori = t.transform.rotation
                        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
                        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
                        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                        bridge.update_amcl_pose(rx, ry, yaw_deg)
                except Exception:
                    pass

            node.create_timer(0.1, tf_timer_cb)
            print("[ROS2 Active] Started 10Hz TF Listener for live pose sync (map/odom -> base_footprint)...")
        except Exception as e:
            print(f"[ROS2 Active] TF Listener initialization warning: {e}")

        def amcl_cb(msg):
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            yaw_deg = math.degrees(yaw_rad)

            cov = msg.pose.covariance
            bridge.update_amcl_pose(pos.x, pos.y, yaw_deg, cov[0], cov[7], cov[35])

        def odom_cb(msg):
            v = msg.twist.twist.linear.x
            w = msg.twist.twist.angular.z
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
            yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
            
            bridge.update_odom(v, w, pos.x, pos.y, yaw_deg)
            # If TF hasn't set pose yet, use raw odom pose as fallback
            if not bridge.has_pose:
                bridge.update_amcl_pose(pos.x, pos.y, yaw_deg)

        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl_cb, amcl_qos)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl_cb, 10)
        node.create_subscription(PoseWithCovarianceStamped, '/lsm_bot_1/amcl_pose', amcl_cb, amcl_qos)
        node.create_subscription(PoseWithCovarianceStamped, '/lsm_bot_1/amcl_pose', amcl_cb, 10)

        node.create_subscription(Odometry, '/odom', odom_cb, odom_qos)
        node.create_subscription(Odometry, '/lsm_bot_1/odom', odom_cb, odom_qos)

        print("[ROS2 Active] Listening to /amcl_pose (TRANSIENT_LOCAL), /odom and connected to Gazebo Nav2 /goal_pose...")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[Standalone Mode] ROS2 not loaded. Running standalone WebSocket telemetry bridge...")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

if __name__ == '__main__':
    main()

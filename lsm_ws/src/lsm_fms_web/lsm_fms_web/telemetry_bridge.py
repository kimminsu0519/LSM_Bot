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
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
    HAS_ROS2 = True
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

        # Robot Telemetry State (lsm_bot_1)
        self.robot_state = {
            "type": "telemetry",
            "robot_id": "lsm_bot_1",
            "x": 0.0,
            "y": 0.0,
            "yaw_deg": 0.0,
            "v": 0.0,
            "w": 0.0,
            "battery": 100,
            "status": "IDLE",
            "covX": 0.002,
            "covY": 0.002,
            "covYaw": 0.01
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

            elif msg_type == "dispatch_route":
                self.active_route = data.get("route", [])
                self.current_step = 0
                self.robot_state["status"] = "NAVIGATING"
                print(f"[TelemetryBridge] Route Dispatched: {self.active_route}")
                
                resp = {
                    "type": "route_progress",
                    "robot_id": self.robot_state["robot_id"],
                    "current_step": 0,
                    "route": self.active_route
                }
                self.ws_server.broadcast(resp)

        except Exception as e:
            print(f"[TelemetryBridge] Error handling WS message: {e}")

    def _broadcast_loop(self):
        while True:
            # Broadcast 10Hz telemetry message
            self.ws_server.broadcast(self.robot_state)
            time.sleep(0.1)

    def update_amcl_pose(self, x, y, yaw_deg, cov_x=0.002, cov_y=0.002, cov_yaw=0.01):
        self.robot_state["x"] = round(x, 3)
        self.robot_state["y"] = round(y, 3)
        self.robot_state["yaw_deg"] = round(yaw_deg, 1)
        self.robot_state["covX"] = round(cov_x, 4)
        self.robot_state["covY"] = round(cov_y, 4)
        self.robot_state["covYaw"] = round(cov_yaw, 4)

def main():
    # Find waypoints_graph2.yaml
    default_yaml = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../lsm_navigation/config/waypoints_graph2.yaml'))
    if not os.path.exists(default_yaml):
        default_yaml = os.path.abspath(os.path.join(os.path.dirname(__file__), '../web/config/waypoints_graph2.yaml'))

    print("=== LSM FMS Telemetry & Route Dispatch WebSocket Bridge ===")
    print(f"Graph Data: {default_yaml}")

    bridge = FMSBridgeTelemetryNode(default_yaml)

    if HAS_ROS2:
        rclpy.init()
        node = Node('lsm_fms_telemetry_bridge_node')
        
        def amcl_cb(msg):
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            # Quaternion to Yaw
            siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            yaw_deg = math.degrees(yaw_rad)

            cov = msg.pose.covariance
            bridge.update_amcl_pose(pos.x, pos.y, yaw_deg, cov[0], cov[7], cov[35])

        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl_cb, 10)
        node.create_subscription(PoseWithCovarianceStamped, '/lsm_bot_1/amcl_pose', amcl_cb, 10)

        print("[ROS2 Active] Listening to /amcl_pose and /lsm_bot_1/amcl_pose topics...")
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

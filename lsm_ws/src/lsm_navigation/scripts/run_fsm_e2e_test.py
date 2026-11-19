#!/usr/bin/env python3
"""
LSM Bot 5-Stage Logistics FSM End-to-End (E2E) Integration Test Runner
Package: lsm_navigation

Exercises the full 5-stage logistics mission cycle on live Gazebo/Nav2/EKF/FSM stack:
  0: CHARGING
  1: MOVING_UNLOADED  (wp-charge-1 -> wp-ws-a-4)
  2: WORKING_LOADING  (@ wp-ws-a-4, triggers /workstation/trigger_mock EVENT_CARGO_LOADED=2)
  3: MOVING_LOADED    (wp-ws-a-4 -> wp-ws-a-5)
  4: WORKING_UNLOADING (@ wp-ws-a-5, triggers /workstation/trigger_mock EVENT_CARGO_UNLOADED=3)
  1: MOVING_UNLOADED  (wp-ws-a-5 -> wp-charge-1)
  0: CHARGING         (mission complete)

Simultaneously logs Gazebo Ground Truth (/ground_truth_poses), Raw Wheel Odometry (/odom),
EKF Filtered Odometry (/odometry/filtered), and AMCL Global Pose (/amcl_pose & /lsm01/robot_status)
to quantify 28.14m 3-Leg full-cycle localization accuracy, odometry drift, and station docking error.
"""

import os
import sys
import math
import time
import threading
from datetime import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from lsm_interfaces.msg import RobotStatus, WorkstationEvent
from lsm_interfaces.srv import TriggerMockEvent
from lsm_interfaces.action import DispatchTask


STATE_NAMES = {
    0: "CHARGING",
    1: "MOVING_UNLOADED",
    2: "WORKING_LOADING",
    3: "MOVING_LOADED",
    4: "WORKING_UNLOADING",
}

STATION_WORLD_COORDS = {
    "wp-charge-1": (2.40, 0.30),
    "wp-ws-a-4":   (4.50, 2.10),
    "wp-ws-a-5":   (6.90, 2.10),
}


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def to_local_se2(pose, origin):
    dx = pose[0] - origin[0]
    dy = pose[1] - origin[1]
    c = math.cos(origin[2])
    s = math.sin(origin[2])
    lx = dx * c + dy * s
    ly = -dx * s + dy * c
    lyaw = wrap_pi(pose[2] - origin[2])
    return lx, ly, lyaw


class FSME2ETestRunner(Node):
    def __init__(self, log_path: str):
        super().__init__('fsm_e2e_test_runner')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.log_path = log_path
        self.log_lines = []
        self.lock = threading.Lock()

        self.latest_status = None
        self.last_state = None
        self.transitions = []
        self.status_msg_count = 0

        self.gt_pose = None
        self.raw_pose = None
        self.ekf_pose = None
        self.amcl_cov = (0.0, 0.0, 0.0)

        self.gt0 = None
        self.raw0 = None
        self.ekf0 = None

        self.raw_pos_errs = []
        self.ekf_pos_errs = []
        self.amcl_pos_errs = []
        self.raw_yaw_errs = []
        self.ekf_yaw_errs = []
        self.amcl_yaw_errs = []
        self.map_odom_jumps = []

        self.loading_triggered = False
        self.unloading_triggered = False
        self.nav_flag_created = False

        qos = QoSProfile(
            depth=20,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(RobotStatus, '/lsm01/robot_status', self._status_cb, 10)
        self.create_subscription(TFMessage, '/ground_truth_poses', self._gt_cb, qos)
        self.create_subscription(Odometry, '/odom', self._raw_cb, qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._ekf_cb, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.dispatch_client = ActionClient(self, DispatchTask, '/fms/dispatch_task')
        self.mock_client = self.create_client(TriggerMockEvent, '/workstation/trigger_mock')

    def publish_charger_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = 1.0
        msg.pose.pose.orientation.w = 0.0
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.02
        self.init_pose_pub.publish(msg)

    def _gt_cb(self, msg: TFMessage):
        for tf in msg.transforms:
            if 'LSM01_Waffle' in tf.child_frame_id and '/' not in tf.child_frame_id and 'link' not in tf.child_frame_id:
                t = tf.transform.translation
                r = tf.transform.rotation
                self.gt_pose = (float(t.x), float(t.y), quat_to_yaw(r))
                return
        if msg.transforms and msg.transforms[0].child_frame_id == '':
            tf = msg.transforms[0]
            t = tf.transform.translation
            r = tf.transform.rotation
            self.gt_pose = (float(t.x), float(t.y), quat_to_yaw(r))

    def _raw_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.raw_pose = (float(p.x), float(p.y), quat_to_yaw(q))

    def _ekf_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.ekf_pose = (float(p.x), float(p.y), quat_to_yaw(q))

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        self.amcl_cov = (float(c[0]), float(c[7]), float(c[35]))

    def _record(self, line: str):
        now_wall = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        sim_sec = self.get_clock().now().nanoseconds * 1e-9
        entry = f"[{now_wall} | sim={sim_sec:7.2f}s] {line}"
        print(entry, flush=True)
        with self.lock:
            self.log_lines.append(entry)

    def _sample_telemetry(self, msg: RobotStatus):
        if self.gt_pose is None or self.raw_pose is None or self.ekf_pose is None:
            return
        if self.gt0 is None:
            self.gt0 = self.gt_pose
            self.raw0 = self.raw_pose
            self.ekf0 = self.ekf_pose

        gx, gy, gyaw = to_local_se2(self.gt_pose, self.gt0)
        rx, ry, ryaw = to_local_se2(self.raw_pose, self.raw0)
        ex, ey, eyaw = to_local_se2(self.ekf_pose, self.ekf0)

        # AMCL global pose in world coordinates: world = map + (2.4, 0.3)
        amcl_wx = float(msg.pose.position.x) + 2.4
        amcl_wy = float(msg.pose.position.y) + 0.3
        amcl_wyaw = quat_to_yaw(msg.pose.orientation)

        raw_err = math.hypot(rx - gx, ry - gy)
        ekf_err = math.hypot(ex - gx, ey - gy)
        amcl_err = math.hypot(amcl_wx - self.gt_pose[0], amcl_wy - self.gt_pose[1])

        raw_y_err = abs(math.degrees(wrap_pi(ryaw - gyaw)))
        ekf_y_err = abs(math.degrees(wrap_pi(eyaw - gyaw)))
        amcl_y_err = abs(math.degrees(wrap_pi(amcl_wyaw - self.gt_pose[2])))

        self.raw_pos_errs.append(raw_err)
        self.ekf_pos_errs.append(ekf_err)
        self.amcl_pos_errs.append(amcl_err)
        self.raw_yaw_errs.append(raw_y_err)
        self.ekf_yaw_errs.append(ekf_y_err)
        self.amcl_yaw_errs.append(amcl_y_err)

    def _status_cb(self, msg: RobotStatus):
        self.status_msg_count += 1
        self.latest_status = msg
        self._sample_telemetry(msg)

        s = int(msg.current_state)
        if s != self.last_state:
            sname = STATE_NAMES.get(s, str(s))
            old_name = STATE_NAMES.get(self.last_state, "INIT") if self.last_state is not None else "INIT"
            px = msg.pose.position.x
            py = msg.pose.position.y
            wx = px + 2.4
            wy = py + 0.3

            gt_info = ""
            if self.gt_pose is not None:
                gwx, gwy, gwyaw = self.gt_pose
                amcl_gt_dist = math.hypot(wx - gwx, wy - gwy)
                gt_info = f" | GT_world=({gwx:.3f}, {gwy:.3f}) | AMCL_vs_GT={amcl_gt_dist*100.0:.2f}cm"

                # Check target station error if arriving at station
                st_target = None
                if s == 2:
                    st_target = ("wp-ws-a-4", STATION_WORLD_COORDS["wp-ws-a-4"])
                elif s == 4:
                    st_target = ("wp-ws-a-5", STATION_WORLD_COORDS["wp-ws-a-5"])
                elif s == 0 and self.last_state == 1:
                    st_target = ("wp-charge-1", STATION_WORLD_COORDS["wp-charge-1"])

                if st_target is not None:
                    st_id, (tx, ty) = st_target
                    amcl_st_err = math.hypot(wx - tx, wy - ty) * 100.0
                    gt_st_err = math.hypot(gwx - tx, gwy - ty) * 100.0
                    gt_info += f" | STATION[{st_id}] AMCL_err={amcl_st_err:.2f}cm, GT_err={gt_st_err:.2f}cm"

            self._record(
                f"STATE TRANSITION: {old_name}({self.last_state}) -> {sname}({s}) | "
                f"task='{msg.current_task_id}' | map=({px:.3f}, {py:.3f}) | world=({wx:.3f}, {wy:.3f}){gt_info} | "
                f"battery={msg.battery_level:.1f}%"
            )
            self.transitions.append((s, sname))
            self.last_state = s

    def call_mock_trigger(self, station_id: str, event_type: int):
        req = TriggerMockEvent.Request()
        req.station_id = station_id
        req.event_type = int(event_type)
        ev_label = "CARGO_LOADED(2)" if event_type == 2 else "CARGO_UNLOADED(3)"
        self._record(f"SERVICE CALL -> /workstation/trigger_mock(station_id='{station_id}', event_type={ev_label})")
        fut = self.mock_client.call_async(req)
        t_wait = time.time()
        while rclpy.ok() and not fut.done() and time.time() - t_wait < 5.0:
            time.sleep(0.05)
        if fut.done():
            res = fut.result()
            self._record(f"SERVICE RESP <- success={res.success}, message='{res.message}'")
            return res.success
        self._record("SERVICE RESP <- TIMEOUT")
        return False

    def record_summary_metrics(self):
        if not self.raw_pos_errs:
            return
        raw_rmse = float(np.sqrt(np.mean(np.square(self.raw_pos_errs))))
        ekf_rmse = float(np.sqrt(np.mean(np.square(self.ekf_pos_errs))))
        amcl_rmse = float(np.sqrt(np.mean(np.square(self.amcl_pos_errs))))

        raw_max = float(np.max(self.raw_pos_errs))
        ekf_max = float(np.max(self.ekf_pos_errs))
        amcl_max = float(np.max(self.amcl_pos_errs))

        raw_final = float(self.raw_pos_errs[-1])
        ekf_final = float(self.ekf_pos_errs[-1])
        amcl_final = float(self.amcl_pos_errs[-1])

        raw_yaw_rmse = float(np.sqrt(np.mean(np.square(self.raw_yaw_errs))))
        ekf_yaw_rmse = float(np.sqrt(np.mean(np.square(self.ekf_yaw_errs))))
        amcl_yaw_rmse = float(np.sqrt(np.mean(np.square(self.amcl_yaw_errs))))

        raw_yaw_final = float(self.raw_yaw_errs[-1])
        ekf_yaw_final = float(self.ekf_yaw_errs[-1])
        amcl_yaw_final = float(self.amcl_yaw_errs[-1])

        self._record("================ 28.14m 3-LEG E2E LOCALIZATION & DRIFT SUMMARY ================")
        self._record(f"Samples (10Hz): {len(self.raw_pos_errs)} | AMCL Final Cov(xx,yy,yaw)=({self.amcl_cov[0]:.5f}, {self.amcl_cov[1]:.5f}, {self.amcl_cov[2]:.5f})")
        self._record(f"[Dead-Reckoning] Raw /odom         vs GT: XY RMSE={raw_rmse:.4f}m (Max={raw_max:.4f}m, Final={raw_final:.4f}m) | Yaw RMSE={raw_yaw_rmse:.2f}deg (Final={raw_yaw_final:.2f}deg)")
        self._record(f"[Dead-Reckoning] 6-Axis EKF        vs GT: XY RMSE={ekf_rmse:.4f}m (Max={ekf_max:.4f}m, Final={ekf_final:.4f}m) | Yaw RMSE={ekf_yaw_rmse:.2f}deg (Final={ekf_yaw_final:.2f}deg)")
        self._record(f"[Global Pose   ] AMCL + 6-Axis EKF vs GT: XY RMSE={amcl_rmse:.4f}m (Max={amcl_max:.4f}m, Final={amcl_final:.4f}m) | Yaw RMSE={amcl_yaw_rmse:.2f}deg (Final={amcl_yaw_final:.2f}deg)")
        self._record("===============================================================================")

    def save_log(self):
        with open(self.log_path, 'w', encoding='utf-8') as f:
            f.write("\n".join(self.log_lines) + "\n")


def main():
    log_path = "/tmp/fsm_e2e_transitions.log"
    flag_path = "/tmp/fsm_e2e_nav_active"
    if os.path.exists(flag_path):
        os.remove(flag_path)

    rclpy.init()
    node = FSME2ETestRunner(log_path)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node._record("Waiting for /lsm01/robot_status, /fms/dispatch_task, and /workstation/trigger_mock...")
    t_wait_status = time.time()
    while rclpy.ok() and node.latest_status is None:
        if time.time() - t_wait_status > 20.0:
            node._record("ERROR: Timed out waiting 20s for /lsm01/robot_status (check ROS_DOMAIN_ID)!")
            node.save_log()
            return 1
        time.sleep(0.1)

    if not node.dispatch_client.wait_for_server(timeout_sec=10.0):
        node._record("ERROR: /fms/dispatch_task action server not available!")
        node.save_log()
        return 1
    if not node.mock_client.wait_for_service(timeout_sec=10.0):
        node._record("ERROR: /workstation/trigger_mock service not available!")
        node.save_log()
        return 1

    # Ensure AMCL is aligned to wp-charge-1 (map=(0.0, 0.0, yaw=180deg) == world=(2.4, 0.3, yaw=180deg))
    if abs(node.latest_status.pose.position.x) > 0.25 or abs(node.latest_status.pose.position.y) > 0.25:
        node._record("Aligning AMCL /initialpose to wp-charge-1: map=(0.000, 0.000, yaw=180deg) <-> world=(2.400, 0.300)...")
        for _ in range(3):
            node.publish_charger_initial_pose()
            time.sleep(0.4)
        time.sleep(1.0)
        node.gt0 = None
        node.raw_pos_errs.clear()
        node.ekf_pos_errs.clear()
        node.amcl_pos_errs.clear()
        node.raw_yaw_errs.clear()
        node.ekf_yaw_errs.clear()
        node.amcl_yaw_errs.clear()

    time.sleep(0.5)
    task_id = "TASK_A4_A5_E2E_001"
    route_spec = "wp-ws-a-4->wp-ws-a-5->wp-charge-1"

    goal = DispatchTask.Goal()
    goal.task_id = task_id
    goal.task_type = DispatchTask.Goal.TASK_TYPE_PICKUP
    goal.target_station_id = route_spec
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = node.get_clock().now().to_msg()

    node._record(f"ACTION GOAL -> /fms/dispatch_task(task_id='{task_id}', target_station_id='{route_spec}')")
    send_fut = node.dispatch_client.send_goal_async(goal)
    while rclpy.ok() and not send_fut.done():
        time.sleep(0.05)

    goal_handle = send_fut.result()
    if not goal_handle or not goal_handle.accepted:
        node._record("ERROR: DispatchTask goal rejected!")
        node.save_log()
        return 1
    node._record("ACTION ACCEPTED -> DispatchTask goal accepted by amr_state_machine_node")

    res_fut = goal_handle.get_result_async()
    nav_entered_time = None

    while rclpy.ok() and not res_fut.done():
        st = node.latest_status
        if st is not None:
            cur = int(st.current_state)
            if cur == RobotStatus.STATE_MOVING_UNLOADED:
                if nav_entered_time is None:
                    nav_entered_time = time.time()
                elif not node.nav_flag_created and time.time() - nav_entered_time > 6.0:
                    with open(flag_path, 'w') as fp:
                        fp.write("nav_active\n")
                    node.nav_flag_created = True
                    node._record("MID-NAVIGATION FLAG SET -> Ready for RViz2 & FMS Web UI capture")

            if cur == RobotStatus.STATE_WORKING_LOADING and not node.loading_triggered:
                node.loading_triggered = True
                time.sleep(2.0)
                node.call_mock_trigger("wp-ws-a-4", TriggerMockEvent.Request.EVENT_CARGO_LOADED)

            if cur == RobotStatus.STATE_WORKING_UNLOADING and not node.unloading_triggered:
                node.unloading_triggered = True
                time.sleep(2.0)
                node.call_mock_trigger("wp-ws-a-5", TriggerMockEvent.Request.EVENT_CARGO_UNLOADED)

        time.sleep(0.2)

    result_wrap = res_fut.result()
    res = result_wrap.result
    status_code = result_wrap.status
    node._record(
        f"ACTION RESULT <- status={status_code} (SUCCEEDED={status_code == GoalStatus.STATUS_SUCCEEDED}), "
        f"success={res.success}, total_execution_time={res.total_execution_time:.2f}s, message='{res.message}'"
    )
    time.sleep(0.5)
    node.record_summary_metrics()

    seq = [s for s, _ in node.transitions]
    expected = [0, 1, 2, 3, 4, 1, 0]
    node._record(f"FSM SEQUENCE CHECK: observed={seq} | expected={expected} | match={seq == expected}")
    node.save_log()

    rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    return 0 if (res.success and seq == expected) else 1


if __name__ == '__main__':
    sys.exit(main())

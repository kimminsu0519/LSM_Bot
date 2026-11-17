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

Writes a timestamped transition & telemetry log to /tmp/fsm_e2e_transitions.log
and touches /tmp/fsm_e2e_nav_active once moving so screenshots can be captured mid-mission.
"""

import os
import sys
import time
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

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

        self.loading_triggered = False
        self.unloading_triggered = False
        self.nav_flag_created = False

        self.create_subscription(RobotStatus, '/lsm01/robot_status', self._status_cb, 10)
        self.dispatch_client = ActionClient(self, DispatchTask, '/fms/dispatch_task')
        self.mock_client = self.create_client(TriggerMockEvent, '/workstation/trigger_mock')

    def _record(self, line: str):
        now_wall = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        sim_sec = self.get_clock().now().nanoseconds * 1e-9
        entry = f"[{now_wall} | sim={sim_sec:7.2f}s] {line}"
        print(entry, flush=True)
        with self.lock:
            self.log_lines.append(entry)

    def _status_cb(self, msg: RobotStatus):
        self.status_msg_count += 1
        self.latest_status = msg
        s = int(msg.current_state)
        if s != self.last_state:
            sname = STATE_NAMES.get(s, str(s))
            old_name = STATE_NAMES.get(self.last_state, "INIT") if self.last_state is not None else "INIT"
            px = msg.pose.position.x
            py = msg.pose.position.y
            # Convert map -> world using map_origin (-2.4, -0.3)
            wx = px + 2.4
            wy = py + 0.3
            self._record(
                f"STATE TRANSITION: {old_name}({self.last_state}) -> {sname}({s}) | "
                f"task='{msg.current_task_id}' | map=({px:.3f}, {py:.3f}) | world=({wx:.3f}, {wy:.3f}) | "
                f"battery={msg.battery_level:.1f}%"
            )
            self.transitions.append((s, sname))
            self.last_state = s

        if s in (1, 3) and not self.nav_flag_created and self.status_msg_count > 5:
            # Wait ~4s into movement before flagging mid-navigation screenshot readiness
            pass

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

    def save_log(self):
        with open(self.log_path, 'w', encoding='utf-8') as f:
            f.write("\n".join(self.log_lines) + "\n")


def main():
    if "ROS_DOMAIN_ID" not in os.environ:
        os.environ["ROS_DOMAIN_ID"] = "13"

    log_path = "/tmp/fsm_e2e_transitions.log"
    flag_path = "/tmp/fsm_e2e_nav_active"
    if os.path.exists(flag_path):
        os.remove(flag_path)

    rclpy.init()
    node = FSME2ETestRunner(log_path)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node._record("Waiting for /lsm01/robot_status, /fms/dispatch_task, and /workstation/trigger_mock...")
    while rclpy.ok() and node.latest_status is None:
        time.sleep(0.1)

    if not node.dispatch_client.wait_for_server(timeout_sec=10.0):
        node._record("ERROR: /fms/dispatch_task action server not available!")
        node.save_log()
        return 1
    if not node.mock_client.wait_for_service(timeout_sec=10.0):
        node._record("ERROR: /workstation/trigger_mock service not available!")
        node.save_log()
        return 1

    time.sleep(1.0)
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
    node.save_log()

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

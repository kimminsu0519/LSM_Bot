#!/usr/bin/env python3
"""
LSM Bot EKF Wheel-Slip Benchmark & Quantitative Comparison Script
Package: lsm_navigation

Drives LSM01_Waffle through an aggressive slip-inducing maneuver (1.82 rad/s spin,
rapid reversals, S-curve slalom, and straight dash) while synchronously logging:
  1. Gazebo Harmonic Ground Truth Pose (/ground_truth_poses -> LSM01_Waffle)
  2. Raw Differential-Drive Wheel Odometry (/odom)
  3. EKF Sensor-Fused Odometry (/odometry/filtered = /odom + 100Hz /imu)

Outputs:
  - ~/dev/LSM_repository/docs/portfolio_assets/ekf_comparison.csv
  - ~/dev/LSM_repository/docs/portfolio_assets/ekf_comparison_plot.png (300 DPI)
  - ~/dev/LSM_repository/docs/portfolio_assets/04_ekf_comparison_plot.png (300 DPI)
"""

import os
import sys
import csv
import math
import time
import argparse
import threading

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def to_local_frame(pose, origin):
    """Transform (x, y, yaw) into local SE(2) frame anchored at origin=(x0, y0, yaw0)."""
    dx = pose[0] - origin[0]
    dy = pose[1] - origin[1]
    c = math.cos(origin[2])
    s = math.sin(origin[2])
    lx = dx * c + dy * s
    ly = -dx * s + dy * c
    lyaw = wrap_pi(pose[2] - origin[2])
    return lx, ly, lyaw


class EKFSlipExperimentNode(Node):
    def __init__(self):
        super().__init__('ekf_slip_experiment_node')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.lock = threading.Lock()
        self.gt_pose = None       # (x, y, yaw_rad)
        self.raw_pose = None      # (x, y, yaw_rad)
        self.ekf_pose = None      # (x, y, yaw_rad)

        qos = QoSProfile(
            depth=20,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(TFMessage, '/ground_truth_poses', self._gt_cb, qos)
        self.create_subscription(Odometry, '/odom', self._raw_odom_cb, qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._ekf_odom_cb, qos)

    def _gt_cb(self, msg: TFMessage):
        for tf in msg.transforms:
            if 'LSM01_Waffle' in tf.child_frame_id and '/' not in tf.child_frame_id and 'link' not in tf.child_frame_id:
                t = tf.transform.translation
                r = tf.transform.rotation
                with self.lock:
                    self.gt_pose = (float(t.x), float(t.y), quat_to_yaw(r))
                return
        # ros_gz_bridge Pose_V -> TFMessage leaves child_frame_id empty; index 0 is the dynamic model (LSM01_Waffle)
        if msg.transforms and msg.transforms[0].child_frame_id == '':
            tf = msg.transforms[0]
            t = tf.transform.translation
            r = tf.transform.rotation
            with self.lock:
                self.gt_pose = (float(t.x), float(t.y), quat_to_yaw(r))

    def _raw_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.lock:
            self.raw_pose = (float(p.x), float(p.y), quat_to_yaw(q))

    def _ekf_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.lock:
            self.ekf_pose = (float(p.x), float(p.y), quat_to_yaw(q))

    def get_snapshot(self):
        with self.lock:
            return self.gt_pose, self.raw_pose, self.ekf_pose

    def send_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def sim_now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def drive_duration(node: EKFSlipExperimentNode, v: float, w: float, duration_sim_s: float):
    t_start = node.sim_now_sec()
    while rclpy.ok():
        node.send_cmd(v, w)
        if node.sim_now_sec() - t_start >= duration_sim_s:
            break
        time.sleep(0.02)


def generate_plot_and_metrics(csv_path: str, out_plot_paths: list):
    data = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({k: float(v) for k, v in row.items()})

    if len(data) < 10:
        raise RuntimeError(f"Insufficient samples in {csv_path}: {len(data)}")

    t = np.array([r['time_s'] for r in data])
    gt_x = np.array([r['gt_x'] for r in data])
    gt_y = np.array([r['gt_y'] for r in data])
    raw_x = np.array([r['raw_x'] for r in data])
    raw_y = np.array([r['raw_y'] for r in data])
    ekf_x = np.array([r['ekf_x'] for r in data])
    ekf_y = np.array([r['ekf_y'] for r in data])

    raw_pos_err = np.array([r['raw_pos_err_m'] for r in data])
    ekf_pos_err = np.array([r['ekf_pos_err_m'] for r in data])
    raw_yaw_err = np.abs(np.array([r['raw_yaw_err_deg'] for r in data]))
    ekf_yaw_err = np.abs(np.array([r['ekf_yaw_err_deg'] for r in data]))

    raw_xy_rmse = float(np.sqrt(np.mean(raw_pos_err ** 2)))
    ekf_xy_rmse = float(np.sqrt(np.mean(ekf_pos_err ** 2)))
    raw_xy_max = float(np.max(raw_pos_err))
    ekf_xy_max = float(np.max(ekf_pos_err))

    raw_yaw_rmse = float(np.sqrt(np.mean(raw_yaw_err ** 2)))
    ekf_yaw_rmse = float(np.sqrt(np.mean(ekf_yaw_err ** 2)))
    raw_yaw_max = float(np.max(raw_yaw_err))
    ekf_yaw_max = float(np.max(ekf_yaw_err))
    raw_yaw_final = float(raw_yaw_err[-1])
    ekf_yaw_final = float(ekf_yaw_err[-1])

    xy_rmse_imp = (1.0 - ekf_xy_rmse / max(raw_xy_rmse, 1e-6)) * 100.0
    xy_max_imp = (1.0 - ekf_xy_max / max(raw_xy_max, 1e-6)) * 100.0
    yaw_rmse_imp = (1.0 - ekf_yaw_rmse / max(raw_yaw_rmse, 1e-6)) * 100.0
    yaw_max_imp = (1.0 - ekf_yaw_max / max(raw_yaw_max, 1e-6)) * 100.0

    metrics = {
        "samples": len(data),
        "duration_s": float(t[-1] - t[0]),
        "raw_xy_rmse_m": raw_xy_rmse,
        "ekf_xy_rmse_m": ekf_xy_rmse,
        "xy_rmse_improvement_pct": xy_rmse_imp,
        "raw_xy_max_m": raw_xy_max,
        "ekf_xy_max_m": ekf_xy_max,
        "xy_max_improvement_pct": xy_max_imp,
        "raw_yaw_rmse_deg": raw_yaw_rmse,
        "ekf_yaw_rmse_deg": ekf_yaw_rmse,
        "yaw_rmse_improvement_pct": yaw_rmse_imp,
        "raw_yaw_max_deg": raw_yaw_max,
        "ekf_yaw_max_deg": ekf_yaw_max,
        "yaw_max_improvement_pct": yaw_max_imp,
        "raw_yaw_final_deg": raw_yaw_final,
        "ekf_yaw_final_deg": ekf_yaw_final,
    }

    print("\n================ EKF WHEEL-SLIP QUANTITATIVE RESULTS ================")
    print(f"Samples: {metrics['samples']} | Sim Duration: {metrics['duration_s']:.2f} s")
    print(f"XY Trajectory RMSE  : Raw={raw_xy_rmse:.4f} m | EKF={ekf_xy_rmse:.4f} m | Improvement={xy_rmse_imp:+.1f}%")
    print(f"XY Max Position Err : Raw={raw_xy_max:.4f} m | EKF={ekf_xy_max:.4f} m | Improvement={xy_max_imp:+.1f}%")
    print(f"Yaw RMSE            : Raw={raw_yaw_rmse:.3f} deg | EKF={ekf_yaw_rmse:.3f} deg | Improvement={yaw_rmse_imp:+.1f}%")
    print(f"Yaw Max Error       : Raw={raw_yaw_max:.3f} deg | EKF={ekf_yaw_max:.3f} deg | Improvement={yaw_max_imp:+.1f}%")
    print(f"Final Yaw Drift     : Raw={raw_yaw_final:.3f} deg | EKF={ekf_yaw_final:.3f} deg")
    print("=====================================================================\n")

    # Configure clean, publication-grade plot (English labels to guarantee zero font glitches)
    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 10,
        'axes.edgecolor': '#CBD5E1',
        'axes.linewidth': 1.0,
    })

    fig = plt.figure(figsize=(14.5, 6.8), dpi=300)
    fig.patch.set_facecolor('#FFFFFF')
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1.0], hspace=0.36, wspace=0.25)

    ax_traj = fig.add_subplot(gs[:, 0])
    ax_pos = fig.add_subplot(gs[0, 1])
    ax_yaw = fig.add_subplot(gs[1, 1])

    # 1. 2D XY Trajectory Comparison
    ax_traj.plot(gt_x, gt_y, color='#0F172A', linewidth=2.4, label='Gazebo Ground Truth', zorder=3)
    ax_traj.plot(raw_x, raw_y, color='#DC2626', linestyle='--', linewidth=1.9, alpha=0.9, label='Before: Raw Wheel Odom (/odom)', zorder=2)
    ax_traj.plot(ekf_x, ekf_y, color='#0284C7', linewidth=2.1, alpha=0.95, label='After: EKF Fused (/odometry/filtered)', zorder=4)

    ax_traj.scatter([gt_x[0]], [gt_y[0]], color='#16A34A', s=80, marker='o', edgecolors='white', linewidths=1.5, zorder=5, label='Start (0, 0)')
    ax_traj.scatter([gt_x[-1]], [gt_y[-1]], color='#0F172A', s=80, marker='X', zorder=5, label='GT End')
    ax_traj.scatter([raw_x[-1]], [raw_y[-1]], color='#DC2626', s=70, marker='^', zorder=5)
    ax_traj.scatter([ekf_x[-1]], [ekf_y[-1]], color='#0284C7', s=70, marker='s', zorder=5)

    ax_traj.set_title('2D XY Trajectory under Wheel-Slip & High-Rate Turns', fontsize=12, fontweight='bold', pad=10, color='#0F172A')
    ax_traj.set_xlabel('Local X Displacement (m)', fontweight='bold')
    ax_traj.set_ylabel('Local Y Displacement (m)', fontweight='bold')
    ax_traj.grid(True, linestyle=':', alpha=0.6, color='#94A3B8')
    ax_traj.axis('equal')
    ax_traj.legend(loc='upper left', frameon=True, facecolor='#F8FAFC', edgecolor='#CBD5E1', fontsize=8.8)

    summary_text = (
        f"Before vs. After EKF Benchmark\n"
        f"──────────────────────────────\n"
        f"XY RMSE   : {raw_xy_rmse:.3f} m -> {ekf_xy_rmse:.3f} m ({xy_rmse_imp:+.1f}%)\n"
        f"XY Max    : {raw_xy_max:.3f} m -> {ekf_xy_max:.3f} m ({xy_max_imp:+.1f}%)\n"
        f"Yaw RMSE  : {raw_yaw_rmse:.2f}° -> {ekf_yaw_rmse:.2f}° ({yaw_rmse_imp:+.1f}%)\n"
        f"Yaw Max   : {raw_yaw_max:.2f}° -> {ekf_yaw_max:.2f}° ({yaw_max_imp:+.1f}%)"
    )
    ax_traj.text(
        0.025, 0.03, summary_text,
        transform=ax_traj.transAxes,
        fontsize=8.6,
        family='DejaVu Sans Mono',
        verticalalignment='bottom',
        bbox=dict(boxstyle='round,pad=0.55', facecolor='#F8FAFC', edgecolor='#0284C7', linewidth=1.2, alpha=0.95)
    )

    # 2. Euclidean XY Position Error over Time
    ax_pos.plot(t, raw_pos_err, color='#DC2626', linestyle='--', linewidth=1.8, label=f'Raw /odom (RMSE: {raw_xy_rmse:.3f} m)')
    ax_pos.plot(t, ekf_pos_err, color='#0284C7', linewidth=2.0, label=f'EKF Filtered (RMSE: {ekf_xy_rmse:.3f} m)')
    ax_pos.fill_between(t, ekf_pos_err, raw_pos_err, where=(raw_pos_err >= ekf_pos_err), color='#0284C7', alpha=0.10)
    ax_pos.set_title('Euclidean XY Position Error vs. Ground Truth', fontsize=11, fontweight='bold', pad=8, color='#0F172A')
    ax_pos.set_ylabel('Position Error (m)', fontweight='bold')
    ax_pos.grid(True, linestyle=':', alpha=0.6, color='#94A3B8')
    ax_pos.legend(loc='upper left', frameon=True, facecolor='#F8FAFC', edgecolor='#CBD5E1', fontsize=8.5)

    # 3. Absolute Heading (Yaw) Error over Time
    ax_yaw.plot(t, raw_yaw_err, color='#DC2626', linestyle='--', linewidth=1.8, label=f'Raw /odom (RMSE: {raw_yaw_rmse:.2f}°)')
    ax_yaw.plot(t, ekf_yaw_err, color='#0284C7', linewidth=2.0, label=f'EKF Filtered (RMSE: {ekf_yaw_rmse:.2f}°)')
    ax_yaw.fill_between(t, ekf_yaw_err, raw_yaw_err, where=(raw_yaw_err >= ekf_yaw_err), color='#059669', alpha=0.10)
    ax_yaw.set_title('Absolute Heading (Yaw) Drift Error over Time', fontsize=11, fontweight='bold', pad=8, color='#0F172A')
    ax_yaw.set_xlabel('Elapsed Simulation Time (s)', fontweight='bold')
    ax_yaw.set_ylabel('Yaw Error (deg)', fontweight='bold')
    ax_yaw.grid(True, linestyle=':', alpha=0.6, color='#94A3B8')
    ax_yaw.legend(loc='upper left', frameon=True, facecolor='#F8FAFC', edgecolor='#CBD5E1', fontsize=8.5)

    fig.suptitle(
        'LSM Bot EKF Sensor Fusion (Wheel Odometry + 100Hz IMU) Performance under Wheel-Slip',
        fontsize=13.5, fontweight='bold', color='#0F172A', y=0.98
    )

    for p in out_plot_paths:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        fig.savefig(p, dpi=300, bbox_inches='tight')
        print(f"[Plot Saved] {p}")
    plt.close(fig)
    return metrics


def turn_to_world_yaw(node: EKFSlipExperimentNode, target_yaw: float, tol: float = 0.08):
    t_start = node.sim_now_sec()
    while rclpy.ok() and node.sim_now_sec() - t_start < 15.0:
        gt, _, _ = node.get_snapshot()
        err = wrap_pi(target_yaw - gt[2])
        if abs(err) <= tol:
            break
        w = max(-1.2, min(1.2, 1.8 * err))
        if abs(w) < 0.35:
            w = 0.35 if err > 0 else -0.35
        node.send_cmd(0.0, w)
        time.sleep(0.02)
    drive_duration(node, 0.0, 0.0, 0.5)


def move_to_central_aisle(node: EKFSlipExperimentNode):
    """Move robot into unobstructed central East-West aisle (y ~ 5.4, x in [1.5, 4.0], yaw ~ 0 deg)."""
    gt, _, _ = node.get_snapshot()
    # 1. Move X inside [1.5, 3.5] if too close to West wall
    if gt[0] < 1.5:
        turn_to_world_yaw(node, 0.0)
        t_s = node.sim_now_sec()
        while rclpy.ok() and node.sim_now_sec() - t_s < 10.0:
            gt, _, _ = node.get_snapshot()
            if gt[0] >= 2.0:
                break
            node.send_cmd(0.30, 0.0)
            time.sleep(0.02)
        drive_duration(node, 0.0, 0.0, 0.5)

    # 2. Move Y to central aisle (y ~ 5.4)
    gt, _, _ = node.get_snapshot()
    if abs(gt[1] - 5.4) > 0.4:
        target_y_heading = math.pi / 2.0 if gt[1] < 5.4 else -math.pi / 2.0
        turn_to_world_yaw(node, target_y_heading)
        t_s = node.sim_now_sec()
        while rclpy.ok() and node.sim_now_sec() - t_s < 20.0:
            gt, _, _ = node.get_snapshot()
            if (target_y_heading > 0 and gt[1] >= 5.3) or (target_y_heading < 0 and gt[1] <= 5.5):
                break
            node.send_cmd(0.35, 0.0)
            time.sleep(0.02)
        drive_duration(node, 0.0, 0.0, 0.5)

    # 3. Face straight East (yaw = 0.0 rad) down the 35m central aisle
    turn_to_world_yaw(node, 0.0)
    drive_duration(node, 0.0, 0.0, 1.5)


def run_live_experiment(csv_path: str, out_plot_paths: list):
    rclpy.init()
    node = EKFSlipExperimentNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("[EKF Experiment] Waiting for /ground_truth_poses, /odom, /odometry/filtered and sim clock...")
    t_wait = time.time()
    while rclpy.ok():
        gt, raw, ekf = node.get_snapshot()
        if gt is not None and raw is not None and ekf is not None and node.sim_now_sec() > 0.1:
            break
        if time.time() - t_wait > 25.0:
            raise RuntimeError(f"Timed out waiting for topics: gt={gt is not None}, raw={raw is not None}, ekf={ekf is not None}")
        time.sleep(0.05)

    gt, raw, ekf = node.get_snapshot()
    print(f"[EKF Experiment] Streams ready! GT=({gt[0]:.2f}, {gt[1]:.2f}, {math.degrees(gt[2]):.1f} deg)")

    print("[EKF Experiment] Positioning into unobstructed central East-West aisle (y~5.4, yaw~0 deg)...")
    move_to_central_aisle(node)
    gt, raw, ekf = node.get_snapshot()
    print(f"[EKF Experiment] Open-aisle starting pose: GT=({gt[0]:.2f}, {gt[1]:.2f}, {math.degrees(gt[2]):.1f} deg)")

    # Anchor SE(2) local reference frame at t=0
    gt0, raw0, ekf0 = node.get_snapshot()
    t0_sim = node.sim_now_sec()

    # Slip-inducing maneuver schedule: (duration_s, linear_v, angular_w, label)
    phases = [
        (2.0, 0.00, +1.82, "Phase 1a: Max-rate CCW Spin (+1.82 rad/s)"),
        (2.0, 0.00, -1.82, "Phase 1b: Reversal CW Spin (-1.82 rad/s)"),
        (1.3, 0.42, +1.40, "Phase 2a: High-Accel S-Curve Left"),
        (2.5, 0.42, -1.40, "Phase 2b: High-Accel S-Curve Right"),
        (2.5, 0.42, +1.40, "Phase 2c: High-Accel S-Curve Left"),
        (1.3, 0.42, -1.40, "Phase 2d: High-Accel S-Curve Right"),
        (2.8, 0.00, +1.82, "Phase 3a: J-Turn Braking & Spin (+1.82 rad/s)"),
        (4.5, 0.42,  0.00, "Phase 3b: Straight Dash (Yaw->XY Divergence)"),
        (2.0, 0.00,  0.00, "Phase 4: Full Stop & Settle"),
    ]

    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    rows = []
    last_log_sim = -1.0

    print("[EKF Experiment] Starting 23.0s aggressive wheel-slip test maneuver...")
    for dur, v, w, label in phases:
        print(f"  -> {label} (v={v:.2f} m/s, w={w:+.2f} rad/s, dt={dur:.1f}s)")
        p_start = node.sim_now_sec()
        while rclpy.ok():
            node.send_cmd(v, w)
            now_sim = node.sim_now_sec()
            if now_sim - last_log_sim >= 0.05:  # 20Hz synchronized logging
                gt_c, raw_c, ekf_c = node.get_snapshot()
                gx, gy, gyaw = to_local_frame(gt_c, gt0)
                rx, ry, ryaw = to_local_frame(raw_c, raw0)
                ex, ey, eyaw = to_local_frame(ekf_c, ekf0)

                raw_pos_err = math.hypot(rx - gx, ry - gy)
                ekf_pos_err = math.hypot(ex - gx, ey - gy)
                raw_yaw_err = math.degrees(wrap_pi(ryaw - gyaw))
                ekf_yaw_err = math.degrees(wrap_pi(eyaw - gyaw))

                rows.append([
                    round(now_sim - t0_sim, 4),
                    round(gx, 5), round(gy, 5), round(math.degrees(gyaw), 3),
                    round(rx, 5), round(ry, 5), round(math.degrees(ryaw), 3),
                    round(ex, 5), round(ey, 5), round(math.degrees(eyaw), 3),
                    round(raw_pos_err, 5), round(ekf_pos_err, 5),
                    round(raw_yaw_err, 3), round(ekf_yaw_err, 3),
                ])
                last_log_sim = now_sim

            if now_sim - p_start >= dur:
                break
            time.sleep(0.01)

    node.send_cmd(0.0, 0.0)

    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time_s',
            'gt_x', 'gt_y', 'gt_yaw_deg',
            'raw_x', 'raw_y', 'raw_yaw_deg',
            'ekf_x', 'ekf_y', 'ekf_yaw_deg',
            'raw_pos_err_m', 'ekf_pos_err_m',
            'raw_yaw_err_deg', 'ekf_yaw_err_deg',
        ])
        writer.writerows(rows)

    print(f"[CSV Saved] {csv_path} ({len(rows)} rows)")
    node.destroy_node()
    rclpy.shutdown()

    return generate_plot_and_metrics(csv_path, out_plot_paths)


def main():
    default_assets = os.path.expanduser("~/dev/LSM_repository/docs/portfolio_assets")
    default_csv = os.path.join(default_assets, "ekf_comparison.csv")
    default_plots = [
        os.path.join(default_assets, "ekf_comparison_plot.png"),
        os.path.join(default_assets, "04_ekf_comparison_plot.png"),
    ]

    parser = argparse.ArgumentParser(description="LSM Bot EKF Wheel-Slip Experiment & Plotter")
    parser.add_argument('--csv', default=default_csv, help="Output CSV path")
    parser.add_argument('--replot-only', action='store_true', help="Regenerate plot & metrics from existing CSV")
    args, _ = parser.parse_known_args()

    if args.replot_only:
        generate_plot_and_metrics(args.csv, default_plots)
    else:
        run_live_experiment(args.csv, default_plots)


if __name__ == '__main__':
    main()

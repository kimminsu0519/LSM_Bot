# LSM Bot 최종 검증 체크리스트 (Verification Gate)

- **프로젝트**: LSM Bot (ROS 2 Jazzy, Gazebo Harmonic 8.11, Modern C++17 `-O3` + Python 3.12 `lsm_venv`)
- **검증일**: 2026-11-18
- **원칙**: 모든 항목은 추측 없이 실제 빌드·실행·토픽 계측·이미지 육안 검수를 통해 직접 관측된 수치만 기록했습니다.

---

## 1. 검증 항목 총괄 요약표

| 카테고리 | 검증 항목 | 검증 방법 / 명령어 | 실측 결과 | 판정 |
| :--- | :--- | :--- | :--- | :---: |
| **패키지 / 빌드** | `ros-jazzy-robot-localization` & `Eigen3` / `yaml-cpp` | `dpkg -l \| grep -E "robot-localization\|eigen3\|yaml-cpp"` | `robot-localization 3.8.4`, `libeigen3-dev 3.4.0`, `libyaml-cpp-dev 0.8.0` 확인 | **PASS** |
| **패키지 / 빌드** | C++17 노드 3종 및 전체 워크스페이스 빌드 | `colcon build --symlink-install` (`-std=c++17 -O3`) | `amr_state_machine_node`, `precision_docking_node`, `topological_route_planner_node` 포함 8개 패키지 빌드 완료, 경고/에러 **0건** | **PASS** |
| **패키지 / 이식성** | 하드코딩 절대경로 제거 검증 | `grep -rn "/home/kms" lsm_ws/src/` | 소스코드 내 하드코딩 경로 **0건** (`ament_index_cpp`/`ament_index_python` 적용) | **PASS** |
| **마찰 모드 분리** | `slip_mode:=true/false` 분리 | `navigation.launch.py slip_mode:=true/false` | 기본값 `slip_mode:=false`(대칭 마찰 `mu=1.0`), 벤치마크 시 `slip_mode:=true`(비대칭 `0.35 / 0.28`) 정상 전환 | **PASS** |
| **토픽 주파수** | EKF 출력 주파수 (`/odometry/filtered`) | `ros2 topic hz --use-sim-time /odometry/filtered` | Sim **`30.010 Hz`** (`min 0.032s, max 0.034s, std 0.00064s, window 31`) | **PASS** |
| **토픽 주파수** | IMU 센서 주파수 (`/imu`) | `ros2 topic hz --use-sim-time /imu` | Sim **`83.086 ~ 90.819 Hz`** (`min 0.000s, max 0.097s, std 0.01401s, window 84`) | **PASS** |
| **토픽 주파수** | 5단계 FSM 상태 발행 (`/lsm01/robot_status`) | `ros2 topic hz --use-sim-time /lsm01/robot_status` | Sim **`10.000 Hz`** (`min 0.072s, max 0.137s, std 0.01360s, window 12`) | **PASS** |
| **TF 트리** | `odom -> base_footprint` 단일 퍼블리셔 검증 | `ros2 topic info --spin-time 3 /tf -v` & `view_frames` | `/tf` 발행자 **3개** (`robot_state_publisher`, `ekf_filter_node`, `amcl`)만 존재 (`ros_gz_bridge` 미포함); `map->odom` (`10.37 Hz`), `odom->base_footprint` (`30.245 Hz`) | **PASS** |
| **EKF 3-Way Ablation** | `slip_mode:=true` 휠 슬립 3-Way 비교 실험 | `python3 lsm_ws/src/lsm_navigation/scripts/run_ekf_slip_experiment.py` (393 samples, 20.89s) | **Case A (Raw)**: XY RMSE `0.2278m`, Yaw RMSE `14.009°`, Final `13.524°`<br>**Case B (Main `yaw=false`)**: XY RMSE **`0.0315m (+86.2%)`**, Yaw RMSE **`1.793° (+87.2%)`**, Final **`0.900°`**<br>**Case C (Ref `yaw=true`)**: XY RMSE `0.0313m (+86.2%)`, Yaw RMSE `1.283° (+90.8%)`, Final `0.000°` | **PASS** |
| **C++17 vs Python 벤치마크** | 코어 알고리즘·지터·메모리·도킹 정밀도 비교 | `python3 lsm_ws/src/lsm_navigation/scripts/run_cpp_vs_python_benchmark.py` | • 206노드 A\*: `57.57 µs → 1.47 µs` (**`39.09x`**)<br>• 2D EKF Step: `32.08 µs → 0.295 µs` (**`108.74x`**)<br>• 360° LiDAR 감시: `61.65 µs → 0.866 µs` (**`71.19x`**)<br>• 50Hz 지터: `63.59 µs → 12.11 µs` (**`5.25x`**)<br>• RSS 메모리: `97.09 MB → 27.63 MB` (**`3.51x`**)<br>• 도킹 오차: `29.27 cm → 3.42 cm` (**`8.56x`, `-88.3%`**) | **PASS** |
| **FSM E2E 통합** | C++17 FSM + 50Hz PID 도킹 5단계 사이클 완주 | `python3 lsm_ws/src/lsm_navigation/scripts/run_fsm_e2e_test.py` (`TASK_A4_A5_E2E_001`) | `observed=[0, 1, 2, 3, 4, 1, 0]` 일치 (`match=True`), `SUCCEEDED=True`, 총 소요 **`216.17s`**, 작업장 정착 오차 `1.43 cm`(`wp-ws-a-4`) / `0.72 cm`(`wp-ws-a-5`) 달성 | **PASS** |
| **시각 자료 검수** | 포트폴리오 에셋 6종(+CSV 2종) 육안 검수 | `read_file`을 통한 전체 PNG(`01~06`) 육안 확인 | 6종 스크린샷·그래프·TF트리 모두 팝업 가림·잘림·텍스트 겹침 없음 확인 | **PASS** |

---

## 2. 포트폴리오 시각 자료 및 리포트 산출물 목록

### 2.1 포트폴리오 시각 자료 (`docs/portfolio_assets/`)

| 파일명 | 검수 확인 사항 |
| :--- | :--- |
| [`01_gazebo_warehouse_sim.png`](../portfolio_assets/01_gazebo_warehouse_sim.png) | 원격 제어 오버레이 및 우측 패널 완전 제거, Gazebo Harmonic 3D 창고(A/B/C 구역) 및 충전 패드 위 `LSM01_Waffle` 클린 캡처 |
| [`02_rviz_amcl_nav2_sync.png`](../portfolio_assets/02_rviz_amcl_nav2_sync.png) | 우측 빈 `Docking`/`Selector` 패널 제거, 창고 전체 수평 뷰(`TopDownOrtho`), `WS-A-4` 진입 구간 LiDAR 스캔 1:1 정합 및 AMCL/Nav2 경로 표시 |
| [`03_fms_web_fsm_dashboard.png`](../portfolio_assets/03_fms_web_fsm_dashboard.png) | 1920×1080 Chrome 캡처, `WS ONLINE`, `FSM STATE: 1: MOVING_UNLOADED`, 녹색 파선 A* 경로(`6.94 m / 9 nodes`), 우측 큐 표시 |
| [`04_ekf_comparison_plot.png`](../portfolio_assets/04_ekf_comparison_plot.png) | 300 DPI **3-Way Ablation** EKF 비교 그래프 ([`ekf_comparison.csv`](../portfolio_assets/ekf_comparison.csv), 393 rows) |
| [`05_tf_frames_tree.png`](../portfolio_assets/05_tf_frames_tree.png) | `map -> odom (10.37 Hz) -> base_footprint (30.245 Hz)` 단일 퍼블리셔 TF 트리 |
| [`06_cpp_vs_python_benchmark_plot.png`](../portfolio_assets/06_cpp_vs_python_benchmark_plot.png) | 300 DPI **C++17 (`rclcpp/Eigen3/STL`) vs. Python 3.12 (`rclpy/NumPy`)** 4-패널 정량 비교 그래프 ([`cpp_vs_python_benchmark.csv`](../portfolio_assets/cpp_vs_python_benchmark.csv)) |

### 2.2 기술 검증 리포트 (`docs/reports/`)

1. [`01_EKF_Sensor_Fusion_Experiment_Report.md`](./01_EKF_Sensor_Fusion_Experiment_Report.md)
2. [`02_FSM_E2E_Integration_Test_Report.md`](./02_FSM_E2E_Integration_Test_Report.md)
3. [`03_Final_Verification_Checklist.md`](./03_Final_Verification_Checklist.md)
4. [`04_Cpp_Core_Optimization_and_Benchmark_Report.md`](./04_Cpp_Core_Optimization_and_Benchmark_Report.md)

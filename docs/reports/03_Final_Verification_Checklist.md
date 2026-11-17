# LSM Bot 최종 검증 체크리스트 (Verification Gate)

- **프로젝트**: LSM Bot (ROS 2 Jazzy, Gazebo Harmonic 8.11, Python 3.12 `lsm_venv`)
- **검증일**: 2026-11-17
- **원칙**: 모든 항목은 추측 없이 실제 빌드·실행·토픽 계측·이미지 검수를 통해 직접 관측된 수치만 기록했습니다.

---

## 1. 검증 항목 총괄 요약표

| 카테고리 | 검증 항목 | 검증 방법 / 명령어 | 실측 결과 | 판정 |
| :--- | :--- | :--- | :--- | :---: |
| **패키지 / 빌드** | `ros-jazzy-robot-localization` 설치 | `dpkg -l \| grep robot-localization` | `3.8.4-1noble.20261105.105419` 설치 확인 | **PASS** |
| **패키지 / 빌드** | ROS 2 워크스페이스 전체 빌드 | `colcon build --symlink-install` (하드웨어 의존 미사용 `turtlebot3` 서브패키지는 `COLCON_IGNORE` 처리) | 8개 패키지 빌드 완료, 에러 **0건** | **PASS** |
| **토픽 주파수** | EKF 출력 주파수 (`/odometry/filtered`) | `ros2 topic hz --use-sim-time /odometry/filtered` & wall-time 계측 | Sim **`29.997 Hz`** (`min 0.031s, max 0.036s, std 0.00089s`), Wall `24.041 Hz` (@ Gazebo RTF `0.668`) | **PASS** |
| **토픽 주파수** | IMU 센서 주파수 (`/imu`) | `ros2 topic hz --use-sim-time /imu` & wall-time 계측 | Sim **`90.819 Hz`**, Wall `82.282 Hz` (`min 0.008s, max 0.019s, std 0.00178s`) | **PASS** |
| **토픽 주파수** | 5단계 FSM 상태 발행 (`/lsm01/robot_status`) | `ros2 topic hz --use-sim-time /lsm01/robot_status` & wall-time 계측 | Sim **`10.000 Hz`** (`min 0.097s, max 0.103s, std 0.00158s`), Wall `8.265 Hz` | **PASS** |
| **TF 트리** | `odom -> base_footprint` 단일 퍼블리셔 검증 | `ros2 topic info /tf -v` & `ros2 run tf2_tools view_frames` | `/tf` 발행자 **3개** (`robot_state_publisher`, `ekf_filter_node`, `amcl`)만 존재 (`ros_gz_bridge` 미포함); `map->odom` (`10.294 Hz`), `odom->base_footprint` (`30.259 Hz`) | **PASS** |
| **EKF 정량 성능** | 휠 슬립 조건 Before/After 비교 실험 | `python3 .../run_ekf_slip_experiment.py` (`lsm_venv`, 397 samples, 20.95s) | XY RMSE `0.2702m -> 0.0351m` (**+87.0%**), XY Max `0.4791m -> 0.0608m` (**+87.3%**), Yaw RMSE `15.820° -> 1.444°` (**+90.9%**), Final Yaw Drift `16.968° -> 0.000°` | **PASS** |
| **FSM E2E 통합** | 5단계 물류 사이클 완주 및 `SequentialNavigator` 회귀 | `python3 .../run_fsm_e2e_test.py` (`lsm_venv`, `TASK_A4_A5_E2E_001`) | `observed=[0, 1, 2, 3, 4, 1, 0]` 일치 (`match=True`), `SUCCEEDED=True`, 총 소요 **`237.83s`**, 3개 Leg(`6.94m`, `11.65m`, `9.55m`) A* 순차 주행 및 3중 좌표계 정합 유지 | **PASS** |
| **시각 자료 검수** | 포트폴리오 에셋 5종(+CSV/그래프) 생성 및 이미지 검수 | 직접 파일 열람(`read_file`)으로 해상도/겹침/정합 상태 육안 확인 | 5종 스크린샷·그래프·TF트리 모두 화면 깨짐 없이 정상 확인 | **PASS** |

---

## 2. 세부 검증 로그 증적

### 2.1 `/tf` 단일 퍼블리셔 (`ros2 topic info /tf -v` 발췌)

```text
Type: tf2_msgs/msg/TFMessage

Publisher count: 3

Node name: robot_state_publisher
Node namespace: /
...
Node name: ekf_filter_node
Node namespace: /
...
Node name: amcl
Node namespace: /
...
Subscription count: 14
```
- `ros_gz_bridge`가 `/tf` 퍼블리셔 목록에서 완전히 제거되었으며, `odom -> base_footprint` 프레임 변환은 `ekf_filter_node` 단독으로 `30.259 Hz` 주기로 발행됨을 확인했습니다([`05_tf_frames_tree.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/05_tf_frames_tree.png)).

### 2.2 주요 토픽 주파수 실측 (`--use-sim-time` 및 Wall-Time)

Gazebo 물리 연산 부하(Real-Time Factor ≈ `0.668`) 환경에서 시뮬레이션 시간 기준(`--use-sim-time`)으로는 목표 설계 주기(`30 Hz`, `100 Hz`급, `10 Hz`)가 정확히 유지됨을 확인했습니다.

- **`/odometry/filtered` (Sim-time)**: `average rate: 29.997` (`min: 0.031s max: 0.036s std dev: 0.00089s window: 92`)
- **`/odometry/filtered` (Wall-time)**: `average rate: 24.041` (`min: 0.000s max: 0.305s std dev: 0.02802s window: 99`)
- **`/imu` (Sim-time)**: `average rate: 90.819` (`min: 0.000s max: 0.195s std dev: 0.01325s window: 367`)
- **`/imu` (Wall-time)**: `average rate: 82.282` (`min: 0.008s max: 0.019s std dev: 0.00178s window: 250`)
- **`/lsm01/robot_status` (Sim-time)**: `average rate: 10.000` (`min: 0.097s max: 0.103s std dev: 0.00158s window: 12`)
- **`/lsm01/robot_status` (Wall-time)**: `average rate: 8.265` (`min: 0.107s max: 0.146s std dev: 0.00782s window: 27`)

---

## 3. 포트폴리오 시각 자료 및 리포트 산출물 목록

### 3.1 포트폴리오 시각 자료 (`docs/portfolio_assets/`)

| 파일명 | 용량 | 검수 확인 사항 |
| :--- | :---: | :--- |
| [`01_gazebo_warehouse_sim.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/01_gazebo_warehouse_sim.png) | 448 KB | Gazebo Harmonic 3D 창고(A/B/C 컬러 구역) 및 충전 패드 위 `LSM01_Waffle` 정상 렌더링 |
| [`02_rviz_amcl_nav2_sync.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/02_rviz_amcl_nav2_sync.png) | 181 KB | `lsm_warehouse_map` 벽면과 LiDAR 스캔 1:1 정합, 녹색 AMCL 파티클 수렴, Nav2 코스트맵 및 글로벌/로컬 경로 표시 (`Navigation/Localization: active`, `ETA: 3s`, `Distance remaining: 0.67 m`) |
| [`03_fms_web_fsm_dashboard.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/03_fms_web_fsm_dashboard.png) | 231 KB | 1920×1080 Chrome 캡처, `WS ONLINE`, `FSM STATE: 1: MOVING_UNLOADED`, `TASK ID: TASK_A4_A5_E2E_001`, Map/ROS2/Yaw 실시간 포즈, 녹색 파선 A* 경로(`6.94 m / 9 nodes`), 우측 큐(`PASSED/CURRENT/NEXT/PENDING`) 정상 표시 |
| [`04_ekf_comparison_plot.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/04_ekf_comparison_plot.png) | 692 KB | 300 DPI 고해상도 XY 궤적 + 시간별 Yaw 오차 2단 그래프, 텍스트 박스 겹침 없음 ([`ekf_comparison_plot.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/ekf_comparison_plot.png) 동일) |
| [`ekf_comparison.csv`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/ekf_comparison.csv) | 42 KB | 397개 동기 샘플 시계열 원천 데이터 |
| [`05_tf_frames_tree.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/05_tf_frames_tree.png) | 408 KB | `map -> odom -> base_footprint -> base_link -> {caster_back_left_link, caster_back_right_link, imu_link, base_scan}` 계층 구조 및 주파수(`10.294 Hz` / `30.259 Hz`) 정상 확인 |

### 3.2 기술 검증 리포트 (`docs/reports/`)

1. [`01_EKF_Sensor_Fusion_Experiment_Report.md`](file:///home/kms/dev/LSM_repository/docs/reports/01_EKF_Sensor_Fusion_Experiment_Report.md)
2. [`02_FSM_E2E_Integration_Test_Report.md`](file:///home/kms/dev/LSM_repository/docs/reports/02_FSM_E2E_Integration_Test_Report.md)
3. [`03_Final_Verification_Checklist.md`](file:///home/kms/dev/LSM_repository/docs/reports/03_Final_Verification_Checklist.md)

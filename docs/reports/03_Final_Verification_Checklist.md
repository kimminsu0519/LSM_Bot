# LSM Bot 최종 검증 체크리스트 (Verification Gate)

- **프로젝트**: LSM Bot (ROS 2 Jazzy, Gazebo Harmonic 8.11, Python 3.12 `lsm_venv`)
- **검증일**: 2026-11-18
- **원칙**: 모든 항목은 추측 없이 실제 빌드·실행·토픽 계측·이미지 육안 검수를 통해 직접 관측된 수치만 기록했습니다.

---

## 1. 검증 항목 총괄 요약표

| 카테고리 | 검증 항목 | 검증 방법 / 명령어 | 실측 결과 | 판정 |
| :--- | :--- | :--- | :--- | :---: |
| **패키지 / 빌드** | `ros-jazzy-robot-localization` 설치 | `dpkg -l \| grep robot-localization` | `3.8.4-1noble.20261105.105419` 설치 확인 | **PASS** |
| **패키지 / 빌드** | ROS 2 워크스페이스 전체 빌드 | `colcon build --symlink-install` (`lsm_venv` `py_compile` 포함) | 8개 패키지 빌드 완료 (`3.12s`), 에러 **0건** | **PASS** |
| **패키지 / 이식성** | 하드코딩 절대경로 제거 검증 | `grep -rn "/home/kms" lsm_ws/src/` | 소스코드 내 하드코딩 경로 **0건** (`ament_index_python` + 상대 경로 폴백 적용) | **PASS** |
| **마찰 모드 분리** | `slip_mode:=true/false` 분리 | `navigation.launch.py slip_mode:=true/false` | 기본값 `slip_mode:=false`(대칭 마찰 `mu=1.0`), 벤치마크 시 `slip_mode:=true`(비대칭 `0.35 / 0.28`) 정상 전환 | **PASS** |
| **토픽 주파수** | EKF 출력 주파수 (`/odometry/filtered`) | `ros2 topic hz --use-sim-time /odometry/filtered` | Sim **`30.010 Hz`** (`min 0.032s, max 0.034s, std 0.00064s, window 31`) | **PASS** |
| **토픽 주파수** | IMU 센서 주파수 (`/imu`) | `ros2 topic hz --use-sim-time /imu` | Sim **`83.086 ~ 90.819 Hz`** (`min 0.000s, max 0.097s, std 0.01401s, window 84`) | **PASS** |
| **토픽 주파수** | 5단계 FSM 상태 발행 (`/lsm01/robot_status`) | `ros2 topic hz --use-sim-time /lsm01/robot_status` | Sim **`10.000 Hz`** (`min 0.072s, max 0.137s, std 0.01360s, window 12`) | **PASS** |
| **TF 트리** | `odom -> base_footprint` 단일 퍼블리셔 검증 | `ros2 topic info /tf -v` & `ros2 run tf2_tools view_frames` | `/tf` 발행자 **3개** (`robot_state_publisher`, `ekf_filter_node`, `amcl`)만 존재 (`ros_gz_bridge` 미포함); `map->odom` (`10.37 Hz`), `odom->base_footprint` (`30.245 Hz`) | **PASS** |
| **EKF 3-Way Ablation** | `slip_mode:=true` 휠 슬립 3-Way 비교 실험 | `python3 lsm_ws/src/lsm_navigation/scripts/run_ekf_slip_experiment.py` (393 samples, 20.89s) | **Case A (Raw)**: XY RMSE `0.2278m`, Yaw RMSE `14.009°`, Final `13.524°`<br>**Case B (Main `yaw=false`)**: XY RMSE **`0.0315m (+86.2%)`**, Yaw RMSE **`1.793° (+87.2%)`**, Final **`0.900°`** (No GT snap)<br>**Case C (Ref `yaw=true`)**: XY RMSE `0.0313m (+86.2%)`, Yaw RMSE `1.283° (+90.8%)`, Final `0.000°` | **PASS** |
| **FSM E2E 통합** | `slip_mode:=false` 5단계 물류 사이클 완주 | `python3 lsm_ws/src/lsm_navigation/scripts/run_fsm_e2e_test.py` (`TASK_A4_A5_E2E_001`) | `observed=[0, 1, 2, 3, 4, 1, 0]` 일치 (`match=True`), `SUCCEEDED=True`, 총 소요 **`218.35s`** (`19.48s` 단축), 3개 Leg(`6.94m`, `11.65m`, `9.55m`) A* 순차 주행 완료 | **PASS** |
| **시각 자료 검수** | 포트폴리오 에셋 5종(+CSV/그래프) 재촬영 및 검수 | `read_file`을 통한 전체 PNG 육안 확인 | `01` 원격 데스크톱 오버레이 및 우측 패널 완전 제거, `02` 창고 전체 구도 및 우측 빈 `Docking` 패널 제거, `04` 3-Way Ablation 그래프 겹침 없음 확인 | **PASS** |

---

## 2. 세부 검증 로그 증적

### 2.1 `/tf` 단일 퍼블리셔 및 프레임 트리 (`view_frames`)

- `ros_gz_bridge`가 `/tf` 퍼블리셔 목록에서 완전히 제거되었으며, `odom -> base_footprint` 프레임 변환은 `ekf_filter_node` 단독으로 `30.245 Hz` 주기로 발행됨을 확인했습니다([`05_tf_frames_tree.png`](../portfolio_assets/05_tf_frames_tree.png)).

### 2.2 주요 토픽 주파수 실측 (`--use-sim-time`)

- **`/odometry/filtered` (Sim-time)**: `average rate: 30.010` (`min: 0.032s max: 0.034s std dev: 0.00064s window: 31`)
- **`/imu` (Sim-time)**: `average rate: 83.086` (`min: 0.000s max: 0.097s std dev: 0.01401s window: 84`) / `90.819` (`window: 367`)
- **`/lsm01/robot_status` (Sim-time)**: `average rate: 10.000` (`min: 0.072s max: 0.137s std dev: 0.01360s window: 12`)

---

## 3. 포트폴리오 시각 자료 및 리포트 산출물 목록

### 3.1 포트폴리오 시각 자료 (`docs/portfolio_assets/`)

| 파일명 | 용량 | 검수 확인 사항 |
| :--- | :---: | :--- |
| [`01_gazebo_warehouse_sim.png`](../portfolio_assets/01_gazebo_warehouse_sim.png) | 187 KB | 원격 제어 오버레이(`Connected ...`) 및 우측 `Entity Tree`/`Grid config` 패널 완전 제거, Gazebo Harmonic 3D 창고(A/B/C 구역) 및 충전 패드 위 `LSM01_Waffle` 클린 캡처 |
| [`02_rviz_amcl_nav2_sync.png`](../portfolio_assets/02_rviz_amcl_nav2_sync.png) | 149 KB | 우측 빈 `Docking`/`Selector` 패널 제거, 창고 전체 A/B 구역 수평 뷰(`TopDownOrtho`), `WS-A-4` 진입 구간 LiDAR 스캔 1:1 정합, 녹색 AMCL 파티클 수렴 및 코스트맵/경로 표시 (`Navigation/Localization/Feedback: active`, `ETA: 2 s`, `Distance remaining: 0.65 m`) |
| [`03_fms_web_fsm_dashboard.png`](../portfolio_assets/03_fms_web_fsm_dashboard.png) | 231 KB | 1920×1080 Chrome 캡처, `WS ONLINE`, `FSM STATE: 1: MOVING_UNLOADED`, `TASK ID: TASK_A4_A5_E2E_001`, 녹색 파선 A* 경로(`6.94 m / 9 nodes`), 우측 큐(`PASSED/CURRENT/NEXT/PENDING`) 정상 표시 |
| [`04_ekf_comparison_plot.png`](../portfolio_assets/04_ekf_comparison_plot.png) | 745 KB | 300 DPI 고해상도 **3-Way Ablation** 그래프 (`Ground Truth` vs. `Case A: Raw /odom` vs. `Case B: EKF Gyro yaw=false` vs. `Case C: EKF AHRS yaw=true`), 텍스트 겹침 없음 ([`ekf_comparison_plot.png`](../portfolio_assets/ekf_comparison_plot.png) 동일) |
| [`ekf_comparison.csv`](../portfolio_assets/ekf_comparison.csv) | 56 KB | 393개 동기 샘플 3-Way Ablation 시계열 원천 데이터 |
| [`05_tf_frames_tree.png`](../portfolio_assets/05_tf_frames_tree.png) | 408 KB | `map -> odom (10.37 Hz) -> base_footprint (30.245 Hz) -> base_link -> {caster_back_left_link, caster_back_right_link, imu_link, base_scan}` 계층 구조 확인 |

### 3.2 기술 검증 리포트 (`docs/reports/`)

1. [`01_EKF_Sensor_Fusion_Experiment_Report.md`](./01_EKF_Sensor_Fusion_Experiment_Report.md)
2. [`02_FSM_E2E_Integration_Test_Report.md`](./02_FSM_E2E_Integration_Test_Report.md)
3. [`03_Final_Verification_Checklist.md`](./03_Final_Verification_Checklist.md)

# EKF 센서 퓨전(`robot_localization`) 구현 및 휠 슬립 정량 실험 리포트

- **프로젝트**: LSM Bot (Smart Warehouse Autonomous Mobile Robot & FMS)
- **작성일**: 2026-11-17
- **실행 환경**: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.11, Python 3.12 (`lsm_venv`), `ros-jazzy-robot-localization` (`3.8.4-1noble.20261105.105419`)

---

## 1. 개요 및 목적

기존 LSM Bot 시뮬레이션은 Gazebo `gz-sim-diff-drive-system` 플러그인이 계산한 휠 오도메트리(`/odom`)만으로 `odom -> base_footprint` TF를 직접 발행하고 있었습니다. 이 방식은 다음과 같은 한계가 존재합니다.

1. **슬립 및 급선회 취약성**: 바퀴 미끄러짐(Wheel Slip)이나 제자리 급선회(`±1.82 rad/s`), 급가감속 시 엔코더 기반 적분 오차가 급격히 누적되어 Yaw 헤딩 및 XY 위치가 실제 포즈(Ground Truth)에서 이탈합니다.
2. **센서 단일 의존성**: 관성 측정 장치(IMU)의 각속도 및 절대 방향 측정치를 반영하지 못해 Nav2 DWB 컨트롤러 및 AMCL 지역화 부담이 커집니다.

이를 해결하기 위해 **`robot_localization` 기반 2D 확장 칼만 필터(`ekf_filter_node`)**를 도입하여 30Hz 휠 오도메트리(`/odom`)와 100Hz IMU(`/imu`)를 융합하고, `ekf_filter_node`가 `odom -> base_footprint` TF의 **단일 발행자(Sole Publisher)** 역할을 수행하도록 아키텍처를 재구성했습니다.

---

## 2. EKF 센서 퓨전 아키텍처 및 구현 내역

### 2.1 TF 파이프라인 및 데이터 흐름

```
[Gazebo Harmonic 8.11]
  ├── gz-sim-diff-drive-system (<tf_topic> 제거) ──▶ /odom (30 Hz: vx, vy, vyaw)
  └── tb3_imu sensor (100 Hz, Gaussian Noise)    ──▶ /imu  (imu_link, 100 Hz: yaw, vyaw, ax)
                                                            │
                                                            ▼
[robot_localization :: ekf_filter_node (30 Hz, two_d_mode: true)]
  ├── publishes: /odometry/filtered (map/odom frame fused odometry)
  └── broadcasts: odom -> base_footprint TF (단독 퍼블리셔)
                                                            │
                        ┌───────────────────────────────────┴──────────────────────────┐
                        ▼                                                              ▼
       [Nav2 Stack (nav2_params.yaml)]                             [FMS Telemetry Bridge]
         - bt_navigator (odom_topic: /odometry/filtered)             - telemetry_bridge.py
         - controller_server (odom_topic: /odometry/filtered)          (/odometry/filtered 우선 구독,
         - amcl (map -> odom)                                           미수신 시 2s 후 /odom 폴백)
```

### 2.2 패키지 및 설정 변경 요약

| 구분 | 대상 파일 | 주요 변경 내용 |
| :--- | :--- | :--- |
| 의존성 | [`lsm_navigation/package.xml`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/package.xml) | `<exec_depend>robot_localization</exec_depend>` 추가 |
| URDF / 센서 | [`lsm_bot_description/urdf/turtlebot3_waffle_gz.urdf`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_bot_description/urdf/turtlebot3_waffle_gz.urdf) | `DiffDrive` 내 `<tf_topic>tf</tf_topic>` 주석 처리(TF 중복 차단), `imu_link` 관성 모멘트 추가 및 `tb3_imu` 센서(`update_rate: 100`, `topic: imu`, 각속도 stddev `2e-4`, 선가속도 stddev `1.7e-2`) 추가 |
| 브릿지 | [`lsm_bot_description/config/ros_gz_bridge.yaml`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_bot_description/config/ros_gz_bridge.yaml) | `imu` (`sensor_msgs/msg/Imu` <-> `gz.msgs.IMU`, `GZ_TO_ROS`, `frame_id: "imu_link"`) 추가, `ground_truth_poses` (`/world/lsm_custom_warehouse/dynamic_pose/info` <-> `tf2_msgs/msg/TFMessage`) 추가, raw `tf` 브릿지 주석 처리 |
| GUI 설정 | [`lsm_bot_description/config/lsm_gui.config`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_bot_description/config/lsm_gui.config) | `<start_paused>false</start_paused>` 설정으로 시뮬레이터 기동 즉시 클럭/센서 활성화 |
| EKF 파라미터 | [`lsm_navigation/config/ekf.yaml`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/config/ekf.yaml) | 30Hz 2D EKF(`world_frame: odom`, `publish_tf: true`, `use_sim_time: true`), `/odom` + `/imu` 퓨전 벡터 및 Q/P0 행렬 정의 |
| 런치 및 Nav2 | [`lsm_navigation/launch/navigation.launch.py`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/launch/navigation.launch.py), [`nav2_params.yaml`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/config/nav2_params.yaml) | `ekf_filter_node` 런치 등록, `bt_navigator` 및 `controller_server`의 `odom_topic`을 `/odometry/filtered`로 연결 |

---

## 3. EKF 상태 벡터 및 Q / R 공분산 튜닝 근거

[`ekf.yaml`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/config/ekf.yaml)의 상태 벡터는 REP-105 및 `robot_localization` 표준 15차원 상태를 사용합니다:
`[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]` (`two_d_mode: true`로 3D 성분 강제 구속).

### 3.1 센서 입력 행렬 (`odom0`, `imu0`)

- **`odom0: /odom` (30 Hz)**
  - `odom0_config`: `[false, false, false,  false, false, false,  true, true, false,  false, false, true,  false, false, false]`
  - **설계 의도**: 슬립이 누적된 절대 위치/방위각(`x, y, yaw`)은 직접 융합에서 **배제**하고, 순간 몸체 속도 성분(`vx`, non-holonomic 구속 `vy`, 각속도 `vyaw`)만 융합하여 슬립 적분 오차가 글로벌 포즈를 오염시키지 않도록 설계했습니다.
- **`imu0: /imu` (100 Hz, `imu_link`)**
  - `imu0_config`: `[false, false, false,  false, false, true,  false, false, false,  false, false, true,  true, false, false]`
  - `imu0_relative: true`, `imu0_remove_gravitational_acceleration: true`
  - **설계 의도**: 초기 스폰 방위각 기준 상대 절대 `yaw`, 각속도 `vyaw`, 종방향 가속도 `ax`를 융합하여 고속 선회 및 휠 슬립 구간에서 헤딩을 강하게 고정(Anchor)합니다.

### 3.2 15×15 프로세스 노이즈 공분산(`Q`) 및 초기 공분산(`P0`) 튜닝

| 상태 변수 | `Q` 대각 값 | `P0` 대각 값 | 튜닝 근거 (실제 시뮬레이션 특성 반영) |
| :--- | :---: | :---: | :--- |
| `x, y` | `0.05` | `1e-9` | 속도 적분 시 지터 없이 부드러운 위치 예측을 허용하는 중간 수준의 예측 불확실성 부여 |
| `yaw` | `0.08` | `1e-9` | 급선회(`1.82 rad/s`) 시 예측 모델보다 100Hz IMU 방향 관측에 빠르게 수렴하도록 상대적으로 높은 프로세스 노이즈 설정 |
| `vx, vy` | `0.05` | `1e-9` | Gazebo DiffDrive twist 공분산(`0.0` → 내부적으로 `1e-9` 처리)과 조화를 이루어 선속도 변화를 지연 없이 추적하되 캐스터 피칭 외란 억제 |
| `vyaw` | `0.04` | `1e-9` | 슬립 발생 시에도 IMU 자이로(`vyaw`) 업데이트를 부드럽게 추적 |
| `ax` | `0.001` | `1e-9` | 정적 기울기나 제동 시 중력 성분이 `vx` 드리프트로 누설되는 것을 방지하기 위한 타이트한 가속도 프로세스 노이즈 |
| 기타 (`z, roll, pitch, vz, vroll, vpitch, ay, az`) | `0.01 ~ 0.06` | `1e-9` | `two_d_mode: true`에 의해 0으로 평면 구속 (`z=0.06, roll/pitch=0.03, vz=0.04, vroll/vpitch=0.01, ay=0.01, az=0.015`) |

> **튜닝 과정에서 발견된 특이사항**:
> 1. Gazebo Harmonic `DiffDrive` 플러그인은 `/odom`의 twist 공분산을 `0.0`으로 발행하며, `robot_localization`은 이를 `1e-9`로 치환합니다. 과도하게 좁은 Mahalanobis 게이트를 적용하면 선회 중 `/odom`의 `vx` 관측치가 기각될 수 있어, 별도 거부 임계치 없이 위 `Q` 행렬 균형으로 정합을 확보했습니다.
> 2. `ros_gz_bridge`가 Gazebo `Pose_V`를 `TFMessage` (`/ground_truth_poses`)로 변환할 때 `child_frame_id`가 빈 문자열(`''`)로 전달되므로, 벤치마크 스크립트([`run_ekf_slip_experiment.py`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/scripts/run_ekf_slip_experiment.py))에서는 동적 모델 인덱스 `0` (`LSM01_Waffle`)을 Ground Truth로 파싱하여 로컬 SE(2) 좌표계로 정렬·비교했습니다.

---

## 4. 휠 슬립 재현 조건 및 EKF Before/After 정량 실험

### 4.1 슬립 유발 물리 조건 및 주행 시나리오

이상적인 평지 마찰에서는 `/odom`과 Ground Truth 차이가 거의 나타나지 않으므로, [`turtlebot3_waffle_gz.urdf`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_bot_description/urdf/turtlebot3_waffle_gz.urdf)에 비대칭 저마찰 조건을 부여해 실제 창고 바닥 슬립을 재현했습니다.
- **물리 마찰 설정**: `wheel_left_link` (`mu1/mu2 = 0.35`), `wheel_right_link` (`mu1/mu2 = 0.28`), 전/후방 캐스터 (`mu1/mu2 = 0.01`), 접촉 강성 `kp = 500000.0`, 감쇠 `kd = 10.0`
- **실험 시작 포즈**: 장애물이 없는 중앙 동-서 통로 `GT = (2.09 m, 5.34 m, 3.9 deg)`
- **9단계 고기동 슬립 프로파일** ([`run_ekf_slip_experiment.py`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/scripts/run_ekf_slip_experiment.py), `lsm_venv` 활성화 실행):
  1. `Phase 1a`: 최대 각속도 반시계 제자리 선회 (`v=0.00 m/s, w=+1.82 rad/s, dt=2.0s`)
  2. `Phase 1b`: 급반전 시계 제자리 선회 (`v=0.00 m/s, w=-1.82 rad/s, dt=2.0s`)
  3. `Phase 2a ~ 2d`: 고속 S자 슬라럼 연속 기동 (`v=0.42 m/s, w=±1.40 rad/s`, `1.3s -> 2.5s -> 2.5s -> 1.3s`)
  4. `Phase 3a`: 급제동 J-Turn 선회 (`v=0.00 m/s, w=+1.82 rad/s, dt=2.8s`)
  5. `Phase 3b`: 직선 고속 대시 (`v=0.42 m/s, w=0.00 rad/s, dt=4.5s` — 누적된 Yaw 오차가 XY 위치 오차로 발산하는 구간 검증)
  6. `Phase 4`: 완전 정지 및 안정화 (`v=0.00 m/s, w=0.00 rad/s, dt=2.0s`)

### 4.2 실측 정량 비교 결과 (Ground Truth 대비)

총 **397개 동기 샘플**, 시뮬레이션 시간 **20.95s** 구간에 대한 실측 데이터([`ekf_comparison.csv`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/ekf_comparison.csv)) 집계 결과입니다.

| 평가 지표 (vs Gazebo Ground Truth) | Before: 순수 휠 오돔 (`/odom`) | After: EKF 센서 퓨전 (`/odometry/filtered`) | 개선율 / 감소량 |
| :--- | :---: | :---: | :---: |
| **XY 궤적 RMSE (m)** | `0.2702 m` | **`0.0351 m`** | **`+87.0%`** (0.2351 m 감소) |
| **XY 최대 위치 오차 (Max Error, m)** | `0.4791 m` | **`0.0608 m`** | **`+87.3%`** (0.4183 m 감소) |
| **Yaw 방위각 RMSE (deg)** | `15.820 deg` | **`1.444 deg`** | **`+90.9%`** (14.376 deg 감소) |
| **Yaw 최대 오차 (Max Error, deg)** | `26.870 deg` | **`3.949 deg`** | **`+85.3%`** (22.921 deg 감소) |
| **종료 시점 Yaw 누적 드리프트 (deg)** | `16.968 deg` | **`0.000 deg`** | **`+100.0%`** (잔류 오차 0.000° 수렴) |

### 4.3 실제 터미널 실행 로그 스니펫

```text
[EKF Experiment] Positioning into unobstructed central East-West aisle (y~5.4, yaw~0 deg)...
[EKF Experiment] Open-aisle starting pose: GT=(2.09, 5.34, 3.9 deg)
[EKF Experiment] Starting 23.0s aggressive wheel-slip test maneuver...
  -> Phase 1a: Max-rate CCW Spin (+1.82 rad/s) (v=0.00 m/s, w=+1.82 rad/s, dt=2.0s)
  -> Phase 1b: Reversal CW Spin (-1.82 rad/s) (v=0.00 m/s, w=-1.82 rad/s, dt=2.0s)
  -> Phase 2a: High-Accel S-Curve Left (v=0.42 m/s, w=+1.40 rad/s, dt=1.3s)
  -> Phase 2b: High-Accel S-Curve Right (v=0.42 m/s, w=-1.40 rad/s, dt=2.5s)
  -> Phase 2c: High-Accel S-Curve Left (v=0.42 m/s, w=+1.40 rad/s, dt=2.5s)
  -> Phase 2d: High-Accel S-Curve Right (v=0.42 m/s, w=-1.40 rad/s, dt=1.3s)
  -> Phase 3a: J-Turn Braking & Spin (+1.82 rad/s) (v=0.00 m/s, w=+1.82 rad/s, dt=2.8s)
  -> Phase 3b: Straight Dash (Yaw->XY Divergence) (v=0.42 m/s, w=+0.00 rad/s, dt=4.5s)
  -> Phase 4: Full Stop & Settle (v=0.00 m/s, w=+0.00 rad/s, dt=2.0s)
[CSV Saved] /home/kms/dev/LSM_repository/docs/portfolio_assets/ekf_comparison.csv (397 rows)

================ EKF WHEEL-SLIP QUANTITATIVE RESULTS ================
Samples: 397 | Sim Duration: 20.95 s
XY Trajectory RMSE  : Raw=0.2702 m | EKF=0.0351 m | Improvement=+87.0%
XY Max Position Err : Raw=0.4791 m | EKF=0.0608 m | Improvement=+87.3%
Yaw RMSE            : Raw=15.820 deg | EKF=1.444 deg | Improvement=+90.9%
Yaw Max Error       : Raw=26.870 deg | EKF=3.949 deg | Improvement=+85.3%
Final Yaw Drift     : Raw=16.968 deg | EKF=0.000 deg
=====================================================================
```

---

## 5. 생성된 시각화 에셋 및 TF 트리 검증

1. **EKF Before/After 비교 그래프 (300 DPI)**:
   - [`04_ekf_comparison_plot.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/04_ekf_comparison_plot.png) (동일 사본: [`ekf_comparison_plot.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/ekf_comparison_plot.png))
   - **관찰 결과**: Raw Wheel Odom(적색 파선)은 Phase 1~3a의 비대칭 휠 슬립으로 인해 Phase 3b 직진 대시 구간에서 방향이 틀어진 채 XY 궤적이 크게 이탈(`0.4791 m`)하지만, EKF Filtered(청색 실선)는 100Hz IMU 각도 융합을 통해 Ground Truth(녹색/흑색 기준선) 궤적과 사실상 중첩되며 RMSE `0.0351 m`를 유지했습니다.
2. **시계열 원천 데이터 CSV**: [`ekf_comparison.csv`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/ekf_comparison.csv) (397행, `time_s`, `gt_*`, `raw_*`, `ekf_*`, `raw_pos_err_m`, `ekf_pos_err_m`, `raw_yaw_err_deg`, `ekf_yaw_err_deg`)
3. **TF 단일 퍼블리셔 검증**: [`05_tf_frames_tree.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/05_tf_frames_tree.png)
   - `map -> odom` (`10.294 Hz`, AMCL), `odom -> base_footprint` (`30.259 Hz`, `ekf_filter_node` 단독 발행) 확인.

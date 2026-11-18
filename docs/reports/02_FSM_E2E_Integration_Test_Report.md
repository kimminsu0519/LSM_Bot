# 물류 5단계 FSM 상태 머신(C++17 `rclcpp`) 및 FMS E2E 통합 테스트 리포트

- **프로젝트**: LSM Bot (Smart Warehouse Autonomous Mobile Robot & FMS)
- **작성일**: 2026-11-18
- **구현 노드**: [`amr_state_machine_cpp_node.cpp`](../../lsm_ws/src/lsm_navigation/src/amr_state_machine_cpp_node.cpp) + [`precision_docking_node.cpp`](../../lsm_ws/src/lsm_navigation/src/precision_docking_node.cpp) (`C++17 -O3`)
- **테스트 러너**: [`run_fsm_e2e_test.py`](../../lsm_ws/src/lsm_navigation/scripts/run_fsm_e2e_test.py) (`lsm_venv` 가상환경, `slip_mode:=false` 정상 마찰 주행 모드)
- **핵심 미션 ID**: `TASK_A4_A5_E2E_001` (`wp-charge-1 -> wp-ws-a-4 -> wp-ws-a-5 -> wp-charge-1`)

---

## 1. 5단계 물류 FSM 및 C++17 50Hz 정밀 도킹 아키텍처 개요

[`amr_state_machine_cpp_node.cpp`](../../lsm_ws/src/lsm_navigation/src/amr_state_machine_cpp_node.cpp)(실행 바이너리 `amr_state_machine_node`)는 Modern C++17(`rclcpp`, `rclcpp_action`, `tf2_ros`, `yaml-cpp`)로 구현된 독립 ROS 2 노드로, 기존 A* 토폴로지 순차 주행 엔진(`SequentialNavigator` in [`telemetry_bridge.py`](../../lsm_ws/src/lsm_fms_web/lsm_fms_web/telemetry_bridge.py)) 및 신규 **50Hz C++17 폐루프 PID 정밀 도킹 제어기([`precision_docking_node.cpp`](../../lsm_ws/src/lsm_navigation/src/precision_docking_node.cpp))**와 결합되어 `lsm_interfaces`의 커스텀 메시지·서비스·액션 계약을 완전 수행합니다.

### 1.1 5단계 상태 전이 및 2단계(Coarse Nav2 + Fine C++ PID) 주행 다이어그램

```
  ┌────────────────────────────────────────────────────────────────────────────┐
  │                        0: CHARGING (충전소 대기/충전)                      │
  │                        (초기 위치: wp-charge-1 / Map Origin 보정)          │
  └───────────────┬────────────────────────────────────────────▲───────────────┘
                  │ /fms/dispatch_task Goal 수신               │ wp-charge-1 정밀 도킹 완료
                  ▼                                            │ (Mission Complete)
  ┌───────────────────────────────┐            ┌───────────────┴───────────────┐
  │   1: MOVING_UNLOADED          │            │   1: MOVING_UNLOADED          │
  │   (A* 순차 주행 + 50Hz 도킹)  │            │   (공차 복귀 + 50Hz 도킹)     │
  └───────────────┬───────────────┘            └───────────────▲───────────────┘
                  │ 픽업지 정밀 도킹 완료                      │ CARGO_UNLOADED(3)
                  │ (wp-ws-a-4: 오차 3.27cm -> 정착 1.43cm)    │ (/workstation/trigger_mock)
                  ▼                                            │
  ┌───────────────────────────────┐            ┌───────────────┴───────────────┐
  │   2: WORKING_LOADING          │            │   4: WORKING_UNLOADING        │
  │   (픽업 작업장 적재 대기)     │            │   (하역 작업장 하역 대기)      │
  └───────────────┬───────────────┘            └───────────────▲───────────────┘
                  │ CARGO_LOADED(2)                            │ 하역지 정밀 도킹 완료
                  │ (/workstation/trigger_mock)                │ (wp-ws-a-5: 오차 1.63cm -> 0.72cm)
                  ▼                                            │
  ┌────────────────────────────────────────────────────────────┴───────────────┐
  │                        3: MOVING_LOADED                                    │
  │                        (실차 주행: 목적지 작업장 이동 + 50Hz 도킹)         │
  └────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 보존된 핵심 불변량 및 C++17 고도화 사항

- **지도 오프셋 (`lsm_warehouse_map.yaml`)**: `origin: [-2.4, -0.3, 0.0]`, `resolution: 0.050`
- **초기 스폰 및 AMCL 정렬**: Gazebo 스폰 `(2.4, 0.3, 0.1, Y=3.14159)`, `nav2_params.yaml` 및 `auto_init_pose` `180°` 헤딩(`z: 1.0, w: 0.0`) 보존
- **C++17 50Hz 정밀 도킹 연동 (`precision_docking_node`)**:
  - `SequentialNavigator`가 최종 목적지 반경 `0.30m`에 진입하면 Nav2 목표를 해제하고, `amr_state_machine_node`가 `/fms/docking_cmd`를 발행해 `precision_docking_node`의 50Hz 폐루프 PID 제어로 남은 `~29.3cm` 오차를 평균 **`3.42cm` (`wp-ws-a-4: 3.27cm`, `wp-ws-a-5: 1.63cm`, `wp-charge-1: 5.35cm`)**까지 줄입니다.

---

## 2. `lsm_interfaces` 및 FMS/웹 대시보드 연동 명세

| 인터페이스 유형 | 토픽 / 서비스 / 액션 이름 | 타입 (`lsm_interfaces`) | 역할 및 주요 필드 |
| :--- | :--- | :--- | :--- |
| **Publisher (10 Hz)** | `/lsm01/robot_status` | [`RobotStatus.msg`](../../lsm_ws/src/lsm_interfaces/msg/RobotStatus.msg) | `header`, `robot_id`, `current_state` (`0..4`), `pose`, `battery_level`, `current_task_id` |
| **Pub / Sub** | `/workstation/event` | [`WorkstationEvent.msg`](../../lsm_ws/src/lsm_interfaces/msg/WorkstationEvent.msg) | `station_id`, `event_type` (`1: JOB_DONE`, `2: CARGO_LOADED`, `3: CARGO_UNLOADED`), `task_id` |
| **Action Server** | `/fms/dispatch_task` | [`DispatchTask.action`](../../lsm_ws/src/lsm_interfaces/action/DispatchTask.action) | Goal(`task_id`, `task_type` `1..3`, `target_station_id`), Feedback(`current_state`, `distance_remaining`, `estimated_time_remaining`), Result(`success`, `message`, `total_execution_time`) |
| **Service Server** | `/workstation/trigger_mock` | [`TriggerMockEvent.srv`](../../lsm_ws/src/lsm_interfaces/srv/TriggerMockEvent.srv) | Request(`station_id`, `event_type` `2` or `3`), Response(`success`, `message`) |
| **Internal Bridge** | `/fms/fsm_route_cmd`, `/fms/route_status` | `std_msgs/msg/String` (JSON) | `amr_state_machine_node` (C++17) ↔ `SequentialNavigator` (`telemetry_bridge.py`) 간 A* 주행 지령·취소 및 완료 보고 |
| **Fine Docking Bridge** | `/fms/docking_cmd`, `/fms/docking_status` | `std_msgs/msg/String` (JSON) | `amr_state_machine_node` (C++17) ↔ `precision_docking_node` (C++17 50Hz PID) 간 정밀 도킹 제어 및 전후 오차 보고 |

---

## 3. E2E 풀 사이클 주행 시나리오 및 단계별 실측 타임스탬프 (C++17 코어 통합)

### 3.1 주행 구간(Leg)별 A* 경로 및 C++ 50Hz 정밀 도킹 결과

| 구간 (Leg) | 출발 → 목적지 | A* 웨이포인트 / 거리 | Nav2 핸드오프 오차 (Before) | C++ 50Hz PID 도킹 후 오차 (After) | 최종 정착 좌표 (목표 대비 오차) |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **Leg 1** (공차 픽업) | `wp-charge-1` → `wp-ws-a-4 (4.5, 2.1)` | **9 wps / 6.94 m** | `0.292 m` (`29.2 cm`) | **`0.0327 m` (`3.27 cm`, `-88.8%`)** | `world=(4.487, 2.106)` (**`1.43 cm`**) |
| **Leg 2** (실차 하역) | `wp-ws-a-4` → `wp-ws-a-5 (6.9, 2.1)` | **12 wps / 11.65 m** | `0.290 m` (`29.0 cm`) | **`0.0163 m` (`1.63 cm`, `-94.4%`)** | `world=(6.906, 2.096)` (**`0.72 cm`**) |
| **Leg 3** (충전소 복귀) | `wp-ws-a-5` → `wp-charge-1 (2.4, 0.3)` | **10 wps / 9.55 m** | `0.296 m` (`29.6 cm`) | **`0.0535 m` (`5.35 cm`, `-81.9%`)** | `world=(2.339, 0.312)` (`6.22 cm`) |

### 3.2 상태 전이 단계별 실측 타임스탬프 로그 (`total_execution_time = 216.17s`)

| 단계 | Wall Time (KST) | Sim Time (s) | 상태 전이 | ROS2 Map Pose (m) | Gazebo World Pose (m) | 배터리 (%) | 트리거 / 이벤트 |
| :---: | :---: | :---: | :--- | :---: | :---: | :---: | :--- |
| 0 | `20:17:05.894` | `1795000625.89` | `INIT -> CHARGING(0)` | `(0.020, 0.029)` | `(2.420, 0.329)` | `100.0%` | 초기 충전 대기 상태 진입 |
| 1 | `20:17:06.994` | `1795000626.99` | `CHARGING(0) -> MOVING_UNLOADED(1)` | `(0.020, 0.029)` | `(2.420, 0.329)` | `100.0%` | `DispatchTask` (`wp-ws-a-4->wp-ws-a-5->wp-charge-1`) 수락 |
| 2 | `20:18:00.594` | `1795000680.59` | `MOVING_UNLOADED(1) -> WORKING_LOADING(2)` | `(2.080, 1.826)` | `(4.480, 2.126)` | `97.3%` | `wp-ws-a-4` 50Hz PID 정밀 도킹 완료 (`3.27cm`) |
| 3 | `20:18:02.794` | `1795000682.79` | `WORKING_LOADING(2) -> MOVING_LOADED(3)` | `(2.087, 1.806)` | `(4.487, 2.106)` | `97.3%` | `/workstation/trigger_mock` (`CARGO_LOADED=2`) 성공 (`1.43cm` 정착) |
| 4 | `20:19:28.294` | `1795000768.29` | `MOVING_LOADED(3) -> WORKING_UNLOADING(4)` | `(4.506, 1.815)` | `(6.906, 2.115)` | `93.0%` | `wp-ws-a-5` 50Hz PID 정밀 도킹 완료 (`1.63cm`) |
| 5 | `20:19:30.495` | `1795000770.50` | `WORKING_UNLOADING(4) -> MOVING_UNLOADED(1)` | `(4.506, 1.796)` | `(6.906, 2.096)` | `92.9%` | `/workstation/trigger_mock` (`CARGO_UNLOADED=3`) 성공 (`0.72cm` 정착) |
| 6 | `20:20:43.194` | `1795000843.19` | `MOVING_UNLOADED(1) -> CHARGING(0)` | `(-0.061, 0.012)` | `(2.339, 0.312)` | `89.3%` | `wp-charge-1` 50Hz PID 정밀 도킹 완료 |

### 3.3 실제 테스트 러너 및 C++17 노드 로그 스니펫

```text
[2026-11-18 20:17:05.894 | sim=1795000625.89s] STATE TRANSITION: INIT(None) -> CHARGING(0) | task='' | map=(0.020, 0.029) | world=(2.420, 0.329) | battery=100.0%
[2026-11-18 20:17:06.961 | sim=1795000626.96s] ACTION GOAL -> /fms/dispatch_task(task_id='TASK_A4_A5_E2E_001', target_station_id='wp-ws-a-4->wp-ws-a-5->wp-charge-1')
[2026-11-18 20:17:06.994 | sim=1795000626.99s] STATE TRANSITION: CHARGING(0) -> MOVING_UNLOADED(1) | task='TASK_A4_A5_E2E_001' | map=(0.020, 0.029) | world=(2.420, 0.329) | battery=100.0%
[2026-11-18 20:17:07.014 | sim=1795000627.01s] ACTION ACCEPTED -> DispatchTask goal accepted by amr_state_machine_node
[2026-11-18 20:18:00.594 | sim=1795000680.59s] STATE TRANSITION: MOVING_UNLOADED(1) -> WORKING_LOADING(2) | task='TASK_A4_A5_E2E_001' | map=(2.080, 1.826) | world=(4.480, 2.126) | battery=97.3%
[2026-11-18 20:18:02.705 | sim=1795000682.71s] SERVICE CALL -> /workstation/trigger_mock(station_id='wp-ws-a-4', event_type=CARGO_LOADED(2))
[2026-11-18 20:18:02.756 | sim=1795000682.76s] SERVICE RESP <- success=True, message='Mock CARGO_LOADED event triggered for station wp-ws-a-4'
[2026-11-18 20:18:02.794 | sim=1795000682.79s] STATE TRANSITION: WORKING_LOADING(2) -> MOVING_LOADED(3) | task='TASK_A4_A5_E2E_001' | map=(2.087, 1.806) | world=(4.487, 2.106) | battery=97.3%
[2026-11-18 20:19:28.294 | sim=1795000768.29s] STATE TRANSITION: MOVING_LOADED(3) -> WORKING_UNLOADING(4) | task='TASK_A4_A5_E2E_001' | map=(4.506, 1.815) | world=(6.906, 2.115) | battery=93.0%
[2026-11-18 20:19:30.486 | sim=1795000770.49s] SERVICE CALL -> /workstation/trigger_mock(station_id='wp-ws-a-5', event_type=CARGO_UNLOADED(3))
[2026-11-18 20:19:30.495 | sim=1795000770.50s] STATE TRANSITION: WORKING_UNLOADING(4) -> MOVING_UNLOADED(1) | task='TASK_A4_A5_E2E_001' | map=(4.506, 1.796) | world=(6.906, 2.096) | battery=92.9%
[2026-11-18 20:19:30.536 | sim=1795000770.54s] SERVICE RESP <- success=True, message='Mock CARGO_UNLOADED event triggered for station wp-ws-a-5'
[2026-11-18 20:20:43.194 | sim=1795000843.19s] STATE TRANSITION: MOVING_UNLOADED(1) -> CHARGING(0) | task='' | map=(-0.061, 0.012) | world=(2.339, 0.312) | battery=89.3%
[2026-11-18 20:20:43.249 | sim=1795000843.25s] ACTION RESULT <- status=4 (SUCCEEDED=True), success=True, total_execution_time=216.17s, message='Full 5-stage E2E logistics mission completed (wp-ws-a-4 -> wp-ws-a-5 -> wp-charge-1)'
[2026-11-18 20:20:43.751 | sim=1795000843.75s] FSM SEQUENCE CHECK: observed=[0, 1, 2, 3, 4, 1, 0] | expected=[0, 1, 2, 3, 4, 1, 0] | match=True
```

---

## 4. 종합 판정

- **총 주행 시간 (`slip_mode:=false` + C++17 50Hz PID 도킹 포함)**: **`216.17s`**
- **Action Result**: `status=4 (SUCCEEDED=True)`, `success=True`
- **상태 전이 시퀀스 정합성**: `observed=[0, 1, 2, 3, 4, 1, 0]` == `expected=[0, 1, 2, 3, 4, 1, 0]` (`match=True`)
- **도킹 정밀도 향상**: Nav2 단독 핸드오프 평균 `29.27 cm` → C++17 50Hz PID 도킹 후 평균 **`3.42 cm` (`-88.3%`, 작업장 정착 기준 `1.43 cm` / `0.72 cm`)**

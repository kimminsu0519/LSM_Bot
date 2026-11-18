# 물류 5단계 FSM 상태 머신 및 FMS E2E 통합 테스트 리포트

- **프로젝트**: LSM Bot (Smart Warehouse Autonomous Mobile Robot & FMS)
- **작성일**: 2026-11-18
- **테스트 러너**: [`run_fsm_e2e_test.py`](../../lsm_ws/src/lsm_navigation/scripts/run_fsm_e2e_test.py) (`lsm_venv` 가상환경, `slip_mode:=false` 정상 마찰 주행 모드)
- **핵심 미션 ID**: `TASK_A4_A5_E2E_001` (`wp-charge-1 -> wp-ws-a-4 -> wp-ws-a-5 -> wp-charge-1`)

---

## 1. 5단계 물류 FSM 아키텍처 개요

[`amr_state_machine_node.py`](../../lsm_ws/src/lsm_navigation/lsm_navigation/amr_state_machine_node.py)는 기존 A* 토폴로지 경로 탐색 및 10Hz 순차 주행 엔진(`SequentialNavigator` in [`telemetry_bridge.py`](../../lsm_ws/src/lsm_fms_web/lsm_fms_web/telemetry_bridge.py))을 침해하지 않는 독립 ROS 2 노드로 설계되었으며, `lsm_interfaces`의 커스텀 메시지·서비스·액션 계약을 완전 구현합니다.

### 1.1 5단계 상태 전이 다이어그램

```
  ┌────────────────────────────────────────────────────────────────────────────┐
  │                        0: CHARGING (충전소 대기/충전)                      │
  │                        (초기 위치: wp-charge-1 / Map Origin 보정)          │
  └───────────────┬────────────────────────────────────────────▲───────────────┘
                  │ /fms/dispatch_task Goal 수신               │ wp-charge-1 도착
                  ▼                                            │ (Mission Complete)
  ┌───────────────────────────────┐            ┌───────────────┴───────────────┐
  │   1: MOVING_UNLOADED          │            │   1: MOVING_UNLOADED          │
  │   (공차 주행: 픽업 스테이션)   │            │   (공차 복귀: 충전 스테이션)   │
  └───────────────┬───────────────┘            └───────────────▲───────────────┘
                  │ 픽업지 도착                                │ CARGO_UNLOADED(3)
                  │ (wp-ws-a-4)                                │ (/workstation/trigger_mock)
                  ▼                                            │
  ┌───────────────────────────────┐            ┌───────────────┴───────────────┐
  │   2: WORKING_LOADING          │            │   4: WORKING_UNLOADING        │
  │   (픽업 작업장 적재 대기)     │            │   (하역 작업장 하역 대기)      │
  └───────────────┬───────────────┘            └───────────────▲───────────────┘
                  │ CARGO_LOADED(2)                            │ 하역지 도착
                  │ (/workstation/trigger_mock)                │ (wp-ws-a-5)
                  ▼                                            │
  ┌────────────────────────────────────────────────────────────┴───────────────┐
  │                        3: MOVING_LOADED                                    │
  │                        (실차 주행: 목적지 작업장 이동)                     │
  └────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 보존된 핵심 불변량 및 안정성 보강 사항

- **지도 오프셋 (`lsm_warehouse_map.yaml`)**: `origin: [-2.4, -0.3, 0.0]`, `resolution: 0.050`
- **초기 스폰 및 AMCL 정렬**: Gazebo 스폰 `(2.4, 0.3, 0.1, Y=3.14159)`, `nav2_params.yaml` 및 `auto_init_pose` `180°` 헤딩(`z: 1.0, w: 0.0`) 보존
- **토폴로지 그래프 (`waypoints_graph.yaml`)**: 총 206개 노드(`active_waypoints: 156`, `reserved_waypoints: 50`, `208` directed edges) A* 경로 탐색 및 `SequentialNavigator` 순차 주행 연동 유지
- **경로 이식성 및 취소(Cancel) 안전성 보강**:
  - 모든 파이썬 노드(`telemetry_bridge.py`, `amr_state_machine_node.py`, `lsm_fms_route_server.py`)에서 하드코딩된 절대 경로를 제거하고 `ament_index_python` 패키지 공유 디렉터리 우선 조회 + 상대 경로 폴백 구조로 통일했습니다.
  - `DispatchTask` 액션 취소 요청, 타임아웃, 또는 외부 Nav2 목표 취소(`STATUS_CANCELED`) 발생 시 `SequentialNavigator.cancel_active_route()`가 즉시 활성 웨이포인트 큐를 비우고 Nav2 목표를 취소(`cancel_goal_async`)하도록 보강했습니다.
  - 미션 수행 중 중복 Goal 유입 시 `GoalResponse.REJECT`로 작업 충돌을 방지합니다.

---

## 2. `lsm_interfaces` 및 FMS/웹 대시보드 연동 명세

### 2.1 ROS 2 인터페이스 계약

| 인터페이스 유형 | 토픽 / 서비스 / 액션 이름 | 타입 (`lsm_interfaces`) | 역할 및 주요 필드 |
| :--- | :--- | :--- | :--- |
| **Publisher (10 Hz)** | `/lsm01/robot_status` | [`RobotStatus.msg`](../../lsm_ws/src/lsm_interfaces/msg/RobotStatus.msg) | `header`, `robot_id`, `current_state` (`0..4`), `pose`, `battery_level`, `current_task_id` |
| **Pub / Sub** | `/workstation/event` | [`WorkstationEvent.msg`](../../lsm_ws/src/lsm_interfaces/msg/WorkstationEvent.msg) | `station_id`, `event_type` (`1: JOB_DONE`, `2: CARGO_LOADED`, `3: CARGO_UNLOADED`), `task_id` |
| **Action Server** | `/fms/dispatch_task` | [`DispatchTask.action`](../../lsm_ws/src/lsm_interfaces/action/DispatchTask.action) | Goal(`task_id`, `task_type` `1..3`, `target_station_id`), Feedback(`current_state`, `distance_remaining`, `estimated_time_remaining`), Result(`success`, `message`, `total_execution_time`) |
| **Service Server** | `/workstation/trigger_mock` | [`TriggerMockEvent.srv`](../../lsm_ws/src/lsm_interfaces/srv/TriggerMockEvent.srv) | Request(`station_id`, `event_type` `2` or `3`), Response(`success`, `message`) |
| **Internal Bridge** | `/fms/fsm_route_cmd`, `/fms/route_status` | `std_msgs/msg/String` (JSON) | `amr_state_machine_node` ↔ `SequentialNavigator`(`telemetry_bridge.py`) 간 A* 다구간 주행 지령·취소 및 완료 보고 |

### 2.2 FMS WebSocket 브릿지 및 웹 UI 통합

- **[`telemetry_bridge.py`](../../lsm_ws/src/lsm_fms_web/lsm_fms_web/telemetry_bridge.py)**:
  - `/lsm01/robot_status`를 구독하여 Port `9090` Pure Python WebSocket 텔레메트리 JSON에 `fsm_state`, `fsm_state_name`, `current_task_id`, `battery` 필드를 포함해 브로드캐스트합니다.
  - 웹 클라이언트 명령 `fsm_dispatch_task` → `/fms/dispatch_task` 액션 전송, `fsm_trigger_mock` → `/workstation/trigger_mock` 서비스 호출, `cancel_route` → 활성 경로 즉시 중단 브릿지를 지원합니다.
- **웹 UI ([`index.html`](../../lsm_ws/src/lsm_fms_web/web/index.html), [`developer.html`](../../lsm_ws/src/lsm_fms_web/web/developer.html))**:
  - **5단계 FSM 실시간 상태 배지** 및 5-Step 스텝퍼 시각화(`0: CHARGING` ~ `4: WORKING_UNLOADING`), 미션 디스패치 및 `CARGO_LOADED` / `CARGO_UNLOADED` 수동 트리거 버튼을 제공합니다.

---

## 3. E2E 풀 사이클 주행 시나리오 및 단계별 실측 타임스탬프 (`slip_mode:=false`)

### 3.1 주행 구간(Leg)별 A* 경로 탐색 결과 (`telemetry_bridge.py` 실측 로그)

| 구간 (Leg) | 출발 → 목적지 | A* 웨이포인트 수 / 총 거리 | 도달 판정 실측 (목표 공차) |
| :--- | :--- | :---: | :--- |
| **Leg 1** (공차 픽업) | `wp-charge-1` → `wp-ws-a-4` | **9 wps / 6.94 m** | `wp-ws-a-4 (4.5, 2.1)` 도달 (`dist=0.29m <= 0.3m`) |
| **Leg 2** (실차 하역) | `wp-ws-a-4` → `wp-ws-a-5` | **12 wps / 11.65 m** | `wp-ws-a-5 (6.9, 2.1)` 도달 (`dist=0.29m <= 0.3m`) |
| **Leg 3** (충전소 복귀) | `wp-ws-a-5` → `wp-charge-1` | **10 wps / 9.55 m** | `wp-charge-1 (2.4, 0.3)` 복귀 (`world=(2.583, 0.529)`, 잔여거리 `0.293m <= 0.3m`) |

### 3.2 상태 전이 단계별 실측 타임스탬프 로그 (`slip_mode:=false` 정상 마찰)

[`run_fsm_e2e_test.py`](../../lsm_ws/src/lsm_navigation/scripts/run_fsm_e2e_test.py) 실행 시 `/lsm01/robot_status`에서 관측된 실제 전이 타임스탬프 및 좌표 기록입니다. (비대칭 슬립 모드를 분리하고 정상 대칭 마찰 `mu=1.0`을 기본 적용함으로써 총 주행 시간이 기존 `237.83s`에서 **`218.35s`로 `19.48s (8.2%)` 단축**되었습니다.)

| 단계 | Wall Time (KST) | Sim Time (s) | 상태 전이 | ROS2 Map Pose (m) | Gazebo World Pose (m) | 배터리 (%) | 트리거 / 이벤트 |
| :---: | :---: | :---: | :--- | :---: | :---: | :---: | :--- |
| 0 | `17:58:07.568` | `1794992287.57` | `INIT -> CHARGING(0)` | `(0.007, 0.028)` | `(2.407, 0.328)` | `100.0%` | 초기 충전 대기 상태 진입 |
| 1 | `17:58:09.472` | `1794992289.47` | `CHARGING(0) -> MOVING_UNLOADED(1)` | `(0.007, 0.028)` | `(2.407, 0.328)` | `100.0%` | `DispatchTask` (`wp-ws-a-4->wp-ws-a-5->wp-charge-1`) 수락 |
| 2 | `17:59:03.584` | `1794992343.58` | `MOVING_UNLOADED(1) -> WORKING_LOADING(2)` | `(1.862, 1.977)` | `(4.262, 2.277)` | `98.2%` | `wp-ws-a-4` 도착, 적재 대기 |
| 3 | `17:59:05.690` | `1794992345.69` | `WORKING_LOADING(2) -> MOVING_LOADED(3)` | `(1.949, 1.895)` | `(4.349, 2.195)` | `98.1%` | `/workstation/trigger_mock` (`CARGO_LOADED=2`) 성공 |
| 4 | `18:00:34.746` | `1794992434.75` | `MOVING_LOADED(3) -> WORKING_UNLOADING(4)` | `(4.303, 2.010)` | `(6.703, 2.310)` | `95.2%` | `wp-ws-a-5` 도착, 하역 대기 |
| 5 | `18:00:37.026` | `1794992437.03` | `WORKING_UNLOADING(4) -> MOVING_UNLOADED(1)` | `(4.357, 1.994)` | `(6.757, 2.294)` | `95.2%` | `/workstation/trigger_mock` (`CARGO_UNLOADED=3`) 성공 |
| 6 | `18:01:47.710` | `1794992507.71` | `MOVING_UNLOADED(1) -> CHARGING(0)` | `(0.183, 0.229)` | `(2.583, 0.529)` | `92.8%` | `wp-charge-1` 복귀 도킹 완료 |

### 3.3 실제 노드 및 테스트 러너 로그 스니펫

**① 테스트 러너 전체 전이 로그 (`/tmp/fsm_e2e_transitions.log`):**
```text
[2026-11-18 17:58:07.568 | sim=1794992287.57s] STATE TRANSITION: INIT(None) -> CHARGING(0) | task='' | map=(0.007, 0.028) | world=(2.407, 0.328) | battery=100.0%
[2026-11-18 17:58:09.323 | sim=1794992289.32s] ACTION GOAL -> /fms/dispatch_task(task_id='TASK_A4_A5_E2E_001', target_station_id='wp-ws-a-4->wp-ws-a-5->wp-charge-1')
[2026-11-18 17:58:09.376 | sim=1794992289.38s] ACTION ACCEPTED -> DispatchTask goal accepted by amr_state_machine_node
[2026-11-18 17:58:09.472 | sim=1794992289.47s] STATE TRANSITION: CHARGING(0) -> MOVING_UNLOADED(1) | task='TASK_A4_A5_E2E_001' | map=(0.007, 0.028) | world=(2.407, 0.328) | battery=100.0%
[2026-11-18 17:59:03.584 | sim=1794992343.58s] STATE TRANSITION: MOVING_UNLOADED(1) -> WORKING_LOADING(2) | task='TASK_A4_A5_E2E_001' | map=(1.862, 1.977) | world=(4.262, 2.277) | battery=98.2%
[2026-11-18 17:59:05.666 | sim=1794992345.67s] SERVICE CALL -> /workstation/trigger_mock(station_id='wp-ws-a-4', event_type=CARGO_LOADED(2))
[2026-11-18 17:59:05.690 | sim=1794992345.69s] STATE TRANSITION: WORKING_LOADING(2) -> MOVING_LOADED(3) | task='TASK_A4_A5_E2E_001' | map=(1.949, 1.895) | world=(4.349, 2.195) | battery=98.1%
[2026-11-18 17:59:05.717 | sim=1794992345.72s] SERVICE RESP <- success=True, message='Mock CARGO_LOADED event triggered for station wp-ws-a-4'
[2026-11-18 18:00:34.746 | sim=1794992434.75s] STATE TRANSITION: MOVING_LOADED(3) -> WORKING_UNLOADING(4) | task='TASK_A4_A5_E2E_001' | map=(4.303, 2.010) | world=(6.703, 2.310) | battery=95.2%
[2026-11-18 18:00:36.884 | sim=1794992436.88s] SERVICE CALL -> /workstation/trigger_mock(station_id='wp-ws-a-5', event_type=CARGO_UNLOADED(3))
[2026-11-18 18:00:36.935 | sim=1794992436.94s] SERVICE RESP <- success=True, message='Mock CARGO_UNLOADED event triggered for station wp-ws-a-5'
[2026-11-18 18:00:37.026 | sim=1794992437.03s] STATE TRANSITION: WORKING_UNLOADING(4) -> MOVING_UNLOADED(1) | task='TASK_A4_A5_E2E_001' | map=(4.357, 1.994) | world=(6.757, 2.294) | battery=95.2%
[2026-11-18 18:01:47.710 | sim=1794992507.71s] STATE TRANSITION: MOVING_UNLOADED(1) -> CHARGING(0) | task='' | map=(0.183, 0.229) | world=(2.583, 0.529) | battery=92.8%
[2026-11-18 18:01:47.866 | sim=1794992507.87s] ACTION RESULT <- status=4 (SUCCEEDED=True), success=True, total_execution_time=218.35s, message='Full 5-stage E2E logistics mission completed (wp-ws-a-4 -> wp-ws-a-5 -> wp-charge-1)'
[2026-11-18 18:01:48.368 | sim=1794992508.37s] FSM SEQUENCE CHECK: observed=[0, 1, 2, 3, 4, 1, 0] | expected=[0, 1, 2, 3, 4, 1, 0] | match=True
```

**② `amr_state_machine_node` 및 `SequentialNavigator` 실행 발췌:**
```text
[spawn_waffle_warehouse] slip_mode=false -> Symmetric normal warehouse tire friction enabled (wheel_left=1.0, wheel_right=1.0)
[amr_state_machine_node-6] [INFO] [1794991927.721677795] [amr_state_machine_node]: [FSM Ready] robot_id=LSM01, initial_state=CHARGING (0), loaded 206 waypoints, map_origin=(-2.4, -0.3)
[amr_state_machine_node-6] [INFO] [1794992289.338027850] [amr_state_machine_node]: [FSM TRANSITION] CHARGING(0) -> MOVING_UNLOADED(1) | task=TASK_A4_A5_E2E_001 | Dispatched to pickup wp-ws-a-4
[amr_state_machine_node-6] [INFO] [1794992343.579702224] [amr_state_machine_node]: [FSM TRANSITION] MOVING_UNLOADED(1) -> WORKING_LOADING(2) | task=TASK_A4_A5_E2E_001 | Arrived at pickup wp-ws-a-4; awaiting CARGO_LOADED
[amr_state_machine_node-6] [INFO] [1794992345.687242576] [amr_state_machine_node]: [FSM TRANSITION] WORKING_LOADING(2) -> MOVING_LOADED(3) | task=TASK_A4_A5_E2E_001 | Cargo loaded; transporting to wp-ws-a-5
[amr_state_machine_node-6] [INFO] [1794992434.686544315] [amr_state_machine_node]: [FSM TRANSITION] MOVING_LOADED(3) -> WORKING_UNLOADING(4) | task=TASK_A4_A5_E2E_001 | Arrived at delivery wp-ws-a-5; awaiting CARGO_UNLOADED
[amr_state_machine_node-6] [INFO] [1794992436.909129056] [amr_state_machine_node]: [FSM TRANSITION] WORKING_UNLOADING(4) -> MOVING_UNLOADED(1) | task=TASK_A4_A5_E2E_001 | Cargo unloaded; returning to charger wp-charge-1
[amr_state_machine_node-6] [INFO] [1794992507.683540387] [amr_state_machine_node]: [FSM TRANSITION] MOVING_UNLOADED(1) -> CHARGING(0) | task=TASK_A4_A5_E2E_001 | Docked at wp-charge-1; cycle complete
```

---

## 4. 종합 판정

- **총 주행 시간 (`slip_mode:=false`)**: **`218.35s`** (기존 상시 슬립 조건 `237.83s` 대비 **`19.48s` 단축**)
- **Action Result**: `status=4 (SUCCEEDED=True)`, `success=True`
- **상태 전이 시퀀스 정합성**: `observed=[0, 1, 2, 3, 4, 1, 0]` == `expected=[0, 1, 2, 3, 4, 1, 0]` (`match=True`)
- **실시간 관제 UI 및 RViz2 정합 확인**:
  - [`02_rviz_amcl_nav2_sync.png`](../portfolio_assets/02_rviz_amcl_nav2_sync.png)
  - [`03_fms_web_fsm_dashboard.png`](../portfolio_assets/03_fms_web_fsm_dashboard.png)

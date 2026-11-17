# 물류 5단계 FSM 상태 머신 및 FMS E2E 통합 테스트 리포트

- **프로젝트**: LSM Bot (Smart Warehouse Autonomous Mobile Robot & FMS)
- **작성일**: 2026-11-17
- **테스트 러너**: [`run_fsm_e2e_test.py`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/scripts/run_fsm_e2e_test.py) (`lsm_venv` 가상환경)
- **핵심 미션 ID**: `TASK_A4_A5_E2E_001` (`wp-charge-1 -> wp-ws-a-4 -> wp-ws-a-5 -> wp-charge-1`)

---

## 1. 5단계 물류 FSM 아키텍처 개요

[`amr_state_machine_node.py`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/lsm_navigation/amr_state_machine_node.py)는 기존 A* 토폴로지 경로 탐색 및 10Hz 순차 주행 엔진(`SequentialNavigator` in [`telemetry_bridge.py`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_fms_web/lsm_fms_web/telemetry_bridge.py))을 침해하지 않는 독립 ROS 2 노드로 설계되었으며, `lsm_interfaces`의 커스텀 메시지·서비스·액션 계약을 완전 구현합니다.

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

### 1.2 보존된 기존 핵심 불변량 (Invariant)

- **지도 오프셋 (`lsm_warehouse_map.yaml`)**: `origin: [-2.4, -0.3, 0.0]`, `resolution: 0.050`
- **초기 스폰 및 AMCL 정렬**: Gazebo 스폰 `(2.4, 0.3, 0.1, Y=3.14159)`, `nav2_params.yaml` 및 `auto_init_pose` `180°` 헤딩(`z: 1.0, w: 0.0`) 보존
- **토폴로지 그래프 (`waypoints_graph.yaml`)**: 총 206개 노드(`active_waypoints: 156`, `reserved_waypoints: 50`, `208` directed edges) A* 경로 탐색 및 `SequentialNavigator` 순차 주행 연동 유지

---

## 2. `lsm_interfaces` 및 FMS/웹 대시보드 연동 명세

### 2.1 ROS 2 인터페이스 계약

| 인터페이스 유형 | 토픽 / 서비스 / 액션 이름 | 타입 (`lsm_interfaces`) | 역할 및 주요 필드 |
| :--- | :--- | :--- | :--- |
| **Publisher (10 Hz)** | `/lsm01/robot_status` | [`RobotStatus.msg`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_interfaces/msg/RobotStatus.msg) | `header`, `robot_id`, `current_state` (`0..4`), `pose`, `battery_level`, `current_task_id` |
| **Pub / Sub** | `/workstation/event` | [`WorkstationEvent.msg`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_interfaces/msg/WorkstationEvent.msg) | `station_id`, `event_type` (`1: JOB_DONE`, `2: CARGO_LOADED`, `3: CARGO_UNLOADED`), `task_id` |
| **Action Server** | `/fms/dispatch_task` | [`DispatchTask.action`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_interfaces/action/DispatchTask.action) | Goal(`task_id`, `task_type` `1..3`, `target_station_id`), Feedback(`current_state`, `distance_remaining`, `estimated_time_remaining`), Result(`success`, `message`, `total_execution_time`) |
| **Service Server** | `/workstation/trigger_mock` | [`TriggerMockEvent.srv`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_interfaces/srv/TriggerMockEvent.srv) | Request(`station_id`, `event_type` `2` or `3`), Response(`success`, `message`) |
| **Internal Bridge** | `/fms/fsm_route_cmd`, `/fms/route_status` | `std_msgs/msg/String` (JSON) | `amr_state_machine_node` ↔ `SequentialNavigator`(`telemetry_bridge.py`) 간 A* 다구간 주행 지령 및 완료 보고 |

### 2.2 FMS WebSocket 브릿지 및 웹 UI 통합

- **`telemetry_bridge.py`** ([`telemetry_bridge.py`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_fms_web/lsm_fms_web/telemetry_bridge.py)):
  - `/lsm01/robot_status`를 구독하여 Port `9090` Pure Python WebSocket 텔레메트리 JSON에 `fsm_state`, `fsm_state_name`, `current_task_id`, `battery` 필드를 포함해 브로드캐스트.
  - 웹 클라이언트 명령 `fsm_dispatch_task` → `/fms/dispatch_task` 액션 전송, `fsm_trigger_mock` → `/workstation/trigger_mock` 서비스 호출 브릿지 구현.
  - `/fms/fsm_route_cmd` 수신 시 기존 `SequentialNavigator` A* 웨이포인트 시퀀스 실행 및 웹 UI 경로/진행상태(`route_plan`, `route_progress`) 동시 갱신.
- **웹 UI ([`index.html`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_fms_web/web/index.html), [`developer.html`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_fms_web/web/developer.html))**:
  - **5단계 FSM 실시간 상태 배지** 및 5-Step 스텝퍼 시각화(`0: CHARGING` ~ `4: WORKING_UNLOADING`) 구현.
  - 픽업→배송→복귀 미션 디스패치 및 `CARGO_LOADED` / `CARGO_UNLOADED` 수동 트리거 버튼 연동.

---

## 3. E2E 풀 사이클 주행 시나리오 및 단계별 실측 타임스탬프

### 3.1 주행 구간(Leg)별 A* 경로 탐색 결과 (`telemetry_bridge.py` 실측 로그)

| 구간 (Leg) | 출발 → 목적지 | A* 웨이포인트 수 / 총 거리 | 도달 판정 실측 (목표 공차) |
| :--- | :--- | :---: | :--- |
| **Leg 1** (공차 픽업) | `wp-charge-1` → `wp-ws-a-4` | **9 wps / 6.94 m** | `wp-ws-a-4 (4.5, 2.1)` 도달 (`dist=0.29m <= 0.3m`) |
| **Leg 2** (실차 하역) | `wp-ws-a-4` → `wp-ws-a-5` | **12 wps / 11.65 m** | `wp-ws-a-5 (6.9, 2.1)` 도달 (`dist=0.29m <= 0.3m`) |
| **Leg 3** (충전소 복귀) | `wp-ws-a-5` → `wp-charge-1` | **10 wps / 9.55 m** | `wp-charge-1 (2.4, 0.3)` 복귀 (`world=(2.592, 0.528)`, 잔여거리 `0.298m`) |

### 3.2 상태 전이 단계별 실측 타임스탬프 로그

아래 표는 [`run_fsm_e2e_test.py`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/scripts/run_fsm_e2e_test.py) 실행 시 `/lsm01/robot_status`에서 관측된 실제 전이 타임스탬프 및 좌표 기록입니다.

| 단계 | Wall Time (KST) | Sim Time (s) | 상태 전이 | ROS2 Map Pose (m) | Gazebo World Pose (m) | 배터리 (%) | 트리거 / 이벤트 |
| :---: | :---: | :---: | :--- | :---: | :---: | :---: | :--- |
| 0 | `19:26:14.553` | `1794911174.55` | `INIT -> CHARGING(0)` | `(0.014, 0.049)` | `(2.414, 0.349)` | `100.0%` | 초기 충전 대기 상태 진입 |
| 1 | `19:26:15.711` | `1794911175.71` | `CHARGING(0) -> MOVING_UNLOADED(1)` | `(0.014, 0.049)` | `(2.414, 0.349)` | `100.0%` | `DispatchTask` (`wp-ws-a-4->wp-ws-a-5->wp-charge-1`) 수락 |
| 2 | `19:27:16.991` | `1794911236.99` | `MOVING_UNLOADED(1) -> WORKING_LOADING(2)` | `(1.846, 1.934)` | `(4.246, 2.234)` | `98.2%` | `wp-ws-a-4` 도착, 적재 대기 |
| 3 | `19:27:19.301` | `1794911239.30` | `WORKING_LOADING(2) -> MOVING_LOADED(3)` | `(1.880, 1.902)` | `(4.280, 2.202)` | `98.2%` | `/workstation/trigger_mock` (`CARGO_LOADED=2`) 성공 |
| 4 | `19:28:53.900` | `1794911333.90` | `MOVING_LOADED(3) -> WORKING_UNLOADING(4)` | `(4.311, 2.022)` | `(6.711, 2.322)` | `95.2%` | `wp-ws-a-5` 도착, 하역 대기 |
| 5 | `19:28:56.138` | `1794911336.14` | `WORKING_UNLOADING(4) -> MOVING_UNLOADED(1)` | `(4.343, 1.975)` | `(6.743, 2.275)` | `95.2%` | `/workstation/trigger_mock` (`CARGO_UNLOADED=3`) 성공 |
| 6 | `19:30:13.410` | `1794911413.41` | `MOVING_UNLOADED(1) -> CHARGING(0)` | `(0.192, 0.228)` | `(2.592, 0.528)` | `92.8%` | `wp-charge-1` 복귀 도킹 완료 |

### 3.3 실제 노드 및 테스트 러너 로그 스니펫

**① 테스트 러너 전체 전이 로그 (`/tmp/fsm_e2e_transitions.log`):**
```text
[2026-11-17 19:26:14.553 | sim=1794911174.55s] STATE TRANSITION: INIT(None) -> CHARGING(0) | task='' | map=(0.014, 0.049) | world=(2.414, 0.349) | battery=100.0%
[2026-11-17 19:26:15.556 | sim=1794911175.56s] ACTION GOAL -> /fms/dispatch_task(task_id='TASK_A4_A5_E2E_001', target_station_id='wp-ws-a-4->wp-ws-a-5->wp-charge-1')
[2026-11-17 19:26:15.610 | sim=1794911175.61s] ACTION ACCEPTED -> DispatchTask goal accepted by amr_state_machine_node
[2026-11-17 19:26:15.711 | sim=1794911175.71s] STATE TRANSITION: CHARGING(0) -> MOVING_UNLOADED(1) | task='TASK_A4_A5_E2E_001' | map=(0.014, 0.049) | world=(2.414, 0.349) | battery=100.0%
[2026-11-17 19:27:16.991 | sim=1794911236.99s] STATE TRANSITION: MOVING_UNLOADED(1) -> WORKING_LOADING(2) | task='TASK_A4_A5_E2E_001' | map=(1.846, 1.934) | world=(4.246, 2.234) | battery=98.2%
[2026-11-17 19:27:19.155 | sim=1794911239.16s] SERVICE CALL -> /workstation/trigger_mock(station_id='wp-ws-a-4', event_type=CARGO_LOADED(2))
[2026-11-17 19:27:19.206 | sim=1794911239.21s] SERVICE RESP <- success=True, message='Mock CARGO_LOADED event triggered for station wp-ws-a-4'
[2026-11-17 19:27:19.301 | sim=1794911239.30s] STATE TRANSITION: WORKING_LOADING(2) -> MOVING_LOADED(3) | task='TASK_A4_A5_E2E_001' | map=(1.880, 1.902) | world=(4.280, 2.202) | battery=98.2%
[2026-11-17 19:28:53.900 | sim=1794911333.90s] STATE TRANSITION: MOVING_LOADED(3) -> WORKING_UNLOADING(4) | task='TASK_A4_A5_E2E_001' | map=(4.311, 2.022) | world=(6.711, 2.322) | battery=95.2%
[2026-11-17 19:28:56.015 | sim=1794911336.02s] SERVICE CALL -> /workstation/trigger_mock(station_id='wp-ws-a-5', event_type=CARGO_UNLOADED(3))
[2026-11-17 19:28:56.067 | sim=1794911336.07s] SERVICE RESP <- success=True, message='Mock CARGO_UNLOADED event triggered for station wp-ws-a-5'
[2026-11-17 19:28:56.138 | sim=1794911336.14s] STATE TRANSITION: WORKING_UNLOADING(4) -> MOVING_UNLOADED(1) | task='TASK_A4_A5_E2E_001' | map=(4.343, 1.975) | world=(6.743, 2.275) | battery=95.2%
[2026-11-17 19:30:13.410 | sim=1794911413.41s] STATE TRANSITION: MOVING_UNLOADED(1) -> CHARGING(0) | task='' | map=(0.192, 0.228) | world=(2.592, 0.528) | battery=92.8%
[2026-11-17 19:30:13.432 | sim=1794911413.43s] ACTION RESULT <- status=4 (SUCCEEDED=True), success=True, total_execution_time=237.83s, message='Full 5-stage E2E logistics mission completed (wp-ws-a-4 -> wp-ws-a-5 -> wp-charge-1)'
[2026-11-17 19:30:13.933 | sim=1794911413.93s] FSM SEQUENCE CHECK: observed=[0, 1, 2, 3, 4, 1, 0] | expected=[0, 1, 2, 3, 4, 1, 0] | match=True
```

**② `amr_state_machine_node` 및 `SequentialNavigator` 실행 발췌:**
```text
[amr_state_machine_node-6] [INFO] [1794910831.078374180] [amr_state_machine_node]: [FSM Ready] robot_id=LSM01, initial_state=CHARGING (0), loaded 206 waypoints, map_origin=(-2.4, -0.3)
[amr_state_machine_node-6] [INFO] [1794911175.567527862] [amr_state_machine_node]: [DispatchTask] Received goal: task_id='TASK_A4_A5_E2E_001', type=1, target='wp-ws-a-4->wp-ws-a-5->wp-charge-1'
[telemetry_bridge-2] [SequentialNavigator] Reached Waypoint [9/9] 'wp-ws-a-4' (dist=0.29m <= 0.3m)
[telemetry_bridge-2] [SequentialNavigator] Route Completed! Arrived at final destination: 'wp-ws-a-4'
[telemetry_bridge-2] [FSM -> SequentialNavigator] Route wp-ws-a-4 -> wp-ws-a-5 (12 wps, 11.65m)
[telemetry_bridge-2] [SequentialNavigator] Reached Waypoint [12/12] 'wp-ws-a-5' (dist=0.29m <= 0.3m)
[telemetry_bridge-2] [SequentialNavigator] Route Completed! Arrived at final destination: 'wp-ws-a-5'
[telemetry_bridge-2] [FSM -> SequentialNavigator] Route wp-ws-a-5 -> wp-charge-1 (10 wps, 9.55m)
[amr_state_machine_node-6] [INFO] [1794911413.397915558] [amr_state_machine_node]: [FSM TRANSITION] MOVING_UNLOADED(1) -> CHARGING(0) | task=TASK_A4_A5_E2E_001 | Docked at wp-charge-1; cycle complete
```

---

## 4. 종합 판정

- **총 주행 시간**: **237.83s** (`sim=1794911175.61s` ~ `1794911413.43s`)
- **Action Result**: `status=4 (SUCCEEDED=True)`, `success=True`
- **상태 전이 시퀀스 정합성**: `observed=[0, 1, 2, 3, 4, 1, 0]` == `expected=[0, 1, 2, 3, 4, 1, 0]` (`match=True`)
- **실시간 관제 UI 확인**: [`03_fms_web_fsm_dashboard.png`](file:///home/kms/dev/LSM_repository/docs/portfolio_assets/03_fms_web_fsm_dashboard.png) (1920x1080 캡처에서 `WS ONLINE`, `FSM STATE: 1: MOVING_UNLOADED`, `TASK ID: TASK_A4_A5_E2E_001`, A* 경로 `6.94 m / 9 nodes` 및 우측 큐 `PASSED/CURRENT/NEXT/PENDING` 동시 동작 확인)
- *(비고)* E2E 테스트 통과(`task-371`) 이후 테스트 러너 [`run_fsm_e2e_test.py`](file:///home/kms/dev/LSM_repository/lsm_ws/src/lsm_navigation/scripts/run_fsm_e2e_test.py)의 종료 시 스핀 스레드 정리(`rclpy.shutdown()` → `spin_thread.join()`) 순서 2줄을 보완하였으며, `amr_state_machine_node.py` 및 `telemetry_bridge.py` 런타임 코드는 E2E 검증 시점과 동일합니다.

# [Workstation Event Message]
# 작업장 공정 완료 및 적재/하역 이벤트 알림 (발행자: workstation_mock_node)

std_msgs/Header header         # 타임스탬프 및 프레임 ID
string station_id             # 작업장/스테이션 ID (예: "A-1"~"A-6", "B-1-1"~"B-4-4", "C-1"~"C-4", "CHARGE-1")
uint8 event_type               # 발생한 이벤트 유형 (1: JOB_DONE, 2: CARGO_LOADED, 3: CARGO_UNLOADED)
string task_id                # 연관된 작업 ID (예: "TASK_A1_B11_001")

# [Event Type Enums]
uint8 EVENT_JOB_DONE=1         # 1: JOB_DONE (작업장에서 생산/공정 완료 -> AMR 이송 요청 발생)
uint8 EVENT_CARGO_LOADED=2     # 2: CARGO_LOADED (AMR에 물품 적재 완료)
uint8 EVENT_CARGO_UNLOADED=3   # 3: CARGO_UNLOADED (AMR에서 물품 하역 완료)


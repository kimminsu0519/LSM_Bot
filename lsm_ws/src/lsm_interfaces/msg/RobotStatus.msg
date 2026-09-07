# [Robot Status Message]
# AMR 실시간 상태, 좌표, 배터리 정보 모니터링 (10Hz 발행)

std_msgs/Header header         # 타임스탬프 및 프레임 ID (frame_id: "map")
string robot_id               # 로봇 식별자 (예: "LSM01", "LSM02")
uint8 current_state           # AMR 5단계 FSM 상태 코드 (0 ~ 4)
geometry_msgs/Pose pose       # AMR 현재 Map 좌표계 기준 위치 (x, y, z) 및 자세 (orientation)
float32 battery_level         # 배터리 잔량 퍼센트 (0.0 ~ 100.0%)
string current_task_id        # 현재 수행 중인 작업 ID (예: "TASK_A1_B11_001", 미수행 시 "")

# [State Enums] AMR 5단계 상태 정의
uint8 STATE_CHARGING=0          # 0: CHARGING (충전소 대기 및 충전 상태)
uint8 STATE_MOVING_UNLOADED=1   # 1: MOVING_UNLOADED (공차 주행 - 픽업 위치로 이동 중)
uint8 STATE_WORKING_LOADING=2   # 2: WORKING_LOADING (픽업 작업장에서 물품 적재 중)
uint8 STATE_MOVING_LOADED=3     # 3: MOVING_LOADED (실차 주행 - 목적지 작업장으로 이동 중)
uint8 STATE_WORKING_UNLOADING=4 # 4: WORKING_UNLOADING (목적지 작업장에서 물품 하역 중)


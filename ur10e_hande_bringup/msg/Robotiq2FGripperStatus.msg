# Robotiq 2F Gripper Status Message
# 이 메시지는 Robotiq Hande 그리퍼의 상태를 보고하는데 사용됩니다.

uint8 g_act      # 그리퍼 활성화 상태 (0: 비활성화, 1: 활성화)
uint8 g_gto      # 이동 중 상태 (0: 정지, 1: 이동 중)
uint8 g_sta      # 그리퍼 상태 (0: 리셋, 1: 활성화 중, 3: 활성화됨)
uint8 g_obj      # 물체 감지 상태 (0: 이동 중, 1: 외부 물체 감지, 2: 내부 물체 감지, 3: 목표 위치 도달)
uint8 g_flt      # 폴트 상태 (0: 정상, 그 외: 오류 코드)
uint8 g_pr       # 목표 위치 에코 (0-255)
uint8 g_po       # 현재 위치 (0-255)
uint8 g_cu       # 현재 전류 (0-255)
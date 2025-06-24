# Robotiq 2F Gripper Command Message
# 이 메시지는 Robotiq Hande 그리퍼에 명령을 전송하는데 사용됩니다.

uint8 r_act      # 그리퍼 활성화 (0: 비활성화, 1: 활성화)
uint8 r_gto      # 이동 요청 (0: 중지, 1: 이동)
uint8 r_atr      # 자동 해제 (0: 비활성화, 1: 활성화)
uint8 r_pr       # 목표 위치 (0-255, 0: 완전 열림, 255: 완전 닫힘)
uint8 r_sp       # 속도 (0-255)
uint8 r_fr       # 힘 (0-255)
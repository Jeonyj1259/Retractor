# test_zaber.py
import time
from zaber_control import ZaberStage

PORT_ZABER = "COM8"          # 네 Zaber 포트
MICROSTEP_UM = 0.49609375    # µm/step, 스펙에서 가져온 값
DESIRED_SPEED_MM_S = -5.0    # -방향으로 5 mm/s

# mm/s -> native 속도 (step/s) 변환
vel_native = int(DESIRED_SPEED_MM_S / (MICROSTEP_UM / 1000.0))
print(f"Native velocity = {vel_native} step/s (≈ {DESIRED_SPEED_MM_S} mm/s)")

stage = ZaberStage(PORT_ZABER)

input("엔터를 누르면 움직이기 시작합니다...")

stage.move_velocity(vel_native)
print("이동 중... 2초간 대기")
time.sleep(2.0)

print("stop() 호출")
stage.stop()
time.sleep(0.5)

print("close()")
stage.close()
print("테스트 종료")

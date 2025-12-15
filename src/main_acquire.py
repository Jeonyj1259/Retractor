# main_acquire.py

from time import sleep

# ✅ 여기에서 RFT_UART 모듈을 import (파일 이름이 RFT_UART.py 이기 때문에)
from RFT_UART import (
    RFTseries,
    COMMAND_STOP_FT_DATA_OUTPUT,
    COMMNAD_READ_MODEL_NAME,      # 원래 파일에 이렇게 써 있을 거라 그대로 사용
    COMMAND_START_FT_DATA_OUTPUT,
    commandSetFilter,
)

# zaber_control.py를 아직 안 만들었거나 비어 있어도 에러 안 나게 try/except 처리
try:
    from zaber_control import ZaberStage
except ImportError:
    class ZaberStage:
        def __init__(self, port: str):
            self.port = port
            print(f"[Zaber] (stub) Using port: {port}")


# 👉 포트 번호는 네 환경에 맞게 수정!
PORT_RFT = "COM7"    # Robotus F/T 센서가 연결된 COM 포트
PORT_ZABER = "COM8"  # 나중에 Zaber 스테이지 포트 (지금은 안 써도 됨)


def init_robotus() -> RFTseries:
    """Robotus F/T 센서 초기화."""
    print("[Robotus] Initializing...")
    rft = RFTseries(PORT_RFT)

    # 혹시 기존에 데이터 출력 중이면 정지
    rft.sendCommand(COMMAND_STOP_FT_DATA_OUTPUT)
    sleep(0.1)
    rft.ser.flush()

    # 모델명 요청 (통신 확인용, 결과는 RFT_UART 안에서 처리하거나 무시)
    rft.sendCommand(COMMNAD_READ_MODEL_NAME)
    sleep(0.1)

    # 데이터 출력 시작
    rft.sendCommand(COMMAND_START_FT_DATA_OUTPUT)
    sleep(0.1)

    # 하드웨어 필터 설정 (필요에 따라 파라미터 조정)
    rft.sendCommand(commandSetFilter(1, 10))
    sleep(0.1)

    # 하드 타레
    rft.hardTare()
    print("[Robotus] Init done.")
    return rft


def main():
    # Zaber는 지금은 그냥 생성만 (stub이든 실제든)
    stage = ZaberStage(PORT_ZABER)

    # Robotus 초기화
    rft = init_robotus()

    print("\n[Main] Reading 10 samples from Robotus...")
    try:
        for i in range(10):
            fx, fy, fz, tx, ty, tz = rft.getTareFT()
            print(
                f"{i:02d}: "
                f"Fx={fx:.3f}, Fy={fy:.3f}, Fz={fz:.3f}, "
                f"Tx={tx:.3f}, Ty={ty:.3f}, Tz={tz:.3f}"
            )
            sleep(0.05)  # 50ms 간격 (테스트용, 나중에 1000Hz 수집으로 바꿀 예정)

    finally:
        print("\n[Main] Stopping output & closing serial...")
        try:
            rft.sendCommand(COMMAND_STOP_FT_DATA_OUTPUT)
        except Exception as e:
            print(f"[WARN] stop command failed: {e}")
        try:
            rft.close()
        except Exception as e:
            print(f"[WARN] serial close failed: {e}")

    print("[Main] Done.")


if __name__ == "__main__":
    main()

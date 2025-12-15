# zaber_control.py
"""
간단한 Zaber ASCII 제어 래퍼.
- 포트만 주면 /<addr> 주소의 축을 velocity 모드로 움직이거나 stop 할 수 있음.
- 현재 위치 읽기(get_position) + native 단위 절대 이동(move_abs_native) 지원.
"""

import serial


class ZaberStage:
    def __init__(self, port: str, baudrate: int = 115200, addr: int = 1):
        self.port = port
        self.addr = addr
        self.ser = serial.Serial(
            port,
            baudrate=baudrate,
            timeout=0.2,
            write_timeout=0.2,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print(f"[Zaber] Connected to {port} at {baudrate} baud (addr={addr}).")

    # ---------- 내부 유틸 ----------

    def _write_cmd(self, cmd: str):
        """
        ASCII 커맨드 한 줄 전송.
        예: 'move vel 10000' → '/1 move vel 10000\\r\\n'
        """
        line = f"/{self.addr} {cmd}\r\n"
        self.ser.write(line.encode("ascii"))

    def _read_reply(self) -> str:
        """
        응답 한 줄 읽기 (없으면 빈 문자열 반환).
        """
        try:
            reply = self.ser.readline().decode("ascii", errors="ignore").strip()
        except Exception:
            reply = ""
        return reply

    # ---------- 위치 관련 ----------

    def get_position(self) -> int:
        """
        현재 절대 위치를 native 단위(int)로 읽어온다.
        Zaber ASCII 응답에서 마지막에 있는 정수 토큰을 파싱하는 방식.
        """
        self._write_cmd("get pos")
        reply = self._read_reply()
        # 예시 응답: '/1 0 OK IDLE POS 123456'
        for token in reversed(reply.split()):
            try:
                return int(token)
            except ValueError:
                continue
        raise ValueError(f"Could not parse position from reply: {reply}")

    def move_abs_native(self, native_pos: int):
        """
        native 단위 절대 위치로 이동.
        mm 단위가 아니라, 장치의 스텝/카운트 단위 그대로 사용.
        """
        print(f"[Zaber] move abs {native_pos}")
        self._write_cmd(f"move abs {native_pos}")
        return self._read_reply()

    # ---------- 동작 명령 ----------

    def move_velocity(self, vel_native: int):
        """
        속도 모드로 계속 움직이기.
        vel_native: native 속도 단위 (step/s 또는 count/s).
        sign(±)으로 방향 결정.
        """
        print(f"[Zaber] move vel {vel_native}")
        self._write_cmd(f"move vel {vel_native}")
        return self._read_reply()

    def stop(self):
        """즉시 정지."""
        print("[Zaber] stop()")
        self._write_cmd("stop")
        return self._read_reply()

    def home(self):
        """홈 센서까지 이동 (필요시 사용)."""
        print("[Zaber] home()")
        self._write_cmd("home")
        return self._read_reply()

    def close(self):
        """시리얼 포트 닫기."""
        print("[Zaber] close()")
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

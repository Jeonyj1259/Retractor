import serial
import time
import threading
from RFT_UART_command import *
from RFT_UART_response import *

class RFTseries:
    __response = dict()
    DT = 50
    DF = 1000
    offsetFx, offsetFy, offsetFz, offsetTx, offsetTy, offsetTz = 0, 0, 0, 0, 0, 0
    def __init__(self, port, baud=115200):
        # default 115200 bps
        # 1 stop bit, No parity, No flow control, 8 data bits
        self.ser = serial.Serial(port, baud)
        self.ser.flush()
        self._running = True
        self.__thread = threading.Thread(target=self.__readResponseRunner)
        self.__thread.daemon = True
        self.__thread.start()

    def close(self):
            # 1) 스레드 종료 요청
        self._running = False

        # 2) 스레드가 살아있으면 잠깐 기다렸다 종료
        try:
            if hasattr(self, "_RFTseries__readerThread"):
                if self.__readerThread.is_alive():
                    self.__readerThread.join(timeout=0.5)
        except Exception as e:
            print(f"[RFT] reader thread join error: {e}")

        # 3) 시리얼 포트 닫기
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception as e:
            print(f"[RFT] serial close error: {e}")
    ## Command Packet Structure
    # SOP : 0x55
    # Data Field  : 8 bytes
    # Checksum : 1 byte, summation of data field
    # EOP : 0xAA
    def sendCommand(self, command):
        checksum = sum(command) & 0xFF
        if len(command) != 8:
            raise ValueError('Data field must be 8 bytes long')
        packet = b'\x55' + command + checksum.to_bytes(1, 'big') + b'\xAA'
        self.ser.write(packet)
        return packet
    ## Response Packet Structure
    # SOP : 0x55
    # Data Field  : 16 bytes
    # Checksum : 1 byte
    # EOP : 0xAA
    def __readResponseRunner(self):
        while self._running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    if self.ser.read() == b'\x55':
                        data = self.ser.read(16)
                        checksum = self.ser.read()
                        eop = self.ser.read()
                        responseID = data[0]
                        self.__response[responseID] = data
                else:
                    time.sleep(0.001)  # CPU 100% 방지
            except serial.SerialException:
                # 포트가 닫혔거나 에러난 경우 → 조용히 종료
                break
            except Exception as e:
                print(f"[RFT] reader thread error: {e}")
                break
    def getResponse(self, responseID):
        return self.__response.get(responseID)
    def hardTare(self):
        self.sendCommand(commandSetBias(True))
    def softTare(self):
        self.offsetFx, self.offsetFy, self.offsetFz, self.offsetTx, self.offsetTy, self.offsetTz, _ = responseReadFTData(self.getResponse(ID_START_FT_DATA_OUTPUT))
    def getTareFT(self):
        rawFx, rawFy, rawFz, rawTx, rawTy, rawTz, _ = responseReadFTData(self.getResponse(ID_START_FT_DATA_OUTPUT))
        return rawFx - self.offsetFx, rawFy - self.offsetFy, rawFz - self.offsetFz, rawTx - self.offsetTx, rawTy - self.offsetTy, rawTz - self.offsetTz

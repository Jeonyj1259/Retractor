COMMNAD_READ_MODEL_NAME                 = bytes.fromhex('01 00 00 00 00 00 00 00')
COMMAND_READ_SERIAL_NUMBER              = bytes.fromhex('02 00 00 00 00 00 00 00')
COMMAND_READ_FIRMWARE_VERSION           = bytes.fromhex('03 00 00 00 00 00 00 00')
COMMAND_READ_BAUDRATE                   = bytes.fromhex('07 00 00 00 00 00 00 00')
COMMAND_READ_FILTER                     = bytes.fromhex('09 00 00 00 00 00 00 00')
COMMAND_READ_FT_DATA                    = bytes.fromhex('0A 00 00 00 00 00 00 00')
COMMAND_START_FT_DATA_OUTPUT            = bytes.fromhex('0B 00 00 00 00 00 00 00')
COMMAND_STOP_FT_DATA_OUTPUT             = bytes.fromhex('0C 00 00 00 00 00 00 00')
COMMAND_READ_DATA_OUTPUT_RATE           = bytes.fromhex('10 00 00 00 00 00 00 00')
COMMAND_READ_COUNT_OVERLOAD_OCCURRENCE  = bytes.fromhex('12 00 00 00 00 00 00 00')
def commandSetBaudrate(baudrate: int):
    if baudrate == 115200:
        return bytes.fromhex('06 00 00 00 00 00 00 00')
    elif baudrate == 921600:
        return bytes.fromhex('06 01 00 00 00 00 00 00')
    elif baudrate == 460800:
        return bytes.fromhex('06 02 00 00 00 00 00 00')
    elif baudrate == 230400:
        return bytes.fromhex('06 03 00 00 00 00 00 00')
    elif baudrate == 115200:
        return bytes.fromhex('06 04 00 00 00 00 00 00')
    elif baudrate == 57600:
        return bytes.fromhex('06 05 00 00 00 00 00 00')
    else:
        print("baudrate not supported. Supported baudrates are 115200, 921600, 460800, 230400, 115200, 57600")
        raise ValueError('Invalid baudrate')
def commandSetFilter(type_: int, parameter: int):
    if type_ not in (0, 1):
        raise ValueError("type_ 값은 0 또는 1이어야 합니다.")
    if not (0 <= parameter <= 14):
        raise ValueError("parameter 값은 0~14 범위여야 합니다.")

    return b'\x08' + type_.to_bytes(1, 'big') + parameter.to_bytes(1, 'big') + b'\x00\x00\x00\x00\x00'
 
def commandSetDataOutputRate(hz: int):
    paramDict = { 200: 0, 10: 1, 20: 2,
                  50: 3, 100: 4, 200: 5,
                  333: 6, 500: 7, 1000: 8 }
    parameter = paramDict.get(hz)
    if parameter == None:
        print("Invalid hz. Supported hz are 200, 10, 20, 50, 100, 200, 333, 500, 1000")
        raise ValueError('Invalid hz')
    return b'\17' + int.to_bytes(parameter) + b'\00\00\00\00\00\00'  # b'\17' -> b'\15'
def commandSetBias(bias: bool):
    return b'\21' + (b'\01' if bias else b'\00') + b'\00\00\00\00\00\00'  # b'\21' -> b'\17'
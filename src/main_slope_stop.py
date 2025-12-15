# main_slope_stop.py
"""
Robotus F/T 센서를 고주파수(TARGET_FS)로 읽으면서
Fz에 EMA 저역통과 필터를 적용하고,
윈도우 기반 dFz/dt(slope)가 어떤 임계값을 넘으면

1) Zaber 속도 이동을 stop()
2) 정지 당시 현재 위치를 읽고
3) 실험 시작 당시 Zaber 위치로 복귀(move abs)
4) Robotus/플롯/CSV/PNG 정리 후 종료

까지 자동으로 수행하는 스크립트.
"""

import os
import csv
import time
import threading
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from RFT_UART import (
    RFTseries,
    COMMAND_STOP_FT_DATA_OUTPUT,
    COMMNAD_READ_MODEL_NAME,
    COMMAND_START_FT_DATA_OUTPUT,
    commandSetFilter,
)

from zaber_control import ZaberStage


# ========= 사용자 설정 =========
PORT_RFT   = "COM7"   # Robotus F/T 센서 포트
PORT_ZABER = "COM8"   # Zaber 스테이지 포트

TARGET_FS = 1000.0    # 목표 샘플링 주파수 [Hz]
DT = 1.0 / TARGET_FS

AXIS_FOR_SLOPE = "fz"     # slope 기준 축 (지금은 Fz)
TARGET_SLOPE   = 200.0    # [N/s] 윈도우 기반 dFz/dt 임계값 (그래프 보고 조정)
MIN_TIME_FOR_TRIGGER = 0.05   # 계측 시작 후 너무 초반 스파이크는 무시 [s]

# EMA 필터 시간 상수 (신호 부드럽게 정도)
TAU = 0.03  # [s] 0.02 ~ 0.05 사이에서 조정 추천 (값이 클수록 더 부드러움)

# 윈도우 기반 slope 계산용
WINDOW_FOR_SLOPE = 0.01  # [s] 최근 10ms 동안의 평균 기울기
FZ_MIN_FOR_SLOPE = 0.05  # [N] 이 force 이상일 때만 slope 트리거 체크

# Zaber 관련
ZABER_USE = True              # 실험에서 Zaber 실제로 쓸 때만 True
MICROSTEP_UM = 0.49609375     # [µm/step] (스테이지 스펙에서 가져온 값)
DESIRED_SPEED_MM_S = -5.0     # [mm/s] -방향으로 5 mm/s

# mm/s -> native 속도 (step/s) 변환
ZABER_VEL_NATIVE = int(DESIRED_SPEED_MM_S / (MICROSTEP_UM / 1000.0))
# ==============================


# 플롯에 보일 데이터 포인트 수
MAX_POINTS_PLOT = 5000

# 공유 데이터 버퍼 (플롯용)
time_data = deque(maxlen=MAX_POINTS_PLOT)
fx_data   = deque(maxlen=MAX_POINTS_PLOT)
fy_data   = deque(maxlen=MAX_POINTS_PLOT)
fz_data   = deque(maxlen=MAX_POINTS_PLOT)
tx_data   = deque(maxlen=MAX_POINTS_PLOT)
ty_data   = deque(maxlen=MAX_POINTS_PLOT)
tz_data   = deque(maxlen=MAX_POINTS_PLOT)

fz_filt_data = deque(maxlen=MAX_POINTS_PLOT)  # EMA filtering된 Fz
slope_data   = deque(maxlen=MAX_POINTS_PLOT)  # dFz/dt (윈도우 기반)

# 전체 로그 (CSV 저장용)
log_rows: list[list[float]] = []

# 상태 변수
data_lock = threading.Lock()
running = True
target_reached = False

# 저장 경로
BASE_DIR    = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(BASE_DIR, ".."))
timestamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
SAVE_DIR    = os.path.join(PROJECT_DIR, f"rft_slope_{timestamp}")
os.makedirs(SAVE_DIR, exist_ok=True)

CSV_PATH = os.path.join(SAVE_DIR, f"rft_slope_{timestamp}.csv")
FIG_PATH = os.path.join(SAVE_DIR, f"rft_slope_{timestamp}.png")


def init_robotus() -> RFTseries:
    """Robotus F/T 센서 초기화."""
    print("[Robotus] Initializing...")
    rft = RFTseries(PORT_RFT)

    # 혹시 기존에 데이터 출력 중이면 정지
    rft.sendCommand(COMMAND_STOP_FT_DATA_OUTPUT)
    time.sleep(0.1)
    rft.ser.flush()

    # 모델명 요청 (통신 확인용)
    rft.sendCommand(COMMNAD_READ_MODEL_NAME)
    time.sleep(0.1)

    # 데이터 출력 시작
    rft.sendCommand(COMMAND_START_FT_DATA_OUTPUT)
    time.sleep(0.1)

    # 하드웨어 필터 설정 (필요하면 조절)
    rft.sendCommand(commandSetFilter(1, 1))  # 거의 안 거는 수준 예시
    time.sleep(0.1)

    # 하드 타레
    rft.hardTare()
    print("[Robotus] Init done.")
    return rft


def acquisition_loop(rft: RFTseries, zaber: ZaberStage | None):
    """
    고주파수(TARGET_FS)로 Robotus에서 데이터를 읽고,
    Fz에 EMA 필터 적용 + 윈도우 기반 dFz/dt 계산 + slope 임계값 체크.
    임계값을 넘으면 Zaber를 stop() 하고 루프 종료.
    """
    global running, target_reached

    t0 = time.perf_counter()
    next_t = t0

    fz_filt = None

    # EMA 계수
    alpha = DT / (TAU + DT)

    # 윈도우 기반 slope 계산용 버퍼: (time, fz_filtered)
    slope_window: deque[tuple[float, float]] = deque()

    print(f"[Acq] Started acquisition loop at target {TARGET_FS} Hz (alpha={alpha:.4f})")

    while running:
        now = time.perf_counter()
        t_rel = now - t0

        # 1) 센서 값 읽기
        fx, fy, fz, tx, ty, tz = rft.getTareFT()

        # 2) EMA 필터 (Fz 기준)
        if fz_filt is None:
            fz_filt = fz
        else:
            fz_filt = alpha * fz + (1.0 - alpha) * fz_filt

        # 2-1) 윈도우 버퍼 업데이트
        slope_window.append((t_rel, fz_filt))
        # WINDOW_FOR_SLOPE 보다 오래된 샘플 제거
        while slope_window and (t_rel - slope_window[0][0]) > WINDOW_FOR_SLOPE:
            slope_window.popleft()

        # 3) slope 계산 (윈도우 처음 값 기준 평균 기울기)
        if len(slope_window) >= 2:
            t_old, f_old = slope_window[0]
            dt_local = t_rel - t_old
            if dt_local > 0:
                slope = (fz_filt - f_old) / dt_local
            else:
                slope = 0.0
        else:
            slope = 0.0

        # 4) 공유 버퍼/로그에 저장
        with data_lock:
            time_data.append(t_rel)
            fx_data.append(fx)
            fy_data.append(fy)
            fz_data.append(fz)
            tx_data.append(tx)
            ty_data.append(ty)
            tz_data.append(tz)

            fz_filt_data.append(fz_filt)
            slope_data.append(slope)

            log_rows.append([
                t_rel, fx, fy, fz, tx, ty, tz, fz_filt, slope
            ])

        # 5) 목표 slope 도달 체크 (양의 기울기만)
        if (
            t_rel >= MIN_TIME_FOR_TRIGGER
            and abs(fz_filt) >= FZ_MIN_FOR_SLOPE
            and slope        >= TARGET_SLOPE
        ):
            print(f"[Acq] TARGET SLOPE REACHED: slope={slope:.2f} at t={t_rel:.4f}s (Fz_filt={fz_filt:.4f})")
            target_reached = True
            running = False

            # 여기서 바로 Zaber 정지 (추가 안전)
            if zaber is not None:
                try:
                    zaber.stop()
                    print("[Acq] Zaber stop() called due to slope trigger.")
                except Exception as e:
                    print(f"[Acq] Zaber stop() failed: {e}")
            break

        # 6) 타이밍 맞추기 (고주파수 루프)
        next_t += DT
        sleep_time = next_t - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_t = time.perf_counter()

    print("[Acq] Acquisition loop ended.")


def save_and_cleanup(
    rft: RFTseries | None,
    zaber: ZaberStage | None,
    reason: str = "",
    zaber_start_pos: int | None = None,
):
    """
    센서 정지, Zaber 정지 및 시작 위치 복귀, CSV/그림 저장, 리소스 반환.
    정지 시점의 현재 위치와 시작 위치를 모두 로그로 남긴다.
    """
    global running
    running = False

    print(f"[Cleanup] reason = {reason}")

    # Zaber 안전 정지 및 시작 위치 복귀
    if zaber is not None:
        # 0) 정지 시점 current pos 읽어보기 (디버깅용)
        current_pos = None
        try:
            current_pos = zaber.get_position()
            # mm 단위로도 로그
            step_mm = MICROSTEP_UM / 1000.0
            cur_mm = current_pos * step_mm
            print(f"[Cleanup] Zaber current pos (native) = {current_pos} (~ {cur_mm:.3f} mm)")
        except Exception as e:
            print(f"[WARN] Zaber get_position in cleanup failed: {e}")

        # 1) 속도 모드 정지
        try:
            zaber.stop()
        except Exception as e:
            print(f"[WARN] Zaber stop in cleanup failed: {e}")

        # 2) 시작 위치 정보가 있으면 그 위치로 복귀
        if zaber_start_pos is not None:
            try:
                step_mm = MICROSTEP_UM / 1000.0
                start_mm = zaber_start_pos * step_mm
                print(
                    f"[Cleanup] Returning Zaber to start pos "
                    f"(native={zaber_start_pos}, ~{start_mm:.3f} mm)"
                )
                zaber.move_abs_native(zaber_start_pos)
            except Exception as e:
                print(f"[WARN] Zaber move_abs_native failed: {e}")
        else:
            print("[Cleanup] zaber_start_pos is None → 복귀 명령 생략.")

        # 3) 시리얼 포트 닫기
        try:
            zaber.close()
        except Exception as e:
            print(f"[WARN] Zaber close failed: {e}")

    # Robotus 정지 및 close
    if rft is not None:
        try:
            rft.sendCommand(COMMAND_STOP_FT_DATA_OUTPUT)
        except Exception as e:
            print(f"[WARN] stop cmd failed: {e}")
        try:
            rft.close()
        except Exception as e:
            print(f"[WARN] serial close failed: {e}")

    # CSV 저장
    try:
        headers = [
            "time_s", "fx", "fy", "fz", "tx", "ty", "tz",
            "fz_filtered", "slope_dFz_dt",
        ]
        with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(headers)
            w.writerows(log_rows)
        print(f"[Cleanup] CSV saved: {CSV_PATH}")
    except Exception as e:
        print(f"[ERROR] CSV save failed: {e}")

    # 그림 저장 (플롯이 열려 있다면)
    try:
        plt.savefig(FIG_PATH, dpi=150)
        print(f"[Cleanup] Figure saved: {FIG_PATH}")
    except Exception as e:
        print(f"[WARN] Figure save failed: {e}")


def main():
    global running, target_reached

    # Zaber 초기화 및 시작 위치 읽기
    zaber = None
    zaber_start_pos: int | None = None

    if ZABER_USE:
        try:
            zaber = ZaberStage(PORT_ZABER)
            try:
                zaber_start_pos = zaber.get_position()
                step_mm = MICROSTEP_UM / 1000.0
                start_mm = zaber_start_pos * step_mm
                print(
                    f"[Main] Zaber start position (native) = {zaber_start_pos} "
                    f"(~ {start_mm:.3f} mm)"
                )
            except Exception as e:
                print(f"[Zaber] get_position failed: {e}")
                zaber_start_pos = None
        except Exception as e:
            print(f"[Zaber] Init failed: {e}")
            zaber = None

    # Robotus 초기화
    rft = init_robotus()

    # Zaber를 일정 속도로 움직이기 (velocity 모드)
    if zaber is not None and ZABER_USE:
        try:
            zaber.move_velocity(ZABER_VEL_NATIVE)
            print(
                f"[Main] Zaber.move_velocity({ZABER_VEL_NATIVE}) called "
                f"(≈ {DESIRED_SPEED_MM_S:.2f} mm/s)."
            )
        except Exception as e:
            print(f"[Zaber] move_velocity failed: {e}")

    # 수집 스레드 시작
    acq_thread = threading.Thread(
        target=acquisition_loop,
        args=(rft, zaber),
        daemon=True,
    )
    acq_thread.start()

    # ==== 플롯 설정 (Fz raw, filtered, slope) ====
    plt.style.use("seaborn-v0_8-darkgrid")
    fig, (ax_f, ax_s) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    ax_f.set_title("Fz (raw & filtered)")
    ax_f.set_ylabel("Fz [N]")

    ax_s.set_title("dFz/dt (slope)")
    ax_s.set_xlabel("Time [s]")
    ax_s.set_ylabel("Slope [N/s]")

    line_fz_raw,  = ax_f.plot([], [], label="Fz raw")
    line_fz_filt, = ax_f.plot([], [], label="Fz filtered")
    line_slope,   = ax_s.plot([], [], label="dFz/dt")

    ax_f.legend(loc="upper left")
    ax_s.legend(loc="upper left")

    def update_plot(frame):
        with data_lock:
            t = list(time_data)
            fz_raw = list(fz_data)
            fz_filt = list(fz_filt_data)
            slope = list(slope_data)

        if len(t) == 0:
            return line_fz_raw, line_fz_filt, line_slope

        line_fz_raw.set_data(t, fz_raw)
        line_fz_filt.set_data(t, fz_filt)
        line_slope.set_data(t, slope)

        ax_f.relim()
        ax_f.autoscale_view()
        ax_s.relim()
        ax_s.autoscale_view()

        return line_fz_raw, line_fz_filt, line_slope

    # FuncAnimation에 cache_frame_data=False를 줘서 warning 제거
    ani = FuncAnimation(
        fig,
        update_plot,
        interval=50,           # 20 Hz 정도로 플롯 업데이트
        cache_frame_data=False,
    )

    try:
        # 창이 살아 있고, 아직 target_reached가 아닐 때만 반복
        while plt.fignum_exists(fig.number) and not target_reached:
            plt.pause(0.05)   # GUI 이벤트 처리 + 50ms 대기

        if target_reached:
            print("[Main] Target slope reached → auto cleanup.")
            save_and_cleanup(
                rft,
                zaber,
                reason="Target slope reached.",
                zaber_start_pos=zaber_start_pos,
            )
        else:
            print("[Main] Window closed by user → cleanup.")
            save_and_cleanup(
                rft,
                zaber,
                reason="Figure closed by user.",
                zaber_start_pos=zaber_start_pos,
            )

    except KeyboardInterrupt:
        save_and_cleanup(
            rft,
            zaber,
            reason="KeyboardInterrupt (Ctrl+C).",
            zaber_start_pos=zaber_start_pos,
        )

    finally:
        if plt.fignum_exists(fig.number):
            plt.close(fig)
        print("[Main] Done.")


if __name__ == "__main__":
    main()

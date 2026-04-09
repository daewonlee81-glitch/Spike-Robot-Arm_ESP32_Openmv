#!/usr/bin/env pybricks-micropython
"""
EV3 파이브릭스 태양광 추적 로봇
- 컬러센서 1,2,3 (Port S1~S3): 주변광(ambient) 모드
- 모터B (왼쪽 바퀴) + 모터C (오른쪽 바퀴): 제자리 회전

동작 방식:
1. 제자리에서 360도 천천히 회전하며 가장 밝은 방향 탐색
2. 그 방향으로 되돌아 이동
3. 센서 L/R 비교로 미세 조정 반복 (지속 추적)
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

# ── 하드웨어 초기화 ──────────────────────────────────────────────
ev3 = EV3Brick()
motor_l = Motor(Port.B)           # 왼쪽 바퀴
motor_r = Motor(Port.C)           # 오른쪽 바퀴

sensor_l = ColorSensor(Port.S1)   # 왼쪽 센서
sensor_m = ColorSensor(Port.S2)   # 중앙 센서
sensor_r = ColorSensor(Port.S3)   # 오른쪽 센서

# ── 파라미터 ─────────────────────────────────────────────────────
SCAN_SPEED      = 120   # 스캔 회전 속도 (deg/s, 각 바퀴)
TRACK_SPEED     = 80    # 미세 추적 속도 (deg/s)
SCAN_STEP_WAIT  = 80    # 스캔 샘플링 간격 (ms)
TRACK_INTERVAL  = 150   # 추적 루프 간격 (ms)
LIGHT_THRESHOLD = 3     # 좌우 차이 허용치 (이하면 정지)


# ── 유틸리티 ─────────────────────────────────────────────────────
def read_ambient():
    """(좌, 중, 우) 주변광 반환"""
    return sensor_l.ambient(), sensor_m.ambient(), sensor_r.ambient()


def total_light():
    l, m, r = read_ambient()
    return l + m + r


def spin_left(speed):
    """제자리 왼쪽 회전 (모터B 후진, 모터C 전진)"""
    motor_l.run(-speed)
    motor_r.run(speed)


def spin_right(speed):
    """제자리 오른쪽 회전 (모터B 전진, 모터C 후진)"""
    motor_l.run(speed)
    motor_r.run(-speed)


def stop_motors():
    motor_l.hold()
    motor_r.hold()


# ── 1단계: 360도 전체 스캔 ───────────────────────────────────────
def full_scan():
    """
    오른쪽으로 천천히 한 바퀴 돌며 가장 밝은 '시간 t'를 기록.
    반환: 스캔에 걸린 전체 시간 중 최적 시점까지의 비율(0~1),
          최대 밝기
    """
    ev3.screen.clear()
    ev3.screen.print("360도 스캔 중...")

    # 한 바퀴 돌 때 걸리는 총 시간(ms) 측정용 — 먼저 짧게 보정
    # 실제로는 샘플 인덱스와 총 인덱스로 비율 계산
    samples = []

    spin_right(SCAN_SPEED)

    # 샘플 수: 360도 / (SCAN_SPEED * SCAN_STEP_WAIT/1000) ≈ steps
    # 한 바퀴 예상 시간 기반으로 샘플링
    # 간단하게 time-based: 일정 횟수 샘플 후 360도 완료로 간주
    # EV3 제자리 회전 1바퀴 = 바퀴간 거리에 따라 다름 (약 3~4초 @ 120 deg/s)
    # 여기서는 motor_l 각도를 기준으로 사용 (reset 후 360도 이상 회전 시 종료)

    motor_l.reset_angle(0)

    # 오른쪽 회전 시 motor_l은 정방향 → 360도(한 바퀴) 이상
    # 로봇 한 바퀴 = 모터 회전각 (로봇 지름 / 바퀴 지름 * 360)
    # EV3 기본: 로봇 폭 약 12cm, EV3 바퀴 지름 5.6cm → 약 770도
    # 실제 조립에 따라 다르므로 시간 기반(4000ms) 방식 사용
    SCAN_DURATION = 4000  # ms — 한 바퀴 예상 시간 (조립에 맞게 조정)
    elapsed = 0

    best_idx   = 0
    best_light = -1
    idx        = 0

    while elapsed < SCAN_DURATION:
        brightness = total_light()
        samples.append(brightness)
        if brightness > best_light:
            best_light = brightness
            best_idx   = idx
        idx     += 1
        elapsed += SCAN_STEP_WAIT
        wait(SCAN_STEP_WAIT)

    stop_motors()
    total_samples = len(samples)

    ev3.screen.print("최고: {}lx ({}%)".format(
        best_light, best_idx * 100 // total_samples))

    return best_idx, total_samples, best_light


# ── 2단계: 최적 방향으로 역회전 ─────────────────────────────────
def return_to_best(best_idx, total_samples):
    """
    스캔 후 현재 위치에서 best_idx 지점으로 되돌아가기.
    스캔을 오른쪽으로 했으므로, 남은 구간만큼 계속 오른쪽 or
    지나쳐 왔으면 왼쪽으로 되감기.
    전략: 360도 중 best 지점의 비율 → 남은 각도를 역방향(왼쪽)으로 회전.
    """
    ev3.screen.print("최적 방향으로 이동...")

    # 스캔 완료 = 360도. 이미 지나온 비율
    passed_ratio  = best_idx / total_samples          # 0~1
    remain_ratio  = 1.0 - passed_ratio                # 돌아가야 할 비율
    return_time   = int(remain_ratio * 4000)          # ms (SCAN_DURATION 기준)

    # remain이 0.5보다 작으면 왼쪽(역방향), 아니면 계속 오른쪽이 더 빠름
    if remain_ratio <= 0.5:
        spin_left(SCAN_SPEED)
        wait(return_time)
    else:
        spin_right(SCAN_SPEED)
        wait(int(passed_ratio * 4000))

    stop_motors()
    wait(300)


# ── 3단계: 미세 추적 루프 ────────────────────────────────────────
def fine_tracking():
    """
    센서L(S1)과 센서R(S3)을 비교해 더 밝은 쪽으로 조금씩 회전.
    중앙(S2)이 가장 밝고 좌우 차이가 작으면 정지.
    """
    ev3.screen.clear()
    ev3.screen.print("미세 추적 중...")

    while True:
        l, m, r = read_ambient()
        diff = l - r  # 양수: 왼쪽 밝음 → 왼쪽 회전 / 음수: 오른쪽 밝음 → 오른쪽 회전

        if abs(diff) <= LIGHT_THRESHOLD:
            stop_motors()
        elif diff > 0:
            spin_left(TRACK_SPEED)
        else:
            spin_right(TRACK_SPEED)

        ev3.screen.clear()
        ev3.screen.print("L:{:2d} M:{:2d} R:{:2d}".format(l, m, r))
        ev3.screen.print("차이: {:+d}".format(diff))

        wait(TRACK_INTERVAL)


# ── 메인 ────────────────────────────────────────────────────────
def main():
    ev3.speaker.beep(frequency=600, duration=300)
    ev3.screen.clear()
    ev3.screen.print("태양광 추적 시작")
    wait(500)

    # 1. 360도 스캔
    best_idx, total_samples, best_light = full_scan()

    # 빛이 너무 약하면 대기
    if best_light < 5:
        ev3.screen.print("빛 부족 — 대기")
        ev3.speaker.beep(frequency=200, duration=800)
        wait(5000)
        return

    # 2. 최적 방향으로 복귀
    return_to_best(best_idx, total_samples)

    ev3.speaker.beep(frequency=900, duration=150)
    wait(100)
    ev3.speaker.beep(frequency=1100, duration=150)

    # 3. 미세 추적 (무한 루프)
    fine_tracking()


main()

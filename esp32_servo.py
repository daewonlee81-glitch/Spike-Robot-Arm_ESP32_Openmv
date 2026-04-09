"""
ESP32 서보 제어 코드 (PUPRemote Sensor)
핀: servo21=Pin21, servo19=Pin19, servo22=Pin22, gripper=Pin20
통신: PUPRemote (LEGO 허브와 연결)
"""

from machine import Pin, PWM
import time
from pupremote import PUPRemoteSensor

# ── 서보 설정 ─────────────────────────────────────────
servo21 = PWM(Pin(21), freq=50)
servo19 = PWM(Pin(19), freq=50)
servo22 = PWM(Pin(22), freq=50)
servo20 = PWM(Pin(20), freq=50)   # 그리퍼

# ── 유틸 ─────────────────────────────────────────────
def clamp(v, vmin, vmax):
    if v < vmin: return vmin
    if v > vmax: return vmax
    return v

def set_angle(servo, angle):
    angle = clamp(angle, -90, 90)
    duty = int((angle + 90) * 102 / 180 + 26)
    servo.duty(duty)

# ── 상태 변수 ─────────────────────────────────────────
cur21 = 0.0
cur19 = 0.0
cur22 = 0.0
cur20 = 0.0   # 그리퍼

target21 = 0.0
target19 = 0.0
target22 = 0.0
target20 = 0.0

acc21 = 0.0
acc19 = 0.0
acc22 = 0.0
acc20 = 0.0

speed21 = 30.0
speed19 = 30.0
speed22 = 30.0
speed20 = 30.0

TICK_MS = 10

# ── 누산기 기반 이동 ──────────────────────────────────
def step_toward_acc(current, target, speed_dps, acc):
    if current == target:
        return current, 0.0
    delta = speed_dps * (TICK_MS / 1000.0)
    acc += delta
    move = int(acc)
    acc -= move
    if current < target:
        current = min(current + move, target)
    else:
        current = max(current - move, target)
    return current, acc

# ── 그리퍼 약전류 유지 설정 ──────────────────────────
# 목표 도달 후 500ms마다 한 번만 PWM 신호 → 평균 전류 대폭 감소
GRIPPER_HOLD_MS  = 500
gripper_hold_tmr = 0
gripper_prev_tgt = 0.0

def update_servos():
    global cur21, cur19, cur22, cur20
    global acc21, acc19, acc22, acc20
    global gripper_hold_tmr, gripper_prev_tgt

    cur21, acc21 = step_toward_acc(cur21, target21, speed21, acc21)
    cur19, acc19 = step_toward_acc(cur19, target19, speed19, acc19)
    cur22, acc22 = step_toward_acc(cur22, target22, speed22, acc22)
    cur20, acc20 = step_toward_acc(cur20, target20, speed20, acc20)

    set_angle(servo21, int(cur21))
    set_angle(servo19, int(cur19))
    set_angle(servo22, int(cur22))

    now = time.ticks_ms()

    # 목표가 바뀌면 즉시 정상 PWM으로 이동
    if target20 != gripper_prev_tgt:
        gripper_prev_tgt = target20
        gripper_hold_tmr = now
        set_angle(servo20, int(cur20))
    elif int(cur20) != int(target20):
        # 이동 중 → 정상 PWM
        set_angle(servo20, int(cur20))
        gripper_hold_tmr = now
    elif time.ticks_diff(now, gripper_hold_tmr) >= GRIPPER_HOLD_MS:
        # 목표 도달 후 500ms마다 한 번 신호 (약전류 유지)
        set_angle(servo20, int(cur20))
        gripper_hold_tmr = now
    # else: 대기 중 → PWM 신호 없음 (전류 차단)

# ── 목표 설정 ─────────────────────────────────────────
def set_target(a21, a19, a22, a20, speed):
    global target21, target19, target22, target20
    global speed21, speed19, speed22, speed20
    global acc21, acc19, acc22, acc20

    target21 = float(clamp(a21, -10,  15))  # 범위 축소 (과부하 방지)
    target19 = float(clamp(a19,   0,  45))
    target22 = float(clamp(a22, -20,  30))
    target20 = float(clamp(a20, -15,  15))  # 그리퍼 범위

    speed = clamp(speed, 1, 100)
    dps = speed * 5.0
    speed21 = dps
    speed19 = dps
    speed22 = dps
    speed20 = dps

    acc21 = 0.0
    acc19 = 0.0
    acc22 = 0.0
    acc20 = 0.0

# ── PUPRemote 명령 함수 ───────────────────────────────
# a21, a19, a22, a20(그리퍼), speed
def speedmove(a21, a19, a22, a20, speed):
    set_target(a21, a19, a22, a20, speed)
    return 1

def getpos():
    return int(cur21), int(cur19), int(cur22), int(cur20)

# ── 초기 위치 정렬 ────────────────────────────────────
for i in range(30, -1, -1):
    set_angle(servo21, i)
    set_angle(servo19, -i)
    set_angle(servo22, i)
    time.sleep(0.01)

set_angle(servo20, 0)
time.sleep(0.2)

cur21 = 0.0
cur19 = 0.0
cur22 = 0.0
cur20 = 0.0

set_target(-15, 35, 30, 0, 3)

# ── PUPRemote 설정 ────────────────────────────────────
p = PUPRemoteSensor(max_packet_size=16)
p.add_command('speedmove', to_hub_fmt='b',  from_hub_fmt='5b')  # a21,a19,a22,a20,speed
p.add_command('getpos',    to_hub_fmt='4b', from_hub_fmt='')    # cur21,cur19,cur22,cur20

# ── 메인 루프 ─────────────────────────────────────────
import sys
import select

last_update = time.ticks_ms()
last_print  = time.ticks_ms()
PRINT_MS    = 100   # PC로 관절 위치 전송 주기 (ms)

serial_buf  = ""

while True:
    p.process()

    now = time.ticks_ms()

    if time.ticks_diff(now, last_update) >= TICK_MS:
        update_servos()
        last_update = now

    # USB Serial → PC로 현재 관절 위치 전송
    if time.ticks_diff(now, last_print) >= PRINT_MS:
        print(str(int(cur21)) + "," +
              str(int(cur19)) + "," +
              str(int(cur22)) + "," +
              str(int(cur20)))
        last_print = now

    # PC → USB Serial 명령 수신 (학습 모드)
    # 포맷: "move,a21,a19,a22,a20,speed\n"
    if select.select([sys.stdin], [], [], 0)[0]:
        ch = sys.stdin.read(1)
        if ch:
            serial_buf += ch
            if '\n' in serial_buf:
                line       = serial_buf.strip()
                serial_buf = ""
                parts      = line.split(',')
                if parts[0] == 'move' and len(parts) == 6:
                    set_target(int(parts[1]), int(parts[2]),
                               int(parts[3]), int(parts[4]),
                               int(parts[5]))

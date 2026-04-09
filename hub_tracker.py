"""
SPIKE Prime 허브 - 물체 추적 + 그리퍼 자동 집기
- Port E : OpenMV H7 Plus (blob 수신)
- Port C : ESP32 로봇팔 (PUPRemote)
- Port F : 턴테이블 (수평 추적)

상태 머신:
  TRACK   → 물체 추적 중
  GRAB    → 목표 위치 도달 → 그리퍼 닫기
  HOLD    → 집은 상태 유지
  RELEASE → 그리퍼 열기 → TRACK 복귀
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Direction, Stop
from pybricks.tools import wait, StopWatch
from pupremote_hub import PUPRemoteHub

hub       = PrimeHub()
turntable = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)

# ── OpenMV 연결 ───────────────────────────────────────
cam = PUPRemoteHub(Port.E, max_packet_size=16)
cam.add_command('blob', to_hub_fmt='3h', from_hub_fmt='')

# ── ESP32 로봇팔 연결 ─────────────────────────────────
arm = PUPRemoteHub(Port.C, max_packet_size=16)
arm.add_command('speedmove', to_hub_fmt='b',  from_hub_fmt='5b')
arm.add_command('getpos',    to_hub_fmt='4b', from_hub_fmt='')

# ── 이미지 중심 ───────────────────────────────────────
IMG_CX = 160   # 320 / 2
IMG_CY = 120   # 240 / 2

# ── 비례 제어 게인 ────────────────────────────────────
KP_X    = 0.3
KP_A21  = 0.03
KP_A19  = 0.04
KP_A22  = 0.02
DEAD_X  = 15
DEAD_Y  = 10

# ── 그리퍼 집기 조건 ──────────────────────────────────
GRAB_DEAD_X  = 40      # 수평 오차 허용 범위 (px)
GRAB_DEAD_Y  = 100     # 수직 오차 허용 범위 (px) - 팔 범위 한계로 완화
GRAB_SIZE    = 500     # 최소 물체 크기
GRAB_COUNT   = 5       # 연속으로 N회 조건 충족 시 집기 시작

GRIPPER_OPEN  =  15    # 그리퍼 열림 각도
GRIPPER_CLOSE = -15    # 그리퍼 닫힘 각도
HOLD_MS       = 2000   # 집은 상태 유지 시간 (ms)
RELEASE_MS    = 1000   # 그리퍼 열기 후 대기 시간 (ms)

# ── 물체 정지 감지 ────────────────────────────────────
STILL_FRAMES  = 10     # 정지 판정에 필요한 연속 프레임 수
STILL_THRESH  = 8      # 이 픽셀 이하로 움직이면 정지로 판정

# ── 관절 범위 ─────────────────────────────────────────
A21_MIN, A21_MAX = -10, 15
A19_MIN, A19_MAX =   0, 45
A22_MIN, A22_MAX = -20, 30

# ── 학습된 최적 속도 테이블 ───────────────────────────
LEARNED_SPEED = {
    (-15, 10, -15): 20, (-15, 10,  0):  5, (-15, 10, 15): 20, (-15, 10, 30): 50,
    (-15, 20,-30): 20,  (-15, 20,-15): 20, (-15, 20,  0): 20, (-15, 20, 15): 50, (-15, 20, 30): 20,
    (-15, 30,-30):  5,  (-15, 30,-15): 10, (-15, 30,  0): 20, (-15, 30, 15): 20, (-15, 30, 30):  5,
    (  0, 10,-30): 20,  (  0, 10,-15):  5, (  0, 10,  0):  5, (  0, 10, 15): 20, (  0, 10, 30):  5,
    (  0, 20,-30):  5,  (  0, 20,-15):  5, (  0, 20,  0): 10, (  0, 20, 15): 10, (  0, 20, 30): 30,
    (  0, 30,-30):  5,  (  0, 30,-15): 20, (  0, 30,  0): 50, (  0, 30, 15): 30, (  0, 30, 30): 50,
    ( 15, 10,-30):  5,  ( 15, 10,-15): 10, ( 15, 10,  0):  5, ( 15, 10, 15):  5, ( 15, 10, 30):  5,
    ( 15, 20,-30): 10,  ( 15, 20,-15): 20, ( 15, 20,  0): 20, ( 15, 20, 15): 50, ( 15, 20, 30):  5,
    ( 15, 30,-30): 50,  ( 15, 30,-15):  5, ( 15, 30,  0): 50, ( 15, 30, 15): 10, ( 15, 30, 30): 30,
    ( 30, 10,-30): 30,  ( 30, 10,-15): 10, ( 30, 10,  0): 30, ( 30, 10, 15): 10, ( 30, 10, 30): 50,
    ( 30, 20,-30): 50,  ( 30, 20,-15):  5, ( 30, 20,  0): 10, ( 30, 20, 15):  5, ( 30, 20, 30):  5,
    ( 30, 30,-30): 30,  ( 30, 30,-15): 10, ( 30, 30,  0): 50, ( 30, 30, 15): 10, ( 30, 30, 30): 20,
}
A21_STEPS = [-15, 0, 15, 30]
A19_STEPS = [10, 20, 30]
A22_STEPS = [-30, -15, 0, 15, 30]

def nearest_step(val, steps):
    return min(steps, key=lambda s: abs(s - val))

def lookup_speed(a21, a19, a22):
    k = (nearest_step(a21, A21_STEPS),
         nearest_step(a19, A19_STEPS),
         nearest_step(a22, A22_STEPS))
    return LEARNED_SPEED.get(k, 10)

# ── 현재 관절 목표값 ──────────────────────────────────
cur_a21 = -10
cur_a19 =  25
cur_a22 =  10

# ── 로봇팔 명령 ───────────────────────────────────────
def arm_set(a21, a19, a22, a20=0, speed=None):
    if speed is None:
        speed = lookup_speed(a21, a19, a22)
    arm.call('speedmove', a21, a19, a22, a20, speed)

# ── 상태 머신 ─────────────────────────────────────────
STATE_TRACK   = 0
STATE_GRAB    = 1
STATE_HOLD    = 2
STATE_RELEASE = 3

state       = STATE_TRACK
grab_count  = 0
timer       = StopWatch()
cur_gripper = GRIPPER_OPEN

# ── 물체 정지 감지용 히스토리 ─────────────────────────
prev_cx     = 0
prev_cy     = 0
still_count = 0

# ── 초기 자세 ─────────────────────────────────────────
print("초기 자세 설정...")
arm_set(cur_a21, cur_a19, cur_a22, GRIPPER_OPEN, speed=10)
turntable.reset_angle(0)
wait(2000)
print("추적 시작!")

# ── 메인 루프 ─────────────────────────────────────────
while True:
    data = cam.call('blob')

    # ── 상태: GRAB (그리퍼 닫기) ──────────────────────
    if state == STATE_GRAB:
        turntable.hold()
        arm_set(int(cur_a21), int(cur_a19), int(cur_a22), GRIPPER_CLOSE, speed=5)
        cur_gripper = GRIPPER_CLOSE
        print("집기 시도!")
        hub.light.on(Color.YELLOW)
        timer.reset()
        state = STATE_HOLD
        wait(50)
        continue

    # ── 상태: HOLD (집은 상태 유지) ───────────────────
    if state == STATE_HOLD:
        if timer.time() >= HOLD_MS:
            state = STATE_RELEASE
            timer.reset()
        wait(50)
        continue

    # ── 상태: RELEASE (그리퍼 열기) ───────────────────
    if state == STATE_RELEASE:
        arm_set(int(cur_a21), int(cur_a19), int(cur_a22), GRIPPER_OPEN, speed=5)
        cur_gripper = GRIPPER_OPEN
        print("그리퍼 열기 → 추적 복귀")
        hub.light.on(Color.GREEN)
        if timer.time() >= RELEASE_MS:
            grab_count = 0
            state = STATE_TRACK
        wait(50)
        continue

    # ── 상태: TRACK (추적) ────────────────────────────
    if data is None or (data[0] == -1):
        turntable.hold()
        grab_count = 0
        hub.light.on(Color.RED)
        wait(50)
        continue

    cx, cy, size = data
    hub.light.on(Color.BLUE)

    ex = cx - IMG_CX
    ey = cy - IMG_CY

    # 수평 추적 (턴테이블)
    if abs(ex) > DEAD_X:
        spd = int(ex * KP_X * 100)
        spd = max(-500, min(500, spd))
        turntable.run(spd)
    else:
        turntable.hold()

    # 수직 추적 (3관절)
    if abs(ey) > DEAD_Y:
        cur_a21 = cur_a21 + ey * KP_A21
        cur_a19 = cur_a19 + ey * KP_A19
        cur_a22 = cur_a22 - ey * KP_A22
        cur_a21 = max(A21_MIN, min(A21_MAX, cur_a21))
        cur_a19 = max(A19_MIN, min(A19_MAX, cur_a19))
        cur_a22 = max(A22_MIN, min(A22_MAX, cur_a22))
        arm_set(int(cur_a21), int(cur_a19), int(cur_a22), GRIPPER_OPEN, speed=10)

    # ── 물체 정지 감지 ────────────────────────────────
    moved = abs(cx - prev_cx) + abs(cy - prev_cy)
    if moved <= STILL_THRESH:
        still_count += 1
    else:
        still_count = 0
    prev_cx = cx
    prev_cy = cy

    is_still      = still_count >= STILL_FRAMES
    is_centered   = abs(ex) <= GRAB_DEAD_X and abs(ey) <= GRAB_DEAD_Y
    is_close      = size >= GRAB_SIZE

    # 디버그 출력
    print("cx=" + str(cx) + " cy=" + str(cy)
          + " sz=" + str(size)
          + " still=" + str(still_count)
          + " cnt=" + str(grab_count))

    # ── 집기 조건: 중앙 정렬 + 충분한 크기 + 정지 ────
    if is_centered and is_close and is_still:
        grab_count += 1
    else:
        grab_count = 0

    if grab_count >= GRAB_COUNT:
        state = STATE_GRAB

    wait(50)

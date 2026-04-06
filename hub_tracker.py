"""
SPIKE Prime 허브 - 빨강 물체 추적 + 로봇팔 제어
- Port E : OpenMV H7 Plus (blob 수신)
- Port C : ESP32 로봇팔 (PUPRemote)
- Port F : 턴테이블 (수평 추적)

추적 구조:
  수평(cx) → 턴테이블 좌우 회전
  수직(cy) → 로봇팔 a19 (상하 이동)
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Direction, Stop
from pybricks.tools import wait
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
KP_X   = 0.3   # 수평 오차 → 턴테이블 속도
KP_Y   = 0.04  # 수직 오차 → a19 각도 변화량
DEAD_X = 15    # 수평 데드존 (px) — 이 안에서는 턴테이블 정지
DEAD_Y = 10    # 수직 데드존 (px)

# ── 팔 관절 범위 ──────────────────────────────────────
A21_HOME =  -15
A19_MIN  =   10
A19_MAX  =   35
A22_HOME =   30
A20_HOME =    0

# ── 현재 팔 목표값 ────────────────────────────────────
cur_a19 = 25   # 수직 추적에 사용할 관절 초기값

# ── 로봇팔 즉시 명령 (도달 대기 없음) ────────────────
def arm_set(a21, a19, a22, a20=0, speed=20):
    arm.call('speedmove', a21, a19, a22, a20, speed)

# ── 초기 자세 ─────────────────────────────────────────
print("초기 자세 설정...")
arm_set(A21_HOME, cur_a19, A22_HOME, A20_HOME, speed=10)
turntable.reset_angle(0)
wait(2000)
print("추적 시작!")

# ── 추적 메인 루프 ────────────────────────────────────
while True:
    data = cam.call('blob')

    if data is None:
        turntable.hold()
        wait(50)
        continue

    cx, cy, size = data

    # 물체 없음 (-1, -1)
    if cx == -1:
        turntable.hold()
        wait(50)
        continue

    # ── 수평 추적 (턴테이블) ──────────────────────────
    ex = cx - IMG_CX   # 양수: 오른쪽, 음수: 왼쪽

    if abs(ex) > DEAD_X:
        spd = int(ex * KP_X * 100)          # 턴테이블 속도
        spd = max(-500, min(500, spd))       # 속도 제한
        turntable.run(spd)
    else:
        turntable.hold()

    # ── 수직 추적 (a19 관절) ──────────────────────────
    ey = cy - IMG_CY   # 양수: 아래, 음수: 위

    if abs(ey) > DEAD_Y:
        delta = ey * KP_Y
        cur_a19 = cur_a19 + delta
        cur_a19 = max(A19_MIN, min(A19_MAX, cur_a19))
        arm_set(A21_HOME, int(cur_a19), A22_HOME, A20_HOME, speed=10)

    wait(50)

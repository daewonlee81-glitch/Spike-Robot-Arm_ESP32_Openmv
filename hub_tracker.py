"""
SPIKE Prime 허브 - 빨강 물체 추적 + 로봇팔 제어
- Port E : OpenMV H7 Plus (blob 수신)
- Port C : ESP32 로봇팔 (PUPRemote)
- Port F : 턴테이블 (수평 추적)

추적 구조:
  수평(cx) → 턴테이블 좌우 회전
  수직(cy) → a21(하단) + a19(중단) + a22(상단) 연동 이동
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
KP_X    = 0.3   # 수평 오차 → 턴테이블 속도
KP_A21  = 0.03  # 수직 오차 → a21 변화량 (하단 관절)
KP_A19  = 0.04  # 수직 오차 → a19 변화량 (중단 관절)
KP_A22  = 0.02  # 수직 오차 → a22 변화량 (상단 관절)
DEAD_X  = 15    # 수평 데드존 (px)
DEAD_Y  = 10    # 수직 데드존 (px)

# ── 관절 범위 ─────────────────────────────────────────
A21_MIN, A21_MAX = -15, 30
A19_MIN, A19_MAX =  10, 35
A22_MIN, A22_MAX = -30, 30
A20_HOME = 0

# ── 현재 관절 목표값 (초기 자세) ─────────────────────
cur_a21 = -15
cur_a19 =  25
cur_a22 =  10

# ── 로봇팔 즉시 명령 (도달 대기 없음) ────────────────
def arm_set(a21, a19, a22, a20=0, speed=10):
    arm.call('speedmove', a21, a19, a22, a20, speed)

# ── 초기 자세 ─────────────────────────────────────────
print("초기 자세 설정...")
arm_set(cur_a21, cur_a19, cur_a22, A20_HOME, speed=10)
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

    # 물체 없음
    if cx == -1:
        turntable.hold()
        wait(50)
        continue

    # ── 수평 추적 (턴테이블) ──────────────────────────
    ex = cx - IMG_CX   # 양수: 오른쪽, 음수: 왼쪽

    if abs(ex) > DEAD_X:
        spd = int(ex * KP_X * 100)
        spd = max(-500, min(500, spd))
        turntable.run(spd)
    else:
        turntable.hold()

    # ── 수직 추적 (3관절 연동) ────────────────────────
    ey = cy - IMG_CY   # 양수: 아래, 음수: 위

    if abs(ey) > DEAD_Y:
        # 아래 방향(ey > 0): 팔 내리기
        # 위 방향(ey < 0): 팔 올리기
        cur_a21 = cur_a21 + ey * KP_A21
        cur_a19 = cur_a19 + ey * KP_A19
        cur_a22 = cur_a22 - ey * KP_A22   # a22는 반대 방향으로 보정

        cur_a21 = max(A21_MIN, min(A21_MAX, cur_a21))
        cur_a19 = max(A19_MIN, min(A19_MAX, cur_a19))
        cur_a22 = max(A22_MIN, min(A22_MAX, cur_a22))

        arm_set(int(cur_a21), int(cur_a19), int(cur_a22), A20_HOME, speed=10)

    wait(50)

"""
SPIKE Prime 턴테이블 중심 정렬
- Motor F      : 턴테이블 Large Motor
- ColorSensor D: 파랑(출발) → 초록(도착) 감지
- 시계반대방향으로 파랑 탐색 → 시계방향으로 초록 탐색 → 중심 이동
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Direction, Stop
from pybricks.tools import wait

hub       = PrimeHub()
turntable = Motor(Port.F, positive_direction=Direction.CLOCKWISE)
sensor    = ColorSensor(Port.D)

SEARCH_SPEED = 150  # 파랑 탐색 속도 (deg/s)
SCAN_SPEED   = 120  # 초록 탐색 속도 (deg/s)
MOVE_SPEED   = 400  # 중심 이동 속도 (deg/s)

# ── STEP 1: 시계반대방향 회전 → 파랑 탐색 ───────────
print("파랑 마커 탐색 중 (시계반대방향)...")
turntable.run(-SEARCH_SPEED)

while True:
    if sensor.color() == Color.BLUE:
        turntable.hold()
        break
    wait(10)

turntable.reset_angle(0)
print("파랑 확인 → 0° 초기화")
wait(200)

# ── STEP 2: 시계방향 회전 → 초록 탐색 ───────────────
print("초록 마커 탐색 중 (시계방향)...")
turntable.run(SCAN_SPEED)

while True:
    angle = turntable.angle()

    if angle > 30 and sensor.color() == Color.GREEN:
        turntable.hold()
        green_angle = turntable.angle()
        print("초록 감지 →", green_angle, "°")
        break

    wait(10)

# ── STEP 3: 중심 = 엔코더 절반 ───────────────────────
center_angle = green_angle / 2
print("중심 위치  →", center_angle, "°")

# ── STEP 4: 중심으로 이동 ────────────────────────────
print("중심으로 이동 중...")
turntable.run_target(MOVE_SPEED, center_angle, then=Stop.HOLD)
print("완료!")

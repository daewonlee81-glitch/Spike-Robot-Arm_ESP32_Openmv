"""
SPIKE Prime 허브 통합 제어
- Motor F      : 턴테이블 (Port F)
- ColorSensor D: 파랑/초록 마커 감지 (Port D)
- Port C       : ESP32 로봇팔 (PUPRemote)

관절 정보:
  a21 = 1번 관절 (가장 아래, -15 ~ 30)
  a19 = 2번 관절 (10 ~ 35)
  a22 = 3번 관절 (-30 ~ 30)
  a20 = 그리퍼   (-15 ~ 15)

순서:
  1. 시계반대방향 → 파랑 탐색 → 0° 초기화 → 팔 최초 위치
  2. 시계방향 → 초록 탐색 → 각도 기록
  3. 중심(절반) 이동
  4. 1초 대기
  5. 파랑(0°) 이동 → 팔 내리기 → 그리퍼 동작 → 복귀
  6. 중심 이동
  7. 1초 대기
  8. 초록 이동 → 팔 내리기 → 그리퍼 동작 → 복귀
  9. 중심 이동
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Direction, Stop
from pybricks.tools import wait
from pupremote_hub import PUPRemoteHub

hub       = PrimeHub()
turntable = Motor(Port.F, positive_direction=Direction.CLOCKWISE)
sensor    = ColorSensor(Port.D)

arm = PUPRemoteHub(Port.C, max_packet_size=16)
arm.add_command('speedmove', to_hub_fmt='b',  from_hub_fmt='5b')  # a21,a19,a22,a20,speed
arm.add_command('getpos',    to_hub_fmt='4b', from_hub_fmt='')    # cur21,cur19,cur22,cur20

SPEED = 900  # 턴테이블 속도 (deg/s)

# ── 로봇팔 이동 + 도달 대기 ──────────────────────────
def move_and_wait(a21, a19, a22, a20=0, speed=30, tolerance=2, timeout=6000):
    arm.call('speedmove', a21, a19, a22, a20, speed)
    elapsed = 0
    while elapsed < timeout:
        pos = arm.call('getpos')
        if pos:
            c21, c19, c22, c20 = pos
            if (abs(c21 - a21) <= tolerance and
                abs(c19 - a19) <= tolerance and
                abs(c22 - a22) <= tolerance and
                abs(c20 - a20) <= tolerance):
                return True
        wait(50)
        elapsed += 50
    print("팔 이동 timeout")
    return False

# ── 그리퍼만 동작 (팔 위치 유지) ─────────────────────
def grip(a21, a19, a22, a20, speed=10):
    return move_and_wait(a21, a19, a22, a20=a20, speed=speed)

# ── 픽업 동작 시퀀스 (파랑/초록 공통) ────────────────
def pickup_sequence():
    # 1. 기본 자세 (그리퍼 열기)
    reached = move_and_wait(-15, 35, 30, a20=15, speed=10)
    print("  기본 자세(그리퍼 열기):", reached)

    # 2. 팔 내리기 (그리퍼 유지)
    reached = move_and_wait(30, 10, -30, a20=15, speed=10)
    print("  팔 내림:", reached)

    # 3. 그리퍼 닫기 (팔 위치 유지)
    reached = grip(30, 10, -30, a20=-15, speed=10)
    print("  그리퍼 닫기:", reached)
    wait(300)

    # 4. 팔 복귀 (그리퍼 닫힌 채로)
    reached = move_and_wait(-15, 35, 30, a20=-15, speed=10)
    print("  복귀:", reached)

    # 5. 그리퍼 0°로 초기화
    reached = grip(-15, 35, 30, a20=0, speed=10)
    print("  그리퍼 초기화:", reached)

    print("  현재 위치:", arm.call('getpos'))

# ── STEP 1: 시계반대방향 → 파랑 탐색 ────────────────
print("파랑 탐색 중...")
turntable.run(-SPEED)

while True:
    if sensor.color() == Color.BLUE:
        turntable.hold()
        break
    wait(10)

turntable.reset_angle(0)
print("파랑 확인 → 0° 초기화")

print("로봇팔 최초 위치 이동...")
reached = move_and_wait(-15, 35, 30, a20=0, speed=10)
print("최초 위치 도달:", reached)
wait(200)

# ── STEP 2: 시계방향 → 초록 탐색 ────────────────────
print("초록 탐색 중...")
turntable.run(SPEED)

while True:
    angle = turntable.angle()
    if angle > 30 and sensor.color() == Color.GREEN:
        turntable.hold()
        green_angle = turntable.angle()
        print("초록 감지 →", green_angle, "°")
        break
    wait(10)

# ── STEP 3: 중심 이동 ────────────────────────────────
center_angle = green_angle / 2
print("중심 이동 →", center_angle, "°")
turntable.run_target(SPEED, center_angle, then=Stop.HOLD)
print("중심 도달")

# ── STEP 4: 1초 대기 ─────────────────────────────────
print("1초 대기...")
wait(1000)

# ── STEP 5: 파랑(0°) 이동 → 팔 동작 ─────────────────
print("파랑 위치 이동...")
turntable.run_target(SPEED, 0, then=Stop.HOLD)
print("파랑 도달 → 로봇팔 동작")
pickup_sequence()

# ── STEP 6: 중심 이동 ────────────────────────────────
print("중심 이동...")
turntable.run_target(SPEED, center_angle, then=Stop.HOLD)
print("중심 도달")

# ── STEP 7: 1초 대기 ─────────────────────────────────
print("1초 대기...")
wait(1000)

# ── STEP 8: 초록 이동 → 팔 동작 ─────────────────────
print("초록 위치 이동...")
turntable.run_target(SPEED, green_angle, then=Stop.HOLD)
print("초록 도달 → 로봇팔 동작")
pickup_sequence()

# ── STEP 9: 중심 이동 ────────────────────────────────
print("중심 이동...")
turntable.run_target(SPEED, center_angle, then=Stop.HOLD)
print("중심 도달")

print("모든 동작 완료!")

"""
기어비 자동 측정
- 파랑 마커를 컬러센서로 두 번 감지 → 1회전 모터 각도 계산
- 측정 후 GEAR_RATIO 값을 turntable_center.py에 입력
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Stop
from pybricks.tools import wait

hub = PrimeHub()
turntable = Motor(Port.F)
sensor    = ColorSensor(Port.D)

SCAN_SPEED = 60   # 천천히 돌려야 감지 정확

turntable.reset_angle(0)

first_angle  = None
second_angle = None
last_detected = -50  # 시작 직후 감지 방지

print("기어비 측정 시작 - 파랑 마커를 두 번 감지합니다...")

turntable.run(SCAN_SPEED)

while True:
    color = sensor.color()
    angle = turntable.angle()

    if color == Color.BLUE and abs(angle - last_detected) > 50:
        last_detected = angle

        if first_angle is None:
            first_angle = angle
            print("1번째 파랑 감지:", first_angle, "도")

        elif second_angle is None:
            second_angle = angle
            print("2번째 파랑 감지:", second_angle, "도")
            break

    # 3회전 넘으면 실패
    if angle > 360 * 15:
        print("오류: 파랑 마커를 두 번 찾지 못했습니다.")
        turntable.hold()
        raise SystemExit

    wait(10)

turntable.hold()

# ── 기어비 계산 ───────────────────────────────────────
one_revolution = second_angle - first_angle
gear_ratio = one_revolution / 360

print()
print("1회전 모터 각도:", one_revolution, "도")
print("GEAR_RATIO =", round(gear_ratio, 2))
print()
print("turntable_center.py 의 GEAR_RATIO 를 위 값으로 수정하세요.")

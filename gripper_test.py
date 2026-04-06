"""
ESP32 그리퍼 테스트
- GPIO 20 : 그리퍼 서보
- 0° → 15° → 1초 대기 → 0° → -15°
"""

from machine import Pin, PWM
import time

# ── 그리퍼 서보 설정 ──────────────────────────────────
gripper = PWM(Pin(20), freq=50)

# ── 유틸 ─────────────────────────────────────────────
def clamp(v, vmin, vmax):
    if v < vmin: return vmin
    if v > vmax: return vmax
    return v

def set_angle(servo, angle):
    angle = clamp(angle, -90, 90)
    duty = int((angle + 90) * 102 / 180 + 26)
    servo.duty(duty)

# ── 동작 ─────────────────────────────────────────────
print("0° 초기화")
set_angle(gripper, 0)
time.sleep(1)

print("15°로 이동")
set_angle(gripper, 15)
time.sleep(1)

print("1초 대기...")
time.sleep(1)

print("0°로 복귀")
set_angle(gripper, 0)
time.sleep(0.5)

print("-15°로 이동")
set_angle(gripper, -15)
time.sleep(1)

print("완료")

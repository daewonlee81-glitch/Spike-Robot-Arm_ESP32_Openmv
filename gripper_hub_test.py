"""
Pybricks 그리퍼 작동 확인 코드
- 팔은 최초 위치 유지
- 그리퍼: 0° → 15° → 0° → -15° 순서로 테스트
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Port
from pybricks.tools import wait
from pupremote_hub import PUPRemoteHub

hub = PrimeHub()

arm = PUPRemoteHub(Port.C, max_packet_size=16)
arm.add_command('speedmove', to_hub_fmt='b',  from_hub_fmt='5b')
arm.add_command('getpos',    to_hub_fmt='4b', from_hub_fmt='')

# ── 도달 대기 함수 ────────────────────────────────────
def move_and_wait(a21, a19, a22, a20, speed=30, tolerance=2, timeout=6000):
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
    print("timeout")
    return False

# ── 팔 최초 위치 고정값 ───────────────────────────────
ARM = (-15, 35, 30)  # a21, a19, a22 고정

# ── 그리퍼 테스트 ─────────────────────────────────────
print("그리퍼 0° 초기화")
move_and_wait(*ARM, 0, speed=10)
print("현재 위치:", arm.call('getpos'))
wait(1000)

print("그리퍼 → 15°")
move_and_wait(*ARM, 15, speed=10)
print("현재 위치:", arm.call('getpos'))
wait(1000)

print("그리퍼 → 0°")
move_and_wait(*ARM, 0, speed=10)
print("현재 위치:", arm.call('getpos'))
wait(1000)

print("그리퍼 → -15°")
move_and_wait(*ARM, -15, speed=10)
print("현재 위치:", arm.call('getpos'))
wait(1000)

print("완료!")

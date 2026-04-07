"""
SPIKE Prime 허브 - 로봇팔 3관절 움직임 학습 및 재생
- Port C : ESP32 로봇팔 (GPIO 21, 19, 22)
- Port E : OpenMV H7 Plus 마커 추적 (위치 기록용)

[ 학습 모드 ] 허브 불빛: 노랑
  LEFT  버튼 : 선택 관절 각도 감소 (-5°)
  RIGHT 버튼 : 선택 관절 각도 증가 (+5°)
  CENTER 짧게: 현재 자세 저장
  CENTER 길게: 다음 관절로 전환 (21 → 19 → 22 → 21)

  LEFT + RIGHT 동시: 학습 종료 → 재생 모드

[ 재생 모드 ] 허브 불빛: 초록
  CENTER 버튼: 저장된 동작 순서대로 재생
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pupremote_hub import PUPRemoteHub

hub       = PrimeHub()
turntable = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)

# ── ESP32 로봇팔 연결 ─────────────────────────────────
arm = PUPRemoteHub(Port.C, max_packet_size=16)
arm.add_command('speedmove', to_hub_fmt='b',  from_hub_fmt='5b')
arm.add_command('getpos',    to_hub_fmt='4b', from_hub_fmt='')

# ── OpenMV 마커 연결 ──────────────────────────────────
cam = PUPRemoteHub(Port.E, max_packet_size=16)
cam.add_command('marker', to_hub_fmt='3h', from_hub_fmt='')

# ── 관절 범위 ─────────────────────────────────────────
JOINTS = {
    'a21': {'min': -15, 'max': 30,  'color': Color.RED},
    'a19': {'min':  10, 'max': 35,  'color': Color.YELLOW},
    'a22': {'min': -30, 'max': 30,  'color': Color.BLUE},
}
JOINT_KEYS = ['a21', 'a19', 'a22']
STEP = 5   # 버튼 한 번에 이동 각도

# ── 현재 관절 각도 ────────────────────────────────────
cur = {'a21': -15, 'a19': 25, 'a22': 10}

# ── 저장 시퀀스 ───────────────────────────────────────
sequence   = []   # [ {'a21':x, 'a19':y, 'a22':z, 'cx':cx, 'cy':cy}, ... ]

# ── 유틸 ─────────────────────────────────────────────
def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

def send_arm(speed=20):
    arm.call('speedmove',
             cur['a21'], cur['a19'], cur['a22'], 0, speed)

def move_and_wait(a21, a19, a22, speed=15, tolerance=2, timeout=5000):
    arm.call('speedmove', a21, a19, a22, 0, speed)
    elapsed = 0
    while elapsed < timeout:
        pos = arm.call('getpos')
        if pos:
            c21, c19, c22, _ = pos
            if (abs(c21 - a21) <= tolerance and
                abs(c19 - a19) <= tolerance and
                abs(c22 - a22) <= tolerance):
                return True
        wait(50)
        elapsed += 50
    return False

# ── 초기 자세 ─────────────────────────────────────────
print("초기 자세로 이동...")
move_and_wait(cur['a21'], cur['a19'], cur['a22'], speed=10)
wait(500)

# ═══════════════════════════════════════════════════
# 학습 모드
# ═══════════════════════════════════════════════════
joint_idx    = 0   # 현재 선택된 관절 인덱스
selected     = JOINT_KEYS[joint_idx]
hub.light.on(JOINTS[selected]['color'])

print("=== 학습 모드 ===")
print(f"선택 관절: {selected}")
print("LEFT/RIGHT: 각도 조절 | CENTER 짧게: 저장 | CENTER 길게: 관절 전환")
print("LEFT+RIGHT 동시: 학습 종료")

sw = StopWatch()

while True:
    pressed = hub.buttons.pressed()

    # ── LEFT + RIGHT 동시: 학습 종료 ─────────────────
    if Button.LEFT in pressed and Button.RIGHT in pressed:
        hub.speaker.beep(frequency=1000, duration=300)
        print(f"학습 완료 — {len(sequence)}개 자세 저장")
        while hub.buttons.pressed():
            wait(20)
        break

    # ── LEFT: 각도 감소 ───────────────────────────────
    if Button.LEFT in pressed:
        cur[selected] = clamp(
            cur[selected] - STEP,
            JOINTS[selected]['min'],
            JOINTS[selected]['max']
        )
        send_arm()
        print(f"  {selected}: {cur[selected]}°")
        hub.speaker.beep(frequency=500, duration=50)
        while Button.LEFT in hub.buttons.pressed():
            wait(20)

    # ── RIGHT: 각도 증가 ──────────────────────────────
    if Button.RIGHT in pressed:
        cur[selected] = clamp(
            cur[selected] + STEP,
            JOINTS[selected]['min'],
            JOINTS[selected]['max']
        )
        send_arm()
        print(f"  {selected}: {cur[selected]}°")
        hub.speaker.beep(frequency=700, duration=50)
        while Button.RIGHT in hub.buttons.pressed():
            wait(20)

    # ── CENTER: 저장(짧게) / 관절 전환(길게) ──────────
    if Button.CENTER in pressed:
        sw.reset()
        while Button.CENTER in hub.buttons.pressed():
            wait(20)
        duration = sw.time()

        if duration < 800:
            # 짧게: 현재 자세 저장
            marker_data = cam.call('marker')
            cx, cy = (-1, -1)
            if marker_data and marker_data[0] != -1:
                cx, cy = marker_data[0], marker_data[1]

            pose = {
                'a21': cur['a21'],
                'a19': cur['a19'],
                'a22': cur['a22'],
                'cx' : cx,
                'cy' : cy
            }
            sequence.append(pose)
            print(f"[{len(sequence)}] 저장 | a21={cur['a21']} a19={cur['a19']} a22={cur['a22']} | 마커=({cx},{cy})")
            hub.speaker.beep(frequency=900, duration=100)

        else:
            # 길게: 다음 관절로 전환
            joint_idx = (joint_idx + 1) % len(JOINT_KEYS)
            selected  = JOINT_KEYS[joint_idx]
            hub.light.on(JOINTS[selected]['color'])
            print(f"선택 관절 전환 → {selected}")
            hub.speaker.beep(frequency=600, duration=200)

    wait(30)

# ═══════════════════════════════════════════════════
# 재생 모드
# ═══════════════════════════════════════════════════
hub.light.on(Color.GREEN)
print()
print("=== 재생 모드 ===")
print("CENTER 버튼: 재생")

if len(sequence) == 0:
    print("저장된 자세 없음")
else:
    while True:
        if Button.CENTER in hub.buttons.pressed():
            hub.light.on(Color.BLUE)
            print("재생 중...")

            for i, pose in enumerate(sequence):
                reached = move_and_wait(
                    pose['a21'], pose['a19'], pose['a22'], speed=15
                )
                print(f"  [{i+1}/{len(sequence)}] "
                      f"a21={pose['a21']} a19={pose['a19']} a22={pose['a22']}"
                      f" → {'도달' if reached else 'timeout'}")
                wait(300)

            hub.light.on(Color.GREEN)
            print("재생 완료")
            hub.speaker.beep(frequency=1200, duration=300)

            while Button.CENTER in hub.buttons.pressed():
                wait(20)

        wait(50)

"""
SPIKE Prime 허브 - 로봇팔 움직임 학습 및 재생
- Port C : ESP32 로봇팔 (PUPRemote)
- Port E : OpenMV H7 Plus 마커 추적

[ 학습 모드 ]
  왼쪽 버튼  : 현재 자세 저장
  오른쪽 버튼 : 학습 종료 → 재생 모드로 전환

[ 재생 모드 ]
  가운데 버튼 : 저장된 동작 순서대로 재생
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait
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

# ── 도달 대기 ─────────────────────────────────────────
def move_and_wait(a21, a19, a22, a20=0, speed=20, tolerance=2, timeout=5000):
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
    return False

# ── 현재 자세 읽기 ─────────────────────────────────────
def read_pose():
    pos = arm.call('getpos')
    if pos:
        return list(pos)   # [a21, a19, a22, a20]
    return None

# ── 저장된 시퀀스 ─────────────────────────────────────
sequence   = []   # [ [a21, a19, a22, a20], ... ]
marker_log = []   # [ [cx, cy], ... ] — OpenMV 마커 좌표 기록

# ═══════════════════════════════════════════════════
# 학습 모드
# ═══════════════════════════════════════════════════
hub.light.on(Color.YELLOW)
print("=== 학습 모드 시작 ===")
print("왼쪽 버튼  : 현재 자세 저장")
print("오른쪽 버튼: 학습 종료")

while True:
    pressed = hub.buttons.pressed()

    # ── 왼쪽 버튼: 현재 자세 저장 ────────────────────
    if Button.LEFT in pressed:
        pose = read_pose()
        marker_data = cam.call('marker')

        if pose:
            sequence.append(pose)

            cx, cy = (-1, -1)
            if marker_data:
                cx, cy = marker_data[0], marker_data[1]
            marker_log.append([cx, cy])

            print(f"[{len(sequence)}] 저장 | 관절: {pose} | 마커: ({cx},{cy})")
            hub.speaker.beep(frequency=800, duration=100)
        else:
            print("자세 읽기 실패")
            hub.speaker.beep(frequency=300, duration=200)

        # 버튼 뗄 때까지 대기
        while Button.LEFT in hub.buttons.pressed():
            wait(20)

    # ── 오른쪽 버튼: 학습 종료 ───────────────────────
    if Button.RIGHT in pressed:
        hub.speaker.beep(frequency=1000, duration=200)
        print(f"학습 완료 — 총 {len(sequence)}개 자세 저장")
        break

    wait(50)

# ═══════════════════════════════════════════════════
# 재생 모드
# ═══════════════════════════════════════════════════
hub.light.on(Color.GREEN)
print()
print("=== 재생 모드 ===")
print("가운데 버튼: 재생 시작")

if len(sequence) == 0:
    print("저장된 자세 없음")
else:
    while True:
        pressed = hub.buttons.pressed()

        # ── 가운데 버튼: 재생 ─────────────────────────
        if Button.CENTER in pressed:
            hub.light.on(Color.BLUE)
            print("재생 중...")

            for i, pose in enumerate(sequence):
                a21, a19, a22, a20 = pose
                reached = move_and_wait(a21, a19, a22, a20, speed=15)
                print(f"  [{i+1}/{len(sequence)}] {pose} → {'도달' if reached else 'timeout'}")
                wait(300)

            hub.light.on(Color.GREEN)
            print("재생 완료")
            hub.speaker.beep(frequency=1200, duration=300)

            # 버튼 뗄 때까지 대기
            while Button.CENTER in hub.buttons.pressed():
                wait(20)

        wait(50)

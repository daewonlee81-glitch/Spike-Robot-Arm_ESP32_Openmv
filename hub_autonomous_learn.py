"""
SPIKE Prime - 로봇팔 자율 학습 + 추론
- Port C : ESP32 로봇팔 (GPIO 21, 19, 22)
- Port E : OpenMV 마커 추적 (arm marker)

[ Phase 1: 자율 탐색 ]
  관절 조합을 순차적으로 이동하면서
  OpenMV 마커(cx, cy)와 관절 각도를 쌍으로 기록

[ Phase 2: 추론 ]
  목표 (cx, cy)가 주어지면
  데이터셋에서 가장 가까운 관절 각도를 찾아 이동
  CENTER 버튼: 추론 모드 전환 / 재실행
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Button, Color
from pybricks.tools import wait
from pupremote_hub import PUPRemoteHub

hub       = PrimeHub()
turntable = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)

arm = PUPRemoteHub(Port.C, max_packet_size=16)
arm.add_command('speedmove', to_hub_fmt='b',  from_hub_fmt='5b')
arm.add_command('getpos',    to_hub_fmt='4b', from_hub_fmt='')

cam = PUPRemoteHub(Port.E, max_packet_size=16)
cam.add_command('marker', to_hub_fmt='3h', from_hub_fmt='')

# ── 탐색 범위 설정 ────────────────────────────────────
# step을 크게 할수록 빠르고 데이터 적음
# step을 작게 할수록 느리고 정밀함
A21_RANGE = range(-15, 31, 15)   # -15, 0, 15, 30       (4개)
A19_RANGE = range(10,  36, 10)   # 10, 20, 30, (35 근사) (3개)
A22_RANGE = range(-30, 31, 15)   # -30,-15, 0, 15, 30   (5개)
# 총 조합: 4 × 3 × 5 = 60개 → 약 60초 소요

SETTLE_MS  = 800   # 이동 후 안정화 대기 (ms)
MOVE_SPEED = 15    # 탐색 이동 속도

# ── 데이터셋 ─────────────────────────────────────────
# [(a21, a19, a22, cx, cy), ...]
dataset = []

# ── 유틸 ─────────────────────────────────────────────
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

def get_marker():
    data = cam.call('marker')
    if data and data[0] != -1:
        return data[0], data[1]
    return None, None

# ── 가장 가까운 관절 찾기 (최근접 이웃) ──────────────
def find_nearest(target_cx, target_cy):
    best_joints = None
    best_dist   = float('inf')
    for entry in dataset:
        a21, a19, a22, cx, cy = entry
        dist = (cx - target_cx) ** 2 + (cy - target_cy) ** 2
        if dist < best_dist:
            best_dist   = dist
            best_joints = (a21, a19, a22)
    return best_joints, best_dist

# ═══════════════════════════════════════════════════
# Phase 1: 자율 탐색 및 데이터 수집
# ═══════════════════════════════════════════════════
hub.light.on(Color.YELLOW)
print("=== Phase 1: 자율 탐색 시작 ===")

total = len(list(A21_RANGE)) * len(list(A19_RANGE)) * len(list(A22_RANGE))
count = 0

for a21 in A21_RANGE:
    for a19 in A19_RANGE:
        for a22 in A22_RANGE:
            count += 1

            # 관절 이동
            reached = move_and_wait(a21, a19, a22, speed=MOVE_SPEED)
            wait(SETTLE_MS)

            # 마커 위치 읽기
            cx, cy = get_marker()

            if cx is not None:
                dataset.append((a21, a19, a22, cx, cy))
                print(f"[{count}/{total}] a21={a21:4} a19={a19:4} a22={a22:4}"
                      f" → marker({cx},{cy})")
                hub.speaker.beep(frequency=600, duration=30)
            else:
                print(f"[{count}/{total}] a21={a21:4} a19={a19:4} a22={a22:4}"
                      f" → 마커 없음 (스킵)")

print(f"\n탐색 완료: {len(dataset)}/{total}개 데이터 수집")

if len(dataset) == 0:
    print("데이터 없음 — 마커 및 카메라 확인 필요")
    raise SystemExit

# ═══════════════════════════════════════════════════
# Phase 2: 추론 모드
# ═══════════════════════════════════════════════════
hub.light.on(Color.GREEN)
hub.speaker.beep(frequency=1000, duration=300)
print("\n=== Phase 2: 추론 모드 ===")
print("CENTER 버튼: 현재 마커 위치 기준으로 최적 자세 이동")

while True:
    if Button.CENTER in hub.buttons.pressed():
        hub.light.on(Color.BLUE)

        # 현재 마커 위치 읽기
        cx, cy = get_marker()

        if cx is None:
            print("마커 감지 안됨")
            hub.speaker.beep(frequency=300, duration=200)
        else:
            print(f"현재 마커 위치: ({cx}, {cy})")

            # 최근접 관절 찾기
            joints, dist = find_nearest(cx, cy)
            a21, a19, a22 = joints
            print(f"최적 자세: a21={a21} a19={a19} a22={a22} (거리={dist:.1f})")

            # 이동
            reached = move_and_wait(a21, a19, a22, speed=15)
            print(f"이동 완료: {'도달' if reached else 'timeout'}")
            hub.speaker.beep(frequency=900, duration=100)

        hub.light.on(Color.GREEN)

        while Button.CENTER in hub.buttons.pressed():
            wait(20)

    wait(50)

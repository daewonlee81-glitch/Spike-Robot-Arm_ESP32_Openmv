"""
SPIKE Prime - 로봇팔 자율 학습 + 추론
- Port C : ESP32 로봇팔 (GPIO 21, 19, 22)
- Port E : OpenMV 노랑 구간 추적

[ Phase 1: 자율 탐색 ]
  관절 조합을 순차적으로 이동하면서
  OpenMV 마커(cx, cy)와 관절 각도를 쌍으로 기록

[ Phase 2: 연속 추론 ]
  마커 위치를 실시간으로 읽어
  데이터셋에서 가장 가까운 관절 각도를 찾아 자동 이동
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Color
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
A21_RANGE = range(-15, 31, 15)   # -15, 0, 15, 30     (4개)
A19_RANGE = range(10,  36, 10)   # 10, 20, 30          (3개)
A22_RANGE = range(-30, 31, 15)   # -30,-15, 0, 15, 30 (5개)
# 총 조합: 4 × 3 × 5 = 60개

SETTLE_MS    = 800   # 이동 후 안정화 대기 (ms)
MOVE_SPEED   = 15    # 탐색 속도
INFER_SPEED  = 20    # 추론 이동 속도
INFER_GAP    = 15    # 추론 최소 이동 거리 (px) — QQVGA(160x120) 기준

# ── 데이터셋 ─────────────────────────────────────────
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
            move_and_wait(a21, a19, a22, speed=MOVE_SPEED)
            wait(SETTLE_MS)

            cx, cy = get_marker()
            if cx is not None:
                dataset.append((a21, a19, a22, cx, cy))
                print("[" + str(count) + "/" + str(total) + "]"
                      + " a21=" + str(a21)
                      + " a19=" + str(a19)
                      + " a22=" + str(a22)
                      + " -> marker(" + str(cx) + "," + str(cy) + ")")
                hub.speaker.beep(frequency=600, duration=30)
            else:
                print("[" + str(count) + "/" + str(total) + "]"
                      + " a21=" + str(a21)
                      + " a19=" + str(a19)
                      + " a22=" + str(a22)
                      + " -> marker 없음 (skip)")

print("탐색 완료: " + str(len(dataset)) + "/" + str(total) + "개 수집")

if len(dataset) == 0:
    print("데이터 없음 — OpenMV 및 노랑 임계값 확인 필요")
    raise SystemExit

# ═══════════════════════════════════════════════════
# Phase 2: 연속 추론
# ═══════════════════════════════════════════════════
hub.light.on(Color.GREEN)
hub.speaker.beep(frequency=1000, duration=300)
print("\n=== Phase 2: 연속 추론 시작 ===")

prev_cx, prev_cy = -1, -1

while True:
    cx, cy = get_marker()

    if cx is None:
        hub.light.on(Color.RED)
        wait(100)
        continue

    hub.light.on(Color.GREEN)

    # 이전 위치에서 INFER_GAP 이상 변화가 있을 때만 이동
    moved = ((cx - prev_cx) ** 2 + (cy - prev_cy) ** 2) ** 0.5
    if moved < INFER_GAP:
        wait(50)
        continue

    joints, dist = find_nearest(cx, cy)
    a21, a19, a22 = joints
    print("marker(" + str(cx) + "," + str(cy) + ")"
          + " -> a21=" + str(a21)
          + " a19=" + str(a19)
          + " a22=" + str(a22)
          + " dist=" + str(int(dist)))

    move_and_wait(a21, a19, a22, speed=INFER_SPEED)
    prev_cx, prev_cy = cx, cy

    wait(50)

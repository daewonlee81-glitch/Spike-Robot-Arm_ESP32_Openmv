"""
PC 학습 스크립트

연결:
  OpenMV USB Serial → PC        : 노랑 끝단 (cx, cy) 수신
  ESP32  USB Serial ↔ PC        : 관절 위치 수신 + 이동 명령 직접 전송

Hub는 학습 중 사용하지 않음
일반 운용 시에는 Hub → PUPRemote → ESP32 방식 유지

필요 패키지:
  pip install pyserial
"""

import serial
import time
import math
import json

# ── 포트 설정 ─────────────────────────────────────────
OPENMV_PORT = "/dev/cu.usbmodem3965377731332"   # OpenMV Virtual Comm Port
ESP32_PORT  = "/dev/cu.usbserial-110"          # ESP32 USB Serial
# Windows 예시: "COM3", "COM4"
BAUD = 115200

# ── 탐색 범위 ─────────────────────────────────────────
A21_RANGE = range(-15, 31, 15)   # 4개
A19_RANGE = range(10,  36, 10)   # 3개
A22_RANGE = range(-30, 31, 15)   # 5개

SPEED_CANDIDATES = [5, 10, 20, 30, 50]

# ── 연결 ─────────────────────────────────────────────
def connect():
    omv = serial.Serial(OPENMV_PORT, BAUD, timeout=0.1)
    esp = serial.Serial(ESP32_PORT,  BAUD, timeout=0.1)
    time.sleep(1.0)
    print("OpenMV 연결 : " + OPENMV_PORT)
    print("ESP32  연결 : " + ESP32_PORT)
    return omv, esp

# ── OpenMV: 노랑 끝단 위치 읽기 ──────────────────────
def read_marker(omv):
    omv.reset_input_buffer()
    try:
        line = omv.readline().decode(errors='ignore').strip()
        if line:
            parts = line.split(',')
            if len(parts) >= 2:
                cx, cy = int(parts[0]), int(parts[1])
                if cx != -1:
                    return cx, cy
    except:
        pass
    return None, None

# ── ESP32: 현재 관절 위치 읽기 ───────────────────────
def read_joints(esp):
    esp.reset_input_buffer()
    try:
        line = esp.readline().decode(errors='ignore').strip()
        if line:
            parts = line.split(',')
            if len(parts) == 4:
                return int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
    except:
        pass
    return None

# ── ESP32: 이동 명령 직접 전송 ────────────────────────
def send_move(esp, a21, a19, a22, a20=0, speed=20):
    cmd = "move," + str(a21) + "," + str(a19) + "," + \
          str(a22) + "," + str(a20) + "," + str(speed) + "\n"
    esp.write(cmd.encode())

# ── ESP32 도달 대기 ───────────────────────────────────
def wait_arrived(esp, a21, a19, a22, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        joints = read_joints(esp)
        if joints:
            c21, c19, c22, _ = joints
            if abs(c21-a21)<=2 and abs(c19-a19)<=2 and abs(c22-a22)<=2:
                return True
        time.sleep(0.05)
    return False

# ── 궤적 부드러움 계산 ────────────────────────────────
def smoothness(trajectory):
    if len(trajectory) < 3:
        return float('inf')
    velocities = []
    for i in range(1, len(trajectory)):
        dx = trajectory[i][0] - trajectory[i-1][0]
        dy = trajectory[i][1] - trajectory[i-1][1]
        velocities.append(math.sqrt(dx**2 + dy**2))
    mean_v   = sum(velocities) / len(velocities)
    variance = sum((v - mean_v)**2 for v in velocities) / len(velocities)
    return variance

# ── 이동하며 궤적 기록 ────────────────────────────────
def measure_trajectory(omv, esp, a21, a19, a22, speed, duration=2.0):
    send_move(esp, a21, a19, a22, 0, speed)
    trajectory = []
    start      = time.time()
    while time.time() - start < duration:
        cx, cy = read_marker(omv)
        if cx is not None:
            trajectory.append((cx, cy))
        time.sleep(0.02)
    return trajectory

# ═══════════════════════════════════════════════════
# Phase 1: 자율 탐색
# ═══════════════════════════════════════════════════
def phase1_explore(omv, esp):
    print("=== Phase 1: 자율 탐색 ===")
    dataset = []
    poses   = [(a21, a19, a22)
               for a21 in A21_RANGE
               for a19 in A19_RANGE
               for a22 in A22_RANGE]
    total = len(poses)

    for i, (a21, a19, a22) in enumerate(poses):
        send_move(esp, a21, a19, a22, 0, 20)
        arrived = wait_arrived(esp, a21, a19, a22)
        time.sleep(0.3)

        cx, cy  = read_marker(omv)
        status  = "arrived" if arrived else "timeout"

        if cx is not None:
            dataset.append({'a21': a21, 'a19': a19, 'a22': a22,
                             'cx': cx,  'cy': cy})
            print("[" + str(i+1) + "/" + str(total) + "] "
                  + "a21=" + str(a21)
                  + " a19=" + str(a19)
                  + " a22=" + str(a22)
                  + " (" + status + ")"
                  + " -> marker(" + str(cx) + "," + str(cy) + ")")
        else:
            print("[" + str(i+1) + "/" + str(total) + "] "
                  + "a21=" + str(a21)
                  + " a19=" + str(a19)
                  + " a22=" + str(a22)
                  + " (" + status + ")"
                  + " -> marker 없음 (skip)")

    print("탐색 완료: " + str(len(dataset)) + "/" + str(total) + "개")
    return dataset

# ═══════════════════════════════════════════════════
# Phase 2: 속도 프로파일 학습
# ═══════════════════════════════════════════════════
def phase2_learn_speed(omv, esp, dataset):
    print("\n=== Phase 2: 속도 프로파일 학습 ===")
    learned = {}

    for i in range(len(dataset) - 1):
        src = dataset[i]
        dst = dataset[i+1]
        key = (dst['a21'], dst['a19'], dst['a22'])

        best_speed = SPEED_CANDIDATES[0]
        best_score = float('inf')

        print("  [" + str(i+1) + "] "
              + "a21=" + str(dst['a21'])
              + " a19=" + str(dst['a19'])
              + " a22=" + str(dst['a22']))

        for speed in SPEED_CANDIDATES:
            send_move(esp, src['a21'], src['a19'], src['a22'], 0, 30)
            time.sleep(0.8)

            traj  = measure_trajectory(omv, esp,
                                       dst['a21'], dst['a19'], dst['a22'],
                                       speed, duration=1.5)
            score = smoothness(traj)
            print("    speed=" + str(speed)
                  + " score=" + str(round(score, 2))
                  + " pts=" + str(len(traj)))

            if score < best_score:
                best_score = score
                best_speed = speed

        learned[key] = best_speed
        print("    => 최적 speed: " + str(best_speed))

    return learned

# ═══════════════════════════════════════════════════
# Phase 3: 학습된 속도로 재생
# ═══════════════════════════════════════════════════
def phase3_playback(esp, dataset, learned):
    print("\n=== Phase 3: 학습 결과 재생 ===")
    for entry in dataset:
        key   = (entry['a21'], entry['a19'], entry['a22'])
        speed = learned.get(key, 20)
        print("a21=" + str(entry['a21'])
              + " a19=" + str(entry['a19'])
              + " a22=" + str(entry['a22'])
              + " speed=" + str(speed))
        send_move(esp, entry['a21'], entry['a19'], entry['a22'], 0, speed)
        time.sleep(0.5)

# ── 메인 ─────────────────────────────────────────────
if __name__ == "__main__":
    omv, esp = connect()

    dataset = phase1_explore(omv, esp)

    if dataset:
        learned = phase2_learn_speed(omv, esp, dataset)

        result = {
            'dataset': dataset,
            'learned_speed': {str(k): v for k, v in learned.items()}
        }
        with open("learned_motion.json", "w") as f:
            json.dump(result, f, indent=2)
        print("\n저장 완료: learned_motion.json")

        phase3_playback(esp, dataset, learned)

    omv.close()
    esp.close()

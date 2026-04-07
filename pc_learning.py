"""
PC 학습 스크립트
- OpenMV USB Serial → 노랑 끝단 위치 수신
- ESP32 WiFi Socket → 서보 명령 전송
- 자연스러운 움직임 학습 (속도 프로파일 최적화)

필요 패키지:
  pip install pyserial
"""

import serial
import socket
import time
import math
import json

# ── 연결 설정 ─────────────────────────────────────────
OPENMV_PORT  = "COM3"       # ← OpenMV USB 시리얼 포트 (Mac: /dev/cu.usbmodem...)
OPENMV_BAUD  = 115200
ESP32_IP     = "192.168.x.x"  # ← ESP32 WiFi IP (esp32_learning.py 실행 후 확인)
ESP32_PORT   = 8080

# ── 탐색 범위 ─────────────────────────────────────────
A21_RANGE = range(-15, 31, 15)
A19_RANGE = range(10,  36, 10)
A22_RANGE = range(-30, 31, 15)

SPEED_CANDIDATES = [5, 10, 20, 30, 50]   # 학습할 속도 후보

# ── 연결 ─────────────────────────────────────────────
def connect():
    omv = serial.Serial(OPENMV_PORT, OPENMV_BAUD, timeout=0.1)
    esp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    esp.connect((ESP32_IP, ESP32_PORT))
    esp.settimeout(0.1)
    print("OpenMV 연결:", OPENMV_PORT)
    print("ESP32 연결:", ESP32_IP)
    return omv, esp

# ── OpenMV 한 프레임 읽기 ─────────────────────────────
def read_marker(omv):
    try:
        line = omv.readline().decode().strip()
        if line:
            parts = line.split(',')
            if len(parts) == 3:
                cx, cy, size = int(parts[0]), int(parts[1]), int(parts[2])
                if cx != -1:
                    return cx, cy
    except:
        pass
    return None, None

# ── ESP32 서보 명령 ───────────────────────────────────
def send_cmd(esp, a21, a19, a22, speed):
    cmd = str(a21)+","+str(a19)+","+str(a22)+","+str(speed)+"\n"
    esp.send(cmd.encode())

def get_pos(esp):
    try:
        esp.send(b"pos\n")
        resp = esp.recv(32).decode().strip()
        parts = resp.split(',')
        if len(parts) == 3:
            return int(parts[0]), int(parts[1]), int(parts[2])
    except:
        pass
    return None

# ── 궤적 부드러움 계산 ────────────────────────────────
# 속도 분산이 작을수록 부드러운 움직임
def smoothness(trajectory):
    if len(trajectory) < 3:
        return float('inf')
    velocities = []
    for i in range(1, len(trajectory)):
        dx = trajectory[i][0] - trajectory[i-1][0]
        dy = trajectory[i][1] - trajectory[i-1][1]
        velocities.append(math.sqrt(dx**2 + dy**2))
    mean_v = sum(velocities) / len(velocities)
    variance = sum((v - mean_v)**2 for v in velocities) / len(velocities)
    return variance

# ── 한 동작의 궤적 측정 ───────────────────────────────
def measure_trajectory(omv, esp, a21, a19, a22, speed, duration=2.0):
    send_cmd(esp, a21, a19, a22, speed)
    trajectory = []
    start = time.time()
    while time.time() - start < duration:
        cx, cy = read_marker(omv)
        if cx is not None:
            trajectory.append((cx, cy))
        time.sleep(0.02)
    return trajectory

# ═══════════════════════════════════════════════════
# Phase 1: 전체 자세 탐색 + 궤적 기록
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
        print("["+str(i+1)+"/"+str(total)+"] a21="+str(a21)+" a19="+str(a19)+" a22="+str(a22))
        send_cmd(esp, a21, a19, a22, 20)
        time.sleep(1.0)   # 이동 완료 대기

        cx, cy = read_marker(omv)
        if cx is not None:
            dataset.append({'a21': a21, 'a19': a19, 'a22': a22, 'cx': cx, 'cy': cy})
            print("  -> marker("+str(cx)+","+str(cy)+")")
        else:
            print("  -> marker 없음 (skip)")

    print("탐색 완료: "+str(len(dataset))+"/"+str(total)+"개")
    return dataset

# ═══════════════════════════════════════════════════
# Phase 2: 속도 프로파일 학습
# ═══════════════════════════════════════════════════
def phase2_learn_speed(omv, esp, dataset):
    print("\n=== Phase 2: 속도 프로파일 학습 ===")
    learned = {}   # { (a21,a19,a22): best_speed }

    # 인접한 자세 쌍 사이의 최적 속도 탐색
    for i in range(len(dataset) - 1):
        src = dataset[i]
        dst = dataset[i+1]
        key = (dst['a21'], dst['a19'], dst['a22'])

        best_speed = SPEED_CANDIDATES[0]
        best_score = float('inf')

        for speed in SPEED_CANDIDATES:
            # 출발 자세로 복귀
            send_cmd(esp, src['a21'], src['a19'], src['a22'], 30)
            time.sleep(0.8)

            # 목표 자세로 이동하며 궤적 기록
            traj = measure_trajectory(
                omv, esp,
                dst['a21'], dst['a19'], dst['a22'],
                speed, duration=1.5
            )
            score = smoothness(traj)

            print("  speed="+str(speed)+" score="+str(round(score, 2))+" points="+str(len(traj)))

            if score < best_score:
                best_score = score
                best_speed = speed

        learned[key] = best_speed
        print("  최적 speed: "+str(best_speed)+" (score="+str(round(best_score,2))+")")

    return learned

# ═══════════════════════════════════════════════════
# Phase 3: 학습 결과로 자연스러운 재생
# ═══════════════════════════════════════════════════
def phase3_playback(esp, dataset, learned):
    print("\n=== Phase 3: 학습된 속도로 재생 ===")
    for entry in dataset:
        key   = (entry['a21'], entry['a19'], entry['a22'])
        speed = learned.get(key, 20)
        print("a21="+str(entry['a21'])+" a19="+str(entry['a19'])
              +" a22="+str(entry['a22'])+" speed="+str(speed))
        send_cmd(esp, entry['a21'], entry['a19'], entry['a22'], speed)
        time.sleep(0.5)

# ── 메인 ─────────────────────────────────────────────
if __name__ == "__main__":
    omv, esp = connect()

    dataset = phase1_explore(omv, esp)

    if dataset:
        learned = phase2_learn_speed(omv, esp, dataset)

        # 학습 결과 저장
        result = {
            'dataset': dataset,
            'learned_speed': {str(k): v for k, v in learned.items()}
        }
        with open("learned_motion.json", "w") as f:
            json.dump(result, f, indent=2)
        print("\n학습 결과 저장 완료: learned_motion.json")

        phase3_playback(esp, dataset, learned)

    omv.close()
    esp.close()

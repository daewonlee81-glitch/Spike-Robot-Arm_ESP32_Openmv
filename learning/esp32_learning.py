"""
ESP32 - 학습 모드 WiFi 서버
PC에서 WiFi로 직접 서보 명령 수신
기존 PUPRemote 없이 독립 실행

명령 포맷 (TCP):
  "a21,a19,a22,speed\n"  → 서보 이동
  "pos\n"                → 현재 위치 반환 "a21,a19,a22\n"
"""

from machine import Pin, PWM
import network
import socket
import time

# ── WiFi 설정 ─────────────────────────────────────────
WIFI_SSID = "your_ssid"      # ← 실제 WiFi SSID로 변경
WIFI_PASS = "your_password"  # ← 실제 비밀번호로 변경
PORT      = 8080

# ── 서보 설정 ─────────────────────────────────────────
servo21 = PWM(Pin(21), freq=50)
servo19 = PWM(Pin(19), freq=50)
servo22 = PWM(Pin(22), freq=50)

def clamp(v, vmin, vmax):
    if v < vmin: return vmin
    if v > vmax: return vmax
    return v

def set_angle(servo, angle):
    angle = clamp(angle, -90, 90)
    duty  = int((angle + 90) * 102 / 180 + 26)
    servo.duty(duty)

# ── 상태 변수 ─────────────────────────────────────────
cur21 = 0.0
cur19 = 0.0
cur22 = 0.0

target21 = 0.0
target19 = 0.0
target22 = 0.0

acc21 = 0.0
acc19 = 0.0
acc22 = 0.0

speed21 = 30.0
speed19 = 30.0
speed22 = 30.0

TICK_MS = 10

# ── 누산기 기반 이동 ──────────────────────────────────
def step_toward_acc(current, target, speed_dps, acc):
    if current == target:
        return current, 0.0
    delta = speed_dps * (TICK_MS / 1000.0)
    acc  += delta
    move  = int(acc)
    acc  -= move
    if current < target:
        current = min(current + move, target)
    else:
        current = max(current - move, target)
    return current, acc

def update_servos():
    global cur21, cur19, cur22, acc21, acc19, acc22
    cur21, acc21 = step_toward_acc(cur21, target21, speed21, acc21)
    cur19, acc19 = step_toward_acc(cur19, target19, speed19, acc19)
    cur22, acc22 = step_toward_acc(cur22, target22, speed22, acc22)
    set_angle(servo21, int(cur21))
    set_angle(servo19, int(cur19))
    set_angle(servo22, int(cur22))

def set_target(a21, a19, a22, speed):
    global target21, target19, target22
    global speed21, speed19, speed22, acc21, acc19, acc22
    target21 = float(clamp(a21, -15, 30))
    target19 = float(clamp(a19,  10, 35))
    target22 = float(clamp(a22, -30, 30))
    dps      = clamp(speed, 1, 100) * 5.0
    speed21  = dps
    speed19  = dps
    speed22  = dps
    acc21    = 0.0
    acc19    = 0.0
    acc22    = 0.0

# ── WiFi 연결 ─────────────────────────────────────────
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASS)

print("WiFi 연결 중...")
while not wlan.isconnected():
    time.sleep(0.5)

print("WiFi 연결 완료:", wlan.ifconfig()[0])

# ── TCP 서버 ──────────────────────────────────────────
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('', PORT))
srv.listen(1)
srv.setblocking(False)
print("서버 대기 중 — Port:", PORT)

# ── 메인 루프 ─────────────────────────────────────────
conn        = None
buf         = ""
last_update = time.ticks_ms()

while True:
    # 클라이언트 연결 수락
    if conn is None:
        try:
            conn, addr = srv.accept()
            conn.setblocking(False)
            print("PC 연결:", addr)
        except:
            pass

    # 데이터 수신 및 명령 처리
    if conn:
        try:
            data = conn.recv(64)
            if data:
                buf += data.decode()
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()

                    if line == 'pos':
                        resp = str(int(cur21)) + "," + str(int(cur19)) + "," + str(int(cur22)) + "\n"
                        conn.send(resp.encode())

                    elif ',' in line:
                        parts = line.split(',')
                        if len(parts) == 4:
                            a21   = int(parts[0])
                            a19   = int(parts[1])
                            a22   = int(parts[2])
                            speed = int(parts[3])
                            set_target(a21, a19, a22, speed)
            else:
                conn.close()
                conn = None
                print("PC 연결 해제")
        except:
            pass

    # 서보 업데이트
    now = time.ticks_ms()
    if time.ticks_diff(now, last_update) >= TICK_MS:
        update_servos()
        last_update = now

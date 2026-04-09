"""
OpenMV H7 Plus - 펩시 페트병 데이터셋 수집
SD카드에 QVGA 이미지 자동 저장

사용법:
  1. OpenMV에 SD카드 삽입
  2. 이 파일을 OpenMV IDE에서 실행
  3. LED 색상으로 상태 확인:
     - 파랑 깜빡임 : 준비 중 (2초 대기)
     - 초록         : 촬영 중
     - 빨강         : SD카드 오류
  4. 병을 다양한 각도/거리로 카메라 앞에 위치
  5. 3초마다 자동 촬영 (목표: 50장)
  6. 완료 후 SD카드를 PC에 연결하여 images/ 폴더 확인
"""

import sensor, image, os, pyb

# ── LED 설정 ──────────────────────────────────────────
led_red   = pyb.LED(1)
led_green = pyb.LED(2)
led_blue  = pyb.LED(3)

def led_off():
    led_red.off(); led_green.off(); led_blue.off()

# ── 카메라 설정 ───────────────────────────────────────
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320 x 240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_vflip(True)
sensor.set_hmirror(True)

# ── SD카드 폴더 생성 ──────────────────────────────────
# OpenMV 펌웨어 4.x: SD카드 경로 = /sd/
# 내부 플래시 경로 = /flash/
try:
    os.listdir("/sd")
    SAVE_DIR = "/sd/images"
    print("SD카드 사용: " + SAVE_DIR)
except:
    SAVE_DIR = "/flash/images"
    print("내부 플래시 사용: " + SAVE_DIR)

try:
    os.mkdir(SAVE_DIR)
except:
    pass  # 이미 존재하면 무시

# ── 기존 파일 수 확인 ─────────────────────────────────
existing = [f for f in os.listdir(SAVE_DIR) if f.endswith(".jpg")]
count = len(existing)

TARGET = 50      # 목표 촬영 수
INTERVAL = 3000  # 촬영 간격 (ms)

print("=== 데이터셋 수집 시작 ===")
print("저장 위치: " + SAVE_DIR)
print("기존 파일: " + str(count) + "장")
print("목표: " + str(TARGET) + "장")
print("간격: " + str(INTERVAL // 1000) + "초마다 자동 촬영")
print("")
print("병을 다양한 각도/거리로 카메라 앞에 놓으세요")

# ── 준비 깜빡임 ───────────────────────────────────────
for _ in range(3):
    led_blue.on(); pyb.delay(300)
    led_off();     pyb.delay(300)

# ── 촬영 루프 ─────────────────────────────────────────
last_shot = pyb.millis()

while count < TARGET:
    img = pyb.millis()

    # 프레임 갱신 (프리뷰 유지)
    frame = sensor.snapshot()

    now = pyb.millis()
    if now - last_shot >= INTERVAL:
        num = str(count)
        if len(num) == 1: num = "00" + num
        elif len(num) == 2: num = "0" + num
        filename = SAVE_DIR + "/pepsi_" + num + ".jpg"

        try:
            frame.save(filename)
            count += 1
            last_shot = now

            # 초록 LED 깜빡 = 촬영 완료
            led_green.on(); pyb.delay(100); led_off()

            progress = str(count) + "/" + str(TARGET)
            print("촬영: " + filename + "  [" + progress + "]")

        except Exception as e:
            # 빨강 LED = 오류
            led_red.on(); pyb.delay(500); led_off()
            print("저장 오류: " + str(e))

# ── 완료 ─────────────────────────────────────────────
print("")
print("=== 촬영 완료: " + str(count) + "장 저장됨 ===")
print("SD카드를 PC에 연결하여 /images 폴더를 확인하세요")

# 완료 표시: 초록 3회 깜빡
for _ in range(3):
    led_green.on(); pyb.delay(300)
    led_off();      pyb.delay(300)

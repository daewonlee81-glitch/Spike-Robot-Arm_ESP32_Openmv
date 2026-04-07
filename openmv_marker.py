"""
OpenMV H7 Plus - 로봇팔 노랑 구간 추적
로봇팔의 노랑 LEGO 빔을 추적하여 끝단 위치 감지

전략:
  - 노랑 blob 전체 감지
  - cy 가 가장 작은(화면 위쪽 = 끝단) blob 선택
  - cx, cy, size 전송
"""

import sensor, image
from pupremote import PUPRemoteSensor

# ── 카메라 설정 ───────────────────────────────────────
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320 x 240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)

# ── 노랑 임계값 (LAB) ─────────────────────────────────
# Tools → Machine Vision → Threshold Editor → LAB 보정
# LEGO 노랑 기본값 (실내 조명 기준)
YELLOW_THRESHOLD = (60, 100, -15, 10, 40, 127)

# ── 캐시 변수 ─────────────────────────────────────────
_cx   = -1
_cy   = -1
_size =  0

# ── marker 함수 (add_command 전에 정의) ──────────────
def marker():
    return _cx, _cy, _size

# ── PUPRemote 설정 ────────────────────────────────────
p = PUPRemoteSensor(max_packet_size=16)
p.add_command('marker', to_hub_fmt='3h', from_hub_fmt='')

# ── 메인 루프 ─────────────────────────────────────────
while True:
    img = sensor.snapshot()

    blobs = img.find_blobs(
        [YELLOW_THRESHOLD],
        pixels_threshold=150,
        area_threshold=150,
        merge=True
    )

    if blobs:
        # cy가 가장 작은 blob = 화면 위쪽 = 팔 끝단
        tip = min(blobs, key=lambda b: b.cy())

        # 나머지 blob은 회색 박스로 표시
        for b in blobs:
            img.draw_rectangle(b.rect(), color=(100, 100, 100))

        # 끝단 blob은 노랑 박스 + 십자로 강조
        img.draw_rectangle(tip.rect(),     color=(255, 255, 0), thickness=2)
        img.draw_cross(tip.cx(), tip.cy(), color=(255, 0,   0), size=10)

        _cx, _cy, _size = tip.cx(), tip.cy(), tip.pixels()
    else:
        _cx, _cy, _size = -1, -1, 0

    p.process()

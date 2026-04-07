"""
OpenMV H7 Plus - 로봇팔 마커 추적
로봇팔 끝단에 부착된 마커(노랑)를 추적하여
허브로 위치 데이터 전송

전송: cx, cy, size
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

# ── 마커 임계값 (LAB) ─────────────────────────────────
# 노랑 마커 기본값 — Threshold Editor로 보정 필요
# Tools → Machine Vision → Threshold Editor → LAB
MARKER_THRESHOLD = (50, 100, -20, 10, 30, 127)

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
        [MARKER_THRESHOLD],
        pixels_threshold=200,
        area_threshold=200,
        merge=True
    )

    if blobs:
        b = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(b.rect(),   color=(255, 255, 0))
        img.draw_cross(b.cx(), b.cy(), color=(0, 0, 255))
        _cx, _cy, _size = b.cx(), b.cy(), b.pixels()
    else:
        _cx, _cy, _size = -1, -1, 0

    p.process()

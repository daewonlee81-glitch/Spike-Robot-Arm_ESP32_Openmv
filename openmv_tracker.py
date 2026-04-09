"""
OpenMV H7 Plus - 빨강 물체 추적
Port E → SPIKE Prime 허브로 blob 데이터 전송
전송: cx, cy, size (픽셀)
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
sensor.set_vflip(True)      # 카메라 뒤집힘 보정 (상하)
sensor.set_hmirror(True)    # 카메라 뒤집힘 보정 (좌우)

# ── 빨강 임계값 (LAB) ─────────────────────────────────
RED_THRESHOLD = (8, 72, 11, 127, -128, 127)

# ── blob 캐시 ─────────────────────────────────────────
_cx   = -1
_cy   = -1
_size =  0

# ── blob 함수 (add_command 전에 정의) ────────────────
def blob():
    return _cx, _cy, _size

# ── PUPRemote 설정 ────────────────────────────────────
p = PUPRemoteSensor(max_packet_size=16)
p.add_command('blob', to_hub_fmt='3h', from_hub_fmt='')

# ── 메인 루프 ─────────────────────────────────────────
while True:
    img = sensor.snapshot()

    blobs = img.find_blobs(
        [RED_THRESHOLD],
        pixels_threshold=300,
        area_threshold=300,
        merge=True
    )

    if blobs:
        b = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(b.rect(),   color=(255, 0, 0))
        img.draw_cross(b.cx(), b.cy(), color=(0, 255, 0))
        _cx, _cy, _size = b.cx(), b.cy(), b.pixels()
    else:
        _cx, _cy, _size = -1, -1, 0

    p.process()

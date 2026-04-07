"""
OpenMV H7 Plus - 로봇팔 노랑 구간 추적 (고속)
- QQVGA (160x120) 저해상도로 처리 속도 최적화
- draw 연산 제거
- ROI로 처리 영역 제한
"""

import sensor, image
from pupremote import PUPRemoteSensor

# ── 카메라 설정 ───────────────────────────────────────
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160 x 120 (QVGA의 1/4 픽셀)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)

# ── ROI 설정 (처리 영역 제한) ─────────────────────────
# (x, y, width, height) — 로봇팔이 움직이는 영역만 처리
# 전체: 160x120, 필요시 좁혀서 추가 가속 가능
ROI = (0, 0, 160, 120)

# ── 노랑 임계값 (LAB) ─────────────────────────────────
YELLOW_THRESHOLD = (57, 100, -128, 83, 127, 36)

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
        roi=ROI,
        pixels_threshold=50,   # QQVGA 기준 축소
        area_threshold=50,
        merge=True
    )

    if blobs:
        tip = min(blobs, key=lambda b: b.cy())
        _cx, _cy, _size = tip.cx(), tip.cy(), tip.pixels()
    else:
        _cx, _cy, _size = -1, -1, 0

    p.process()

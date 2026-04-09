"""
OpenMV H7 Plus - PC로 노랑 끝단 위치 스트리밍
USB Serial로 cx, cy, size 를 PC에 전송
PUPRemote 없음 — PC와 직접 통신
"""

import sensor, image, time

# ── 카메라 설정 ───────────────────────────────────────
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)    # 320 x 240 (VGA보다 4배 빠름)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)

# ── 노랑 임계값 (LAB) ─────────────────────────────────
# 형식: (L_min, L_max, A_min, A_max, B_min, B_max)
# 노랑 = B 채널 양수(36~127), A 채널 중립
YELLOW_THRESHOLD = (0, 100, -128, 127, 127, 32)

ROI = (0, 0, 320, 240)

# ── 메인 루프 ─────────────────────────────────────────
while True:
    img = sensor.snapshot()

    blobs = img.find_blobs(
        [YELLOW_THRESHOLD],
        roi=ROI,
        pixels_threshold=100,
        area_threshold=100,
        merge=True
    )

    if blobs:
        tip = min(blobs, key=lambda b: b.cy())
        print(str(tip.cx()) + "," + str(tip.cy()) + "," + str(tip.pixels()))
    else:
        print("-1,-1,0")

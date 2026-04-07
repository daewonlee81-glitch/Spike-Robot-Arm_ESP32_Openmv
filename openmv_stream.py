"""
OpenMV H7 Plus - PC로 노랑 끝단 위치 스트리밍
USB Serial로 cx, cy, size 를 PC에 전송
PUPRemote 없음 — PC와 직접 통신
"""

import sensor, image, time

# ── 카메라 설정 ───────────────────────────────────────
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)   # 160 x 120
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)

# ── 노랑 임계값 (LAB) ─────────────────────────────────
YELLOW_THRESHOLD = (57, 100, -128, 83, 127, 36)

ROI = (0, 0, 160, 120)

# ── 메인 루프 ─────────────────────────────────────────
while True:
    img = sensor.snapshot()

    blobs = img.find_blobs(
        [YELLOW_THRESHOLD],
        roi=ROI,
        pixels_threshold=50,
        area_threshold=50,
        merge=True
    )

    if blobs:
        tip = min(blobs, key=lambda b: b.cy())
        # PC로 전송: "cx,cy,size\n"
        print(str(tip.cx()) + "," + str(tip.cy()) + "," + str(tip.pixels()))
    else:
        print("-1,-1,0")

    time.sleep_ms(20)   # 50fps 목표

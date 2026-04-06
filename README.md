# SPIKE Prime + ESP32 + OpenMV 로봇팔 프로젝트

LEGO SPIKE Prime 허브를 중심으로 ESP32 서보 컨트롤러와 OpenMV H7 Plus 카메라를 연동한 로봇팔 제어 프로젝트입니다.
턴테이블 기반의 회전 구조 위에 3축 서보 로봇팔과 그리퍼가 장착되어 있으며, 컬러 마커를 이용한 위치 정렬과 빨강 물체 추적 기능을 포함합니다.

---

## 시스템 구성

```
SPIKE Prime Hub
├── Port C → ESP32 (Outbreak Board)  : 로봇팔 서보 4축 제어
├── Port D → Color Sensor            : 파랑/초록 마커 감지
├── Port E → OpenMV H7 Plus          : 빨강 물체 추적
└── Port F → Large Motor             : 턴테이블 회전
```

---

## 하드웨어 구매처

| 부품 | 구매처 | 링크 |
|------|--------|------|
| LEGO SPIKE Prime | 퓨너스 | [shopfuners.com](https://shopfuners.com/goods/goods_view.php?goodsNo=1166) |
| ESP32 / OpenMV Outbreak Board | Anton's Mindstorms | [antonsmindstorms.com](https://www.antonsmindstorms.com/product-category/hub-expansion-boards/) |
| OpenMV H7 Plus | DFRobot | [dfrobot.com](https://www.dfrobot.com/product-2263.html) |
| Geekservo Gray (서보모터) | AliExpress | AliExpress에서 "Geekservo Gray" 검색 |

---

## 포트 / 핀 맵

| 장치 | 연결 위치 | 역할 |
|------|----------|------|
| Large Motor | SPIKE Port F | 턴테이블 회전 |
| Color Sensor | SPIKE Port D | 파랑/초록 마커 감지 |
| OpenMV H7 Plus | SPIKE Port E | 빨강 blob 추적 |
| ESP32 Outbreak | SPIKE Port C | PUPRemote 통신 |
| Geekservo (1번 관절) | ESP32 GPIO 21 | 로봇팔 하단 관절 (-15° ~ 30°) |
| Geekservo (2번 관절) | ESP32 GPIO 19 | 로봇팔 중단 관절 (10° ~ 35°) |
| Geekservo (3번 관절) | ESP32 GPIO 22 | 로봇팔 상단 관절 (-30° ~ 30°) |
| Geekservo (그리퍼)   | ESP32 GPIO 20 | 그리퍼 (-15° ~ 15°) |

---

## 파일 설명

### ESP32 (MicroPython)

#### `esp32_servo.py`
ESP32에서 실행되는 메인 서보 제어 코드입니다.
4축(관절 3개 + 그리퍼) 서보를 누산기(accumulator) 방식으로 부드럽게 제어하며,
PUPRemote Sensor로 SPIKE Prime 허브와 통신합니다.

- **명령어**
  - `speedmove(a21, a19, a22, a20, speed)` : 목표 각도 및 속도 설정
  - `getpos()` : 현재 4축 위치 반환
- **speed 범위** : 1~100 (내부적으로 5~500 deg/s로 변환)

#### `gripper_test.py`
ESP32에서 그리퍼(GPIO 20)만 단독으로 테스트하는 코드입니다.
`0° → 15° → 0° → -15°` 순서로 동작을 확인합니다.

---

### SPIKE Prime Hub (Pybricks)

#### `hub_control.py`
허브 메인 제어 코드입니다. 전체 동작 시퀀스를 담당합니다.

1. 시계반대 방향으로 회전하여 **파랑 마커** 탐색 → 0° 초기화
2. 시계 방향으로 회전하여 **초록 마커** 탐색 → 각도 기록
3. 두 마커의 **중간 지점(중심)** 으로 이동
4. 파랑 위치 → 로봇팔 픽업 동작 (그리퍼 열기 → 내리기 → 닫기 → 복귀)
5. 초록 위치 → 동일한 픽업 동작 반복

#### `hub_tracker.py`
OpenMV로부터 빨강 물체의 위치(cx, cy)를 받아 로봇팔이 실시간으로 추적하는 코드입니다.

- **수평(cx)** → 턴테이블 비례 속도 제어
- **수직(cy)** → 로봇팔 a19 관절 각도 조정
- 데드존 내에서는 모터 정지하여 떨림 방지

#### `turntable_center.py`
파랑 마커에서 출발하여 초록 마커까지의 엔코더 값을 측정하고,
그 절반 위치(원호의 중심)로 턴테이블을 이동시키는 독립 실행 코드입니다.

#### `gripper_hub_test.py`
Pybricks에서 그리퍼 동작을 확인하는 테스트 코드입니다.
팔을 최초 위치에 고정한 채 그리퍼만 `0° → 15° → 0° → -15°` 로 순차 동작합니다.

#### `measure_gear_ratio.py`
컬러 센서를 이용하여 턴테이블의 기어비를 자동으로 측정하는 유틸리티 코드입니다.
파랑 마커를 두 번 감지하여 모터 1회전 각도를 계산합니다.

---

### OpenMV H7 Plus (MicroPython)

#### `openmv_tracker.py`
OpenMV H7 Plus에서 실행되는 빨강 물체 감지 코드입니다.
LAB 색상 공간 기반의 blob 탐지를 수행하고, 가장 큰 blob의 중심 좌표와 크기를 PUPRemote로 허브에 전송합니다.

- **RED_THRESHOLD** : `(0, 100, 13, 127, -128, 23)` (실제 환경에 맞게 보정 필요)
- 메인 루프에서 항상 캡처하고 캐시에 저장 → 허브 요청 시 즉시 응답

---

## 필요 소프트웨어

| 소프트웨어 | 용도 | 다운로드 |
|-----------|------|---------|
| [Pybricks](https://code.pybricks.com) | SPIKE Prime 허브 코드 작성 및 실행 | 웹 브라우저 실행 (설치 불필요) |
| [OpenMV IDE](https://openmv.io/pages/download) | OpenMV H7 Plus 코드 업로드 및 디버깅 | openmv.io |
| [Thonny](https://thonny.org) 또는 [VS Code](https://code.visualstudio.com) | ESP32 MicroPython 코드 업로드 | thonny.org / code.visualstudio.com |
| [PUPRemote](https://github.com/antonvh/PUPRemote) | SPIKE Prime ↔ ESP32 / OpenMV 통신 라이브러리 | GitHub |

---

## 통신 프로토콜

본 프로젝트는 **PUPRemote** 라이브러리를 사용하여 SPIKE Prime과 ESP32, OpenMV 간 통신합니다.

```
허브 → ESP32  : speedmove(a21, a19, a22, a20, speed)  [5b]
ESP32 → 허브  : return 1                               [b]

허브 → ESP32  : getpos()                               [없음]
ESP32 → 허브  : (cur21, cur19, cur22, cur20)           [4b]

허브 → OpenMV : blob()                                 [없음]
OpenMV → 허브 : (cx, cy, size)                         [3h]
```

---

## 라이선스

MIT License

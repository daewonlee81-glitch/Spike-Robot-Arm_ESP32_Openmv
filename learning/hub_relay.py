"""
SPIKE Prime 허브 - PC 중계 스크립트
PC(USB Serial) 명령을 받아 ESP32로 전달

PC → USB Serial → Hub → PUPRemote → ESP32

명령 포맷 (PC → Hub):
  "move,a21,a19,a22,a20,speed\n" → ESP32 speedmove
  "pos\n"                        → ESP32 getpos → PC로 반환
"""

import sys
from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from pupremote_hub import PUPRemoteHub

hub = PrimeHub()

arm = PUPRemoteHub(Port.C, max_packet_size=16)
arm.add_command('speedmove', to_hub_fmt='b',  from_hub_fmt='5b')
arm.add_command('getpos',    to_hub_fmt='4b', from_hub_fmt='')

hub.light.on(Color.BLUE)
print("ready")   # PC에 준비 신호 전송

buf = ""

while True:
    # USB Serial 수신
    ch = sys.stdin.read(1)
    if ch:
        buf += ch
        if '\n' in buf:
            line = buf.strip()
            buf  = ""

            if line == "pos":
                pos = arm.call('getpos')
                if pos:
                    sys.stdout.write(
                        str(pos[0]) + "," +
                        str(pos[1]) + "," +
                        str(pos[2]) + "," +
                        str(pos[3]) + "\n"
                    )
                else:
                    sys.stdout.write("err\n")

            elif line.startswith("move,"):
                parts = line.split(",")
                if len(parts) == 6:
                    a21   = int(parts[1])
                    a19   = int(parts[2])
                    a22   = int(parts[3])
                    a20   = int(parts[4])
                    speed = int(parts[5])
                    arm.call('speedmove', a21, a19, a22, a20, speed)
                    sys.stdout.write("ok\n")

    wait(5)

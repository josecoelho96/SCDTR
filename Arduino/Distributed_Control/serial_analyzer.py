#!/usr/bin/env python3 -u

# View ports with:
# $ python3 -m serial.tools.list_ports

import serial
import threading


def reader(serial_port):
    ser = serial.Serial(
            port=serial_port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )

    buff = []
    while True:
        c = ser.read().decode("utf-8")

        if len(c) > 0:
            # print("[DEBUG] : {}".format(c))
            if c != '\n':
                buff.append(c)
            else:
                print("[" + serial_port + "] : " + "".join(buff))
                buff = []
                # print("New line!")

if __name__  == '__main__':
    threading.Thread(target=reader, args =("/dev/ttyACM0", )).start()
    threading.Thread(target=reader, args =("/dev/ttyACM1", )).start()

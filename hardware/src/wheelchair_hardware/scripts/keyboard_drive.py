#!/usr/bin/env python3
"""
Direct keyboard control — sends commands straight to ESP32 over serial.
No ROS2 needed. Run standalone in any terminal.

Usage:
  python3 keyboard_drive.py
  python3 keyboard_drive.py /dev/ttyUSB1

Keys:
  W   forward       S   backward
  A   turn left     D   turn right
  X   stop          B   brake
  +   speed up      -   speed down
  Q   quit
"""

import sys
import tty
import termios
import serial
import glob

def find_port():
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    return ports[0] if ports else '/dev/ttyUSB0'

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    print(f'Connecting to {port} ...')
    ser = serial.Serial(port, 115200, timeout=0.1)
    print('Connected.')
    print()
    print('  W = forward    S = backward')
    print('  A = left       D = right')
    print('  X = stop       B = brake')
    print('  + = faster     - = slower')
    print('  Q = quit')
    print()

    try:
        while True:
            ch = getch().upper()
            if ch == 'Q':
                ser.write(b'X')
                print('\nStopped. Bye.')
                break
            elif ch in ('W', 'S', 'A', 'D', 'X', 'B', '+', '-'):
                ser.write(ch.encode())
                labels = {
                    'W': 'FORWARD', 'S': 'BACKWARD',
                    'A': 'LEFT',    'D': 'RIGHT',
                    'X': 'STOP',    'B': 'BRAKE',
                    '+': 'FASTER',  '-': 'SLOWER',
                }
                print(f'\r  → {labels[ch]}          ', end='', flush=True)
    except KeyboardInterrupt:
        ser.write(b'X')
        print('\nStopped.')
    finally:
        ser.close()

if __name__ == '__main__':
    main()

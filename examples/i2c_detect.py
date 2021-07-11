# -*- coding: utf-8 -*-
"""Detect I2C devices connected to MCP2221
   This script is the equivalent to the well known i2c_detect tool in linux.
   With less features, though.
"""

import time
from mcp22xx import MCP2221

def main():
    device = MCP2221.get_device()
    device.i2c_setspeed(100e3)

    start = time.time()
    i2cdevices = device.i2c_detect(0x00, 0x7f)
    duration = time.time() - start

    print("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F")

    for i in range(0x00, 0x80):
        if i % 16 == 0:
            if i > 0:
                print('')
            print("{:02X}:".format(i), end='')

        if i in i2cdevices:
            print(' {:02X}'.format(i), end='')
        else:
            print(' --', end='')

    print()

    print("execution time: %0.0f ms" % (duration * 1000))

if __name__ == "__main__":
    main()

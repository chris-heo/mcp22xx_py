# -*- coding: utf-8 -*-
"""Tests the real clock used by the controller
   This example sets different clocks dividers and transmits it as one byte to the device
   with address 0x50. This can be an EEProm - this only sets the address registers and produces
   no wear on the device. With a logic analyzer or oscilloscope the clock frequency can be measured.
   This data was used to improve the calculation of the MCP2221 I2C clock set feature.
"""
import time
from mcp22xx import MCP2221

def test_performance(device, i2c_speed_div):
    device.i2c_setparameters(False, i2c_speed_div)

    time.sleep(5e-3)

    print("testing with divider %u" % device.i2c_speed_div)
    device.i2c_write(0x50, [i2c_speed_div])
    time.sleep(5e-3)


def main():
    device = MCP2221.get_device()
    for i2c_speed_div in [1, 5, 10, 15, 20, 25, 30, 35, 40, 50, 60,
                          70, 80, 90, 100, 125, 150, 200, 250, 255]:
        test_performance(device, i2c_speed_div)

if __name__ == "__main__":
    main()


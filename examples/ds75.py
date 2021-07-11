# -*- coding: utf-8 -*-
"""Reads the temperature from all DS75 connected"""
from mcp22xx import MCP2221

class MaximDS75():

    Addresses = list(range(0x48, 0x50))

    def __init__(self, i2c_dev, i2c_addr):
        self.dev = i2c_dev
        self.i2c_addr = i2c_addr

    def cfg_write(self, resolution=3, faulttolerance=0,
                  thermostat_polarity=False, thermostat_mode=False, shutdown=False):
        val = 0
        val |= (resolution & 3) << 5
        val |= (faulttolerance & 3) << 3
        if thermostat_polarity:
            val |= (1 << 2)
        if thermostat_mode:
            val |= (1 << 1)
        if shutdown:
            val |= (1 << 0)

    def temp_read(self):
        val = self.dev.i2c_read_register(self.i2c_addr, 0, 2)
        if not val:
            return False

        val2 = val[0] << 8 | val[1]

        if val2 & 0x8000 != 0:
            val2 = ((val2 ^ 0xFFFF) + 1) * -1
        return float(val2) / 256

def main():
    device = MCP2221.get_device()
    device.i2c_setspeed(100e3)

    addresses = device.i2c_detect(MaximDS75.Addresses[0], MaximDS75.Addresses[-1])

    if not addresses:
        print("no sensor found")

    for i in addresses:
        sensor = MaximDS75(device, i)
        sensor.cfg_write()
        print("0x%02X: %0.2f °C" % (i, sensor.temp_read()))


if __name__ == "__main__":
    main()

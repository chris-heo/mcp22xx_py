# -*- coding: utf-8 -*-
"""Tests a MCP4725 (12 bit DAC)"""
from mcp22xx import MCP2221
from enum import Enum

class Mcp4725_Powermode(Enum):
    Normal = 0
    Pd1k = 1
    Pd100k = 2
    Pd500k = 3

class Mcp4725():

    Addresses = [
        0x60, 0x61,
        #0x62, 0x63, 0x64, 0x65, 0x66, 0x67 # only available on customer request
    ]

    def __init__(self, i2c_dev, i2c_addr):
        self.dev = i2c_dev
        self.i2c_addr = i2c_addr

        self.dacval = 0
        self.powermode = Mcp4725_Powermode.Normal

    def write(self, value=None, save_eeprom=False, powermode=None):
        if value is not None:
            value = int(min(4095, max(0, round(value, 0))))
            self.dacval = value
        cmd = 2 # write DAC resistor (no EEPROM)
        if save_eeprom is True:
            cmd |= 1
        if powermode is not None:
            self.powermode = powermode

        self.dev.i2c_write(self.i2c_addr,
                           [(cmd << 5) | ((self.powermode.value & 0x02) << 1),
                            (self.dacval >> 4 & 0xFF), (self.dacval & 0x0F) << 4])

def main():
    device = MCP2221.get_device()
    device.i2c_setspeed(400e3)

    dac = Mcp4725(device, 0x60)

    for i in range(0, 4096):
        dac.write(i)

if __name__ == "__main__":
    main():
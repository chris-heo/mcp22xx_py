# -*- coding: utf-8 -*-
"""Reads the measurement values from a TI INA3221 Current/Voltage Monitor"""
from mcp22xx import MCP2221
import math
import time
from enum import Enum

class Ina3221Channel():
    def __init__(self, device, index, shuntResistance=None, enabled=True):
        self.device = device
        self.index = index
        self.shuntResistance = shuntResistance
        self.enabled = enabled
        self.shuntVoltage = 0
        self.busVoltage = 0

        self.criticalLimit = 0
        self.warningLimit = 0

    def get_voltage(self):
        return self.busVoltage

    def get_current(self):
        if self.shuntResistance <= 0 or self.shuntResistance is None:
            return None
        return self.shuntVoltage / self.shuntResistance

    def get_power(self):
        return self.get_current() * self.busVoltage

    def _drv_set_values(self, shunt, voltage):
        if shunt is not None:
            if shunt & 0x8000:
                # 2's complement for negative values
                shunt = ((shunt ^ 0xFFFF) + 1) * -1
            self.shuntVoltage = float(shunt >> 3) * 40e-6 # 1 LSB = 40 µV

        if voltage is not None:
            if voltage & 0x8000:
                # 2's complement for negative values
                voltage = ((voltage ^ 0xFFFF) + 1) * -1
            self.busVoltage = float(voltage >> 3) * 8e-3 # 1 LSB = 8 mV

    def _drv_set_values_raw(self, arr):
        self._drv_set_values(
            (arr[0] << 8) | arr[1],
            (arr[2] << 8) | arr[3]
        )

    def read(self):
        # goddammit TI, why u not allow to read all regs at once?
        a = self.device.dev.i2c_read_register(self.device.i2c_addr, self.index * 2 + 1, 2)
        b = self.device.dev.i2c_read_register(self.device.i2c_addr, self.index * 2 + 2, 2)

        self._drv_set_values((a[0] << 8) | a[1], (b[0] << 8) | b[1])


class Ina3221Averaging(Enum):
    avg1 = 0
    avg4 = 1
    avg16 = 2
    avg64 = 3
    avg128 = 4
    avg256 = 5
    avg512 = 6
    avg1024 = 7

class Ina3221ConversionTime(Enum):
    ct140us = 0
    ct204us = 1
    ct332us = 2
    ct588us = 3
    ct1100us = 4
    ct2116us = 5
    ct4156us = 6
    ct8244us = 7

class Ina3221OperationMode(Enum):
    Powerdown0 = 0
    ShuntVoltageTriggered = 1
    BusVoltageTriggered = 2
    ShuntBusVoltageTriggered = 3
    Powerdown1 = 4
    ShuntVoltageContinuous = 5
    BusVoltageContinuous = 6
    ShuntBusVoltageContinuous = 7

class Ina3221():
    def __init__(self, i2c_dev, i2c_addr, shuntResistance=[0.1, 0.1, 0.1]):
        self.dev = i2c_dev
        self.i2c_addr = i2c_addr
        self.channels = [
            Ina3221Channel(self, 0, shuntResistance[0]),
            Ina3221Channel(self, 1, shuntResistance[1]),
            Ina3221Channel(self, 2, shuntResistance[2])
        ]

        self.averaging = Ina3221Averaging.avg1
        self.busVoltageConvTime = Ina3221ConversionTime.ct1100us
        self.shuntVoltageConvTime = Ina3221ConversionTime.ct1100us
        self.operationMode = Ina3221OperationMode.ShuntBusVoltageContinuous

    def set_config(self):
        byte0 = 0 << 7 # Reset
        if self.channels[0].enabled:
            byte0 |= 1 << 6
        if self.channels[1].enabled:
            byte0 |= 1 << 5
        if self.channels[2].enabled:
            byte0 |= 1 << 4

        byte0 |= (self.averaging.value & 0x07) << 1
        byte0 |= (self.busVoltageConvTime.value & 0x04) >> 2

        byte1 = (self.busVoltageConvTime.value & 0x03) << 6
        byte1 |= (self.shuntVoltageConvTime.value & 0x07) << 3
        byte1 |= (self.operationMode.value & 0x07) << 0

        self.dev.i2c_write_register(self.i2c_addr, 0x00, [byte0, byte1])

    def readall(self):
        for channel in self.channels:
            channel.read()

def main():
    device = MCP2221.get_device()
    device.i2c_setspeed(400e3)

    mon = Ina3221(device, 0x40)

    while True:
        mon.readall()

        num = 0
        for channel in mon.channels:
            print("Ch%u: %+7.3f V %+7.3f A %+7.3f W" %
                  (num, channel.get_voltage(), channel.get_current(), channel.get_power())
                 )
            num += 1

        print()
        time.sleep(0.2)


if __name__ == "__main__":
    main()

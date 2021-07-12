# -*- coding: utf-8 -*-
"""Tests the real clock used by the controller
   This example sets different clocks dividers and transmits it as one byte to the device
   with address 0x50. This can be an EEProm - this only sets the address registers and produces
   no wear on the device. With a logic analyzer or oscilloscope the clock frequency can be measured.
   This data was used to improve the calculation of the MCP2221 I2C clock set feature.
"""
import time
from mcp22xx.mcp2210 import MCP2210, SpiChannel
from mcp22xx.mcp2210_enums import PinFunction, SpiMode

def test_performance(dev, bitrate):
    cs = dev.gpio[0]
    cs.function = PinFunction.CHIP_SELECT
    spi = SpiChannel(dev, cs, bitrate, SpiMode.MODE0)
    spi.bitrate = int(bitrate)

    txdata = bytearray([
        (bitrate >> 24) & 0xFF,
        (bitrate >> 16) & 0xFF,
        (bitrate >> 8) & 0xFF,
        (bitrate >> 0) & 0xFF,
    ]*15)

    start = time.time()
    rxdata = spi.transfer(txdata)
    dur = time.time() - start


    print("net bitrate: %0.1f kbit/s gross speed: %0.3f kbit/s" % 
        (bitrate / 1000, len(txdata)/dur*8/1000)
    )

def main():
    device = MCP2210.get_device()
    for bitrate in [100, 500, 1000, 5000, 10000, 50000, 
        100000, 500000, 1000000, 5000000, 10000000, 12000000]:
        test_performance(device, bitrate)

if __name__ == "__main__":
    main()

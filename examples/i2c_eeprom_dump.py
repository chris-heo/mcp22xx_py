# -*- coding: utf-8 -*-
"""Measures the performance of MCP2221's I2C interface by reading an EEPROM
   For this test a Microchip 25LC512 with all address pins connected to GND
   (7 bit address: 0x50) was used.
"""
import time
from mcp22xx import MCP2221

def main():
    device = MCP2221.get_device()

    i2c_speed = 400000
    i2c_addr = 0x50 # 7 bit address
    reg_offset = [0, 0] # MSByte, LSByte
    size = 256

    device.i2c_setspeed(i2c_speed)
    speed = device.i2c_getspeed()

    start = time.time()
    data = device.i2c_read_register(i2c_addr, reg_offset, size)
    duration = time.time() - start

    print("data: " + " ".join(["%02X" % e for e in data]))

    print()

    print("SCL freq: %0.2f kHz" % (speed / 1000))
    print("execution time: %0.0f ms" % (duration * 1000))
    # "len(reg_offset) + 2" is added for i2c addr and reg_offset transmission
    throughput = ((size + len(reg_offset) + 2) / duration * 8 / 1000)
    print("throughput: %0.2f kbit/s" % throughput)
    # "* (9 / 8)" is added to correct the efficiency due to ACK/NAK bits
    print("transfer efficiency: %0.2f %%" % (throughput * (9 / 8) / speed * 100 * 1000))


if __name__ == "__main__":
    main()

# -*- coding: utf-8 -*-
"""Measures the performance of MCP2221's I2C interface by reading an EEPROM
   For this test a Microchip 25LC512 with all address pins connected to GND
   (7 bit address: 0x50) was used.
"""
import time
from mcp22xx import MCP2221, I2CFraming

def test_performance(device, i2c_speed, i2c_addr, reg_offset, read_len):
    device.i2c_setspeed(i2c_speed)
    speed = device.i2c_getspeed()

    device.i2c_write(i2c_addr, reg_offset, I2CFraming.NoStop)

    start = time.time()
    data = device.i2c_read(i2c_addr, read_len, True)
    duration = time.time() - start

    #print("data: " + " ".join(["%02X" % e for e in data]))

    print("SCL freq: %0.2f kHz" % (speed / 1000))
    print("read length: %u bytes" % read_len)
    print("execution time: %0.0f ms" % (duration * 1000))
    # "+ 2" is added for i2c addr transmission
    # "/ 8 * 9" is to consider ACK/NAK-bits
    throughput = ((read_len + 1) / 8 * 9 / duration * 8 / 1000)
    print("throughput: %0.2f kbit/s" % throughput)
    # ACK/NAK bits are not considered
    print("transfer efficiency: %0.2f %%" % (throughput / speed * 100 * 1000))
    print()

def main():
    device = MCP2221.get_device()

    i2c_addr = 0x50 # 7 bit address
    reg_offset = [0, 0] # MSByte, LSByte

    for read_len in [2400]:
        for i2c_speed in [50, 100, 150, 200, 250, 300, 350, 400, 450]:
            test_performance(device, i2c_speed * 1000, i2c_addr, reg_offset, read_len)

if __name__ == "__main__":
    main()

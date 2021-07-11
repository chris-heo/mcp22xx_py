# -*- coding: utf-8 -*-
"""Plots the measurement values from a MAG3110 magnetometer (compass) using matplotlib"""

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from mcp22xx import MCP2221

class NxpMAG3110():
    def __init__(self, i2c_dev, i2c_addr):
        self.dev = i2c_dev
        self.i2c_addr = i2c_addr
        self.cfg_write()

    def cfg_write(self):
        self.dev.i2c_write_register(self.i2c_addr, 0x10, [0x01, 0x80])

    @staticmethod
    def _extractaxis(values, index):
        val = (values[index * 2] << 8) | values[(index * 2) + 1]
        if val & 0x8000:
            val = ((val ^ 0xFFFF) + 1) * -1
        return val

    def readdata(self):
        val = self.dev.i2c_read_register(self.i2c_addr, 1, 6)
        #print(val)
        return [self._extractaxis(val, 0), self._extractaxis(val, 1), self._extractaxis(val, 2)]

def main():
    device = MCP2221.get_device()
    device.i2c_setspeed(400e3)
    compass = NxpMAG3110(device, 0x0E)

    fig, ax = plt.subplots()
    ax.set_ylim([-6000, 6000])

    interval = 0.025

    t = list(np.arange(-10, 0, interval))
    x = [0] * len(t)
    y = [0] * len(t)
    z = [0] * len(t)
    line1, = ax.plot(t, x) # blue
    line2, = ax.plot(t, y) # orange
    line3, = ax.plot(t, z) # green

    def animate(i):
        data = compass.readdata()
        x.pop(0)
        x.append(data[0])
        line1.set_ydata(x)

        y.pop(0)
        y.append(data[1])
        line2.set_ydata(y)

        z.pop(0)
        z.append(data[2])
        line3.set_ydata(z)

        return line1, line2, line3,

    # Init only required for blitting to give a clean slate.
    def init():
        line1.set_ydata(np.ma.array(x, mask=True))
        line2.set_ydata(np.ma.array(y, mask=True))
        line2.set_ydata(np.ma.array(z, mask=True))

        return line1, line2, line3,

    ani = animation.FuncAnimation(fig, animate, np.arange(1, 200), init_func=init,
                                  interval=interval * 1000, blit=True)
    plt.show()

if __name__ == "__main__":
    main()
# -*- coding: utf-8 -*-
"""Measures the VIO voltage on hobbyelektronik.org's MCP-USB-Bridge:
   https://hobbyelektronik.org/w/index.php/MCP-USB-Bridge
   Both jumpers ("Ena" and "Meas") must be set for v1
"""
import time
from mcp22xx import MCP2221, Gpio3Func, ReferenceVoltage

def main():
    device = MCP2221.get_device()
    device.gpios[3].set_direction(False)

    # for some very weird reason, the ADC reference must be set after the pin function
    device.gpios[3].set_function(Gpio3Func.ADC3)
    device.adc_setreference(ReferenceVoltage.Int1024mV)

    while True:
        voltages = device.adc_readvoltage()
        print("%0.3f" % (voltages[2] / 3.3 * (22 + 3.3)))
        time.sleep(0.5)

if __name__ == "__main__":
    main()

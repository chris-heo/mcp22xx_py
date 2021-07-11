# -*- coding: utf-8 -*-
"""Interface for Microchip MCP2221 v0.1
   Copyright (c) 2018 chris@hobbyelektronik.org
   License: CC BY-NC-SA 3.0
   Commercial use may be possible but requires the explicit consent of the author.

   Any use of this software that enables or supports
   warfare, fraud or harming individuals is prohibited.

   The above copyright notice and this permission notice shall be
   included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
   BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
   DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

from enum import Enum

#pylint: disable=R0903
class Gpio0Func(Enum):
    """Function enumerator for MCP2221's GP0"""
    GPIO = 0
    SSPND = 1
    LED_URX = 2

#pylint: disable=R0903
class Gpio1Func(Enum):
    """Function enumerator for MCP2221's GP1"""
    GPIO = 0
    CLK_OUT = 1
    ADC1 = 2
    LED_UTX = 3
    IOC = 4

#pylint: disable=R0903
class Gpio2Func(Enum):
    """Function enumerator for MCP2221's GP2"""
    GPIO = 0
    USBCFG = 1
    ADC2 = 2
    DAC1 = 3

#pylint: disable=R0903
class Gpio3Func(Enum):
    """Function enumerator for MCP2221's GP3"""
    GPIO = 0
    LED_I2C = 1
    ADC3 = 2
    DAC2 = 3

#pylint: disable=R0903
class ReferenceVoltage(Enum):
    """Reference voltage enumerator for MCP2221's ADC and DAC"""

    ExtVdd = 0
    IntOff = (0 << 1) | 1 # there is no real use of it
    Int1024mV = (1 << 1) | 1
    Int2048mV = (2 << 1) | 1
    Int4096mV = (3 << 1) | 1

    @staticmethod
    def get_voltage(entry):
        """Returns the voltage for the given ReferenceVoltage enum entry

        Arguments:
            entry {ReferenceVoltage} -- entry to which the voltage shall be returned

        Returns:
            float -- Voltage or None
        """

        voltage = None
        if entry == ReferenceVoltage.ExtVdd:
            voltage = 5.0
        elif entry == ReferenceVoltage.Int1024mV:
            voltage = 1.024
        elif entry == ReferenceVoltage.Int2048mV:
            voltage = 1.024
        elif entry == ReferenceVoltage.Int4096mV:
            voltage = 4.096

        return voltage

    @classmethod
    def _missing_(cls, value):
        if type(value) is int:
            if value > 7:
                # invalid values
                pass
            if value >= 0 and (value & 1) == 0:
                return cls.ExtVdd
        raise ValueError("%r is not a valid %s" % (value, cls.__name__))

#pylint: disable=R0903
class MCP2221Commands(Enum):
    """Command enumerator for MCP2221"""
    StatusSetParameters = 0x10
    ReadFlashData = 0xB0
    WriteFlashData = 0xB1
    SendFlashAccessPassword = 0xB2

    I2cWriteData = 0x90
    I2cWriteDataRepeatedStart = 0x92
    I2cWriteDataNoStop = 0x94
    I2cReadData = 0x91
    I2cReadDataRepeatedStart = 0x93
    I2cGetReadData = 0x40

    SetGpioOutputValues = 0x50
    GetGpioValues = 0x51

    SetSramSettings = 0x60
    GetSramSettings = 0x61

    ResetChip = 0x70

#pylint: disable=R0903
class I2CFraming(Enum):
    """I2C Framing enumerator for MCP2221"""
    Normal = 0
    RepeatedStart = 1
    NoStop = 2

#pylint: disable=R0903
class ClockoutDutycycle(Enum):
    """Duty cycle enumerator for MCP2221 clock output"""
    Dc0Percent = 0
    Dc25Percent = 1
    Dc50Percent = 2
    Dc75Percent = 3

#pylint: disable=R0903
class ClockoutDivider(Enum):
    """Clock divider enumerator for MCP2221 clock output"""
    Cd_2 = 1 #  24 MHz
    Cd_4 = 2 #  12 MHz
    Cd_8 = 3 #   6 MHz
    Cd_16 = 4 #  3 MHz
    Cd_32 = 5 #  1.5 MHz
    Cd_64 = 6 #    750 kHz
    Cd_128 = 7 #   375 kHz

#pylint: disable=R0903
class I2CStates(Enum):
    """Internal I2C Engine states
       This is mostly guesswork as Microchip didn't want to tell me the internal states.
       That's also why the code uses magic numbers instead of the enum.
       Some values can be found in MCP's linux driver sources (i2c-mcp2221.c)
    """
    Idle = 0x00
    CmdExecuted = 0x10 # unsure
    StartTimeout = 0x12
    RestartTimeout = 0x17
    WrAddrRlWsend = 0x21
    WrAddrRLTimeout = 0x23
    WriteAddrNak = 0x25
    WriteDataTimeout = 0x44
    WriteAddrDataAck = 0x45
    DataReadOngoing = 0x50
    ReadDataTimeout = 0x52
    DataReadComplete = 0x55
    DataReadPending = 0x54
    Locked = 0x61 # i2c engine is in a lock state, no clue how to get out of it
    StopTimeout = 0x62
    ReadError = 0x7F

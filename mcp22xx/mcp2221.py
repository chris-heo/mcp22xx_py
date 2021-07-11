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

import time
import hid
#from inspect import getframeinfo, stack

#pylint: disable=W0401
from mcp22xx.mcp2221_enums import (
    Gpio0Func,
    Gpio1Func,
    Gpio2Func,
    Gpio3Func,
    MCP2221Commands,
    ReferenceVoltage,
    I2CFraming,
)

class I2CCommError(Exception):
    """I2C related exceptions"""
    pass

class MCP2221(object):
    """Microchip MCP2221(A) communication class.
       Communication layer for the I2C bridge
    """

    class MCP2221gpio(object):
        """GPIO class for MCP2221"""

        funcs = [Gpio0Func, Gpio1Func, Gpio2Func, Gpio3Func]

        def __init__(self, parent, index):
            self.parent = parent
            self.index = index

            self.output = None
            self.function = None
            self.value = None
            self.adcvalue = None

        def set_function(self, function, immediate=True):
            """Sets the function of this pin

            Arguments:
                function {Gpio[0-3]Func} -- Function of the pin

            Keyword Arguments:
                immediate {bool} -- If True, the function is set immediately. If False,
                                    method 'xyz' of parent object has to be called (default: {True})

            Raises:
                TypeError -- Raised when wrong enum type for function is used
            """

            if not isinstance(function, self.funcs[self.index]):
                raise TypeError("Wrong type used, use enum Gpio%uFunc instead" % self.index)
            else:
                self.function = function

            if immediate:
                self.parent.sram_write_settings(setgpios=True)

        def set_direction(self, output, immediate=True):
            """Sets the direction of the pin to input or output. Only valid if GPIO function is set

            Arguments:
                output {bool} -- True to use the pin output, False for input

            Keyword Arguments:
                immediate {bool} -- [description] (default: {True})
            """

            self.output = output

            if immediate:
                self.parent.set_gpios()

        def set_value(self, high, immediate=True):
            """Sets the value of the pin when it's defined as output

            Arguments:
                high {bool} -- True to set the GPIO high, False for low

            Keyword Arguments:
                immediate {bool} -- [description] (default: {True})
            """

            self.value = high

            if immediate:
                self.parent.set_gpios()

        def get_value(self, immediate=True):
            """Returns the value of the pin when it's defined as input.
               When it is output, the current output value is returned.

            Keyword Arguments:
                immediate {bool} -- [description] (default: {True})

            Returns:
                bool -- Value of the pin
            """

            if self.output is False and immediate is True:
                self.parent.get_gpios()

            return self.value

        def drv_setgpios(self):
            """Returns the GPIO-settings in order to set them via the GPIO function
               This is an internal method and shall not be called by the user

            Returns:
                list -- list of parameters of the GPIO
            """

            cfg = [0] * 4
            cfg[0] = 1 # alter GPx output
            if self.value is True:
                cfg[1] = 1

            cfg[2] = 1 # alter GPx pin direction
            if self.output is not True:
                cfg[3] = 1

            return cfg

        def drv_getgpios(self, data):
            """Applies the GPIO-settings read via the GPIO functions from the device
               This is an internal method and shall not be called by the user

            Arguments:
                data {list} -- list of parameters of the GPIO

            Raises:
                Exception -- [description]
                Exception -- [description]
            """

            pinval, pindir = data
            # 0xEE: not set as GPIO, 0: L, 1: H
            if pinval == 0xEE:
                self.value = None
            elif pinval == 0x00:
                self.value = False
            elif pinval == 0x01:
                self.value = True
            else:
                raise Exception("Invalid value for GPIO%u value" % self.index)

            # 0xEF: not set as GPIO, 0: O, 1: I
            if pindir == 0xEF:
                self.output = None
            elif pindir == 0x00:
                self.output = True
            elif pindir == 0x01:
                self.output = False
            else:
                raise Exception("Invalid value for GPIO%u direction" % self.index)

        def drv_setsram(self):
            """Returns the GPIO-settings in order to set them via the SRAM function
               This is an internal method and shall not be called by the user

            Returns:
                int -- parameters of the GPIO
            """
            data = 0
            if self.function.value == Gpio0Func.GPIO.value:
                if self.value is True:
                    data |= (1 << 4)

                if self.output is False:
                    data |= (1 << 3)
            else:
                if self.function is not None:
                    data |= (self.function.value & 0x07)
            return data

        def drv_getsram(self, data):
            """Applies the GPIO-settings read via the SRAM functions from the device
               This is an internal method and shall not be called by the user

            Arguments:
                data {int} -- parameters of the GPIO

            Raises:
                Exception -- [description]
                Exception -- [description]
            """
            self.value = data & (1 << 4) != 0
            self.output = data & (1 << 3) == 0
            self.function = self.funcs[self.index](data & 0x07)

    MESSAGE_LENGTH = 64

    # These calculations are not quite accurate (but better than in the datasheet)
    I2C_CLKDIV_OFFSET = -5.5
    I2C_CLKDIV_BASEFREQ = 12e6
    I2C_CLKDIV_MIN = 20
    I2C_CLKDIV_MAX = 255

    @classmethod
    def enumerate(cls, vid=0x04D8, pid=0x00DD):
        """Returns a list of all connected USB devices with the given VID/PID-pair

        Keyword Arguments:
            vid {hexadecimal} -- Vendor-ID of the devices (default: {0x04D8})
            pid {hexadecimal} -- Product-ID of the devices (default: {0x00DD})

        Returns:
            list -- Result of hid.enumerate, use the member 'path' for init
        """

        #pylint: disable=E1101,I1101
        return hid.enumerate(vid, pid)

    @classmethod
    def get_device(cls, vid=0x04D8, pid=0x00DD, serial_number=None, index=0):
        """Instantiates a new MCP2221-object by the VID/PID/index

        Keyword Arguments:
            vid {hexadecimal} -- vendor id of the device (default: {0x04D8})
            pid {hexadecimal} -- product id of the device (default: {0x00DD})
            serial_number {string} -- serial number of the device (default: {None})
            index {int} -- index of the device in the enumeration list (default: {0})

        Returns:
            MCP2221 -- Object of the connected device or None
        """

        devices = cls.enumerate(vid, pid)

        if serial_number is not None:
            for entry in devices:
                if entry["serial_number"] == serial_number:
                    return cls(entry["path"])
            return None

        if index >= len(devices):
            return None

        return cls(devices[index]["path"])

    def __init__(self, path):
        self.device = hid.device()
        #pylint: disable=E1101,I1101
        # this is one of the most stupid bugs I've ever encountered:
        # obviously, the MCP2221 doesn't like get probed and opened
        # in a very short period of time. A tiny little pause seems
        # to be a good workaround. Be warned: YMMV!
        # This cost me almost 3 evenings and two mails to Microchip
        # (they suggested to use the MCP2221A)
        time.sleep(0.01)
        self.device.open_path(path)
        self.device_path = path

        self.gpios = [
            self.MCP2221gpio(self, 0),
            self.MCP2221gpio(self, 1),
            self.MCP2221gpio(self, 2),
            self.MCP2221gpio(self, 3),
        ]

        self.adc_reference = None
        self.dac_reference = None
        self.dac_value = 0
        self.adc_values = [0] * 3

        self._i2c_state = 0
        self.i2c_transferlength_req = 0
        self.i2c_transferlength_done = 0
        self.i2c_speed_div = 1
        self.i2c_curaddr = 0
        self.i2c_linestate = [0, 0] # SCL, SDA
        self.i2c_read_pending = 0
        self.i2c_currtimeout = 0

        self.sram_read_settings()
        # it seems that the MCP2221 also dislikes prophylactic
        # i2c_cancel after startup which results in a lockup.

    def __del__(self):
        self.device.close()

    def _get_i2cstate(self):
        return self._i2c_state

    def _set_i2cstate(self, val):
        #caller = getframeinfo(stack()[1][0])
        #caller2 = getframeinfo(stack()[2][0])
        #print("%s:%d - %s .. %s -> 0x%02X" %
        #      (caller.filename, caller.lineno, caller2.function, caller.function, val))
        self._i2c_state = val

    i2c_state = property(_get_i2cstate, _set_i2cstate)

    def device_driver_info(self):
        """HID Device Driver Info"""

        self.device.get_manufacturer_string()
        self.device.get_product_string()
        self.device.get_serial_number_string()

        #print("Manufacturer: %s" % self.device.get_manufacturer_string())
        #print("Product: %s" % self.device.get_product_string())
        #print("Serial No: %s" % self.device.get_serial_number_string())

    def sendcommand(self, data):
        """Send a command to the usb-bridge

        Arguments:
            data {list} -- data to be sent to the bridge

        Returns:
            [list] -- data returned from the bridge
        """
        #print("> " + " ".join("%02X" % i for i in data))

        data.insert(0, 0) # first entry is the endpoint (?)
        datalen = self.MESSAGE_LENGTH + 1
        if data is None:
            return False
        elif len(data) < datalen:
            # add padding to the data, total length must be 65
            data += [0] * (datalen - len(data))
        elif len(data) > datalen:
            # truncate exceeding data
            data = data[0:datalen]

        self.device.write(data)

        rbuf = self.device.read(datalen)

        #print("write %u bytes, read %u bytes" % (len(data), len(rbuf)))

        #print("< " + " ".join("%02X" % i for i in rbuf))
        #print("")

        # check if response belongs to the request
        # caution: send buffer and receive buffer have a leadind byte
        # if not, maybe another software caused to fill the read buffer
        # TODO: clear read buffer before writing new data
        if rbuf[0] != data[1]:
            raise Exception("Wrong response")

        return rbuf

    @classmethod
    def _getpayloadprototype(cls, command):
        """creates an empty payload array that can be sent to the controller with the given command

        Arguments:
            command {MCP2221Commands} -- command that shall be written into the payload

        Returns:
            list -- list of bytes for communication with the controller
        """
        data = [0] * cls.MESSAGE_LENGTH
        data[0] = command.value
        return data

    def reset(self):
        """Reset the MCP2221"""
        buf = [0] * 65
        buf[0:5] = [0x00, MCP2221Commands.ResetChip.value, 0xAB, 0xCD, 0xEF]
        self.device.write(buf)
        # this is quite cruel
        time.sleep(5)
        self.__init__(self.device_path)

    @staticmethod
    def _unpack16le(data, offset):
        """extracts the 16 bit word from data at the given offset with low endianness

        Arguments:
            data {list} -- list of bytes from which the word shall be extracted
            offset {int} -- offset of the first byte of the data to be extracted

        Returns:
            int -- value representing the value at offset
        """

        return data[offset] | (data[offset + 1] << 8)

    def i2c_cancel(self):
        """Cancels the last I2C transaction

        Returns:
            bool -- True if the command was successful, False if not
        """

        return self.i2c_setparameters(True, None)

    def i2c_getstatus(self):
        """Reads the status from the device. Can be read from the object's member variables

        Returns:
            bool -- True if the command was successful, False if not
        """

        return self.i2c_setparameters(False, None)

    @classmethod
    def _i2c_calc_clkdiv(cls, freq):
        clkdiv = int(round(cls.I2C_CLKDIV_BASEFREQ / float(freq) + cls.I2C_CLKDIV_OFFSET, 0))
        return max(cls.I2C_CLKDIV_MIN, min(cls.I2C_CLKDIV_MAX, clkdiv))

    @classmethod
    def _i2c_calc_freq(cls, clkdiv):
        return cls.I2C_CLKDIV_BASEFREQ / (clkdiv - cls.I2C_CLKDIV_OFFSET)

    def i2c_setspeed(self, i2c_speed):
        """Sets the speed for the I2C interface

        Arguments:
            i2c_speed {int} -- Speed of the I2C interface in Hz
        """
        if i2c_speed == 0:
            raise ValueError("I2C speed can't be 0")

        i2c_clkdiv = self._i2c_calc_clkdiv(i2c_speed)
        return self.i2c_setparameters(False, i2c_clkdiv)

    def i2c_getspeed(self):
        """Returns the speed of the I2C interface in Hz

        Returns:
            float -- I2C speed in Hz (bits/s)
        """

        self.i2c_setparameters()
        return self._i2c_calc_freq(self.i2c_speed_div)

    i2c_speed = property(i2c_getspeed, i2c_setspeed)

    def i2c_setparameters(self, i2c_cancel=False, i2c_speed_div=None):
        """Sets the parameters and reads the status of the device

        Keyword Arguments:
            i2c_cancel {bool} -- True to cancel the last I2C transaction (default: {False})
            i2c_speed_div {int} -- Speed of the I2C interface in Hz (default: {None})

        Raises:
            ValueError -- Raised, if the I2C speed was invalid

        Returns:
            bool -- True if the command was successful, False if not
        """

        buf = self._getpayloadprototype(MCP2221Commands.StatusSetParameters)

        if i2c_cancel is True:
            buf[2] = 0x10
        if i2c_speed_div:
            # this information can't be found in the datasheet but in Microchip's linux driver
            buf[3] = 0x20
            buf[4] = i2c_speed_div

        rbuf = self.sendcommand(buf)

        status = rbuf[1]
        # i2ccanceltransferstate = rbuf[2]
        # i2cspeedsetstatus = rbuf[3]
        self.i2c_state = rbuf[8]
        self.i2c_transferlength_req = self._unpack16le(rbuf, 9)
        self.i2c_transferlength_done = self._unpack16le(rbuf, 11)
        # i2cdatabuffcnt = rbuf[13]
        self.i2c_speed_div = rbuf[14]
        self.i2c_currtimeout = rbuf[15]
        self.i2c_curaddr = self._unpack16le(rbuf, 16)
        self.i2c_linestate = [rbuf[22] == 1, rbuf[23] == 1]
        # irqedgestate = rbuf[24]
        self.i2c_read_pending = rbuf[25]
        # mcphwrev = rbuf[46:49]
        self.adc_values = [self._unpack16le(rbuf, 50), self._unpack16le(rbuf, 52),
                           self._unpack16le(rbuf, 54)]

        return status

    def adc_setreference(self, reference):
        """Sets the voltage reference for the ADC

        Arguments:
            reference {ReferenceVoltage} -- reference voltage to be used by the ADC

        Raises:
            TypeError -- wrong type is used for reference
        """

        #return self.set_sram_settings(adcref=reference)

        if not isinstance(reference, ReferenceVoltage):
            raise TypeError("Wrong type used, use enum ReferenceVoltage instead")

        self.adc_reference = reference

        buf = self._getpayloadprototype(MCP2221Commands.SetSramSettings)
        buf[5] = (1 << 7) | (reference.value & 0x07)
        self.sendcommand(buf)

    def adc_read(self):
        """Reads the raw values from the chip's internal ADCs.

        Returns:
            list -- ADC values for GP1, GP2, GP3 - range 0 .. 1023
        """

        buf = self._getpayloadprototype(MCP2221Commands.StatusSetParameters)
        rbuf = self.sendcommand(buf)
        self.adc_values = [self._unpack16le(rbuf, 50), self._unpack16le(rbuf, 52),
                           self._unpack16le(rbuf, 54)]
        return self.adc_values

    def adc_readvoltage(self):
        """Reads the values from the chip's internal ADCs and converts it to the corresponding
        voltage. Caution: Readings for reference = ExtVdd may be inaccurate!

        Raises:
            Exception -- invalid value for reference voltage

        Returns:
            list -- ADC values in volts for GP1, GP2, GP3.
        """

        refvoltage = ReferenceVoltage.get_voltage(self.adc_reference)

        if refvoltage is None:
            raise Exception("invalid value for reference voltage, is ADC reference set?")

        # FIXME: it's not entirely clear whether the chip's ADC maps 1023 to the reference voltage
        # or if it has a virtual value for '1024' (meaning that maximum value is Vref minus 1 LSB)
        return [float(x) / 1023 * refvoltage for x in self.adc_read()]


    def dac_setreference(self, reference):
        """Sets the voltage reference for the DAC

        Arguments:
            reference {ReferenceVoltage} -- reference voltage to be used by the DAC

        Raises:
            TypeError -- wrong type is used for reference
        """
        #return self.set_sram_settings(dacref=reference)

        if not isinstance(reference, ReferenceVoltage):
            raise TypeError("Wrong type used, use enum ReferenceVoltage instead")

        self.dac_reference = reference

        buf = self._getpayloadprototype(MCP2221Commands.SetSramSettings)
        buf[3] = (1 << 7) | (reference.value & 0x07)
        self.sendcommand(buf)

    def dac_setvalue(self, value):
        """Sets the raw value of the DAC

        Arguments:
            value {int} -- value of the DAC, must be between 0 and 31

        Raises:
            ValueError -- value is out of range
        """

        #return self.set_sram_settings(dacval=value)

        if value < 0 or value > 0x1F:
            raise ValueError("Value must be between 0 and 31")

        buf = self._getpayloadprototype(MCP2221Commands.SetSramSettings)
        buf[4] = (1 << 7) | int(value)
        self.sendcommand(buf)

    def dac_setvoltage(self, voltage):
        """Sets the voltage value of the DAC. This might be inaccurate if ExtVdd is used

        Arguments:
            voltage {float} -- Voltage to be set

        Returns:
            float -- Actual voltage being set considering the granularity of the DAC
        """

        maxvoltage = ReferenceVoltage.get_voltage(self.dac_reference)
        dacval = round(float(voltage) / maxvoltage * 31)

        self.dac_setvalue(dacval)

        return float(dacval) / 31 * maxvoltage


    #TODO: read_flash_data
    #TODO: write_flash_data
    #TODO: send_flash_password

    # Sending more than 60 bytes of payload requires to send multiple messages to the chip.
    # (Un)fortunately, the file handle is not locking while the i2c is busy and also there
    # is no buffer on the side of the controller. Each write/read (of any command) usually
    # takes up to 2 ms (USB packet separation time foo, afaik). Also, as the controller has
    # only one core, the USB interrupt may delay data delivery of the I2C. To get the best
    # throughput, the busy time on the I2C was measured (as a function of i2c_speed).
    # The wait time is not indirect proportional to i2c_speed because the throughput drops
    # the higher the speed goes (fixed wait times or bottleneck somewhere?)
    # For some reason, the chip doesn't really care about the inter frame space time, the
    # even when setting to really long values (below timeout), it returns status = 0x01.
    # So I decided to send the data over and over again. Not really nice but it works...
    #def _i2c_write_waittime(self):
    #    i2c_speed = self._i2c_calc_freq(self.i2c_speed_div)
    #    if i2c_speed < 100e3:
    #        return 18.5e-3 - 0.105e-6 * i2c_speed
    #    elif i2c_speed < 130e3:
    #        return 8e-3
    #    elif i2c_speed < 225e3:
    #        return 5.5e-3
    #    else:
    #        return 3.5e-3

    def i2c_write(self, address, data, framing=I2CFraming.Normal):
        """Send data to a I2C-Device

        Arguments:
            address {hexadecimal} -- 7-bit address of the I2C-Device
            data {list} -- data to be sento to the device

        Raises:
            Exception -- [description]
            Exception -- [description]
        """

        cmd = MCP2221Commands.I2cWriteData
        if framing == I2CFraming.NoStop:
            cmd = MCP2221Commands.I2cWriteDataNoStop
        elif framing == I2CFraming.RepeatedStart:
            cmd = MCP2221Commands.I2cWriteDataRepeatedStart

        buf = self._getpayloadprototype(cmd)

        if len(data) > 0xFFFF:
            raise ValueError("data length > 0xFFFF is not supported")

        datalen = len(data)

        buf[1] = datalen & 0xFF
        buf[2] = datalen >> 8
        buf[3] = (address << 1) & 0xFF

        slicelen = 60
        status = -1

        #retransmission_wait = self._i2c_write_waittime()
        retransmissions_max = 20
        retransmissions_total = 0

        for i in range(0, max(len(data), 1), slicelen):
            #ibwt = retransmission_wait # inter block wait time
            tmp = data[i:i+slicelen]
            tmp.extend([0] * (slicelen - len(tmp)))

            buf[4:63] = tmp

            for retry in range(0, retransmissions_max):
                #ts = time.time()
                rbuf = self.sendcommand(buf[:])
                status = rbuf[1]
                self.i2c_state = rbuf[2]

                #print("status: 0x%02X, i2c_state: 0x%02X" % (status, self.i2c_state))

                if status == 0x00:
                    #block successfully transmitted, continue with the next
                    #print("block %u done" % (i / 60))
                    break

                #print("retransmission needed, %u. try" % (retry + 1))
                retransmissions_total += 1

                if retry == (retransmissions_max - 1):
                    self.i2c_cancel()
                    raise I2CCommError("Retransmission limit reached")

                #waited = time.time() - ts
                #towait = ibwt - waited
                #print("already waited %0.2f ms of %0.2f ms" %
                #      (waited * 1000, total_waittime * 1000))
                #if towait > 0:
                #    print("now sleep additional %0.2f ms" % (towait * 1000))
                #    time.sleep(towait)

                #self.i2c_getstatus()
                #print("i2c_state: 0x%02X" % (self.i2c_state))

                # set inter block wait time to 1 as we are most
                # likely just waiting for the last block to be done
                #ibwt = 1

            #for j in range(1, 51):
            #    self.i2c_getstatus()
            #    print("+%0.2f ms wait i2c_state: 0x%02X" %
            #          ((time.time() - ts) * 1000, self.i2c_state))
            #    time.sleep(0.001)

            #print("total retries for this transmission: %u" % retransmissions_total)

            if status != 0:
                self.i2c_cancel()
                return False

        return status == 0x00

    def i2c_detect(self, addr_start=0x00, addr_stop=0x7F):
        """scans I2C-devices available on the bus

        Keyword Arguments:
            addr_start {hexadecimal} -- start address (7-bit) (default: {0x00})
            addr_stop {hexadecimal} -- stop address (7-bit) (default: {0x7F})

        Returns:
            list -- list of 7-bit-addresses found on the bus
        """
        addr_f = min(addr_start, addr_stop, 0x7F)
        addr_t = max(addr_start, addr_stop, 0) + 1

        found_addr = []

        for addr in range(addr_f, addr_t):
            try:
                self.i2c_write(addr, [])
                self.i2c_getstatus()
                if self.i2c_state == 0:
                    found_addr.append(addr)
                else:
                    self.i2c_cancel()
            except:
                pass
            

        return found_addr

    def i2c_read(self, address, length, repeated_start=False):
        """Reads data from a I2C-Device

        Arguments:
            address {hexadecimal} -- 7-bit-address of the I2C-Device
            length {int} -- amount of bytes that should be read

        Keyword Arguments:
            repeated_start {bool} -- True, if the write before was done without stop condition
                                     (default: {False})

        Raises:
            Exception -- [description]
            Exception -- [description]
            Exception -- [description]

        Returns:
            list -- data returned by the I2C-Device
        """

        if length <= 0:
            raise I2CCommError("I2C read with length <= 0 is not supported")

        cmd = MCP2221Commands.I2cReadData
        if repeated_start is True:
            cmd = MCP2221Commands.I2cReadDataRepeatedStart

        if length > 0xFFFF:
            raise I2CCommError("data length > 0xFFFF is not supported")

        buf = self._getpayloadprototype(cmd)
        buf[1] = length & 0xFF
        buf[2] = length >> 8
        buf[3] = ((address << 1) | 1) & 0xFF
        rbuf = self.sendcommand(buf)

        status = rbuf[1]
        #self.i2c_state = rbuf[2] # don't bother it here

        if status != 0:
            raise I2CCommError("Unable to read data")

        self.i2c_getstatus()
        if self.i2c_state == 0x25:
            raise I2CCommError("Got NAK to read request")

        # 0x00: command completed successfully
        # 0x01: I2C engine is busy (command not completed)

        return self._i2c_getdata()

    def _i2c_getdata(self):
        data = []
        buf = self._getpayloadprototype(MCP2221Commands.I2cGetReadData)

        error_waterlevel = 0
        while True:
            rbuf = self.sendcommand(buf[:])

            # status (buf[1]) == 0x41 doesn't necessarily mean an error,
            # it's also returned if no data is avaiable yet
            # so it's better to wait a little between read commands
            # (not completely investigates yet)
            # to prevent infinity loops, a water level is used.
            # Sorry for the magic constants...
            self.i2c_state = rbuf[2]

            if rbuf[1] == 0:
                error_waterlevel = max(0, error_waterlevel - 1)
            elif self.i2c_state != 0x50:
            #else:
                error_waterlevel += 1


            if error_waterlevel > 200:
                raise I2CCommError("too many unsuccessful read attempts")

            if self.i2c_state == 0x25:
                raise I2CCommError("write addr NAK")
            readlen = rbuf[3]

            if readlen == 127:
                continue
            if 0 < readlen <= 60:
                data.extend(rbuf[4:(4 + readlen)])
            if self.i2c_state == 0x55: #0x55: data read done, 0x54: data pending
                return data

        return data

    def i2c_read_register(self, address, register, readlength):
        """Reads a register from a I2C device

        Arguments:
            address {int} -- 7-bit-address of the I2C device
            register {int|list} -- register address, either byte or list of bytes
            readlength {int} -- length of the data to be read

        Returns:
            list|bool -- list of returned bytes from the I2C device or False if device couldn't be
                         accessed
        """

        wdata = []
        if isinstance(register, int):
            wdata = [register & 0xFF]
        elif isinstance(register, list):
            wdata = register

        if not self.i2c_write(address, wdata, I2CFraming.NoStop):
            return False

        retdata = self.i2c_read(address, readlength, True)
        self.i2c_getstatus()
        return retdata

    def i2c_write_register(self, address, register, data):
        """Writes a register to a I2C device

        Arguments:
            address {int} -- 7-bit-address of the I2C device
            register {int|list} -- register address, either byte or list of bytes
            data {int|list} -- data to be written to that register, either byte or list of bytes
        """

        wdata = []
        if isinstance(register, int):
            wdata = [register & 0xFF]
        elif isinstance(register, list):
            wdata = register
        else:
            raise TypeError("Could not convert to list")

        if isinstance(data, int):
            wdata.append(data & 0xFF)
        elif isinstance(data, list):
            wdata.extend(data)
        else:
            raise TypeError("Could not convert to list")

        #print("write: " + " ".join(["%02X" % e for e in wdata]))
        return self.i2c_write(address, wdata)

    def set_gpios(self):
        """Writes the directions and values of all GPIOs

        Returns:
            bool -- True if the command was successful, False if not
        """

        cmd = MCP2221Commands.SetGpioOutputValues
        buf = self._getpayloadprototype(cmd)

        buf[2:6] = self.gpios[0].drv_setgpios()
        buf[6:10] = self.gpios[1].drv_setgpios()
        buf[10:14] = self.gpios[2].drv_setgpios()
        buf[14:17] = self.gpios[3].drv_setgpios()

        rbuf = self.sendcommand(buf)

        return rbuf[1] == 0x00

    def get_gpios(self):
        """Reads the directions and values of the GPIOs

        Returns:
            bool -- True if the command was successful, False if not
        """

        cmd = MCP2221Commands.GetGpioValues
        buf = self._getpayloadprototype(cmd)

        rbuf = self.sendcommand(buf)

        for i in range(len(self.gpios)):
            offset = i * 2 + 2
            #pinval = rbuf[offset]
            #pindir = rbuf[offset+1]
            self.gpios[i].drv_getgpios(rbuf[offset:offset+2])

        return rbuf[1] == 0x00

    #pylint: disable=R0913
    def sram_write_settings(self, clkoutduty=None, clkoutdiv=None, dacref=None, dacval=None,
                            adcref=None, irqrising=None, irqfalling=None, irqclear=False,
                            setgpios=False):
        """Writes the settings to the (volatile) SRAM
        Keyword Arguments:
            clkoutduty {ClockoutDutycycle} -- Duty cycle of the clock, None to leave the value
                                              unchanged (default: {None})
            clkoutdiv {ClockoutDivider} -- Clock output divider, None to leave the value unchanged
                                           (default: {None})
            dacref {ReferenceVoltage} -- Reference voltage used for the DAC, None to leave the value
                                         unchanged (default: {None})
            dacval {int} -- Value for the DAC (0 .. 31), None to leave the value unchanged
                            (default: {None})
            adcref {ReferenceVoltage} -- Reference voltage used for the ADC, None to leave the value
                                         unchanged (default: {None})
            irqrising {bool} -- Fire IRQ on rising edges, None to leave the value unchanged
                                (default: {None})
            irqfalling {bool} -- Fire IRQ on falling edges, None to leave the value unchanged
                                 (default: {None})
            irqclear {bool} -- True to clear the Interrupt (default: {False})
            setgpios {bool} -- True to set the GPIO directions ans values (default: {False})

        Raises:
            Exception -- [description]

        Returns:
            [type] -- [description]
        """

        cmd = MCP2221Commands.SetSramSettings
        buf = self._getpayloadprototype(cmd)

        if clkoutduty is not None and clkoutdiv is not None:
            buf[2] = (1 << 7) | ((clkoutduty.value & 0x03) << 3) | (clkoutdiv.value & 0x07)
        elif clkoutduty != clkoutdiv:
            raise Exception("CLKOUT duty and divider must be set at the same time")

        if dacref is not None:
            if not isinstance(dacref, ReferenceVoltage):
                raise TypeError("Wrong type used, use enum ReferenceVoltage instead")
            self.dac_reference = dacref
            buf[3] = (1 << 7) | (dacref.value & 0x07)

        if dacval is not None:
            if dacval < 0 or dacval > 0x1F:
                raise ValueError("Value must be between 0 and 31")
            buf[4] = (1 << 7) | dacval

        if adcref is not None:
            if not isinstance(adcref, ReferenceVoltage):
                raise TypeError("Wrong type used, use enum ReferenceVoltage instead")
            self.adc_reference = adcref
            buf[5] = (1 << 7) | (adcref.value & 0x07)

        if irqrising is not None:
            buf[6] |= (1 << 7) | (1 << 4)
            if irqrising is True:
                buf[6] |= (1 << 3)

        if irqfalling is not None:
            buf[6] |= (1 << 7) | (1 << 2)
            if irqfalling is True:
                buf[6] |= (1 << 0)

        if irqclear is True:
            buf[6] |= (1 << 0)

        if setgpios is True:
            # this is wrong in the datasheet, ds says it's 0x01 but in fact it's 0x80
            buf[7] = (1 << 7)

        for i in range(len(self.gpios)):
            buf[8 + i] = self.gpios[i].drv_setsram()

        rbuf = self.sendcommand(buf)
        return rbuf[1] == 0x00

    def sram_read_settings(self):
        """Reads the SRAM settings from the device
        """

        cmd = MCP2221Commands.GetSramSettings
        buf = self._getpayloadprototype(cmd)

        rbuf = self.sendcommand(buf)

        #ignore offset = 2 .. 4

        #offset = 5: clock output divider
        #offset = 6: dac reference voltage
        self.dac_reference = ReferenceVoltage((rbuf[6] >> 5) & 0x07)
        self.adc_reference = ReferenceVoltage((rbuf[7] >> 2) & 0x07)
        #offset = 12: usb power attributes
        #offset = 13: USB requested number of mA
        #offset = 14 .. 21: current supplied-password
        self.gpios[0].drv_getsram(rbuf[22])
        self.gpios[1].drv_getsram(rbuf[23])
        self.gpios[2].drv_getsram(rbuf[24])
        self.gpios[3].drv_getsram(rbuf[25])

        return rbuf[1] == 0x00

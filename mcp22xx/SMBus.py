# -*- coding: utf-8 -*-
"""SMBus compatibility layer for Microchip MCP2221 v0.1
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
from mcp22xx.mcp2221 import (
    MCP2221
)

class SMBus(object):
    """SMBus compatibility layer for Microchip MCP2221"""

    @classmethod
    def enumerate(cls, vid=0x04D8, pid=0x00DD):
        """Returns a list of all connected USB devices with the given VID/PID-pair

        Keyword Arguments:
            vid {hexadecimal} -- Vendor-ID of the devices (default: {0x04D8})
            pid {hexadecimal} -- Product-ID of the devices (default: {0x00DD})

        Returns:
            list -- Result of hid.enumerate, use the member 'path' for init
        """
        return MCP2221.enumerate(vid, pid)

    @classmethod
    def get_device(cls, vid=0x04D8, pid=0x00DD, index=0):
        """Instantiates a new MCP2221-object by the VID/PID/index

        Keyword Arguments:
            vid {hexadecimal} -- vendor id of the device (default: {0x04D8})
            pid {hexadecimal} -- product id of the device (default: {0x00DD})
            index {int} -- index of the device in the enumeration list (default: {0})

        Returns:
            MCP2221 -- Object of the connected device or None
        """
        devices = cls.enumerate(vid, pid)
        if index >= len(devices):
            return None

        return cls(devices[index]["path"])

    def __init__(self, path):
        self.interface = MCP2221(path)

    def read_byte(self, address):
        return self.interface.i2c_read(address, 1)[0]

    def write_byte(self, address, value):
        return self.interface.i2c_write(address, [value])

    def read_byte_data(self, address, register):
        result = self.interface.i2c_read_register(address, register, 1)
        if result is False:
            return False
        return result[0]

    def write_byte_data(self, address, register, value):
        self.interface.i2c_write_register(address, register, value)

    def read_word_data(self, address, register):
        result = self.interface.i2c_read_register(address, register, 2)
        if result is False:
            return False
        return result[0] | (result[1] << 8)

    def write_word_data(self, address, register, value):
        return self.interface.i2c_write_register(address, register,
                                                 [value & 0xFF, (value >> 8) & 0xFF])

    def read_i2c_block_data(self, address, register, length):
        return self.interface.i2c_read_register(address, register, length)

    def write_i2c_block_data(self, address, register, data):
        return self.interface.i2c_write_register(address, register, data)

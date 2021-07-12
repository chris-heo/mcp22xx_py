import hid
import time
import ctypes

from .mcp2210_enums import SpiMode, PinFunction, Statuscode, IrqPinMode
from . import mcp2210_msgs
#from .mcp2210_msgs import printstruct

class MCP2210():
    MESSAGE_LENGTH = 64

    @classmethod
    def enumerate(cls, vid: int = 0x04D8, pid: int = 0x00DE) -> list:
        """Returns a list of all connected USB devices with the given VID/PID-pair.

        Args:
            vid (int, optional): Vendor id of the devices. Defaults to 0x04D8.
            pid (int, optional): Prodcut id of the devices. Defaults to 0x00DE.

        Returns:
            list: Result of hid.enumerate, use the member 'path' for init
        """

        #pylint: disable=E1101,I1101
        return hid.enumerate(vid, pid)

    @classmethod
    def get_device(cls, vid=0x04D8, pid=0x00DE, serial_number=None, index=0) -> object:
        """Creates a new MCP2210 instance by the VID/PID/index

        Args:
            vid (int, optional): Vendor id of the device. Defaults to 0x04D8.
            pid (int, optional): Product id of the device. Defaults to 0x00DE.
            serial_number (str, optional): Serial number of the device. Defaults to None.
            index (int, optional): Index of the device in the enumeration list. Defaults to 0.

        Returns:
            MCP2210: Object of the connected device or None if device could not be found.
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

    def __init__(self, path: str):
        """Creates an instance of a MCP2210 object

        Args:
            path (string): Path to the USB device
        """
        self.device = hid.device()
        
        time.sleep(0.01)
        self.device.open_path(path)
        self.device_path = path

        self.gpio_immediate = True

        self.gpio = [None] * 9
        for i in range(len(self.gpio)):
            self.gpio[i] = Gpio(self, i)

        self._remote_wake = None
        self._irq_mode = None
        self._spi_release = None

        self.gpio_settings_get()

    def send_command(self, cmd: object) -> tuple:
        """Send a command to the usb bridge

        Args:
            cmd (object): mcp2210-message to be transmitted

        Raises:
            Exception: Wrong response to message

        Returns:
            (bytearray, bool): returned data from the bridge and command status
        """
        #print("command 0x%02X's len: %u" % (cmd[0], len(cmd)))
        data = cmd

        datalen = self.MESSAGE_LENGTH + 1
        reportid = 0
        data.insert(0, reportid) # first entry is the report id (?)
        
        # expand/trim the data to match the report size
        if data is None:
            return False
        elif len(data) < datalen:
            # add padding to the data, total length must be 65
            data.extend([0] * (datalen - len(data)))
        elif len(data) > datalen:
            # truncate exceeding data
            data = data[0 : datalen]

        self.device.write(data)

        rbuf = bytearray(self.device.read(datalen))

        if rbuf[0] != data[1]:
            raise Exception("Wrong response to message")

        status = Statuscode(rbuf[1])

        return rbuf, status

    def gpio_settings_set(self, immediate: bool = True) -> bool:
        """Writes GPIO settings to the bridge

        Args:
            immediate (bool, optional): True to force setting the values, False
                to only set it if the object's immediate property is True.
                Defaults to True.

        Returns:
            bool: True if the command was successful, False if not executed or
                not successful
        """
        if immediate is False and self.gpio_immediate is False:
            return False

        cmd = mcp2210_msgs.VmGpioSettingsCmd()
        for i, gpio in enumerate(self.gpio):
            cmd.gpio_designation[i] = gpio._function.value
            if gpio._direction_in is True:
                cmd.gpio_direction |= 1 << i
            if gpio._value is True:
                cmd.gpio_default |= 1 << i

        chipsettings = 0

        if self._spi_release is True:
            chipsettings |= 1 << 0

        chipsettings |= self._irq_mode.value << 1

        if self._remote_wake is True:
            chipsettings |= 1 << 4

        cmd.chipsettings = chipsettings

        return cmd.send(self)

    def gpio_settings_get(self, immediate: bool = True) -> bool:
        """Reads the GPIO settings from the bridge

        Args:
            immediate (bool, optional): True to force setting the values, False
                to only set it if the object's immediate property is True.
                Defaults to True.

        Returns:
            bool: True if the command was successful, False if not executed or
                not successful
        """
        if immediate is False and self.gpio_immediate is False:
            return False

        req = mcp2210_msgs.VmGpioSettingsReq()
        ret = req.send(self)

        if ret == False:
            return False

        for i, gpio in enumerate(self.gpio):
            gpio._function = PinFunction(ret.gpio_designation[i])
            mask = 1 << i
            gpio._value = (ret.gpio_default & mask) != 0
            gpio._direction_in = (ret.gpio_direction & mask) != 0

        self._remote_wake = ret.chipsettings & (1 << 4) != 0
        self._irq_mode = IrqPinMode((ret.chipsettings >> 1) & 0x07)
        self._spi_release = ret.chipsettings & (1 << 0) != 0

        return True

    def gpio_val_set(self, immediate: bool = True) -> bool:
        """Writes the GPIO values of the bridge

        Args:
            immediate (bool, optional): Immediately transfer the data. Defaults to True.

        Raises:
            Exception: When GPIO values could not be set

        Returns:
            bool: True if the command was successful, False if not
        """
        if immediate is False and self.gpio_immediate is False:
            return False
        
        cmd = mcp2210_msgs.VmGpioSetValCmd()
        value = 0
        for i, gpio in enumerate(self.gpio):
            if gpio._value is True:
                value |= 1 << i
        
        cmd.gpio_value = value
        resp = cmd.send(self)
        if isinstance(resp, Statuscode):
            raise Exception("Could not set GPIO values: %s" % resp)
        
        # apply readback
        for i, gpio in enumerate(self.gpio):
            gpio._value = (resp.gpio_value & (1 << i)) != 0

        return True

    def gpio_val_get(self, immediate: bool = True) -> bool:
        """Read the GPIO values from the bridge

        Args:
            immediate (bool, optional): Immediately request the data. Defaults to True.

        Raises:
            Exception: When GPIO values could not be retrieved

        Returns:
            bool: True if the command was successful, False if not.
        """
        if immediate is False and self.gpio_immediate is False:
            return False
        
        cmd = mcp2210_msgs.VmGpioGetValCmd()
        resp = cmd.send(self)

        if isinstance(resp, Statuscode):
            raise Exception("Could not set GPIO values: %s" % resp)

        for i, gpio in enumerate(self.gpio):
            gpio._value = (resp.gpio_value & (1 << i)) != 0

        return True

    def gpio_dir_set(self, immediate: bool = True) -> bool:
        """Sets the GPIO values to the bridge

        Args:
            immediate (bool, optional): Immediately send the data. Defaults to True.

        Raises:
            Exception: When GPIO values could not be written

        Returns:
            bool: True if the command was successful, False if not.
        """
        if immediate is False and self.gpio_immediate is False:
            return False
        
        cmd = mcp2210_msgs.VmGpioSetDirCmd()
        value = 0
        for i, gpio in enumerate(self.gpio):
            if gpio._direction_in is True:
                value |= 1 << i
        
        cmd.gpio_direction = value
        resp = cmd.send(self)
        if isinstance(resp, Statuscode):
            raise Exception("Could not set GPIO directions: %s" % resp)
        
        # apply readback
        for i, gpio in enumerate(self.gpio):
            gpio._direction_in = (resp.gpio_direction & (1 << i)) != 0

        return True

    def gpio_dir_get(self, immediate: bool = True) -> bool:
        """Sets the GPIO directions to the bridge

        Args:
            immediate (bool, optional): Immediately send the data. Defaults to True.

        Raises:
            Exception: When GPIO directions could not be read

        Returns:
            bool: True if the command was successful, False if not.
        """
        if immediate is False and self.gpio_immediate is False:
            return False
        
        cmd = mcp2210_msgs.VmGpioGetDirCmd()
        resp = cmd.send(self)

        if isinstance(resp, Statuscode):
            raise Exception("Could not set GPIO directions: %s" % resp)

        for i, gpio in enumerate(self.gpio):
            gpio._direction_in = (resp.gpio_direction & (1 << i)) != 0

        return True

    def irq_get_events(self, reset: bool = False) -> int:
        """Reads the count of interrupt events from the bridge

        Args:
            reset (bool, optional): Reset the interrupt counter. Defaults to False.

        Raises:
            Exception: When events could not be read

        Returns:
            int: count of interrupt events
        """

        cmd = mcp2210_msgs.IrqEventStatusCmd()
        cmd.noreset = 0x00 if reset == True else 0xFF
        resp = cmd.send(self)

        if isinstance(resp, Statuscode):
            raise Exception("Could not get event count")
        
        return resp.event_count

    def remote_wake_set(self, value: bool):
        """Writes the setting for remote wakeup to the bridge

        Args:
            value (bool): True to enable remote wakeup, False to disable
        """
        self._remote_wake = value
        self.gpio_settings_set()
    
    def remote_wake_get(self) -> bool:
        """Reads the settings for remote wakeup from the bridge

        Returns:
            bool: True if remote wakeup is enabled, False if not
        """
        self.gpio_settings_get()
        return self._remote_wake

    remote_wake = property(remote_wake_get, remote_wake_set,
        doc="Sets or gets the setting for remote wakeup"
    )

    def irq_mode_set(self, value: IrqPinMode) -> bool:
        """Sets the interrupt mode of GP6

        Args:
            value (IrqPinMode): Signal transition the interrupt reacts to

        Returns:
            bool: True if the command was successfull, False if not
        """
        self._irq_mode = value
        return self.gpio_settings_set()
    
    def irq_mode_get(self) -> IrqPinMode:
        """Gets the interrupt mode of GP6

        Returns:
            IrqPinMode: Signal transition the interrupt reacts to
        """
        self.gpio_settings_get()
        return self._irq_mode

    irq_mode = property(irq_mode_get, irq_mode_set,
        doc="Sets or gets the interrupt mode of GP6"
    )

    def spi_release_set(self, value: bool):
        self._spi_release = value
        self.gpio_settings_set()
    
    def spi_release_get(self) -> bool:
        self.gpio_settings_get()
        return self._spi_release

    spi_release = property(spi_release_get, spi_release_set, 
        doc="Sets or gets whether the SPI bus is released between the transfers"
    )

class Gpio():
    def __init__(self, device: MCP2210, index: int):
        """Creates a new Gpio instance

        Args:
            device (MCP2210): MCP2210 object the pin is assigned to
            index (int): Index of the pin
        """
        self.device = device
        self.index = index

        self._function = PinFunction.GPIO
        self._value = False
        self._direction_in = True # False is output, True is input

        self.chipselect_idle = True
        self.chipselect_active = False

    def function_set(self, value: PinFunction):
        """Sets the function of the GPIO. The value might be cached,
                see MCP2210.gpio_immediate

        Args:
            value (PinFunction): Function of the GPIO

        Raises:
            TypeError: when the given value is not PinFunction
        """
        if value not in PinFunction:
            raise TypeError("Incompatible type, must be PinFunction")
        self._function = value
        self.device.gpio_settings_set(False)
    
    def function_get(self) -> PinFunction:
        """Gets the function of the GPIO. The value might be cached,
            see MCP2210.gpio_immediate

        Returns:
            PinFunction: Function of the GPIO
        """
        self.device.gpio_settings_get(False)
        return self._function

    function = property(function_get, function_set, 
        doc="Sets or gets the function of the GPIO. The value might be cached."
    )

    def value_set(self, value: bool):
        """Sets the output value of the GPIO. The value might be cached,
            see MCP2210.gpio_immediate

        Args:
            value (bool): Output value of the GPIO
        """
        self._value = value
        self.device.gpio_val_set(False)
    
    def value_get(self) -> bool:
        """Gets the input value of the GPIO. The value might be cached,
            see MCP2210.gpio_immediate

        Returns:
            bool: Input value of the GPIO
        """
        self.device.gpio_val_get(False)
        return self._value

    value = property(value_get, value_set,
        doc="Sets or gets the value of the GPIO. The value might be cached."
    )

    def direction_in_set(self, value: bool):
        """Sets the direction of the pin. The value might be cached,
            see MCP2210.gpio_immediate

        Args:
            value (bool): True for input, False for output
        """
        self._direction_in = value
        self.device.gpio_dir_set(False)
    
    def direction_in_get(self) -> bool:
        """Gets the direction of the pin. The value might be cached,
            see MCP2210.gpio_immediate

        Returns:
            bool: True for input, False for output
        """
        self.device.gpio_dir_get(False)
        return self._direction_in

    direction_in = property(direction_in_get, direction_in_set,
        doc="Sets or gets the direction of the pin. True for input, False for output."
        "Might be cached."
    )

class SpiChannel():
    def __init__(self, device: MCP2210, chipselect_pin: Gpio, 
        bitrate: int = 100000, spi_mode: SpiMode = SpiMode.MODE0):
        """Creates a new SpiChannel object

        Args:
            device (MCP2210): MCP2210 object the SpiChannel is assigned to
            chipselect_pin (Gpio): GPIO that is used as Chipselect-pin
            bitrate (int, optional): Bitrate in bit/s. Defaults to 100000.
            spi_mode (SpiMode, optional): SPI mode of the channel. 
                Defaults to SpiMode.MODE0.

        Raises:
            Exception: When chipselect pin is not a Gpio-object
        """
        
        self.device = device
        if not isinstance(chipselect_pin, Gpio):
            raise Exception("chipselect_pin must be Gpio")
        
        #TODO: check if chipselect_pin is member of device
        self.chipselect_pin = chipselect_pin

        self.bitrate = bitrate

        self.delay_cs_data = 0
        self.delay_data = 0
        self.delay_data_cs = 0

        self.spi_mode = spi_mode

    def transfersettings_set(self, data_len: int):
        """Writes the transfer setting to the bridge

        Args:
            data_len (int): length of the data to be transferred

        Returns:
            mcp2210_msgs.SpiTransferSettingsCmd: reply of the command
            False: command was not successful
        """
        cmd = mcp2210_msgs.SpiTransferSettingsCmd()
        cmd.bitrate = int(self.bitrate)

        cs_idle = 0
        cs_active = 0

        for i, gpio in enumerate(self.device.gpio):
            cs_idle |= (1 if gpio.chipselect_idle is True else 0) << i
            if gpio == self.chipselect_pin:
                cs_active |= (1 if gpio.chipselect_active is True else 0) << i
            else:
                cs_active |= (1 if gpio.chipselect_idle is True else 0) << i
        
        cmd.chipselect_idle = cs_idle
        cmd.chipselect_active = cs_active

        cmd.delay_cs_data = self.delay_cs_data
        cmd.delay_data_cs = self.delay_data_cs
        cmd.delay_data = self.delay_data
        cmd.spi_mode = self.spi_mode.value
        
        cmd.data_len = data_len

        resp = cmd.send(self.device)
        if resp is False:
            return False
        return True

    def transfer_cancel(self) -> bool:
        """Cancels the ongoing transfer

        Returns:
            bool: True if the command was successful, False if not
        """
        cmd = mcp2210_msgs.SpiTransferCancelCmd()
        return cmd.send(self.device)

    def transfer(self, tx_data: bytearray) -> bytearray:
        """transfers the data to and from the connected device

        Args:
            tx_data (bytearray): data to be transferred to the attached device

        Raises:
            Exception: Transfer data length is too long
            Exception: SPI channel could not be reset
            Exception: Could not set spi settings
            Exception: Bus owned by external master
            Exception: Other error
            Exception: Too many retries for transmission

        Returns:
            bytearray: data transferred from the attached device
        """
        datalen = len(tx_data)

        if datalen > 0xFFFF:
            raise Exception("Transfer data length is too long")

        if self.transfer_cancel() != True:
            raise Exception("SPI channel could not be reset")

        if self.transfersettings_set(datalen) != True:
            raise Exception("Could not set spi settings")

        tx_offset = 0
        rx_offset = 0
        tx_retries = 0
        tx_prepare = True
        tx_blocklen = 0
        rx_data = bytearray()

        cmd = mcp2210_msgs.SpiTransferDataCmd()
        #tx_start = time.time()

        while tx_offset < datalen or rx_offset < datalen:
            if tx_prepare:
                tx_blocklen = min(60, datalen - tx_offset)
                cmd.data = tx_data[tx_offset : tx_offset + tx_blocklen]
                tx_prepare = False

            resp = cmd.send(self.device)
            
            #TODO: sleep during transfers to minimize hardware access

            if resp == Statuscode.SPI_NOT_READY:
                raise Exception("Bus owned by external master")
            elif resp == Statuscode.TRANSFER_IN_PROGRESS:
                tx_retries += 1
            elif isinstance(resp, mcp2210_msgs.SpiTransferDataResp):
                #print("retries: %u, %f ms" % (tx_retries, (time.time() - tx_start) * 1000))
                #tx_start = time.time()
                tx_retries = 0
                tx_offset += tx_blocklen
                tx_prepare = True

                if resp.datalength > 0:
                    rx_data.extend(resp.data)
                    rx_offset += resp.datalength
            else:
                raise Exception("Other error: %s" % resp)

            #print("tx: %u, rx: %u" % (tx_offset, rx_offset))

            if tx_retries > 300:
                raise Exception("Too many retries for transmission")

        #print("retries: %u, %f ms" % (tx_retries, (time.time() - tx_start) * 1000))

        return rx_data

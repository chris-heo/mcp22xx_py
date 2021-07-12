import ctypes
from .mcp2210_enums import Commands, Statuscode


def printstruct(s):
    print("<%s>" % s.__class__.__name__)
    for f in s._fields_:
        field_name = f[0]
        field_type = f[1]
        field_info = getattr(type(s), field_name)
        value = getattr(s, field_name)
        valstr = "<?>"
        if isinstance(value, ctypes.Array):
            elemcnt = len(value)
            elemlen = (field_info.size / elemcnt) * 2
            fmt = "%%0%uX" % elemlen
            valstr = "0x[" + ' '.join([fmt % e for e in value]) + "]"
        elif field_type is ctypes.c_ubyte:
            valstr = "0x%02X (%u)" % (value, value)
        elif field_type is ctypes.c_ushort:
            valstr = "0x%04X (%u)" % (value, value)
        elif field_type is ctypes.c_uint32:
            valstr = "0x%08X (%u)" % (value, value)
        
        print("%20s: %s" % (field_name, valstr))

# https://stackoverflow.com/questions/34533409/serializing-a-c-struct-in-python-and-sending-over-a-socket

class GenericReq(ctypes.LittleEndianStructure):
    """Generic MCP2210 Request"""

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("reserved2", ctypes.c_uint8 * 61),
    ]

    def __init__(self, cmd):
        if isinstance(cmd, Commands):
            cmd = cmd.value
        elif isinstance(cmd, int):
            pass
        else:
            raise Exception("Invalid command type")

        super(type(self), self).__init__(
            command=cmd,
        )

class NvramChipsettingsReq(ctypes.LittleEndianStructure):
    """Set Chip NVRAM Parameters - Set Chip Settings Power-up Default"""
    COMMAND = Commands.SET_NVRAM_SETTINGS
    _pack_ = 1
    SUBCOMMAND = 0x20

    _fields_ = [
        ("command", ctypes.c_uint8),
        ("subcommand", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("gpio_designation", ctypes.c_uint8 * 8),
        ("gpio_default", ctypes.c_uint16),
        ("gpio_direction", ctypes.c_uint16),
        ("chipsettings", ctypes.c_uint8),
        ("nvram_param", ctypes.c_uint8),
        ("password", ctypes.c_uint8 * 8),
        ("reserved2", ctypes.c_uint8 * 37),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
            subcommand=self.SUBCOMMAND,
        )

# class nvramset_res1(ctypes.LittleEndianStructure):
#     
# #     _fields_ = [
# 
#     _pack_ = 1        ("command", ctypes.c_uint8), # 0x60
#         ("status", ctypes.c_uint8), # 0xFB
#         ("reserved0", ctypes.c_uint8 * 62),
#     ]
# 
# class nvramset_res2(ctypes.LittleEndianStructure):
#     _fields_ = [
# #         ("command", ctypes.c_uint8), # 0x60
# 
#     _pack_ = 1        ("success", ctypes.c_uint8), # 0x00
#         ("subcommand", ctypes.c_uint8), # 0x20
#         ("reserved0", ctypes.c_uint8 * 61),
#     ]

class NvramSpitransfersettingsReq(ctypes.LittleEndianStructure):
    """Set Chip NVRAM Parameters - Set SPI Power-up Transfer Settings"""
    COMMAND = Commands.SET_NVRAM_SETTINGS
    _pack_ = 1
    SUBCOMMAND = 0x10

    _fields_ = [
        ("command", ctypes.c_uint8),
        ("subcommand", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("bitrate", ctypes.c_uint32), # bitrate in b/s
        ("chipselect_idle", ctypes.c_uint16),
        ("chipselect_active", ctypes.c_uint8),
        ("delay_cs_data", ctypes.c_uint16), # delay in 100 µs steps from cs active to first data bit
        ("delay_data_cs", ctypes.c_uint16), # delay in 100 µs steps from last data bit to cs idle
        ("delay_data", ctypes.c_uint16), # delay in 100 µs steps between data bytes
        ("data_len", ctypes.c_uint16), # bytes to transfer per SPI transaction
        ("spi_mode", ctypes.c_uint8), # mode 0 ... 3
        ("reserved2", ctypes.c_uint8 * 43)
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
            subcommand=self.SUBCOMMAND
        )

class NnvramUsbparamsReq(ctypes.LittleEndianStructure):
    """Set Chip NVRAM Parameters - Set USB Power-up Key Parameters"""
    COMMAND = Commands.SET_NVRAM_SETTINGS
    _pack_ = 1
    SUBCOMMAND = 0x30

    _fields_ = [
        ("command", ctypes.c_uint8),
        ("subcommand", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("vendor_id", ctypes.c_uint16),
        ("product_id", ctypes.c_uint16),
        ("poweroptions", ctypes.c_uint8),
        ("current", ctypes.c_uint8), # in steps of 2 mA
        ("reserved3", ctypes.c_uint8 * 54),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
            subcommand=self.SUBCOMMAND,
        )

class SpiTransferSettingsCmd(ctypes.LittleEndianStructure):
    """Set (VM) SPI Transfer Settings"""
    COMMAND = Commands.SET_SPI_SETTINGS

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("reserved2", ctypes.c_uint8),
        ("bitrate", ctypes.c_uint32), # bitrate in b/s
        ("chipselect_idle", ctypes.c_uint16),
        ("chipselect_active", ctypes.c_uint16),
        ("delay_cs_data", ctypes.c_uint16), # delay in 100 µs steps from cs active to first data bit
        ("delay_data_cs", ctypes.c_uint16), # delay in 100 µs steps from last data bit to cs idle
        ("delay_data", ctypes.c_uint16), # delay in 100 µs steps between data bytes
        ("data_len", ctypes.c_uint16), # bytes to transfer per SPI transaction
        ("spi_mode", ctypes.c_uint8), # mode 0 ... 3
        ("reserved2", ctypes.c_uint8 * 43)
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        )

    def send(self, dev):
        rbuf, status = dev.send_command(bytearray(self))
        if status != Statuscode.SUCCESS:
            return False
        
        return SpiTransferSettingsCmd.from_buffer(rbuf)

class VmGpioSettingsReq(GenericReq):
    """Get (VM) Current Chip Settings"""
    COMMAND = Commands.GET_GPIO_SETTINGS

    def __init__(self):
        #super(__class__, self).__init__()
        self.command = self.COMMAND.value

    def send(self, dev):
        rbuf, status = dev.send_command(bytearray(self))
        if status != Statuscode.SUCCESS:
            return False
        
        return VmGpioSettingsResp.from_buffer(rbuf)

class VmGpioSettingsResp(ctypes.LittleEndianStructure):
    """Set (VM) Current Chip Settings"""
    COMMAND = Commands.GET_GPIO_SETTINGS

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("status", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("gpio_designation", ctypes.c_uint8 * 9),
        ("gpio_default", ctypes.c_uint16),
        ("gpio_direction", ctypes.c_uint16),
        ("chipsettings", ctypes.c_uint8),
        ("nvram_param", ctypes.c_uint8),
        ("reserved2", ctypes.c_uint8 * 45),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        ),

class VmGpioSettingsCmd(ctypes.LittleEndianStructure):
    """Set (VM) Current Chip Settings"""
    COMMAND = Commands.SET_GPIO_SETTINGS

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("status", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("gpio_designation", ctypes.c_uint8 * 9),
        ("gpio_default", ctypes.c_uint16),
        ("gpio_direction", ctypes.c_uint16),
        ("chipsettings", ctypes.c_uint8),
        ("nvram_param", ctypes.c_uint8),
        ("password", ctypes.c_uint8 * 8),
        ("reserved3", ctypes.c_uint8 * 37),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        ),

    def send(self, dev):
        rbuf, status = dev.send_command(bytearray(self))
        if status != Statuscode.SUCCESS:
            return False
        
        return True

class VmGpioGetDirCmd(GenericReq):
    COMMAND = Commands.GET_GPIO_VALUE

    def __init__(self):
        #super(__class__, self).__init__()
        self.command = self.COMMAND.value

    def send(self, dev):
        rbuf, status = dev.send_command(bytearray(self))
        if status != Statuscode.SUCCESS:
            return False
        
        return VmGpioSetDirCmd.from_buffer(rbuf)

class VmGpioSetDirCmd(ctypes.LittleEndianStructure):
    """Set (VM) GPIO Current Pin Direction"""
    COMMAND = Commands.SET_GPIO_DIR

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("reserved2", ctypes.c_uint8),
        ("gpio_direction", ctypes.c_uint16),
        ("reserved3", ctypes.c_uint8 * 58),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        )

    def send(self, dev):
        rbuf, status = dev.send_command(bytearray(self))
        if status != Statuscode.SUCCESS:
            return False
        
        return VmGpioSetDirCmd.from_buffer(rbuf)

class VmGpioGetValCmd(GenericReq):
    COMMAND = Commands.GET_GPIO_VALUE

    def __init__(self):
        #super(__class__, self).__init__()
        self.command = self.COMMAND.value

    def send(self, dev):
        rbuf, status = dev.send_command(bytearray(self))
        if status != Statuscode.SUCCESS:
            return False
        
        return VmGpioSetValCmd.from_buffer(rbuf)

class VmGpioSetValCmd(ctypes.LittleEndianStructure):
    """Set (VM) GPIO Current Pin Value"""
    COMMAND = Commands.SET_GPIO_VALUE

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("reserved2", ctypes.c_uint8),
        ("gpio_value", ctypes.c_uint16),
        ("reserved3", ctypes.c_uint8 * 58),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        )
    
    def send(self, dev):
        rbuf, status = dev.send_command(bytearray(self))
        if status != Statuscode.SUCCESS:
            return status
        
        return VmGpioSetValCmd.from_buffer(rbuf)


class IrqEventStatusCmd(ctypes.LittleEndianStructure):
    """Get (VM) the Current Number of Events From the Interrupt Pin Request"""
    COMMAND = Commands.GET_EVENT_COUNT

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("noreset", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8 * 62),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        )

    def send(self, dev):
        rbuf, status = dev.send_command(bytearray(self))
        if status != Statuscode.SUCCESS:
            return status
        
        return IrqEventStatusResp.from_buffer(rbuf)

class IrqEventStatusResp(ctypes.LittleEndianStructure):
    """Get (VM) the Current Number of Events from the Interrupt Pin Response"""
    COMMAND = Commands.GET_EVENT_COUNT

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("status", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("event_count", ctypes.c_uint16),
        ("reserved2", ctypes.c_uint8 * 58),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        )

class SpiTransferDataCmd(ctypes.LittleEndianStructure):
    """Transfer SPI Data Request"""
    COMMAND = Commands.SPI_TRANSFER

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("datalength", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("_data", ctypes.c_uint8 * 60),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        )

    def data_set(self, data):
        self.datalength = len(data)
        self._data = (ctypes.c_ubyte * 60)(*data)

    def data_get(self):
        return self._data[0 : self.datalength]

    data = property(data_get, data_set)

    def send(self, dev):
        rbuf, status = dev.send_command(bytearray(self))
        if status != Statuscode.SUCCESS:
            return status
        
        return SpiTransferDataResp.from_buffer(rbuf)

class SpiTransferDataResp(ctypes.LittleEndianStructure):
    """Transfer SPI Data Response"""
    COMMAND = Commands.SPI_TRANSFER

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("status", ctypes.c_uint8),
        ("datalength", ctypes.c_uint8),
        ("engine_status", ctypes.c_uint8),
        ("_data", ctypes.c_uint8 * 60),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        )

    def data_get(self):
        return self._data[0 : self.datalength]

    data = property(data_get)

class SpiTransferCancelCmd(ctypes.LittleEndianStructure):
    """Cancel the current SPI transfer Request"""
    COMMAND = Commands.SPI_CANCEL

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("reserved2", ctypes.c_uint8 * 61),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        )

    def send(self, dev):
        rbuf, status = dev.send_command(bytearray(self))
        if status != Statuscode.SUCCESS:
            return False
        
        return True

class SpiTransferCancelResp1(ctypes.LittleEndianStructure):
    """Cancel the current SPI transfer Response"""
    COMMAND = [ 0x11, 0x10 ]

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("status", ctypes.c_uint8),
        ("spi_released", ctypes.c_uint8),
        ("spi_owner", ctypes.c_uint8),
        ("password_accesses", ctypes.c_uint8),
        ("password_guessed", ctypes.c_uint8),
        ("reserved2", ctypes.c_uint8 * 58),
    ]

class SpiBusReleaseCmd(ctypes.LittleEndianStructure):
    """Request SPI bus Release"""
    COMMAND = Commands.SPI_RELEASE

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8 * 61),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        )

class ChipStatusReq(ctypes.LittleEndianStructure):
    """Get MCP2210 Status"""
    COMMAND = Commands.GET_CHIP_STATUS

    _pack_ = 1
    _fields_ = [
        ("command", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reserved1", ctypes.c_uint8),
        ("reserved2", ctypes.c_uint8 * 61),
    ]

    def __init__(self):
        super(type(self), self).__init__(
            command=self.COMMAND.value,
        )
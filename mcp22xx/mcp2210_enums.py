from enum import Enum

class Commands(Enum):
    GET_CHIP_STATUS = 0x10
    SPI_CANCEL = 0x11
    GET_EVENT_COUNT = 0x12
    GET_GPIO_SETTINGS = 0x20
    SET_GPIO_SETTINGS = 0x21
    SET_GPIO_VALUE = 0x30
    GET_GPIO_VALUE = 0x31
    SET_GPIO_DIR = 0x32
    GET_GPIO_DIR = 0x33
    SET_SPI_SETTINGS = 0x40
    GET_SPI_SETTINGS = 0x41
    SPI_TRANSFER = 0x42
    READ_EEPROM = 0x50
    WRITE_EEPROM = 0x51
    SET_NVRAM_SETTINGS = 0x60
    GET_NVRAM_SETTINGS = 0x61
    SEND_ACCESS_PASSWORD = 0x70
    SPI_RELEASE = 0x80

class Statuscode(Enum):
    SUCCESS = 0x00
    SPI_NOT_READY = 0xF7
    TRANSFER_IN_PROGRESS = 0xF8
    UNKNOWN_COMMAND = 0xF9
    EEPROM_ERROR = 0xFB
    ACCESS_REJECTED = 0xFC
    WRONG_PASSWORD = 0xFD

    UNKNOWN_ERROR = -1

    def _missing_(cls, value):
        return cls.UNKNOWN_ERROR

class SpiMode(Enum):
    MODE0 = 0 # CPOL = 0, CPHA = 0 (clock idle low, data on first edge)
    MODE1 = 1 # CPOL = 0, CPHA = 1 (clock idle low, data on second edge)
    MODE2 = 2 # CPOL = 1, CPHA = 0 (clock idle high, data on second edge)
    MODE3 = 3 # CPOL = 1, CPHA = 1 (clock idle high, data on second edge)

class PinFunction(Enum):
    """GPx Pin Designation"""
    GPIO = 0x00
    CHIP_SELECT = 0x01
    DEDICATED = 0x02

    #@classmethod
    #def _missing_(cls, value):
    #    raise ValueError("%r is not a valid %s. Please report this" % (value, cls.__name__))

class IrqPinMode(Enum):
    """Interrupt Pin mode"""
    NO_COUNTING = 0
    FALLING_EDGES = 1
    RISING_EDGES = 2
    LOW_PULSES = 3
    HIGH_PULSES = 4

class ChipProtection(Enum):
    OFF = 0x00
    PASSWORD = 0x40
    LOCKED = 0x80

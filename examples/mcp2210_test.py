import time

from mcp22xx.mcp2210 import MCP2210, SpiChannel
from mcp22xx.mcp2210_enums import PinFunction, IrqPinMode, SpiMode

dev = MCP2210.get_device()

def test_spi(dev, csidx, bitrate, mode, bytes):
    cs = dev.gpio[csidx]
    cs.function = PinFunction.CHIP_SELECT
    spi = SpiChannel(dev, cs, bitrate, mode)
    spi.bitrate = int(1e6)

    txdata = bytearray()

    for i in range(bytes):
        txdata.append(i & 0xFF)

    start = time.time()
    rxdata = spi.transfer(txdata)
    dur = time.time() - start

    print(rxdata)

    print(dur)

def test_output(dev, ioidx):
    pin = dev.gpio[ioidx]
    pin.function = PinFunction.GPIO
    pin.direction_in = False

    state = False
    for i in range(20):
        state = not state
        pin.value = state
        time.sleep(0.1)

def test_input(dev, ioidx):
    pin = dev.gpio[ioidx]
    pin.function = PinFunction.GPIO
    pin.direction_in = True

    for i in range(40):
        print(pin.value)
        time.sleep(0.05)


def test_irq(dev, mode):
    irq = dev.gpio[6]
    irq.direction_in = True
    irq.function = PinFunction.DEDICATED
    dev.irq_mode = mode
    dev.irq_get_events(True)
    for i in range(200):
        print(dev.irq_get_events())
        time.sleep(0.1)

dev.immediate = False
dev.gpio_settings_get()

def highlow(value, text="", truetext="High", falsetext="Low"):
    if value is True:
        text += truetext
    else:
        text += falsetext
    return text

for gpio in dev.gpio:
    print("GPIO%u:" % gpio.index)
    print("%12s: %s" % ("Function", str(gpio.function)))
    print("%12s: %s" % ("Direction", highlow(gpio.direction_in, "", "Input", "Output")))
    print("%12s: %s" % ("Value", highlow(gpio.value)))
    print("%12s: %s" % ("Chip Select", highlow(gpio.chipselect_idle, "Idle: ") + " " + highlow(gpio.chipselect_active, "Active: ")))

dev.immediate = True

test_spi(dev, 2, 1e6, SpiMode.MODE0, 60*100)
test_output(dev, 7)
test_input(dev, 6)
test_irq(dev, IrqPinMode.RISING_EDGES)
# -*- coding: utf-8 -*-
"""Hellschreiber
   Ok, not exactly but you'll get the point. And the name is kinda cool ;-)
   Connect a digital oscilloscope with rather slow timebase (0.5 s/div)
   and around 1 V/div. Connect the probe to GP2. Run the script and wait.
"""

import time
from mcp22xx import MCP2221, Gpio2Func, ReferenceVoltage

def main(txt):
    font = [
        [0x00,                         ], [0x5F,                         ],
        [0x03, 0x00, 0x03,             ], [0x14, 0x7F, 0x14, 0x7F, 0x14, ],
        [0x4C, 0x5A, 0xFF, 0x32, 0x00, ], [0x63, 0x1B, 0x6C, 0x63, 0x00, ],
        [0x36, 0x49, 0x56, 0x20, 0x50, ], [0x03,                         ],
        [0x3E, 0x41,                   ], [0x41, 0x3E,                   ],
        [0x08, 0x2A, 0x1C, 0x2A, 0x08, ], [0x08, 0x08, 0x3E, 0x08, 0x08, ],
        [0x80, 0x40,                   ], [0x08, 0x08, 0x08, 0x08,       ],
        [0x40,                         ], [0x60, 0x1C, 0x03,             ],
        [0x3E, 0x41, 0x41, 0x3E,       ], [0x00, 0x02, 0x7F, 0x00,       ],
        [0x71, 0x49, 0x49, 0x46,       ], [0x41, 0x49, 0x49, 0x36,       ],
        [0x18, 0x14, 0x12, 0x7F,       ], [0x47, 0x45, 0x45, 0x38,       ],
        [0x3E, 0x49, 0x49, 0x30,       ], [0x01, 0x71, 0x0D, 0x03,       ],
        [0x36, 0x49, 0x49, 0x36,       ], [0x06, 0x49, 0x49, 0x3E,       ],
        [0x24,                         ], [0x80, 0x48,                   ],
        [0x08, 0x14, 0x22, 0x41,       ], [0x14, 0x14, 0x14, 0x14,       ],
        [0x41, 0x22, 0x14, 0x08,       ], [0x00, 0x01, 0x59, 0x05, 0x02, ],
        [0x3E, 0x49, 0x55, 0x5D, 0x2E, ], [0x7E, 0x05, 0x05, 0x7E,       ],
        [0x7F, 0x45, 0x45, 0x3A,       ], [0x3E, 0x41, 0x41, 0x41,       ],
        [0x7F, 0x41, 0x41, 0x3E,       ], [0x7F, 0x45, 0x45, 0x41,       ],
        [0x7F, 0x05, 0x05, 0x01,       ], [0x3E, 0x41, 0x45, 0x7D,       ],
        [0x7F, 0x04, 0x04, 0x7F,       ], [0x7F,                         ],
        [0x40, 0x40, 0x3F,             ], [0x7F, 0x14, 0x22, 0x41,       ],
        [0x7F, 0x40, 0x40, 0x40,       ], [0x7F, 0x02, 0x0C, 0x02, 0x7F, ],
        [0x7F, 0x02, 0x04, 0x08, 0x7F, ], [0x3E, 0x41, 0x41, 0x41, 0x3E, ],
        [0x7F, 0x11, 0x11, 0x0E,       ], [0x3E, 0x41, 0x61, 0x41, 0xBE, ],
        [0x7F, 0x11, 0x11, 0x6E,       ], [0x46, 0x49, 0x49, 0x31,       ],
        [0x01, 0x01, 0x7F, 0x01, 0x01, ], [0x3F, 0x40, 0x40, 0x3F,       ],
        [0x07, 0x18, 0x60, 0x18, 0x07, ], [0x7F, 0x20, 0x18, 0x20, 0x7F, ],
        [0x63, 0x14, 0x08, 0x14, 0x63, ], [0x03, 0x04, 0x78, 0x04, 0x03, ],
        [0x71, 0x49, 0x45, 0x43,       ], [0x7F, 0x41,                   ],
        [0x03, 0x1C, 0x60,             ], [0x41, 0x7F,                   ],
        [0x04, 0x02, 0x01, 0x02, 0x04, ], [0x40, 0x40, 0x40, 0x40,       ],
        [0x01, 0x02,                   ], [0x20, 0x54, 0x54, 0x78,       ],
        [0x7F, 0x44, 0x44, 0x38,       ], [0x38, 0x44, 0x44, 0x44,       ],
        [0x38, 0x44, 0x44, 0x7F,       ], [0x38, 0x54, 0x54, 0x18,       ],
        [0x7E, 0x09,                   ], [0x18, 0xA4, 0xA4, 0x7C,       ],
        [0x7F, 0x04, 0x04, 0x78,       ], [0x7D,                         ],
        [0x40, 0x80, 0x84, 0x7D,       ], [0x7F, 0x10, 0x28, 0x44,       ],
        [0x7F,                         ], [0x7C, 0x04, 0x7C, 0x04, 0x78, ],
        [0x7C, 0x04, 0x04, 0x78,       ], [0x38, 0x44, 0x44, 0x38,       ],
        [0xFC, 0x24, 0x24, 0x18,       ], [0x18, 0x24, 0x24, 0xFC,       ],
        [0x7C, 0x08, 0x04,             ], [0x48, 0x54, 0x24,             ],
        [0x3F, 0x44,                   ], [0x3C, 0x40, 0x40, 0x7C,       ],
        [0x1C, 0x20, 0x40, 0x20, 0x1C, ], [0x1C, 0x60, 0x18, 0x60, 0x1C, ],
        [0x6C, 0x10, 0x10, 0x6C,       ], [0x1C, 0xA0, 0xA0, 0x7C,       ],
        [0x64, 0x54, 0x4C, 0x44,       ]
    ]

    # voltage levels for different bits
    vals = [22, 20, 18, 16, 14, 12, 10, 8]

    device = MCP2221.get_device()
    device.gpios[2].set_function(Gpio2Func.DAC1)
    device.dac_setreference(ReferenceVoltage.Int4096mV)
    device.sram_write_settings()

    outdata = []
    for c in txt:
        outdata.extend(font[ord(c)-32])
        outdata += [0]

    for col in outdata:
        for reps in range(4):
            curcol = col
            for y in range(8):
                if curcol & 1 == 1:
                    device.dac_setvalue(vals[y])
                else:
                    device.dac_setvalue(0)
                curcol >>= 1
                time.sleep(0.003)

if __name__ == "__main__":
    main("hobbyelektronik.org")

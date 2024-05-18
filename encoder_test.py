#!/usr/bin/env python3
import colorsys
import time
from colorama import init, Fore, Style
init()

import ioexpander as io
from ioexpander.encoder import Encoder

from core.logger import Level, Logger
from hardware.rotary_encoder import RotaryEncoder, Selector
from hardware.color import Color

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

COLORS = [ Color.GREEN, Color.RED, Color.MAGENTA, Color.BLUE ]

def selectServo(n, rot):
    color = COLORS[n]
    rot.set_rgb(color.rgb)

_rot = None

try:

    _rot = RotaryEncoder(Level.INFO)
    _selector = Selector(3, Level.INFO)

    while True:
        _value = _selector.get_value(_rot.value())
        selectServo(_value, _rot)

        time.sleep(50 / 1000)

except KeyboardInterrupt:
    print('Ctrl-C caught; exiting...')
except Exception as e:
    print('{} encountered, exiting: {}'.format(type(e), e))
finally:
    if _rot:
        _rot.close()



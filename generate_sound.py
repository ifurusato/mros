#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-04-30
# modified: 2024-05-24
#
# Generates the 'hardware/sound.py' file and './sounds.json metadata' file using
# the './sounds-source.json' JSON file and 'hardware/sounds-template.py' as a
# template. The sounds.json is then copied to the ESP32 so that its 'main.py'
# can use it to determine which sound to play.
#
# See: https://github.com/pimoroni/ioe-python/blob/master/REFERENCE.md#function-reference
#

import time
import datetime as dt
import json

from core.stringbuilder import StringBuilder
from colorama import init, Fore, Style
init(autoreset=True)

from upy.sound import Sound

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_enum_defs = StringBuilder()
_order = StringBuilder()
_sounds = Sound.load_sounds('sounds-source.json')

for _sound in _sounds:
    _enum = _sound.as_enum()
#   print(_enum)
    _enum_defs.append(_enum + '\n')
    _order.append(_sound.mnemonic)
    _order.append(' ')

with open('hardware/sound-template.py', 'r') as _template_file:
    _template = _template_file.read()
    _timestamp = dt.datetime.now().strftime('%Y-%m-%d')
    _template = _template.replace('%TIMESTAMP',_timestamp)
    _template = _template.replace('%ORDER',_order.to_string())
    _template = _template.replace('%ENUM',_enum_defs.to_string())

with open('hardware/sound.py', 'w') as _output_file:
    _output_file.write(_template)
    print('wrote file: hardware/sound.py')

print('generating: sounds.json')
from hardware.sound import Sound
Sound.export()

print("export complete, now copy 'sounds.json' to the ESP32.")

#EOF

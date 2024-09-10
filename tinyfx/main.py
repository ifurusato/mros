#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-09-08
# modified: 2024-09-08
#

import utime
from machine import Pin
from tiny_fx import TinyFX
from picofx import MonoPlayer, ColourPlayer
#from picofx.colour import RainbowFX

from pin_set import PinSetFX
from triofx import TrioFX
from rgb_blink import RgbBlinkFX
from picofx.mono import RandomFX, StaticFX, BlinkFX

from color import Color

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

tiny = TinyFX()                         # create a new TinyFX object to interact with the board
player = MonoPlayer(tiny.outputs)       # create a new effect player to control TinyFX's mono outputs
rgb_player = ColourPlayer(tiny.rgb)     # create a new effect player to control TinyFX's RGB output

# set up the effects to play
player.effects = [
#   BlinkFX(speed=0.5, phase=0.0, duty=0.02),
    TrioFX(1, brightness=0.3),
    TrioFX(2, brightness=1.0),
    TrioFX(3, brightness=1.0),
    None,
    None,
    None
]

#   BlinkFX(speed=0.5, phase=0.0, duty=0.02),
#   StaticFX(brightness=1.0),
#   RandomFX(interval=0.01, brightness_min=0.5, brightness_max=1.0),

# create and set up a rainbow effect to play
_colors = [ Color.VERY_DARK_CYAN, Color.VERY_DARK_CYAN, Color.VERY_DARK_CYAN, Color.VERY_DARK_CYAN, Color.RED ]
rgb_player.effects = RgbBlinkFX(speed=1.0, phase=0.0, duty=0.05, color=_colors)

# pair the RGB player with the Mono player so they run in sync
player.pair(rgb_player)

# wrap the code in a try block, to catch any exceptions (including KeyboardInterrupt)
try:

    player.start()   # start the effects running
    # loop until the effect stops or the "Boot" button is pressed
    while player.is_running() and not tiny.boot_pressed():
        utime.sleep(1)

# stop any running effects and turn off all the outputs
finally:
    player.stop()
    tiny.shutdown()

#EOF

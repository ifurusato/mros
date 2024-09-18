#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-18
# modified: 2024-09-13
#
# Itsy Bitsy RP2040 main.
#
# Test file for I2CDriver that uses the callback to print the
# message and set the NeoPixel color.
#

import utime
from machine import Timer
from machine import Pin
from neopixel import Neopixel

from colors import*
from i2c_driver import I2CDriver

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_neopixel = None
_freq_hz  = 0.2

# methods ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def show_color(color):
    '''
    Display the color on the NeoPixel.
    '''
    _neopixel.set_pixel(0, color)
    _neopixel.show()

def poll():
    show_color(COLOR_INDIGO)
    utime.sleep_ms(50)
    show_color(COLOR_BLACK)
    utime.sleep_ms(50)

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# NeoPixel control pin 17
#_neopixel = Neopixel(num_leds=10, state_machine=0, pin=17, mode="RGB")
#_neopixel.brightness(108)
# turn on NeoPixel power pin 16
#_pin16 = Pin(16, Pin.OUT)
#_pin16.value(1)

#_timer = Timer() 
#_timer.init(freq=_freq_hz, mode=Timer.PERIODIC, callback=lambda n: poll())

_driver = I2CDriver()
_driver.enable()

#EOF

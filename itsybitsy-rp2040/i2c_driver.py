#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-18
# modified: 2024-08-18
#
# Extends I2CSlave as a driver file for the Adafruit ItsyBitsy RP2040, using
# its callback to print the message and set the NeoPixel color.
#

import utime
from machine import Pin
from neopixel import Neopixel

from colors import*
from i2c_slave import I2CSlave, I2CSlaveError

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class I2CDriver(I2CSlave):
    '''
    Wraps the functionality of the I2CSlave class with callback support
    for providing visual feedback via a NeoPixel, as found on the Adafruit
    ItsyBitsy RP2040.
    '''
    def __init__(self, i2c_address=0x44, blink=True):
        # NeoPixel control pin 17
        self._neopixel = Neopixel(num_leds=10, state_machine=0, pin=17, mode="RGB")
        self._neopixel.brightness(108)
        # turn on NeoPixel power pin 16
        _pin16 = Pin(16, Pin.OUT)
        _pin16.value(1)
        self._swap_grb = True # if NeoPixel requires GRB instead of RGB
        I2CSlave.__init__(self, i2c_address=i2c_address, blink=blink, callback=self.callback)
        self.message('I2C driver ready.')

    def callback(self, msg, color):
        if msg:
            self.message('response: {}'.format(msg))
        if self.is_visual():
            self.show_color(color)

    def show_color(self, color):
        '''
        Display the color on the NeoPixel.
        '''
        self._neopixel.set_pixel(0, color)
        self._neopixel.show()
        utime.sleep(1)

    def process_payload(self, payload):
        '''
        Override this method to process the payload value in a subclass.
        This uppercases the color value to match the enumerated keys.
        '''
        if payload is None:
            return "empty payload."
        else:
            if payload.startswith('set '):
                try:
                    _key = payload[4:].upper()
                    _color = COLOR[_key]
                    if self._swap_grb:
                        _color = ( _color[1], _color[0], _color[2] )
#                   self.message("overridden process_payload: '{}'; color key: '{}'; color: '{}'".format(payload, _key, _color))
                    self._neopixel.set_pixel(0, _color)
                    self._neopixel.show()
                except KeyError as e:
                    raise I2CSlaveError(I2CSlave.BAD_REQUEST, "unrecognised color name: '{}'".format(_key))
                except Exception as e:
                    raise I2CSlaveError(I2CSlave.BAD_REQUEST, "{} error during payload processing: '{}'".format(type(e), e))
                finally:
                    return "processed payload: '{}'".format(payload)
            else:
                return "unprocessed payload: '{}'".format(payload)

#EOF

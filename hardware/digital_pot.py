#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2021 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-09-19
# modified: 2021-07-21
#

import sys, colorsys, traceback
import ioexpander as io
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from hardware.i2c_scanner import DeviceNotFound

REG_ADCCON0 = 0xa8
REG_PWMCON0 = 0x98

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DigitalPotentiometer(Component):
    '''
    Configures an IO Expander Potentiometer breakout, returning an analog
    value scaled to a specified range. For a center-zero pot simply
    specify the minimum value as (-1.0 * out_max).

    Optional min and max values will resort to the application configuration
    if not provided explicitly as arguments.

    :param config:     The application configuration.
    :param in_min:     (optional, int or float) Minimum input value.
    :param in_max:     (optional, int or float) Maximum input value.
    :param out_min:    (optional, int or float) Minimum output value.
    :param out_max:    (optional, int or float) Maximum output value.
    :param level:      The log level.
    '''
    def __init__(self, config, in_min=None, in_max=None, out_min=None, out_max=None, level=Level.INFO):
#       super().__init__()
        self._log = Logger('digital-pot', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['kros'].get('hardware').get('digital_potentiometer')
        # 0x18 for IO Expander, 0x0E for the potentiometer breakout
#       self._i2c_addr = 0x0E
        self._i2c_addr   = _cfg.get('i2c_address')
        self._pin_red    = _cfg.get('pin_red')
        self._pin_green  = _cfg.get('pin_green')
        self._pin_blue   = _cfg.get('pin_blue')
        self._log.info("pins: red: {}; green: {}; blue: {}".format(self._pin_red, self._pin_green, self._pin_blue))
        self._pot_enc_a  = 12
        self._pot_enc_b  = 3
        self._pot_enc_c  = 11
        self._max_value  = 3.3                     # maximum voltage (3.3v supply)
        self._brightness = _cfg.get('brightness')  # effectively max fraction of period LED will be on
        self._period = int(255 / self._brightness) # add a period large enough to get 0-255 steps at the desired brightness
        # minimum and maximum analog values from IO Expander
        _in_min          = _cfg.get('in_min') if in_min is None else in_min
        _in_max          = _cfg.get('in_max') if in_max is None else in_max
        self.set_input_limits(_in_min, _in_max)
        # minimum and maximum scaled output values
        _out_min         = _cfg.get('out_min') if out_min is None else out_min
        _out_max         = _cfg.get('out_max') if out_max is None else out_max
        self.set_output_limits(_out_min, _out_max)
        # now configure IO Expander
        self._log.info("🥝 configuring IO Expander...")
        try:
            self._ioe = io.IOE(i2c_addr=self._i2c_addr)
            self._ioe.set_mode(self._pot_enc_a, io.PIN_MODE_PP)
            self._ioe.set_mode(self._pot_enc_b, io.PIN_MODE_PP)
            self._ioe.set_mode(self._pot_enc_c, io.ADC)
            self._ioe.output(self._pot_enc_a, 1)
            self._ioe.output(self._pot_enc_b, 0)
            self._ioe.set_pwm_period(self._period)
            self._ioe.set_pwm_control(divider=2)  # PWM as fast as we can to avoid LED flicker
            self._ioe.set_mode(self._pin_red,   io.PWM, invert=True)
            self._ioe.set_mode(self._pin_green, io.PWM, invert=True)
            self._ioe.set_mode(self._pin_blue,  io.PWM, invert=True)

#           _result = self._ioe.get_bit(REG_ADCCON0, 7)
            _result = self._ioe.get_bit(REG_PWMCON0, 6)
            print('REG_PWMCON0: {}'.format(_result))

        except FileNotFoundError:
            raise DeviceNotFound("unable to initialise potentiometer: no device found.")
        except Exception as e:
#           self._log.warning(Fore.BLACK + "unable to initialise IO Expander: {}\n{}".format(e, traceback.format_exc()))
            raise DeviceNotFound("{} error initialising potentiometer: {}".format(type(e), traceback.format_exc()))

        self._log.info("running LED with {} brightness steps.".format(int(self._period * self._brightness)))
        self._log.info("ready.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_input_limits(self, in_min, in_max):
        '''
        Used to change the input minimum and maximum values.
        This accepts either int or float arguments.
        '''
        if isinstance(in_min, int):
            in_min = float(in_min)
        if not isinstance(in_min, float):
            raise ValueError('wrong type for in_min argument: {}'.format(type(in_min)))
        self._in_min = in_min

        if isinstance(in_max, int):
            in_max = float(in_max)
        if not isinstance(in_max, float):
            raise ValueError('wrong type for in_max argument: {}'.format(type(in_max)))
        self._in_max = in_max
        self._log.info('input range:\t{:>5.2f}-{:<5.2f}'.format(self._in_min, self._in_max))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_output_limits(self, out_min, out_max):
        '''
        Used to change the output minimum and maximum values.
        This accepts either int or float arguments.
        '''
        if isinstance(out_min, int):
            out_min = float(out_min)
        if not isinstance(out_min, float):
            raise ValueError('wrong type for out_min argument: {}'.format(type(out_min)))
        self._out_min = out_min
        if isinstance(out_max, int):
            out_max = float(out_max)
        if not isinstance(out_max, float):
            raise ValueError('wrong type for out_max argument: {}'.format(type(out_max)))
        self._out_max = out_max
        self._log.info('output range:\t{:>5.2f}-{:<5.2f}'.format(self._out_min, self._out_max))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def value(self):
        if self.disabled or not self._ioe:
            return 0.0
        _value = self._max_value - self._ioe.input(self._pot_enc_c)
        self._log.debug('raw value: {:<5.2f}'.format(_value))
        return _value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_white(self):
        if self._ioe:
            self._ioe.output(self._pin_red, 255)
            self._ioe.output(self._pin_green, 255)
            self._ioe.output(self._pin_blue, 255)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_black(self):
        if self._ioe:
            self._ioe.output(self._pin_red, 0)
            self._ioe.output(self._pin_green, 0)
            self._ioe.output(self._pin_blue, 0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_rgb(self, value):
        if self._ioe:
            h = value / self._max_value # time.time() / 10.0
            r, g, b = [int(c * self._period * self._brightness) for c in colorsys.hsv_to_rgb(h, 1.0, 1.0)]
            self._ioe.output(self._pin_red, r)
            self._ioe.output(self._pin_green, g)
            self._ioe.output(self._pin_blue, b)
            self._log.debug('value: {:<5.2f}; rgb: {},{},{}'.format(value, r, g, b))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_scaled_value(self, update_led=True):
        '''
        Return a scaled value while also updating the RGB LED if the
        argument is True (the default).
        '''
        if self.disabled:
            return 0.0
        _value = self.value
        if update_led:
            self.set_rgb(_value)
        return self.scale_value(_value) # as float

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def scale_value(self, value):
        '''
                   (out_max - out_min)(value - in_min)
            f(x) = -----------------------------------  + out_min
                            in_max - in_min

            where e.g.:  a = 0.0, b = 1.0, min = 0, max = 330.
        '''
        return (( self._out_max - self._out_min ) * ( value - self._in_min ) / ( self._in_max - self._in_min )) + self._out_min


    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __reset(self):
        '''
        Set the display to black carefully, to be used during closing.
        '''
        if self._ioe:
            self._ioe.output(self._pin_red, 0)
        if self._ioe:
            self._ioe.output(self._pin_green, 0)
        if self._ioe:
            self._ioe.output(self._pin_blue, 0)
        return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            _count = 0
            while _count < 10 and not self.__reset():
                self._log.info("[{:d}] waiting for digital potentiometer reset...")
                time.sleep(0.1)
            Component.disable(self)
            self._log.debug('successfully disabled.')
        else:
            self._log.warning("already disabled.")

#EOF

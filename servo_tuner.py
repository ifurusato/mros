#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2024-05-07
#

import sys, traceback

try:
    import pytest
except Exception:
    print("could not find library 'pytest': install with \"sudo pip3 install pytest\"")
    sys.exit(1)

import ioexpander as io
from adafruit_servokit import ServoKit

from datetime import datetime as dt
from math import isclose
from colorama import init, Fore, Style
init()

from core.rate import Rate
from core.orientation import Orientation
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.message_bus import MessageBus
from core.message_factory import MessageFactory
from hardware.color import Color
from hardware.servo_controller import ServoController
from hardware.servo import Servo
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.rotary_encoder import RotaryEncoder, Selector
from hardware.digital_pot import DigitalPotentiometer

_log = Logger('test', Level.INFO)
COLORS  = [ Color.GREEN, Color.RED, Color.MAGENTA, Color.BLUE ]
IN_MIN  = 0.0    # minimum analog value from IO Expander
IN_MAX  = 3.3    # maximum analog value from IO Expander
OUT_MIN = -90.0  # minimum scaled output value
OUT_MAX =  90.0  # maximum scaled output value

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
@pytest.mark.unit
def test_servos():

    _start_time = dt.now()

    _enable_pfwd_servo = True
    _enable_sfwd_servo = True
    _enable_paft_servo = True
    _enable_saft_servo = True

    # TODO: in configuration

    try:

        _level = Level.INFO

        _rot = RotaryEncoder(Level.INFO)
        _selector = Selector(3, Level.INFO)

        # read YAML configuration
        _config = ConfigLoader(Level.INFO).configure()

        _log.info('creating message bus…')
        _message_bus = MessageBus(_config, _level)
        _log.info('creating message factory…')
        _message_factory = MessageFactory(_message_bus, _level)

        # set up servos ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _servo_ctrl = ServoController(_config, _message_bus, level=Level.INFO)
        _pfwd_servo = _servo_ctrl.get_servo(Orientation.PFWD)
        _sfwd_servo = _servo_ctrl.get_servo(Orientation.SFWD)
        _paft_servo = _servo_ctrl.get_servo(Orientation.PAFT)
        _saft_servo = _servo_ctrl.get_servo(Orientation.SAFT)

        # set up digital potentiometer ┈┈┈┈┈┈┈┈┈┈┈
#       _min_value = -1 * (Servo.MAX_RANGE / 2)
#       _max_value = Servo.MAX_RANGE / 2

        _i2c_scanner = I2CScanner(_config, level=_level)
        if _i2c_scanner.has_hex_address(['0x0E']):
            _log.info('using digital potentiometer...')
            _pot = DigitalPotentiometer(_config, level=_level)
            _pot.set_input_range(IN_MIN, IN_MAX)
            _pot.set_output_range(OUT_MIN, OUT_MAX)
        else:
            raise Exception('no digital potentiometer available.')

        _last_scaled_value = 0.0
        _log.info('starting test...')
        _hz = 20
        _rate = Rate(_hz, Level.ERROR)

        while True:
            _selected = _selector.get_value(_rot.value())
            _color = COLORS[_selected]
            _rot.set_rgb(_color.rgb)
            _value = _pot.value
            _scaled_value = _pot.get_scaled_value(False)
            if _scaled_value != _last_scaled_value: # if not the same as last time
                # math.isclose(3, 15, abs_tol=0.03 * 255) # 3% on a 0-255 scale
                if isclose(_scaled_value, 0.0, abs_tol=0.05 * 90):
                    _pot.set_black()
                    _log.debug(Fore.YELLOW + Style.DIM + 'value: {:5.2f}; scaled value: {:5.2f}'.format(_value, _scaled_value))
                else:
                    _pot.set_rgb(_pot.value)
                    _log.debug(Fore.YELLOW + Style.NORMAL + 'value: {:5.2f}; scaled value: {:5.2f}'.format(_value, _scaled_value))
            _last_scaled_value = _scaled_value

            if _selected == 0 and _enable_sfwd_servo:
                _sfwd_servo.angle = int(_scaled_value)
            if _selected == 1 and _enable_pfwd_servo:
                _pfwd_servo.angle = int(_scaled_value)
            if _selected == 2 and _enable_saft_servo:
                _saft_servo.angle = int(_scaled_value)
            if _selected == 3 and _enable_paft_servo:
                _paft_servo.angle = int(_scaled_value)

            _rate.wait()

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting...')
        if _enable_pfwd_servo:
            _pfwd_servo.center()
        if _enable_sfwd_servo:
            _sfwd_servo.center()
        if _enable_paft_servo:
            _paft_servo.center()
        if _enable_saft_servo:
            _saft_servo.center()
    except DeviceNotFound as e:
        _log.error('no potentiometer found, exiting.')
    except Exception as e:
        _log.error('{} encountered, error in test: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        if _rot:
            _rot.close()

    _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
    _log.info(Fore.YELLOW + 'complete: elapsed: {:d}ms'.format(_elapsed_ms))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    try:
        test_servos()
    except Exception as e:
        print(Fore.RED + 'error in motor test: {}'.format(e) + Style.RESET_ALL)
        traceback.print_exc(file=sys.stdout)
    finally:
        pass

if __name__== "__main__":
    main()

#EOF

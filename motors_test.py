#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2021-08-07
#
# Tests the port and starboard motors for directly by setting their power, from
# a digital potentiometer, without the intermediaries of velocity, slew, or PID
# controllers.
#

import pytest
import sys, numpy, time, traceback
from datetime import datetime as dt
from math import isclose
from colorama import init, Fore, Style
init()

from core.orientation import Orientation
from core.rate import Rate
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.motor_configurer import MotorConfigurer
from hardware.motor import Motor
from hardware.analog_pot import AnalogPotentiometer
from hardware.digital_pot import DigitalPotentiometer

_log = Logger('test', Level.INFO)

IN_MIN  = 0.0    # minimum analog value from IO Expander
IN_MAX  = 3.3    # maximum analog value from IO Expander
OUT_MIN = -80.0  # minimum scaled output value
OUT_MAX =  80.0  # maximum scaled output value

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def key_callback(event):
    _log.info('callback on event: {}'.format(event))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
@pytest.mark.unit
def test_motors():

    _use_digital_pot = True

    _pfwd_motor_enabled = True
    _sfwd_motor_enabled = True
    _pmid_motor_enabled = False
    _smid_motor_enabled = False
    _paft_motor_enabled = True
    _saft_motor_enabled = True

    _pfwd_motor = None
    _sfwd_motor = None
    _pmid_motor = None
    _smid_motor = None
    _paft_motor = None
    _saft_motor = None

    _start_time = dt.now()

    try:

        # read YAML configuration
        _level = Level.INFO
        _config = ConfigLoader(Level.INFO).configure() 

        _i2c_scanner = I2CScanner(_config, _level)

        # add motor controller
        _motor_configurer = MotorConfigurer(_config, _i2c_scanner, motors_enabled=True, level=_level)

        if _pfwd_motor_enabled:
            _pfwd_motor = _motor_configurer.get_motor(Orientation.PFWD)
            _pfwd_motor.enable()
        if _sfwd_motor_enabled:
            _sfwd_motor = _motor_configurer.get_motor(Orientation.SFWD)
            _sfwd_motor.enable()
        if _pmid_motor_enabled:
            _pmid_motor = _motor_configurer.get_motor(Orientation.PMID)
            _pmid_motor.enable()
        if _smid_motor_enabled:
            _smid_motor = _motor_configurer.get_motor(Orientation.SMID)
            _smid_motor.enable()
        if _paft_motor_enabled:
            _paft_motor = _motor_configurer.get_motor(Orientation.PAFT)
            _paft_motor.enable()
        if _saft_motor_enabled:
            _saft_motor = _motor_configurer.get_motor(Orientation.SAFT)
            _saft_motor.enable()

        # report on each motor's ThunderBorg:
        if _pfwd_motor_enabled:
            _i2cAddress = _pfwd_motor.tb.I2cAddress
            _log.info(Fore.RED   + 'PFWD I2C address: 0x{:02X}'.format(_i2cAddress))
        if _sfwd_motor_enabled:
            _i2cAddress = _sfwd_motor.tb.I2cAddress
            _log.info(Fore.GREEN + 'SFWD I2C address: 0x{:02X}'.format(_i2cAddress))
        if _pmid_motor_enabled:
            _i2cAddress = _pmid_motor.tb.I2cAddress
            _log.info(Fore.RED   + 'PMID I2C address: 0x{:02X}'.format(_i2cAddress))
        if _smid_motor_enabled:
            _i2cAddress = _smid_motor.tb.I2cAddress
            _log.info(Fore.GREEN + 'SMID I2C address: 0x{:02X}'.format(_i2cAddress))
        if _paft_motor_enabled:
            _i2cAddress = _paft_motor.tb.I2cAddress
            _log.info(Fore.RED   + 'PAFT I2C address: 0x{:02X}'.format(_i2cAddress))
        if _saft_motor_enabled:
            _i2cAddress = _saft_motor.tb.I2cAddress
            _log.info(Fore.GREEN + 'SAFT I2C address: 0x{:02X}'.format(_i2cAddress))
       
        if _use_digital_pot and _i2c_scanner.has_hex_address(['0x0E']):
            _log.info('using digital potentiometer…')
            # configure digital potentiometer for motor speed
            _pot = DigitalPotentiometer(_config, 0x0E, level=_level)
            _pot.set_input_range(IN_MIN, IN_MAX)
            _pot.set_output_range(OUT_MIN, OUT_MAX)
        else:
            _pot = AnalogPotentiometer(_config, level=_level)
#           raise Exception('cannot continue: no digital potentiometer found.')

        _last_scaled_value = 0.0
        _log.info('starting test…')
        _hz = 20
        _rate = Rate(_hz, Level.ERROR)
        while True:

            if _pfwd_motor_enabled:
                _pfwd_motor.update_target_speed()
            if _sfwd_motor_enabled:
                _sfwd_motor.update_target_speed()
            if _pmid_motor_enabled:
                _pmid_motor.update_target_speed()
            if _smid_motor_enabled:
                _smid_motor.update_target_speed()
            if _paft_motor_enabled:
                _paft_motor.update_target_speed()
            if _saft_motor_enabled:
                _saft_motor.update_target_speed()

            if _use_digital_pot:
                _scaled_value = _pot.get_scaled_value(False)
            else:
                _scaled_value = _pot.get_scaled_value()

            _log.info(Fore.CYAN + Style.BRIGHT + 'scaled value: {}'.format(_scaled_value))

            if _scaled_value != _last_scaled_value: # if not the same as last time
                # math.isclose(3, 15, abs_tol=0.03 * 255) # 3% on a 0-255 scale
                if isclose(_scaled_value, 0.0, abs_tol=0.05 * 90):
                    _pot.set_black()
                    if _pfwd_motor:
                        _pfwd_motor.target_velocity = 0.0
                    if _sfwd_motor:
                        _sfwd_motor.target_velocity = 0.0
                    if _pmid_motor:
                        _pmid_motor.target_velocity = 0.0
                    if _smid_motor:
                        _smid_motor.target_velocity = 0.0
                    if _paft_motor:
                        _paft_motor.target_velocity = 0.0
                    if _saft_motor:
                        _saft_motor.target_velocity = 0.0
                else:
                    _pot.set_rgb(_pot.value)
                    if _pfwd_motor_enabled:
                        _pfwd_motor.target_velocity = _scaled_value
                    if _sfwd_motor_enabled:
                        _sfwd_motor.target_velocity = _scaled_value
                    if _pmid_motor_enabled:
                        _pmid_motor.target_velocity = _scaled_value
                    if _smid_motor_enabled:
                        _smid_motor.target_velocity = _scaled_value
                    if _paft_motor_enabled:
                        _paft_motor.target_velocity = _scaled_value
                    if _saft_motor_enabled:
                        _saft_motor.target_velocity = _scaled_value
            _last_scaled_value = -1 #_scaled_value
            _rate.wait()

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting…')
    except DeviceNotFound as e:
        _log.error('no potentiometer found, exiting.')
    except Exception as e:
        _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        if _pfwd_motor != None:
            _pfwd_motor.set_motor_power(0.0)
        if _sfwd_motor != None:
            _sfwd_motor.set_motor_power(0.0)
        if _pmid_motor != None:
            _pmid_motor.set_motor_power(0.0)
        if _smid_motor != None:
            _smid_motor.set_motor_power(0.0)
        if _paft_motor != None:
            _paft_motor.set_motor_power(0.0)
        if _saft_motor != None:
            _saft_motor.set_motor_power(0.0)

    _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
    _log.info(Fore.YELLOW + 'complete: elapsed: {:d}ms'.format(_elapsed_ms))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    try:
        test_motors()
    except Exception as e:
        print(Fore.RED + 'error in motor test: {}'.format(e) + Style.RESET_ALL)
        traceback.print_exc(file=sys.stdout)
    finally:
        pass

if __name__== "__main__":
    main()

#EOF

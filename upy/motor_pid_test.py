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
from math import isclose
from colorama import Fore, Style

from core.rate import Rate
from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.motor import Motor
from hardware.pid import PID

_log = Logger('test', Level.INFO)

IN_MIN  = 0.0    # minimum analog value from IO Expander
IN_MAX  = 3.3    # maximum analog value from IO Expander
# when setting velocity:
#OUT_MIN = -80.0  # minimum scaled output value
#OUT_MAX =  80.0  # maximum scaled output value
# when setting motor power:
OUT_MIN = -90.0  # minimum scaled output value
OUT_MAX =  90.0  # maximum scaled output value

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def key_callback(event):
    _log.info('callback on event: {}'.format(event))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
@pytest.mark.unit
def test_motors():

    _motor = None

    try:

        # read YAML configuration
        _level = Level.INFO
        _config = ConfigLoader(Level.INFO).configure() 

        # TEMP
        _power = 0.0
        _set_power = 0.0

        # pid_controller
        _label = 'sfwd'
        _kp = 0.09500            # proportional gain
        _ki = 0.00000            # integral gain
        _kd = 0.00000            # derivative gain
        _minimum_output = -10.0  # lower output limit
        _maximum_output = 10.0   # upper output limit
        _sample_freq_hz = 20     # 20Hz equiv to 50ms/0.05s
        _hyst_queue_len = 20     # size of queue used for running average for hysteresis
        _freq_hz    = _sample_freq_hz
        _period_sec = 1.0 / _freq_hz

        _pid = PID(_label, _kp, _ki, _kd, _minimum_output, _maximum_output, period=_period_sec, level=Level.INFO)

        _motor = None # TODO
        _motor.enable()
        _motor.remove_limiters()

        # configure potentiometer from ADC
        _pot = None # TODO 

        _last_scaled_value = 0.0
        _log.info('starting test…')
        _hz = 20
        _rate = Rate(_hz, Level.ERROR)
        while True:
            _scaled_value = _pot.get_scaled_value(False)
            if _scaled_value != _last_scaled_value: # if not the same as last time
                # math.isclose(3, 15, abs_tol=0.03 * 255) # 3% on a 0-255 scale
                if isclose(_scaled_value, 0.0, abs_tol=0.05 * 90):
                    _motor_power = 0.0
                    _log.info('scaled value: {:.2f};\tmotor power: {:.2f}'.format(_scaled_value, _motor_power))
                    _pot.set_black()
                    _motor.set_motor_power(0.0)
                    _last_scaled_value = 0.0
                else:

                    _target_velocity = _scaled_value # was _pot_value
                
                    _pid.setpoint = _target_velocity
#                   _pid.tunings = ( _kp, _ki, _kd )
                    _last_power = _motor.current_power
                    _current_velocity = _motor.get_velocity()
                    _power += _pid(_current_velocity) / 100.0
                    _set_power = _power / 100.0

                    _motor_power = _scaled_value / 100.0
                    _log.info(Style.BRIGHT + 'scaled value: {:.2f};\tmotor power: {:.2f}; '.format(_scaled_value, _motor_power) 
                            + Fore.BLUE + 'power: {:.2f}; set power: {:.2f}'.format(_power, _set_power))
                    _pot.set_rgb(_pot.value)
                    _motor.set_motor_power(_motor_power)
                    _last_scaled_value = _scaled_value
            _rate.wait()

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting…')
    except DeviceNotFound as e:
        _log.error('no potentiometer found, exiting.')
    except Exception as e:
        _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        if _motor != None:
            _motor.set_motor_power(0.0)

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

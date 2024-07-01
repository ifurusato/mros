#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-04-27
# modified: 2024-05-19
#
# A jerk limiter that limits the rate of acceleration of a motor power.
#

import sys
from math import isclose
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class JerkLimiter(Component):
    '''
    A jerk limiter that limits the rate of change of a value. This isn't
    actually a jerk limiter in the formal sense because it doesn't take
    time into account in its calculation, and therefore not acceleration
    (where 'jerk' is defined as the rate of change of acceleration). This
    is just a modest low pass filter using fixed values for clipping. It
    is used to add a safety feature to the values of motor power sent to
    the controller, so due to its position in the controller process it
    is acting in the capacity of minimising jerk.

    This uses the ros:motor:jerk: section of the YAML configuration.

    :param config:           application configuration
    :param orientation:      used only for the logger label
    :param level:            the logging Level
    '''
    def __init__(self, config, orientation, suppressed=False, enabled=True, level=Level.INFO):
        self._log = Logger('jerk:{}'.format(orientation.label), level)
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)
        _maximum_output = config['mros'].get('motor').get('motor_power_limit') # power limit to motor
        _minimum_output = _maximum_output * -1
        _cfg = config['mros'].get('motor').get('jerk_limiter')
        _jerk_tolerance_pc = _cfg.get('jerk_tolerance') # expressed as percent (0-100)
        if _jerk_tolerance_pc < 0 or _jerk_tolerance_pc > 100:
            raise ValueError('jerk tolerance must be expressed as a percentage value (0-100).')
        self._tolerance = ( _jerk_tolerance_pc / 100 ) * _maximum_output
        self._jerk_rate_limit = self._tolerance * 1.3 # defined as the tolerance
        self._clip = lambda n: _minimum_output if n <= _minimum_output else _maximum_output if n >= _maximum_output else n
        self._log.info('jerk limit: {:5.2f}; tolerance: {:5.2f}; minimum output: {:5.2f}; maximum output: {:5.2f}'.format(
                self._jerk_rate_limit, self._tolerance, _minimum_output, _maximum_output))
        if not self.suppressed and self.enabled:
            self._log.info('ready.')
        else:
            self._log.info('ready (enabled: {}; suppressed: {})'.format(self.enabled, self.suppressed))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._log.info('starting jerk limiter with jerk limit of {:5.3f}/cycle.'.format(self._jerk_rate_limit))
        Component.enable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def limit(self, current_value, target_value):
        '''
        The returned result is the limited by maximum amount of change between
        the current value and the target value, using a constant set in
        configuration. If the two arguments are the same (mathematically close)
        we just return the current value. The result is clipped to the minimum
        and maximum output values set in configuration.

        This filter works for either negative or positive values, clipping
        changes larger than the jerk value. If suppressed or disabled this 
        still returns the target value argument, clipped.
        '''
        _value = target_value
        if not self.enabled:
            self._log.debug('disabled; returning target value {:+06.2f}.'.format(target_value))
        elif self.suppressed:
            self._log.debug('suppressed; returning target value {:+06.2f}.'.format(target_value))
        else:
            # math.isclose(3, 15, abs_tol=0.03 * 255) # 3% on a 0-255 scale
            if isclose(current_value, target_value, abs_tol=self._tolerance): # if close to each other
                self._log.info('limit current {:+06.2f} to target value {:+06.2f}.'.format(current_value, target_value))
                pass
            elif target_value > current_value: # increasing ┈┈┈┈┈┈┈┈┈┈
                if not isclose(current_value, target_value, abs_tol=self._tolerance):
                    # only allow the current value plus the jerk limit
                    _value = current_value + self._jerk_rate_limit
                    self._log.info('limit current {:+06.2f} -> target value {:+06.2f}: value: {:5.2f}'.format(current_value, target_value, _value))
                else:
                    self._log.info(Fore.BLACK + 'limit current {:+06.2f} -> target value {:+06.2f}.'.format(current_value, target_value))

            else: # decreasing ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                if abs(current_value - target_value) > self._jerk_rate_limit:
                    # only allow the current value minus the jerk limit
                    _value = current_value - self._jerk_rate_limit
                    self._log.info('limit current {:+06.2f} -> target value {:+06.2f}: value: {:5.2f}'.format(current_value, target_value, _value))
                else:
                    self._log.info(Fore.BLACK + 'limit current {:+06.2f} -> target value {:+06.2f}.'.format(current_value, target_value))

        # clip within save limits
        _value = -1.0 * self._clip(-1.0 * _value) if _value < 0.0 else self._clip(_value)
        self._log.debug(Style.DIM + 'limit current {:+06.2f} -> target value {:+06.2f}, returning '.format(current_value, target_value)
                + Fore.YELLOW + Style.NORMAL + 'value: {:5.2f}'.format(_value))
        return _value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_test_result(self, current_value, target_value):
        _result = self.limit(current_value, target_value)
        self._log.info(Fore.GREEN + 'current: {:5.2f}; '.format(current_value)
                + Fore.MAGENTA + 'target: {:5.2f}; '.format(target_value)
                + Fore.YELLOW  + 'result: {:5.2f}.'.format(_result))
        return _result

#EOF

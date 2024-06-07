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
# A general purpose slew limiter that limits the rate of change of a value.
#

import time
from math import isclose
from enum import Enum
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SlewLimiter(Component):
    '''
    A general purpose slew limiter that limits the rate of change of a value,
    configured for managing velocity values, which vary from -90.0 to +90.0.

    This uses the ros:slew: section of the YAML configuration.

    Parameters:
    :param config:  application configuration
    :param orientation:   used for the logger label
    :param level:   the logging Level
    '''
    def __init__(self, config, orientation, suppressed=False, enabled=True, level=Level.INFO):
        self._log = Logger('slew:{}'.format(orientation.label), level)
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)
        self._millis  = lambda: int(round(time.time() * 1000))
        self._seconds = lambda: int(round(time.time()))
        # slew configuration
        _cfg = config['mros'].get('motor').get('slew_limiter')
        self._minimum_output    = _cfg.get('minimum_output')
        self._maximum_output    = _cfg.get('maximum_output')
        self._log.info('minimum output: {:5.2f}; maximum output: {:5.2f}'.format(self._minimum_output, self._maximum_output))
        self._default_slew_rate = SlewRate.from_string(_cfg.get('default_rate')) # default rate_limit, value change permitted per millisecond
        self.slew_rate = self._default_slew_rate
        self._slew_hysteresis   = _cfg.get('hysteresis')
        self._log.info('hysteresis:\t{:5.2f}'.format(self._slew_hysteresis))
        self._stats_queue       = None
        self._start_time        = self._millis()
        # lambdas
        self._clip = lambda n: self._minimum_output if n <= self._minimum_output \
                else self._maximum_output if n >= self._maximum_output \
                else n
        if not self.suppressed and self.enabled:
            self._log.info('ready.')
        else:
            self._log.info('ready (enabled: {}; suppressed: {})'.format(self.enabled, self.suppressed))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        '''
        Reset the slew rate to the default value provided in the configuration.
        '''
        self._slew_rate = self._default_slew_rate
        self._log.info('slew rate limit reset to default of {}; {:>6.4f}/cycle.'.format(self._slew_rate.label, self._slew_rate.limit))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def slew_rate(self):
        '''
        Return the current slew rate.
        '''
        return self._slew_rate

    @slew_rate.setter
    def slew_rate(self, slew_rate):
        '''
        Sets the slew rate to the argument (an enum whose 'limit' property
        is in value/second). This overrides the value set in configuration.
        '''
        if not isinstance(slew_rate, SlewRate):
            raise ValueError('expected SlewRate argument, not {}'.format(type(slew_rate)))
        self._slew_rate = slew_rate
        self._log.info('slew rate limit set to {}; {:>6.4f}/cycle.'.format(slew_rate.label, self._slew_rate.limit))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _reset_timer(self, value):
        '''
        Resets the elapsed timer.
        '''
        self._start_time = self._millis()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def limit(self, current_value, target_value):
        '''
        Next generation attempt.
        '''
        if not self.is_active:
            return target_value
        elif isclose(target_value, current_value, abs_tol=1e-3):
            return target_value
        elif target_value > current_value: # increasing ..........
            # add a percentage of difference between current and target to current
            _diff = self._slew_rate.ratio * ( target_value - current_value )
#           if abs(_diff) < self._slew_hysteresis:
#               _diff = self._slew_hysteresis
            _value = current_value + _diff
#           self._log.info(Fore.RED + '+value: {:+06.2f}; diff: {:06.2f} ({:3.1f}%); target: {:+06.2f}'.format(\
#                   _value, _diff, 100.0 * self._slew_rate.ratio, target_value))
        else: # decreasing .......................................
            # subtract a percentage of difference between current and target to current
            _diff = self._slew_rate.ratio * ( current_value - target_value )
#           if abs(_diff) < self._slew_hysteresis:
#               _diff = self._slew_hysteresis
            _value = current_value - _diff
#           self._log.info(Fore.BLUE + '-value: {:+06.2f}; diff: {:06.2f} ({:3.1f}%); target: {:+06.2f}'.format(\
#                   _value, _diff, 100.0 * self._slew_rate.ratio, target_value))
        return -1.0 * self._clip(-1.0 * _value) if _value < 0.0 else self._clip(_value)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._log.info('starting slew limiter with rate limit of {:5.3f}/cycle.'.format(self._slew_rate.limit))
        self._start_time = self._millis()
        Component.enable(self)
    
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SlewRate(Enum): # tested to 50.0 velocity:
    #                  num   ratio     pid    limit
#   SLOWEST          = ( 0,  0.009,   0.16, 00.0001 ) # 5.1 sec
    SLOWEST          = ( 0,   0.02,   0.22, 00.0002 ) # 3.1 sec
    SLOWER           = ( 1,   0.05,   0.38, 00.0005 ) # 1.7 sec
    SLOW             = ( 2,   0.08,   0.48, 00.0010 ) # 1.3 sec
    NORMAL           = ( 3,   0.10,   0.58, 00.0050 ) # 1.0 sec
    FAST             = ( 4,   0.25,   0.68, 00.0100 ) # 0.6 sec
    FASTER           = ( 5,   0.50,   0.90, 00.0200 ) # 0.5 sec
    FASTEST          = ( 6,   0.95,   0.90, 00.1000 ) # 0.9 sec

    def __new__(cls, *args, **kwds):
        obj = object.__new__(cls)
        obj._value_ = args[0]
        return obj

    # ignore the first param since it's already set by __new__
    def __init__(self, num, ratio, pid, limit):
        self._ratio = ratio
        self._pid   = pid
        self._limit = limit

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def label(self):
        return self.name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_string(value):
        for r in SlewRate:
            if value.upper() == r.name:
                return r
        raise ValueError('unrecognised value for slew rate: {}'.format(value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def ratio(self):
        return self._ratio

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def limit(self):
        return self._limit

#EOF

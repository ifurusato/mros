#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-04-27
# modified: 2024-06-08
#
# A general purpose slew limiter that limits the rate of change of a value.
#

import time
from math import isclose
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component
from core.orientation import Orientation
from hardware.slew_rate import SlewRate

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SlewLimiter(Component):
    '''
    A general purpose slew limiter that limits the rate of change of a value,
    configured for managing speed values, which vary from -1.0 to +1.0.

    :param config:       application configuration
    :param orientation:  used for the logger label
    :param level:        the logging Level
    '''
    def __init__(self, config, orientation, suppressed=False, enabled=True, level=Level.INFO):
        self._log = Logger('slew:{}'.format(orientation.label), level)
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)
        self._orientation = orientation
        self._millis  = lambda: int(round(time.perf_counter() * 1000))
        # slew configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config.get('mros').get('motor').get('slew_limiter')
        self._minimum_output    = _cfg.get('minimum_output')
        self._maximum_output    = _cfg.get('maximum_output')
        self._log.info('minimum output: {:5.2f}; maximum output: {:5.2f}'.format(self._minimum_output, self._maximum_output))
        self._default_slew_rate = SlewRate.from_string(_cfg.get('default_rate')) # default rate_limit, value change permitted per millisecond
        self.slew_rate = self._default_slew_rate
        self._rate_limit = self.slew_rate.limit
        self._slew_hysteresis   = _cfg.get('hysteresis')
        self._log.info('hysteresis:\t{:5.2f}'.format(self._slew_hysteresis))
        self._stats_queue       = None
        self._last_time         = self._millis()
        self._verbose           = False
        # temporary ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._min = 0.0
        self._max = 0.0
        self._clamp = lambda n: self._min if n < self._min else self._max if n > self._max else n
        # lambdas ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
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
#       self._log.info('slew rate limit reset to default of {}; {:>6.4f}/cycle.'.format(self._slew_rate.label, self._slew_rate.limit))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def slew_rate(self):
        '''
        Return the current SlewRate.
        '''
        return self._slew_rate

    @slew_rate.setter
    def slew_rate(self, slew_rate):
        '''
        Sets the SlewRate to the argument (an enum whose 'limit' property
        is in value/second). This overrides the value set in configuration.
        The limit may be set directly as a float using set_slew_rate_limit(),
        which is mostly for use in testing.
        '''
        if not isinstance(slew_rate, SlewRate):
            raise ValueError('expected SlewRate argument, not {}'.format(type(slew_rate)))
        self._slew_rate = slew_rate
        self._rate_limit = slew_rate.limit
#       self._log.info('slew rate limit set to {}; {:>6.4f}/cycle.'.format(slew_rate.label, self._slew_rate.limit))

    def set_slew_rate_limit(self, limit):
        self._rate_limit = limit

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _reset_timer(self, value):
        '''
        Resets the elapsed timer.
        '''
        self._last_time = self._millis()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def limit(self, current_value, target_value):
        '''
        Next generation attempt.
        '''
        if not self.is_active:
            self._log.warning('slew limiter not active.')
            return target_value
        _value = target_value

        _now = self._millis()
        _elapsed = _now - self._last_time

#       elif isclose(target_value, current_value, abs_tol=1e-3):
#           self._log.info('slew limiter: target is close to current value.')
#           return target_value
        if target_value > current_value: # increasing ┈┈┈┈
            # add a percentage of difference between current and target to current
            self._min = current_value - ( self._rate_limit * _elapsed )
            self._max = current_value + ( self._rate_limit * _elapsed )
#           self._clamp = lambda n: self._min if n < self._min else self._max if n > self._max else n
            _value = self._clamp(target_value)
            if self._verbose:
                if _value == target_value:
                    self._log.info(Fore.YELLOW + '(+) elapsed: {:4.2f}; current: {:+5.3f}; target: {:+5.3f}; min: {:+5.3f}; max: {:+5.3f}; value: {:+5.3f}'.format(\
                            _elapsed, current_value, target_value, self._min, self._max, _value))
                else:
                    self._log.info(Fore.YELLOW + '(+) elapsed: {:4.2f}; current: {:+5.3f}; target: {:+5.3f}; min: {:+5.3f}; max: {:+5.3f}; value: {:+5.3f}'.format(\
                            _elapsed, current_value, target_value, self._min, self._max, _value))

        else: # decreasing ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            # subtract a percentage of difference between current and target to current
            self._min = current_value - ( self._rate_limit * _elapsed )
            self._max = current_value + ( self._rate_limit * _elapsed )
            _value = self._clamp(target_value)
            if self._verbose and self._orientation is Orientation.SAFT:
                if _value == target_value:
                    self._log.info(Fore.MAGENTA + '(-) elapsed: {:4.2f}; current: {:+5.3f}; target: {:+5.3f}; min: {:+5.3f}; max: {:+5.3f}; value: {:+5.3f}'.format(\
                            _elapsed, current_value, target_value, self._min, self._max, _value))
                else:
                    self._log.info(Fore.MAGENTA + '(-) elapsed: {:4.2f}; current: {:+5.3f}; target: {:+5.3f}; min: {:+5.3f}; max: {:+5.3f}; value: {:+5.3f}'.format(\
                            _elapsed, current_value, target_value, self._min, self._max, _value))

#       return -1.0 * self._clip(-1.0 * _value) if _value < 0.0 else self._clip(_value)
        self._last_time = _now
        return _value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._log.info('starting slew limiter with rate limit of {:5.3f}/cycle.'.format(self._slew_rate.limit))
        self._last_time = self._millis()
        Component.enable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        Component.disable(self)
        self._log.info('disabled slew limiter.')

#EOF

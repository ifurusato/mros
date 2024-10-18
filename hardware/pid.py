#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-09-26
# modified: 2021-07-14
#
# This class and the PIDController class were derived from ideas gleaned
# from libraries by both Martin Lundberg and Brett Beauregard, as well as
# helpful discussions with David Anderson of the DPRG.
#

import time, math
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.orientation import Orientation

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class PID(object):
    '''
    The PID controller itself.

    :param label:       a label for logging
    :param kp:          proportional gain constant
    :param ki:          integral gain constant
    :param kd:          derivative gain constant
    :param min_output:  minimum output limit
    :param max_output:  maximum output limit
    :param setpoint:    initial setpoint
    :param period:      period (sample time in seconds), used as a limit to determine if called too soon
    :param level:       log level
    '''
    def __init__(self, label, kp, ki, kd, min_output, max_output, setpoint=0.0, period=0.01, level=Level.INFO):
        self._log = Logger('pid:{}'.format(label), level)
        self._kp           = kp # proportional gain
        self._ki           = ki # integral gain
        self._kd           = kd # derivative gain
        self._setpoint     = setpoint
        self._min_output   = min_output
        self._max_output   = max_output
        self._sp_limit     = None
        self._target       = 0.0
        self._setpoint_clip = lambda n: ( -1.0 * self._sp_limit ) if n <= ( -1.0 * self._sp_limit ) \
                else self._sp_limit if n >= self._sp_limit \
                else n
        if self._min_output is None or self._max_output is None:
            self._log.info('kp:{:7.4f}; ki:{:7.4f}; kd:{:7.4f}; min={}; max={}'.format(
                    self._kp, self._ki, self._kd, self._min_output, self._max_output))
        else:
            self._log.info('kp:{:7.4f}; ki:{:7.4f}; kd:{:7.4f}; min={:>5.2f}; max={:>5.2f}'.format(
                    self._kp, self._ki, self._kd, self._min_output, self._max_output))
        if not isinstance(period, float):
            raise ValueError('wrong type for period argument: {}'.format(type(period)))
        self._period_sec   = period
        self._log.info('period: {:7.4f} sec'.format(self._period_sec))
        self.reset()
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def kp(self):
        return self._kp

    @kp.setter
    def kp(self, kp):
        self._kp = kp

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def ki(self):
        return self._ki

    @ki.setter
    def ki(self, ki):
        self._ki = ki

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def kd(self):
        return self._kd

    @kd.setter
    def kd(self, kd):
        self._kd = kd

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def setpoint(self):
        '''
        Returns the setpoint used by the controller.
        '''
        return self._setpoint

    @setpoint.setter
    def setpoint(self, setpoint):
        '''
        Setter for the set point. If setpoint limit has been set and the
        argument exceeds the limit, the value is set to the limit.
        '''
        if self._sp_limit:
            _tmp_setpoint = self._setpoint_clip(setpoint)
            if setpoint > self._sp_limit:
#               self._log.warning('setpoint {:5.2f} clipped; over limit: {:5.2f}'.format(setpoint, self._sp_limit))
                self._setpoint = self._sp_limit
            elif setpoint < -1.0 * self._sp_limit:
#               self._log.warning('setpoint {:5.2f} clipped; under limit: {:5.2f}'.format(setpoint, self._sp_limit))
                self._setpoint = -1.0 * self._sp_limit
            else:
#               self._log.info(Fore.GREEN + 'setpoint {:5.2f} not clipped; within limit: {:5.2f}'.format(setpoint, self._sp_limit))
                self._setpoint = setpoint
            if _tmp_setpoint != self._setpoint:
                raise Exception('clip didnt work.')
        else:
#           self._log.info(Fore.GREEN + 'setpoint {:5.2f} not clipped; no limit set.'.format(setpoint))
            self._setpoint = setpoint

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def limit(self):
        return self._sp_limit

    @limit.setter
    def limit(self, limit):
        '''
        Setter for the setpoint limit. Set to None (the default) to
        disable this feature. Note that this doesn't affect the setting
        of the setpoint but rather the getting of the setpoint.
        '''
        if limit == None:
            self._log.info('setpoint limit: disabled')
        else:
            self._log.info('setpoint limit: {:5.2f}'.format(limit))
        self._sp_limit = limit

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __call__(self, dt=None):
        '''
        Call the PID controller and calculate and return a control output if
        the period (sample time in seconds) has passed since the last update.
        If no new output is calculated, return the previous output instead
        (or None if no value has been calculated yet).

        :param target: the target value for the setpoint.
        :param dt: If set, uses this value for timestep instead of real time.
                   This can be used in simulations when simulation time is
                   different from real time.
        '''
#       self._log.info(Fore.RED + Style.BRIGHT + 'PID.__call__() setpoint: {:5.2f}; target: {:5.2f}; dt: {}'.format(self._setpoint, self._target, dt))
        _now = time.monotonic()
        if dt is None:
            dt = _now - self._last_time
        elif dt <= 0:
            raise ValueError("dt has nonpositive value {}. Must be positive.".format(dt))
        # display dt in milliseconds
        if dt < self._period_sec and not math.isclose(dt, self._period_sec) and self._last_output is not None:
            # only update every period
            output = self._last_output
        else:
            # compute error terms
            # FIXME setpoint and target?
            _error = self._setpoint - self._target
            d_input = self._target - (self._last_input if self._last_input is not None else self._target)
#           self._log.info(Fore.MAGENTA + 'error: {:5.2f}'.format(_error))
            # compute the proportional, integral and derivative terms
            self._proportional = self._kp * _error
            self._integral    += self._ki * _error * dt
            self._integral     = self._clip(self._integral) # avoid integral windup
            self._derivative   = -self._kd * d_input / dt
            # compute output, clipped to limits
            output = self._clip(self._proportional + self._integral + self._derivative)
            kp, ki, kd = self.constants
            cp, ci, cd = self.components
#           self._log.info('dt={:7.4f}ms '.format(dt * 1000.0) \
#                   + Fore.CYAN + Style.DIM + 'self._target={:5.2f}; error={:6.3f};'.format(self._target, _error) \
#                   + Fore.MAGENTA + ' KP={:<8.5f}; KD={:<8.5f};'.format(kp, kd) \
#                   + Fore.CYAN + Style.BRIGHT + ' P={:8.5f}; I={:8.5f}; D={:8.5f}; sp={:6.3f};'.format(cp, ci, cd, self._setpoint) \
#                   + Style.BRIGHT + ' out: {:<8.5f}'.format(output))
            self._last_output = output

        # keep track of state
        self._last_input  = self._target
        self._last_time   = _now
        return output

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def target(self):
        '''
        Return the current target speed.
        '''
        return self._target

    @target.setter
    def target(self, target):
        '''
        Setter for the output limits using a tuple: (lower, upper).
        Setting 'None' for a value means there is no limit.
        '''
        self._target = target

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def period(self):
        '''
        Return the period (sample time in seconds) as a property.
        '''
        return self._period_sec

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def constants(self):
        '''
        The P-, I- and D- fixed terms, as a tuple.
        '''
        return self._kp, self._ki, self._kd

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def components(self):
        '''
        The P-, I- and D-terms from the last computation as separate
        components as a tuple. Useful for visualizing what the controller
        is doing or when tuning hard-to-tune systems.
        '''
        return self._proportional, self._integral, self._derivative

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def tunings(self):
        '''
        The tunings used by the controller as a tuple: (Kp, Ki, Kd)
        '''
        return self._kp, self._ki, self._kd

    @tunings.setter
    def tunings(self, tunings):
        '''
        Setter for the PID tunings.
        '''
        self._kp, self._ki, self._kd = tunings

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def output_limits(self):
        '''
        The output limits as a 2-tuple: (lower, upper). See also
        the *output_limts* parameter in :meth:`PID.__init__`.
        '''
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        '''
        Setter for the output limits using a tuple: (lower, upper).
        Setting 'None' for a value means there is no limit.
        '''
        if limits is None:
            self._min_output, self._max_output = None, None
            return
        min_output, max_output = limits
        if None not in limits and max_output < min_output:
            raise ValueError('lower limit must be less than upper limit')
        self._min_output  = min_output
        self._max_output  = max_output
        self._integral    = self._clip(self._integral)
        self._last_output = self._clip(self._last_output)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_state(self):
        _fore = Fore.YELLOW
        self._log.info(_fore + 'kp:             \t{}'.format(self._kp))
        self._log.info(_fore + 'ki:             \t{}'.format(self._ki))
        self._log.info(_fore + 'kd:             \t{}'.format(self._kd))
        self._log.info(_fore + 'min_output:     \t{}'.format(self._min_output))
        self._log.info(_fore + 'max_output:     \t{}'.format(self._max_output))
        self._log.info(_fore + 'setpoint:       \t{}'.format(self._setpoint))
        self._log.info(_fore + 'setpoint limit: \t{}'.format(self._sp_limit))
        self._log.info(_fore + 'period:         \t{}'.format(self._period_sec))

        self._log.info(_fore + 'proportional:   \t{}'.format(self._proportional))
        self._log.info(_fore + 'integral:       \t{}'.format(self._integral))
        self._log.info(_fore + 'derivative:     \t{}'.format(self._derivative))
        self._log.info(_fore + 'last_output:    \t{}'.format(self._last_output))
        self._log.info(_fore + 'last_input:     \t{}'.format(self._last_input))
        self._log.info(_fore + 'last_time:      \t{}'.format(self._last_time))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        '''
        Reset the PID controller internals, setting each term to 0 as well
        as cleaning the integral, the last output and the last input
        (derivative calculation).

        Note that after the setpoint has been set to zero if the motors
        have been running the last input and output values remain at
        their previous values. This storing of previous state causes
        problems when starting up again. The reset() function cleans
        any stored state.
        '''
        self._log.info('reset.')
#       self._setpoint     = 0.0
        self._proportional = 0.0
        self._integral     = 0.0
        self._derivative   = 0.0
        self._last_output  = 0.0
        self._last_input   = 0.0
        self._last_time    = time.monotonic()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _clip(self, value):
        '''
        Clip the value between the lower and upper limits. If either limit
        is not set ('None') the original argument is returned.
        '''
        if self._min_output is None or self._max_output is None:
            return value
        else:
            return max(self._min_output, min(value, self._max_output))

#EOF

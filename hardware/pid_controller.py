#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-04-20
# modified: 2024-05-19
#

import sys
from datetime import datetime as dt
import itertools # TEMP
from math import isclose
from collections import deque as Deque
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.event import Event
from core.orientation import Orientation
from core.message import Message
from hardware.pid import PID

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class PIDController(Component):
    '''
    Provides a configurable PID-based motor controller. This also maintains
    a value for the current motor target speed by sampling the step count
    from the motors on the same interval as the PID calls.

    :param config:       The application configuration, read from a YAML file.
    :param motor:        The motor to be controlled.
    :param setpoint:     The initial setpoint or target output
    :param period:       The sample time in seconds before generating a new output value.
                         This PID is expected to be called at a constant rate.
    :param suppressed:   Initial suppressed state.
    :param enabled:      Initial enabled state.
    :param level:        The log level, e.g., Level.INFO.
    '''
    def __init__(self, config, motor, setpoint=0.0, period=0.01, suppressed=False, enabled=True, level=Level.INFO):
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(config)))
        self._config = config
        if motor is None:
            raise ValueError('null motor argument.')
        self._motor = motor
        self._orientation = motor.orientation
        self._log = Logger('pid-ctrl:{}'.format(self._orientation.label), level)
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)
        self._counter = itertools.count() # TEMP
        # PID configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['mros'].get('motor').get('pid_controller')
        _kp         = _cfg.get('kp') # proportional gain
        _ki         = _cfg.get('ki') # integral gain
        _kd         = _cfg.get('kd') # derivative gain
        _min_output = _cfg.get('minimum_output')
        _max_output = _cfg.get('maximum_output')
        _freq_hz    = _cfg.get('sample_freq_hz')
        _period_sec = 1.0 / _freq_hz
        self._log.info('sample frequency: {:d}Hz; period: {:5.2f} sec.'.format(_freq_hz, _period_sec))
        self._pid = PID(self._orientation.label, _kp, _ki, _kd, _min_output, _max_output, period=_period_sec, level=level)
        # used for hysteresis, if queue too small will zero-out motor power too quickly
        _queue_len = _cfg.get('hyst_queue_len')
        self._deque = Deque([], maxlen=_queue_len)
        self._target_speed = 0.0
        self._power        = 0.0
        self._last_power   = 0.0
        self._last_time = dt.now() # for calculating elapsed time
        self._verbose      = True
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pid(self):
        '''
        Return the underlying PID controller.
        '''
        return self._pid

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def orientation(self):
        return self._orientation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def kp(self):
        return self._pid.kp

    @kp.setter
    def kp(self, kp):
        self._pid.kp = kp

    @property
    def ki(self):
        return self._pid.ki

    @ki.setter
    def ki(self, ki):
        self._pid.ki = ki

    @property
    def kd(self):
        return self._pid.kd

    @kd.setter
    def kd(self, kd):
        self._pid.kd = kd

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps(self):
        return self._motor.steps

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_steps(self):
        self._motor._steps = 0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_limit(self, limit):
        '''
        Setter for the limit of the PID setpoint.
        Set to None (the default) to disable this feature.
        '''
        self._pid.set_limit(limit)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def stats(self):
        '''
         Returns statistics a tuple for this PID controller:
            [kp, ki, kd, cp, ci, cd, last_power, current_motor_power, power, _speed, setpoint, steps]
        '''
        kp, ki, kd = self._pid.constants
        cp, ci, cd = self._pid.components
        return kp, ki, kd, cp, ci, cd, self._last_power, self._motor.get_current_power(), self._power, self._motor.target_speed, self._pid.setpoint, self._motor.steps

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_speed(self, target_speed):
        _changed = target_speed != self._target_speed
        if _changed:
            self._target_speed = target_speed 
        if self.enabled:
            _elapsed_ms = round(( dt.now() - self._last_time ).total_seconds() * 1000.0)
            _count = next(self._counter)
            if isclose(target_speed, 0.0, abs_tol=1e-2):
                self._pid.setpoint = 0.0
                self._pid.target   = 0.0
                self._power        = 0.0
#               self._motor.get_velocity().reset_steps()
                self._motor.set_motor_power(0.0)
                if self._verbose:
#                   if _count % 20 == 0:
#                       self._log.info(Fore.WHITE + Style.DIM + 'target speed: {:5.2f}; stopped; power: {:4.2f};\tvelocity: {:4.2f}'.format(target_speed, self._power, self._motor.velocity))
                    pass
            else:
                self._pid.setpoint = target_speed
                # converts speed to power...
                self._pid.target = self._motor.velocity

                _error = self._pid.setpoint - self._pid.target

                _pid_output = self._pid()
#               _pid_output = self._pid(self._motor.velocity)
#               if _changed:
                self._power += _pid_output
#               self._log.info(Fore.YELLOW + '_pid_output: {:4.2f};\t_power: {:4.2f};\tvelocity: {:4.2f}'.format(_pid_output, self._power, self._motor.velocity))
                _motor_power = self._power
                self._motor.set_motor_power(_motor_power)

#           _mean_setpoint = self._get_mean_setpoint(self._pid.setpoint)
#           if _mean_setpoint == 0.0:
#               self._log.info(Fore.WHITE + Style.BRIGHT + 'set power for {} motor: {:<5.2f} to 0.0 (pid output: {:5.2f})'.format(self._orientation.label, _motor_power, _pid_output))
#               self._motor.set_motor_power(0.0)
                if self._verbose:
                    if _count % 20 == 0:
                        self._log.info(Fore.YELLOW + 'target speed: {:5.2f}; current speed: {:5.2f};'.format(target_speed, self._motor.target_speed)
                                + Fore.GREEN + Style.NORMAL + ' pid.setpoint: {:5.2f}; pid output: {:5.2f};'.format(self._pid.setpoint, _pid_output)
#                               + Fore.MAGENTA + ' kp: {:7.4f}; ki: {:7.4f}; kd: {:7.4f})'.format(self._pid.kp, self._pid.ki, self._pid.kd)
#                               + Fore.YELLOW + ' velocity: {:<5.2f};'.format(self._motor.velocity)
                                + Fore.MAGENTA + ' error: {:<5.2f};'.format(_error)
                                + Fore.WHITE + ' motor power: {:<5.2f};'.format(_motor_power)
                                + Fore.CYAN + Style.NORMAL + ' elapsed: {:d}ms'.format(_elapsed_ms))

            self._last_power = self._power
            self._last_time = dt.now()
        else:
            self._log.info(Fore.RED + 'pid controller disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_mean_setpoint(self, value):
        '''
        Returns the mean of setpoint values collected in the queue.
        This is used to provide hysteresis around the PID controller's
        setpoint, eliminating jitter particularly around zero.
        '''
        if value == None:
            raise Exception('null argument')
        self._deque.append(value)
        _n = 0
        _mean = 0.0
        for x in self._deque:
            _n += 1
            _mean += ( x - _mean ) / _n
        return float('nan') if _n < 1 else _mean

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_state(self):
        _fore = Fore.RED if self._orientation == Orientation.PORT else Fore.GREEN
        self._log.info(_fore + 'power:        \t{}'.format(self._power))
        self._log.info(_fore + 'last_power:   \t{}'.format(self._last_power))
        self._pid.print_state()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        self._pid.reset()
        self._motor.stop()
        self._log.info(Fore.GREEN + 'reset.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if self.closed:
            self._log.warning('cannot enable PID loop: already closed.')
        else:
            if self.enabled:
                self._log.warning('PID loop already enabled.')
            else:
                Component.enable(self)

#EOF

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-09-13
# modified: 2021-08-07
#
# This class' configuration involves the physical geometry of the robot and
# the specifics of the motor encoders. Unconfigured it always returns 0.0,
# which is harmless but likewise not useful.
#

import sys, math, time
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.event import Event
from core.message import Message
from core.orientation import Orientation

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Velocity(object):
    '''
    NOTE: the following regards the KR01 robot and has not been updated
    for the MR01 Mars rover.

    Velocity is a property of a motor, and therefore this class is meant
    to be instantiated on a motor. It is used to calculate velocity by
    capturing the step count from a motor encoder, where a specific number
    of encoder ticks/steps per second corresponds to a given velocity,
    which can be converted to an SI unit (e.g., cm/sec).

    Based on some hardware constants from the KR01 robot:

      68.0mm diameter tires
      213.62830mm/21.362830cm wheel circumference
      494 encoder steps per rotation

    we can noodle around and deduce various related values:

      494 steps = 213.62830mm
      494 steps = 21.362830cm

      2312.4278 steps per meter
      23.124278 steps per cm
      2.3124278 steps per mm

      4.681028 rotations per meter
      0.04681028 rotations per cm
      0.004681028 rotations per mm
      0.432445952mm per step

    further deriving:

      1 wheel rotation/sec = 21.362830cm/sec
      494 steps/minute = 8.2333 steps/second @ 1rpm
      4940 steps/minute = 82.3333 steps/second @ 10rpm
      49400 steps/minute = 823.3333 steps/second @ 100rpm
      1 rps = 494 steps/second = 21.362830cm/sec

      2312.4278 steps per second @ 1 m/sec
      2312.4278 steps per second @ 100 cm/sec
      231.24278 steps per second @ 10 cm/sec
      23.124278 steps per second @ 1 cm/sec
      1 cm/sec = 23.124278 steps/sec

    And so for our 50ms (20Hz) loop:

      1/20th wheel rotation = 24.7 steps
      1/20th rotation = 1.06815cm
      1/20th rotation = 10.6815mm
      1.1562139 steps per 1/20th sec @ 1 cm/sec

    The most important for our purposes being:

      494 steps/sec = 21.362830cm/sec

    If we wish to calculate velocity in cm/sec we need to find out how many steps
    have occurred since the last function call, then use that to obtain velocity.

    But this is only true if our duty cycle is exactly 20Hz, which it's not. So
    to properly calculate how many steps we might expect in a certain number of
    milliseconds, we go back to our earlier deduction:

      2312.4278 steps per 1000ms = 1 cm/sec
      2.3124278 steps per 1ms = 1 cm/sec

    and multiply that constant times the number of milliseconds passed since the
    last function call.

    TODO:
      a. measure unloaded wheel speed at maximum power (in steps per second,
         from encoders) to obtain maximum velocity as upper limit
      b. figure out how many steps per second the upper limit represents

    :param config:   the application configuration
    :param motor:    the motor whose velocity is to be measured
    :param level:    the logging level
    '''
    def __init__(self, config, motor, level=Level.INFO):
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        if motor is None:
            raise ValueError('null for motor argument.')
        self._motor = motor
        self._log = Logger('velocity:{}'.format(motor.orientation.label), level)
        # add callback from motor's update method
#       self._motor.add_callback(self.tick)
        # establish sample frequency
        self._freq_hz = config['mros'].get('motor').get('pid_controller').get('sample_freq_hz')
        self._period_ms = 1000.0 / self._freq_hz
        self._log.info('sample frequency:   \t{:d}Hz ({:>5.2f}ms)'.format(self._freq_hz, self._period_ms))
        # now calculate some geometry-based conversions
        _cfg = config['mros'].get('geometry')
        self._steps_per_rotation  = _cfg.get('steps_per_rotation') # 494 encoder steps per wheel rotation
        self._log.info('encoder steps/rotation:\t{:7.2f}'.format(self._steps_per_rotation))
        self._wheel_diameter      = _cfg.get('wheel_diameter') # 68.0mm
        self._log.info('wheel diameter:     \t{:4.1f}mm'.format(self._wheel_diameter))
        self._wheel_circumference = self._wheel_diameter * math.pi / 10.0
        self._log.info('wheel circumference:\t{:7.4f}cm'.format(self._wheel_circumference))
        # convert raw velocity to approximate a percentage
        self._steps_per_cm = self._steps_per_rotation / self._wheel_circumference
        self._log.info('conversion constant:\t{:7.4f} steps/cm'.format(self._steps_per_cm))
        # sanity check: perform conversion for velocity of 1 wheel rotation (e.g., 494 steps)
        # per second, where the returned value should be the circumference (e.g., 21.36cm)
        _test_velocity = self.steps_to_cm(self._steps_per_rotation)
        self._log.info('example conversion:\t{:7.4f}cm/rotation'.format(_test_velocity))
        assert _test_velocity == self._wheel_circumference
        # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._stepcount_timestamp = time.perf_counter()
        self._steps_begin  = 0      # step count at beginning of velocity measurement
        self._velocity     = 0.0    # current velocity
        self._max_velocity = 0.0    # capture maximum velocity attained
        self._enabled      = False
        self._closed       = False
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_steps(self):
        self._steps_begin = 0
#       self._steps_begin = self._motor.steps
        self._stepcount_timestamp = time.perf_counter()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps_per_rotation(self):
        return self._steps_per_rotation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps_per_cm(self):
        return self._steps_per_cm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def steps_to_cm(self, steps):
        return steps / self._steps_per_cm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def tick(self):
        '''
        This should be called regularly every 50ms (i.e., at 20Hz), calculating
        velocity based on the tick/step count of the motor encoder.
        '''
        if self._enabled:
            if self._motor.enabled: # then calculate velocity from motor encoder's step count
                _time_diff_ms = 0.0
                _steps = self._motor.steps
                if self._steps_begin != 0:
                    _time_diff_sec = time.perf_counter() - self._stepcount_timestamp
                    _time_diff_ms = _time_diff_sec * 1000.0
                    _time_error_ms = self._period_ms - _time_diff_ms
                    # we multiply our step count by the percentage error to obtain
                    # what would be the step count for our 50.0ms period
                    _diff_steps = _steps - self._steps_begin
                    if _diff_steps == 0:
#                       self._log.info(Fore.BLUE + Style.DIM + '{:+d} steps diff'.format(_diff_steps))
                        self._velocity = 0.0
                    else:
#                       self._log.info(Fore.BLUE + '{:+d} steps diff'.format(_diff_steps))
                        _time_error_percent = _time_error_ms / self._period_ms
                        _corrected_diff_steps = _diff_steps + ( _diff_steps * _time_error_percent )
                        # multiply the steps per period by the loop frequency (20) to get steps/second
                        _steps_per_sec = _corrected_diff_steps * self._freq_hz
                        _cm_per_sec = self.steps_to_cm(_steps_per_sec)
                        self._velocity = _cm_per_sec
                        self._max_velocity = max(self._velocity, self._max_velocity)
#                       if _steps % 10 == 0:
#                           self._log.info(Fore.BLUE + '{:+d} steps, {:+d}/{:5.2f} diff/corrected; time diff: {:>5.2f}ms; error: {:>5.2f}%;\t'.format(
#                                   self._motor.steps, _diff_steps, _corrected_diff_steps, _time_diff_ms, _time_error_percent * 100.0)
#                                   + Fore.YELLOW + 'motor power: {}; velocity: {:>5.2f} steps/sec; {:<5.2f}cm/sec'.format(self._motor.last_power, _steps_per_sec, self._velocity))
                self._stepcount_timestamp = time.perf_counter()
                self._steps_begin = _steps
#               self._log.info(Fore.BLUE + '{:+d} steps, {:+d} begin'.format(self._motor.steps, self._steps_begin))
            else:
                self._velocity = 0.0
                self._log.warning('tick failed: motor disabled.')
        else:
            self._velocity = 0.0
            self._log.warning('tick failed: disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def value(self):
        '''
        Returns the current velocity value as a property. If the motors is
        stopped this always returns 0.0.
        '''
        if self._motor.is_stopped:
            return 0.0
        else:
            return self._velocity
#       return self._velocity

#   def __call__(self):
#       '''
#       Enables this class to be called as if it were a function, returning
#       the current velocity value.
#       '''
#       return self._velocity

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def max_velocity(self):
        return self._max_velocity

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self._closed:
            self._enabled = True
            self._log.info('enabled.')
        else:
            self._log.warning('cannot enable: already closed.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self._enabled:
            self._enabled = False
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self.disable()
        self._closed = True
        self._log.info('closed.')

#EOF

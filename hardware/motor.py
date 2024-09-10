#!/usr/bin/env python3 # -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-18
# modified: 2021-06-29
#

import sys, itertools, time
from math import isclose
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.component import Component
from core.orientation import Orientation
from hardware.pid_controller import PIDController
from hardware.slew_limiter import SlewLimiter
from hardware.jerk import JerkLimiter
from hardware.velocity import Velocity

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Motor(Component):
    '''
    Controls a motor that uses a Hall Effect encoder to determine the robot's
    velocity and distance traveled.

    This Motor class takes an input as speed (-1.0 to 1.0), pre-processed by
    a SlewLimiter (which buffers sudden changes to the target speed), then is
    passed along to a PIDController, which converts the speed to power (-1.0
    to 1.0), which is then passed through a JerkLimiter to avoid sudden (and
    potentially dangerous) changes to the motor. All three are optional; when
    the PIDController is disabled a speed-to-power dual-axis proportional
    interpolating function is used.

    This uses the mros:motor: section of the configuration. The suppressed state
    of the slew limiter, PID controller and jerk limiter is initially set to the
    opposite of the enabled configuration value.

    There are three reported speed properties of the motor:

        1. speed:           the scaled (0.0-1.0) current target speed of
                            the motor
        2. target speed:    the un-scaled target speed of the motor
        3. modified speed:  the target speed of the motor as modified by
                            any lambdas

    :param config:      application configuration
    :param tb:          reference to the ThunderBorg motor controller
    :param orientation: motor orientation
    :param level:       log level
    '''
    def __init__(self, config, tb, orientation, level=Level.INFO):
        if config is None:
            raise ValueError('null config argument.')
        if tb is None:
            raise ValueError('null thunderborg argument.')
        self._tb = tb
        self._orientation = orientation
        self._log = Logger('motor:{}'.format(orientation.label), level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._log.info('initialising {} motor with {} at address 0x{:02X} as motor controller…'.format(
                orientation.name, type(self._tb).__name__, self._tb.I2cAddress))
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['mros'].get('motor')
        self._scale_factor = _cfg.get('scale_factor')      # a constant that converts target speed to motor speed
        self._max_observed_speed = 0.0                     # the observed max forward speed
#       self._log.info('max speed:\t{:<5.2f}'.format(self._speed_multiplier))
        _max_speed = 1.0
        self._motor_power_limit = _cfg.get('motor_power_limit') # power limit to motor
        self._log.info('motor power limit: {:<5.2f}'.format(self._motor_power_limit))
        self._power_clip = lambda n: ( -1.0 * self._motor_power_limit ) if n <= ( -1.0 * self._motor_power_limit ) \
                else self._motor_power_limit if n >= self._motor_power_limit \
                else n
        self._counter            = itertools.count()
        self.__callbacks         = []
        self.__steps             = 0     # step counter
        self.__max_applied_power = 0.0   # capture maximum power applied
        self.__max_power_ratio   = 0.0   # will be set by MotorConfigurer
        self.__target_speed      = 0.0   # the current target speed of the motor
        self.__modified_target_speed = 0.0 # ...as modified by any lambdas
        self._last_driving_power = 0.0   # last power setting for motor
        self._decoder            = None  # motor encoder
        self._jerk_limiter       = None
        self.__speed_lambdas     = {}
        self._verbose            = True
        # slew limiter ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _enable_slew_limiter     = _cfg.get('enable_slew_limiter')
        _suppress_slew_limiter   = not _enable_slew_limiter
        self._slew_limiter       = SlewLimiter(config, orientation, suppressed=_suppress_slew_limiter,
                enabled=_enable_slew_limiter, level=level)
        # provides closed loop speed feedback ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._velocity           = Velocity(config, self, level=level)
        # add callback from motor's update method
        self.add_callback(self._velocity.tick)
        self._velocity.enable()
        # pid controller ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _enable_pid_controller   = _cfg.get('enable_pid_controller')
        _suppress_pid_controller = not _enable_pid_controller
        self._pid_controller     = PIDController(config, self, suppressed=_suppress_pid_controller,
                enabled=_enable_pid_controller, level=level)
        # jerk limiter ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _enable_jerk_limiter     = _cfg.get('enable_jerk_limiter')
        _suppress_jerk_limiter   = not _enable_jerk_limiter
#       if not _suppress_jerk_limiter and _enable_jerk_limiter:
        self._jerk_limiter       = JerkLimiter(config, orientation, suppressed=_suppress_jerk_limiter,
                enabled=_enable_jerk_limiter, level=level)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_callback(self, callback):
        '''
        Used by the Velocity class to obtain a callback on the motor loop.
        '''
        self.__callbacks.append(callback)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def orientation(self):
        '''
        Returns the orientation of this motor.
        '''
        return self._orientation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   @property
#   def speed_multiplier(self):
#       '''
#       Returns the speed multiplier used to convert the 0.0-1.0 value to
#       the value actually sent to the motor.
#       This is a constant, provided by application configuration.
#       '''
#       return self._speed_multiplier

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def max_observed_speed(self):
        '''
        Returns the maximum observed forward speed.
        '''
        return self._max_observed_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_speed_multiplier(self, name, lambda_function, clear=False):
        '''
        Adds a named speed multiplier to the dict of lambda functions. This
        replaces any existing lambda under the same name.

        This is a function that alters the target speed as a multiplier.

        :param optional clear if True, clear all existing multipliers before adding
        '''
        if clear:
            self.__speed_lambdas.clear()
        if name in self.__speed_lambdas:
            self._log.warning('motor already contains a \'{}\' lambda.'.format(name))
        else:
            self._log.info('🐟 adding \'{}\' lambda to motor {}…'.format(name, self.orientation.name))
            self.__speed_lambdas[name] = lambda_function

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def remove_speed_multiplier(self, name):
        '''
        Removes a named (or partial-named) speed multiplier from the dict of
        lambda functions.
        '''
        for _name, _lambda in self.__speed_lambdas.copy().items():
            if name == _name or name in _name:
#               self._log.info('removing \'{}\' lambda from motor {}…'.format(_name, self.orientation.name))
                del self.__speed_lambdas[_name]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def has_speed_multiplier(self, name):
        '''
        Returns true if a named speed multiplier exists in the dict of lambda functions.
        '''
        return name in self.__speed_lambdas

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear_speed_multipliers(self):
        '''
        Resets the speed multipliers to None, i.e., no function.
        '''
        self.__speed_lambdas.clear()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def speed_multiplier_count(self):
        '''
        Resets the number of speed multipliers.
        '''
        return len(self.__speed_lambdas)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def list_speed_multipliers(self):
        '''
        Lists the current speed multipliers to the log.
        '''
        self._log.info(Fore.GREEN + '  motor {} contains {:d} lambdas.'.format(self.orientation.name, len(self.__speed_lambdas)))
        if len(self.__speed_lambdas) > 0:
            for _lambda in self.__speed_lambdas:
                self._log.info(Fore.GREEN + '    speed multiplier: {}'.format(_lambda))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def remove_limiters(self):
        '''
        Used in testing to remove the slew and jerk limiters if they are by
        default set in configuration.
        '''
        self._log.warning('removing motor limiters…')
        self._slew_limiter = None
        self._jerk_limiter = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def slew_limiter(self):
        '''
        Returns the slew limiter used by this motor.
        This should be used only for testing or to obtain information, not
        for control.
        '''
        return self._slew_limiter

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pid_controller(self):
        '''
        Returns the PID controller used by this motor.
        This should be used only to obtain information, not for control.
        '''
        return self._pid_controller

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def jerk_limiter(self):
        '''
        Returns the jerk limiter used by this motor.
        This should be used only to obtain information, not for control.
        '''
        return self._jerk_limiter

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def decoder(self):
        return self._decoder

    @decoder.setter
    def decoder(self, decoder):
        self._decoder = decoder

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_velocity(self):
        '''
        Return the Velocity object for this Motor. This is a good source
        of ticks.
        '''
        return self._velocity

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def velocity(self):
        '''
        Return the current (calculated) velocity of this Motor as a value.
        This is used as an argument for the PID controller.
        '''
        return self._velocity.value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def modified_speed(self):
        '''
        Return the current lambda-modified target speed of the Motor.
        '''
        return self.__modified_target_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def target_speed(self):
        '''
        Return the current target speed of the Motor.
        '''
        return self.__target_speed

    @target_speed.setter
    def target_speed(self, target_speed):
        '''
        Set the target speed of the Motor. This is affected by the slew
        limiter, if active.
        '''
        if not isinstance(target_speed, float):
            raise ValueError('expected float, not {}'.format(type(target_speed)))
        if self._slew_limiter and self._slew_limiter.is_active:
            _current_target_speed = self.__target_speed
            _new_target_speed = target_speed 
            self.__target_speed = self._slew_limiter.limit(_current_target_speed, _new_target_speed)
#           self._log.info(Fore.GREEN + 'current speed: {:5.2f}; target speed: {:5.2f}; slewed as: {:5.2f}'.format(
#                   _current_target_speed, _new_target_speed, self.__target_speed))
        else:
            self.__target_speed = target_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steps(self):
        return self.__steps

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_steps(self):
        self.__steps = 0

    # max power rate ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def max_power_ratio(self):
        return self.__max_power_ratio

    @max_power_ratio.setter
    def max_power_ratio(self, max_power_ratio):
        self.__max_power_ratio = max_power_ratio
        self._log.info('maximum power ratio: {:<5.2f}'.format(self.__max_power_ratio))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _callback_step_count(self, pulse):
        '''
        This callback is used to capture encoder steps.
        '''
        if self._orientation.side is Orientation.PORT:
            self.__steps = self.__steps + pulse
        elif self._orientation.side is Orientation.STBD:
            self.__steps = self.__steps - pulse

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_stopped(self):
        '''
        Returns True if the motor is entirely stopped, or very nearly stopped.
        '''
        return isclose(self.current_power, 0.0, abs_tol=1e-2)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_in_motion(self):
        '''
        Returns True if the motor is moving, i.e., if the current power
        setting of the motor is not equal to zero. Note that this returns
        False if the value is very close to zero.
        '''
        return self.current_power != 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_moving_ahead(self):
        '''
        Returns True if the motor is moving ahead (forward), i.e., if the
        current power setting of the motor is greater than zero.
        '''
        return self.current_power > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_moving_astern(self):
        '''
        Returns True if the motor is moving astern (reverse), i.e., if the
        current power setting of the motor is less than zero.
        '''
        return self.current_power < 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def update_target_speed(self):
        '''
        Align the current speed and the target speed and motor power. This
        is meant to be called regularly, in a loop.
        '''
        if self.enabled:
            if not self._pid_controller.is_active: # via PID
                self._log.warning('pid controller is not active.')
                return
            for callback in self.__callbacks:
                callback()
            # we start with the current value of the target speed
            self.__modified_target_speed = self.__target_speed
            _returned_value = 0.0
            if len(self.__speed_lambdas) > 0:
#               self._log.info('processing {:d} lambdas…'.format(len(self.__speed_lambdas)))
                for _name, _lambda in self.__speed_lambdas.items():
                    _returned_value = _lambda(self.__modified_target_speed)
                    if isinstance(_returned_value, str):
                        # lambda has returned indicator that it's finished
                        break
                    else:
                        _before_lambda_speed = self.__modified_target_speed
                        self.__modified_target_speed = _returned_value
                        if 'accum' in _name: # update stored target speed if lambda name includes "accum"
                            self.__target_speed = self.__modified_target_speed
#                       self._log.info(Fore.WHITE + 'set: {:5.2f}; before: {:5.2f}; target: {:5.2f}; {} lambda for {} motor.'.format(
#                               self.__target_speed, _before_lambda_speed, self.__modified_target_speed, _name, self._orientation.label))
                if isinstance(_returned_value, str):
                    # this only should apply to brake, halt and stop.
                    _lambda_name = _returned_value
                    self.remove_speed_multiplier(_lambda_name)
                    self._pid_controller.set_speed(0.0)
                    return
#           self._log.info('updating {} target speed to: {:<5.2f} (from {:5.2f})'.format(self._orientation.label, self.__modified_target_speed, self.__target_speed))
            _motor_speed = self.__modified_target_speed * self._scale_factor
            self._pid_controller.set_speed(_motor_speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_motor_power(self, target_power):
        '''
        Direct-drive the motor via a target power argument, whose value must be
        between -1.0 and 1.0, with the actual limits set by the max_power_ratio,
        which alters the value to match the power/motor voltage ratio.

        If the JerkLimiter is active this acts as a sanity check on
        overly-rapid changes to motor power.

        :param target_power:  the target motor power
        '''
        if target_power is None:
            raise ValueError('null target_power argument.')
        elif not self.enabled and target_power > 0.0: # though we'll let the power be set to zero
            raise Exception('motor {} not enabled.'.format(self.orientation.name))
        # even if disabled or suppressed, JerkLimiter still clips
#       if self._jerk_limiter:
#           target_power = self._jerk_limiter.limit(self.current_power, target_power)
        # keep track of highest-applied target power
        self.__max_applied_power = max(abs(target_power), self.__max_applied_power)

        _driving_power = round(self._power_clip(float(target_power * self.max_power_ratio)), 4) # round to 4 decimal

        if self._orientation is Orientation.PFWD:
#           self._log.info(Fore.RED   + 'target power {:5.2f} converted to driving power {:<5.2f} for PFWD motor.'.format(target_power, _driving_power))
            self._tb.SetMotor1(_driving_power)
        elif self._orientation is Orientation.SFWD:
#           self._log.info(Fore.GREEN + 'target power {:5.2f} converted to driving power {:<5.2f} for SFWD motor.'.format(target_power, _driving_power))
            self._tb.SetMotor2(_driving_power)
#       elif self._orientation is Orientation.PMID:
#       elif self._orientation is Orientation.SMID:
        elif self._orientation is Orientation.PAFT:
#           self._log.info(Fore.RED   + 'target power {:5.2f} converted to driving power {:<5.2f} for PAFT motor.'.format(target_power, _driving_power))
            self._tb.SetMotor1(_driving_power)
        elif self._orientation is Orientation.SAFT:
#           self._log.info(Fore.GREEN + 'target power {:5.2f} converted to driving power {:<5.2f} for SAFT motor.'.format(target_power, _driving_power))
            self._tb.SetMotor2(_driving_power)
        elif self._orientation.side is Orientation.PORT:
#           self._log.info(Fore.RED   + 'target power {:5.2f} converted to driving power {:<5.2f} for {} motor.'.format(target_power, _driving_power, self.orientation.name))
            self._tb.SetMotor1(_driving_power)
        elif self._orientation.side is Orientation.STBD:
#           self._log.info(Fore.GREEN + 'target power {:5.2f} converted to driving power {:<5.2f} for {} motor.'.format(target_power, _driving_power, self.orientation.name))
            self._tb.SetMotor2(_driving_power)
        self._last_driving_power = _driving_power

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def tb(self):
        '''
        For diagnostics only; not to be used directly.
        '''
        return self._tb

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def current_power(self):
        '''
        Makes a best attempt at getting the current power value from the motors.
        Note that the motor controller does not report absolute zero when the
        motors are not moving, but a very small positive or negative value. In
        this case we report 0.0.

        Note: a side effect of calling this method is that if it is determined
        that the motor power is close to zero we will therefore set the power
        level to zero. We could have done this elsewhere but it would have been
        more complicated. Because the motors can't move with so little power
        it's just a waste of battery power to do so.
        '''
        value = None
        count = 0
        if self._orientation is Orientation.PFWD or self._orientation is Orientation.PMID or self._orientation is Orientation.PAFT:
            while value == None and count < 20:
                count += 1
                value = self._tb.GetMotor1()
                time.sleep(0.001)
            if value == None or isclose(value, 0.0, abs_tol=1e-1):
                value = 0.0
                self._tb.SetMotor1(value)
        elif self._orientation is Orientation.SFWD or self._orientation is Orientation.SMID or self._orientation is Orientation.SAFT:
            while value == None and count < 20:
                count += 1
                value = self._tb.GetMotor2()
                time.sleep(0.001)
            if value == None or isclose(value, 0.0, abs_tol=1e-2):
                value = 0.0
                self._tb.SetMotor2(value)
        return value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def last_power(self):
        '''
        Returns the last power setting for motor.
        '''
        return self._last_driving_power

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stops the motor immediately.
        '''
        self.target_speed = 0.0
        if self._orientation is Orientation.PFWD or self._orientation is Orientation.PMID or self._orientation is Orientation.PAFT:
            self._tb.SetMotor1(0.0)
        elif self._orientation is Orientation.SFWD or self._orientation is Orientation.SMID or self._orientation is Orientation.SAFT:
            self._tb.SetMotor2(0.0)
        else:
            raise ValueError('unrecognised orientation.')
        self._log.info('{} motor stopped.'.format(self._orientation.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def off(self):
        '''
        Stops the motor entirely.
        '''
        if self._orientation is Orientation.PFWD or self._orientation is Orientation.PMID or self._orientation is Orientation.PAFT:
            self._tb.SetMotor1Off()
        elif self._orientation is Orientation.SFWD or self._orientation is Orientation.SMID or self._orientation is Orientation.SAFT:
            self._tb.SetMotor2Off()
        else:
            raise ValueError('unrecognised orientation.')
        self._log.info('{} motor off.'.format(self._orientation.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_enabled(self):
        return self.enabled

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self.enabled:
            if self._slew_limiter:
                self._slew_limiter.enable()
            if self._jerk_limiter:
                self._jerk_limiter.enable()
            Component.enable(self)
        self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            Component.disable(self)
            if self._slew_limiter:
                self._slew_limiter.disable()
            if self._jerk_limiter:
                self._jerk_limiter.disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')
        self.off() # in any case

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        # just do it anyway
        self.stop()
        if self.enabled:
            self.disable()
        if self.__max_applied_power > 0.0:
            self._log.info('on closing, maximum applied power: {:>5.2f}'.format(self.__max_applied_power))
        self._log.info('closed.')

#EOF

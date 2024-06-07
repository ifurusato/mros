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
from core.speed import Speed
from hardware.pid_ctrl import PIDController
from hardware.slew import SlewLimiter
from hardware.jerk import JerkLimiter
from hardware.velocity import Velocity

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Motor(Component):
    '''
    Controls a motor that uses a Hall Effect encoder to determine the robot's
    velocity and distance traveled.

    This Motor class takes an input as velocity (-100.0 to 100.0) which is
    pre-processed by a SlewLimiter (which buffers sudden changes to the target
    velocity), then is passed along to a PIDController, which converts the
    velocity to power (-1.0 to 1.0), which is then passed through a JerkLimiter
    to avoid sudden (and potentially dangerous) changes to the motor. All three
    are optional; when the PIDController is disabled a velocity-to-power
    dual-axis proportional interpolating function is used.

    This uses the mros:motor: section of the configuration. The suppressed state
    of the slew limiter, PID controller and jerk limiter is initially set to the
    opposite of the enabled configuration value.

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
        self._log.info(Fore.WHITE + 'initialising {} motor with {} at address 0x{:02X} as motor controller…'.format(
                orientation.name, type(self._tb).__name__, self._tb.I2cAddress) + Style.RESET_ALL)
        # configuration ..............................................
        _cfg = config['mros'].get('motor')
        self._max_velocity       = _cfg.get('maximum_velocity') # a constant, the limit to motor velocity
        self._max_fwd_velocity   = self._max_velocity           # a variable, the limit to forward velocity
        self._log.info('max velocity:\t{:<5.2f}'.format(self._max_velocity))
        self._velocity_clip = lambda n: ( -1.0 * self._max_velocity ) if n <= ( -1.0 * self._max_velocity ) \
                else min(self._max_velocity, self._max_fwd_velocity) if n >= self._max_fwd_velocity \
                else n
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
        self.__target_velocity   = 0.0   # the target velocity of the motor
        self._last_driving_power = 0.0   # last power setting for motor
        self._decoder            = None  # motor encoder
        self._slew_limiter       = None
        self._jerk_limiter       = None
        self.__velocity_lambdas  = {}
        self._verbose            = True
        # slew limiter ...............................................
        _enable_slew_limiter     = _cfg.get('enable_slew_limiter')
        _suppress_slew_limiter   = not _enable_slew_limiter
        self._slew_limiter       = SlewLimiter(config, orientation, suppressed=_suppress_slew_limiter,
                enabled=_enable_slew_limiter, level=level)
        # provides closed loop velocity feedback .....................
        self._velocity           = Velocity(config, self, level=level)
        # add callback from motor's update method
        self.add_callback(self._velocity.tick)
        self._velocity.enable()
        # pid controller .............................................
        _enable_pid_controller   = _cfg.get('enable_pid_controller')
        _suppress_pid_controller = not _enable_pid_controller
        self._pid_controller     = PIDController(config, self, suppressed=_suppress_pid_controller,
                enabled=_enable_pid_controller, level=level)
        # jerk limiter ...............................................
        _enable_jerk_limiter     = _cfg.get('enable_jerk_limiter')
        _suppress_jerk_limiter   = not _enable_jerk_limiter
#       if not _suppress_jerk_limiter and _enable_jerk_limiter:
        self._jerk_limiter       = JerkLimiter(config, orientation, suppressed=_suppress_jerk_limiter,
                enabled=_enable_jerk_limiter, level=level)
        self._indicator_callback = None
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_callback(self, callback):
        '''
        Used by the Velocity class to obtain a callback on the motor loop.
        '''
        self.__callbacks.append(callback)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_indicator_callback(self, callback):
        self._log.info('added indicator callback.')
        self._indicator_callback = callback

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def orientation(self):
        '''
        Returns the orientation of this motor.
        '''
        return self._orientation

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def max_velocity(self):
        '''
        Returns the maximum velocity in either direction.
        This is a constant, provided by application configuration.
        '''
        return self._max_velocity

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   @property
#   def max_fwd_velocity(self):
#       '''
#       Returns the maximum forward velocity limit.
#       This is a variable and can be altered to limit forward velocity.
#       '''
#       return self._max_fwd_velocity

#   @max_fwd_velocity.setter
#   def max_fwd_velocity(self, maximum_velocity):
#       '''
#       Sets the maximum forward velocity limit to the argument.
#       '''
#       self._max_fwd_velocity = maximum_velocity

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_velocity_multiplier(self, name, lambda_function):
        '''
        Adds a named velocity multiplier to the dict of lambda functions. This
        replaces any existing lambda under the same name.

        This is a function that alters the target velocity as a multiplier.
        '''
        self._log.info(Fore.GREEN + 'adding \'{}\' lambda to motor {}…'.format(name, self.orientation.name))
        if name in self.__velocity_lambdas:
            self._log.warning('motor already contains a \'{}\' lambda.'.format(name))
        self.__velocity_lambdas[name] = lambda_function

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def remove_velocity_multiplier(self, name):
        '''
        Removes a named velocity multiplier from the dict of lambda functions.
        '''
        if name in self.__velocity_lambdas:
            self._log.info(Fore.GREEN + 'removing \'{}\' lambda from motor {}…'.format(name, self.orientation.name))
            del self.__velocity_lambdas[name]
        else:
            self._log.debug('motor did not contain a \'{}\' lambda.'.format(name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def has_velocity_multiplier(self, name):
        '''
        Returns true if a named velocity multiplier exists in the dict of lambda functions.
        '''
        return name in self.__velocity_lambdas

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _reset_velocity_multiplier(self):
        '''
        Resets the velocity multiplier to None, i.e., no function.
        '''
        self.__velocity_lambdas.clear()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def remove_limiters(self):
        '''
        Used in testing to remove the slew and jerk limiters if they are by
        default set in configuration.
        '''
        self._slew_limiter = None
        self._jerk_limiter = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def slew_limiter(self):
        '''
        Returns the slew limiter used by this motor.
        This should be used only to obtain information, not for control.
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
    @property
    def velocity(self):
        '''
        Return the velocity of this Motor as a value.
        '''
        return self._velocity.value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_velocity(self):
        '''
        Return the Velocity object for this Motor.
        '''
        return self._velocity

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def target_velocity(self):
        '''
        Return the current internal target velocity of the Motor.
        '''
        return self.__target_velocity

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @target_velocity.setter
    def target_velocity(self, target_velocity):
        '''
        Set the target velocity of the Motor.
        '''
        if not isinstance(target_velocity, float):
            raise ValueError('expected float, not {}'.format(type(target_velocity)))
#       self._log.info('set target velocity of motor {} to {:5.2f}.'.format(self._orientation.name, target_velocity))
        self.__target_velocity = target_velocity
        if self._indicator_callback:
            self._indicator_callback(target_velocity)

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
            print(Fore.RED + 'callback PORT: {} steps.'.format(self.__steps))
        elif self._orientation.side is Orientation.STBD:
            self.__steps = self.__steps - pulse
            print(Fore.GREEN + 'callback STBD: {} steps.'.format(self.__steps))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_stopped(self):
        '''
         Returns True if the motor is entirely stopped.
        '''
#       return isclose(self.current_power, 0.0, abs_tol=1e-3)
        return self.current_power == 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_in_motion(self):
        '''
        Returns True if the motor is moving, i.e., if the current power
        setting of the motor is not equal to zero. Note that this returns
        False if the value is very close to zero.
        '''
#       if self._orientation is Orientation.PORT:
#           self._log.info(Fore.RED   + 'PORT current power: {:5.3f}; '.format(self.current_power))
#       else:
#           self._log.info(Fore.GREEN + 'STBD current power: {:5.3f}'.format(self.current_power))
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
    def update_target_velocity(self):
        '''
        If the current velocity doesn't match the target, set the target
        velocity and motor power as an attempt to align them.

        This method is the one that should be called on a regular basis,
        and ties the SlewLimiter, PIDController and JerkLimiter together.

        All of the dunderscored methods are intended as internal methods.
        '''
        if self.enabled:
            for callback in self.__callbacks:
                callback()
            _velocity = self._velocity.value
            _current_target_velocity = self.__target_velocity
            # set the target velocity variable modified by the slew limiter, if active
            if self._slew_limiter and self._slew_limiter.is_active:
                _current_target_velocity = self._slew_limiter.limit(self.velocity, _current_target_velocity)

            if len(self.__velocity_lambdas) > 0:
#               self._log.info(Fore.MAGENTA + 'processing {:d} lambdas…'.format(len(self.__velocity_lambdas)))
                for _name, _lambda in self.__velocity_lambdas.items():
#                   _before_lambda_velocity = _current_target_velocity
                    _current_target_velocity = _lambda(_current_target_velocity)
#                   self._log.info(Fore.WHITE + Style.BRIGHT + 'before: {}; targ_vel: {}; {} lambda for {} motor; value: {}'.format(
#                           _before_lambda_velocity, _current_target_velocity, _name, self._orientation.label, _lambda))

            # use velocity clipper as a sanity checker
            _current_target_velocity = self._velocity_clip(_current_target_velocity)

            # we now convert velocity to power, either by passing the target velocity to the PID controller (when active)
            # otherwise directly setting power to the motor via the proportional interpolator from the Speed Enum.
            if self._pid_controller.is_active: # via PID
#               self._log.info('updating {} target velocity to: {:<5.2f} (from {:5.2f})'.format(self._orientation.label, _current_target_velocity, self.__target_velocity))
                self._pid_controller.set_velocity(_current_target_velocity)
            else: # via Speed
#               _power = Speed.get_proportional_power(_current_target_velocity)
#               self.set_motor_power(_power)
#               self._log.warning('pid controller is not active.')
                raise Exception('unsupported direct speed control of motor.')

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
            raise Exception('motor {} not enabled.'.format(_motor.orientation.name))
#       _current_power = self.current_power
        # even if disabled or suppressed, JerkLimiter still clips
#       if self._jerk_limiter:
#           target_power = self._jerk_limiter.limit(self.current_power, target_power)
        # keep track of highest-applied target power
#       self.__max_applied_power = max(abs(target_power), self.__max_applied_power)

        # okay, let's go .........................
        # driving power is modified by the max power ratio to reduce battery voltage to motor voltage
#       _raw_driving_power = float(target_power * self.max_power_ratio)
        # temporary, just for safety
#       _driving_power = self._power_clip(float(target_power * self.max_power_ratio))
#       _driving_power = self._power_clip(_raw_driving_power)

        _driving_power = round(self._power_clip(float(target_power * self.max_power_ratio)), 4) # round to 4 decimal
#       if self._last_driving_power != _driving_power:
#           if self._orientation.side is Orientation.PORT:
#               self._log.info(Fore.RED   + 'target power {:5.2f} converted to driving power {:<5.2f} for {} motor.'.format(target_power, _driving_power, self.orientation.name))
#               self._tb.SetMotor1(_driving_power)
#           elif self._orientation.side is Orientation.STBD:
#               self._log.info(Fore.GREEN + 'target power {:5.2f} converted to driving power {:<5.2f} for {} motor.'.format(target_power, _driving_power, self.orientation.name))
#               self._tb.SetMotor2(_driving_power)
#           self._last_driving_power = _driving_power

        if self._orientation.side is Orientation.PORT:
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
        '''
        value = None
        count = 0
        if self._orientation is Orientation.PFWD or self._orientation is Orientation.PMID or self._orientation is Orientation.PAFT:
            while value == None and count < 20:
                count += 1
                value = self._tb.GetMotor1()
                time.sleep(0.001)
        elif self._orientation is Orientation.SFWD or self._orientation is Orientation.SMID or self._orientation is Orientation.SAFT:
            while value == None and count < 20:
                count += 1
                value = self._tb.GetMotor2()
                time.sleep(0.001)
        if value == None or isclose(value, 0.0, abs_tol=1e-1):
            return 0.0
        else:
            return value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stops the motor immediately.
        '''
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
            if self._slew_limiter:
                self._slew_limiter.disable()
            if self._jerk_limiter:
                self._jerk_limiter.disable()
            Component.disable(self)
        self._log.info('disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        if self.enabled:
            self.disable()
        if self.__max_applied_power > 0.0:
            self._log.info('on closing, maximum applied power: {:>5.2f}'.format(self.__max_applied_power))
        # just do it anyway
        self.stop()
        self._log.info('closed.')

#EOF

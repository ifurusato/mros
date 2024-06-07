#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2024-06-02
#

import sys, traceback
from threading import Thread
from colorama import init, Fore, Style
init()

from core.rate import Rate
from core.component import Component
from core.direction import Direction
from core.orientation import Orientation
from core.logger import Logger, Level
from hardware.i2c_scanner import I2CScanner
from hardware.motor_configurer import MotorConfigurer
from hardware.slew import SlewRate

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorController(Component):

    STOP_LAMBDA_NAME = "__stop"
    HALT_LAMBDA_NAME = "__halt"
    '''
    The controller for 4-6 motors:

        pfwd: Port-Forward        sfwd: Starboard-Forward
        pmid: Port-Mid            smid: Starboard-Mid
        paft: Port-Aft            saft: Starboard-Aft

    :param config:            the YAML based application configuration
    :param level:             the logging Level
    '''
    def __init__(self, config, use_external_clock=False, suppressed=False, enabled=False, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._log = Logger("motor-ctrl", level)
        Component.__init__(self, self._log, suppressed, enabled)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['mros'].get('motor_controller')
        _i2c_scanner = I2CScanner(config, level)
        self._use_ext_clock = use_external_clock
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._verbose       = _cfg.get('verbose')
        self._loop_freq_hz  = _cfg.get('loop_freq_hz') # main loop frequency
        self._loop_delay_sec = 1 / self._loop_freq_hz
        self._rate          = Rate(self._loop_freq_hz, Level.ERROR)
        self._log.info('loop frequency:\t{}Hz ({:4.2f}s)'.format(self._loop_freq_hz, self._loop_delay_sec))
        self._accel_increment   = _cfg.get('accel_increment')   # normal incremental acceleration
        self._decel_increment   = _cfg.get('decel_increment')   # normal incremental deceleration
        self._log.info('accelerate increment: {:5.2f}; decelerate increment: {:5.2f}'.format(self._accel_increment, self._decel_increment))
        self._halt_slew_rate    = SlewRate.from_string(_cfg.get('halt_rate'))
        self._log.info('halt rate:\t{}'.format(self._halt_slew_rate.name))
        self._slew_limiter_enabled = config['mros'].get('motor').get('enable_slew_limiter')
#       self._millis        = lambda: int(round(time.time() * 1000))
        # motor controller ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _motor_configurer   = MotorConfigurer(config, _i2c_scanner, motors_enabled=True, level=level)
        self._pfwd_motor    = _motor_configurer.get_motor(Orientation.PFWD)
        self._sfwd_motor    = _motor_configurer.get_motor(Orientation.SFWD)
        self._pmid_motor    = _motor_configurer.get_motor(Orientation.PMID)
        self._smid_motor    = _motor_configurer.get_motor(Orientation.SMID)
        self._paft_motor    = _motor_configurer.get_motor(Orientation.PAFT)
        self._saft_motor    = _motor_configurer.get_motor(Orientation.SAFT)
        self._all_motors    = self._get_motors()
        self._is_daemon     = True
        self._loop_thread   = None
        self._loop_enabled  = False
        self._is_stopping   = False
        self.__velocity_lambdas = {}
        # velocity and changes to velocity ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._velocity_change_multiplier = _cfg.get('velocity_change_multiplier') # multiplier for velocity change events
        self._theta         = 0.0
        self._stbd_velocity = 0.0
        self._port_velocity = 0.0
        _max_velocity  = _cfg.get('max_velocity') # max velocity of motors (0-100)
        _min_velocity  = -1 * _max_velocity
        self._clamp = lambda n: max(min(_max_velocity, n), _min_velocity)
        # stop lambda slows down quickly
        _stopping_decel = 3.0
        self._stopping_lambda = lambda v: max(v - _stopping_decel, 0) if v > 0.0 else v
        # halt lambda slows down slowly
        _halting_decel = 1.0
        self._halting_lambda = lambda v: max(v - _halting_decel, 0) if v > 0.0 else v
        # zero lambda always returns a zero value
        self._zero_lambda = lambda: 0
        # finish up…
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motors(self):
        '''
        Returns a list containing all instantiated motors.
        This includes only instantiated motors.
        '''
        return self._all_motors

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_motors(self):
        '''
        Returns a list of all extant motors.
        '''
        _list = []
        if self._pfwd_motor:
            _list.append(self._pfwd_motor)
        if self._sfwd_motor:
            _list.append(self._sfwd_motor)
        if self._pmid_motor:
            _list.append(self._pmid_motor)
        if self._smid_motor:
            _list.append(self._smid_motor)
        if self._paft_motor:
            _list.append(self._paft_motor)
        if self._saft_motor:
            _list.append(self._saft_motor)
        return _list

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motor(self, orientation):
        '''
        Returns the motor corresponding to the orientation.
        '''
        if orientation is Orientation.PFWD:
            return self._pfwd_motor
        elif orientation is Orientation.SFWD:
            return self._sfwd_motor
        elif orientation is Orientation.PMID:
            return self._pmid_motor
        elif orientation is Orientation.SMID:
            return self._smid_motor
        elif orientation is Orientation.PAFT:
            return self._paft_motor
        elif orientation is Orientation.SAFT:
            return self._saft_motor
        else:
            raise Exception('unsupported orientation.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enables the motors. This issues a warning if already enabled, but
        no harm is done in calling it repeatedly.
        '''
        if self.enabled:
            self._log.warning('already enabled.')
        else:
            Component.enable(self)
            if not self._use_ext_clock and not self.loop_is_running:
                self._start_loop()
            self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the motors, halting first if in motion.
        '''
        if self.enabled:
            self._log.info('disabling…')
            self.stop_loop() # stop loop thread
            Component.disable(self)
#           _count = 0
#           while _count < 10 and self.is_in_motion: # if we're moving then halt
#               _count += 1
#               self._log.warning('[{:d}] event: motors are in motion (halting).'.format(_count))
#               self._pfwd_motor = _motor_configurer.get_motor(Orientation.PFWD)
#               self._sfwd_motor = _motor_configurer.get_motor(Orientation.SFWD)
#               self._pmid_motor = _motor_configurer.get_motor(Orientation.PMID)
#               self._smid_motor = _motor_configurer.get_motor(Orientation.SMID)
#               self._paft_motor = _motor_configurer.get_motor(Orientation.PAFT)
#               self._saft_motor = _motor_configurer.get_motor(Orientation.SAFT)
#               self._port_motor.stop()
#               self._stbd_motor.stop()
#               time.sleep(0.1)
            self._log.info('disabled.')
        else:
            self._log.debug('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _start_loop(self):
        '''
        Start the loop Thread.

        If we're using an external clock, calling this method throws an exception.
        '''
        self._log.info('start motor control loop…')
        if not self.enabled:
            raise Exception('not enabled.')
        if self.loop_is_running:
            self._log.warning('loop already running.')
        elif self._loop_thread is None:
            if self._use_ext_clock:
                raise Exception('cannot use thread-based loop: external clock enabled.')
            self._loop_enabled = True
            self._loop_thread = Thread(name='motor_loop', target=MotorController._motor_loop, args=[self, lambda: self._loop_enabled], daemon=self._is_daemon)
            self._loop_thread.start()
            self._log.info('loop enabled.')
        else:
            raise Exception('cannot enable loop: thread already exists.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop_loop(self):
        '''
        Stop the loop.
        '''
        self._log.info(Style.BRIGHT + 'loop stop.')
        if self.loop_is_running:
            self._loop_enabled = False
            self._loop_thread  = None
            self._log.info('loop disabled.')
        else:
            self._log.warning('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def loop_is_running(self):
        '''
        Returns true if using an external clock or if the loop thread is alive.
        '''
        return self._loop_enabled and self._loop_thread != None and self._loop_thread.is_alive()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _motor_loop(self, f_is_enabled):
        '''
        The motors loop, which executes while the flag argument lambda is True.
        '''
        self._log.info('loop start.')
        self._log.info(Style.BRIGHT + 'loop start.')
        try:


            while f_is_enabled():
                for _motor in self._all_motors:
                    self._log.info(Fore.GREEN + '🍓 updating {} motor…'.format(_motor.orientation.name))

#               if len(self.__velocity_lambdas) > 0:
#                   self._log.info(Fore.MAGENTA + 'processing {:d} lambdas…'.format(len(self.__velocity_lambdas)))
#                   for _name, _lambda in self.__velocity_lambdas.items():
#                       _current_target_velocity = _lambda(_current_target_velocity)
#                       self._log.info(Fore.WHITE + Style.BRIGHT + 'before: {}; targ_vel: {}; {} lambda for {} motor; value: {}'.format(
#                               _before_lambda_velocity, _current_target_velocity, _name, self._orientation.label, _lambda))

                    _motor.update_target_velocity()
                # add execute any callbacks here…
                if self._verbose: # print stats
                    self.print_info(next(self._event_counter))
                self._rate.wait()
        except Exception as e:
            self._log.error('error in loop: {}\n{}'.format(e, traceback.format_exc()))
        finally:
            self._log.info(Fore.GREEN + 'exited motor control loop.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _external_callback_method(self):
        '''
        The callback called by the external clock as an alternative to the
        asyncio _loop() method.
        '''
        if self.enabled:
#           _now = self._millis()
            for _motor in self._all_motors:
                _motor.update_target_velocity()
#           _elapsed = _now - self._start_time
#           self._start_time = _now
#           if self._verbose: # print stats
#               self.print_info(next(self._event_counter))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_stopping(self):
        # then we've reached a stop, so remove any stopping features
        self._log.info(Fore.GREEN + '😭 reset stopping.')
        self._is_stopping = False
        for _motor in self._all_motors:
            if _motor.has_velocity_multiplier(MotorController.STOP_LAMBDA_NAME):
                _motor.remove_velocity_multiplier(MotorController.STOP_LAMBDA_NAME)
            if _motor.has_velocity_multiplier(MotorController.HALT_LAMBDA_NAME):
                _motor.remove_velocity_multiplier(MotorController.HALT_LAMBDA_NAME)
        self._reset_slew_rate()

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
    @property
    def is_stopped(self):
        '''
        Returns true if the velocity of both motors is zero.
        '''
        _not_in_motion = not self.is_in_motion
        if self._is_stopping and _not_in_motion:
            self.reset_stopping()
        return _not_in_motion

  # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_in_motion(self):
        '''
        Returns true if any of the motors are moving, i.e., if the motor
        power of any motor is greater than zero.
        '''
        for _motor in self._all_motors:
            if _motor.is_in_motion:
                return True
        return False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_info(self, count):
        if self.is_stopped:
            self._log.info(('[{:04d}] '.format(count) if count else '') + 'velocity: stopped.')
        else:
            self._log.info(('[{:04d}] '.format(count) if count else '')
                    + 'velocity: '
                    # fwd ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                    + Fore.RED   + 'port: {:<5.2f} -> {:<5.2f} / {:<5.2f}'.format(
                            self._pfwd_motor.velocity, self._pfwd_motor.target_velocity, self._pfwd_motor.current_power)
                    + ' {:5.2f}'.format(self._pfwd_motor.current_power)
                    + Fore.CYAN  + ' :: '
                    + Fore.GREEN + 'stbd: {:<5.2f} -> {:<5.2f} / {:<5.2f}'.format(
                            self._sfwd_motor.velocity, self._sfwd_motor.target_velocity, self._sfwd_motor.current_power)
                    + Fore.CYAN + ' :: movement: {}'.format(self._characterise_movement())
                    # mid ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                    + Fore.RED   + 'port: {:<5.2f} -> {:<5.2f} / {:<5.2f}'.format(
                            self._pmid_motor.velocity, self._pmid_motor.target_velocity, self._pmid_motor.current_power)
                    + ' {:5.2f}'.format(self._pmid_motor.current_power)
                    + Fore.CYAN  + ' :: '
                    + Fore.GREEN + 'stbd: {:<5.2f} -> {:<5.2f} / {:<5.2f}'.format(
                            self._smid_motor.velocity, self._smid_motor.target_velocity, self._smid_motor.current_power)
                    + Fore.CYAN + ' :: movement: {}'.format(self._characterise_movement())
                    # aft ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                    + Fore.RED   + 'port: {:<5.2f} -> {:<5.2f} / {:<5.2f}'.format(
                            self._paft_motor.velocity, self._paft_motor.target_velocity, self._paft_motor.current_power)
                    + ' {:5.2f}'.format(self._paft_motor.current_power)
                    + Fore.CYAN  + ' :: '
                    + Fore.GREEN + 'stbd: {:<5.2f} -> {:<5.2f} / {:<5.2f}'.format(
                            self._saft_motor.velocity, self._saft_motor.target_velocity, self._saft_motor.current_power)
                    + Fore.CYAN + ' :: movement: {}'.format(self._characterise_movement()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_velocity(self, orientation, value):
        '''
        Sets the value of the port or starboard velocity.

        As David Anderson says, we should use velocity and rotation, not port and starboard motor speeds.

            port      = velocity + rotation
            starboard = velocity - rotation

        where we use 'theta' as rotation.
        '''
        if self._is_stopping and value != 0.0:
            # then it's no longer the case
            self.reset_stopping()
        _multiplier = 0.5
        _value = value * _multiplier
        _color = None
        _port_style = Style.NORMAL
        _stbd_style = Style.NORMAL
        if orientation is Orientation.PORT:
            _color = Fore.RED
            _port_style = Style.BRIGHT
            self._port_velocity = self._clamp(_value)
            self.set_motor_velocity(Orientation.PORT, self._port_velocity)
        elif orientation is Orientation.STBD:
            _color = Fore.GREEN
            _stbd_style = Style.BRIGHT
            self._stbd_velocity = self._clamp(_value)
            self.set_motor_velocity(Orientation.STBD, self._stbd_velocity)
        else:
            raise Exception('unsupported orientation {}'.format(orientation.name))
#       self._theta = ( self._port_velocity / 100.0 ) - ( self._stbd_velocity / 100.0 )
#       _display_theta = int( 100.0 * self._theta )
#       self._log.info(_color + "add value {}; velocity: ".format(value)
#               + _port_style + " port={:.1f}".format(self._port_velocity)
#               + _stbd_style + " stbd={:.1f}".format(self._stbd_velocity)
#               + Fore.WHITE + Style.BRIGHT + ' theta: {:d}'.format(_display_theta))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_motor_velocity(self, orientation, target_velocity):
        '''
        A convenience method that sets the target velocity and motor power of
        the specified orientation (PORT or STBD). Accepts either ints or floats
        between -100 and 100.

        When the motor controller is disabled any calls to this method will
        override the target velocity argument and set it to zero.
        '''
        if not self.enabled:
            self._log.error('motor controller not enabled.')
            target_velocity = 0.0
        if isinstance(target_velocity, int):
            raise ValueError('expected target velocity as float not int: {:d}'.format(target_velocity))
        if not isinstance(target_velocity, float):
            raise ValueError('expected float, not {}'.format(type(target_velocity)))
        if orientation is Orientation.PORT:
            self._pfwd_motor.target_velocity = target_velocity
#           self._pmid_motor.target_velocity = target_velocity
            self._paft_motor.target_velocity = target_velocity
            self._log.info('set motor velocity ' + Fore.RED   + 'PORT: {:5.2f}'.format(target_velocity))
        elif orientation is Orientation.STBD:
            self._sfwd_motor.target_velocity = target_velocity
#           self._smid_motor.target_velocity = target_velocity
            self._saft_motor.target_velocity = target_velocity
            self._log.info('set motor velocity ' + Fore.GREEN + 'STBD: {:5.2f}'.format(target_velocity))
        else:
            raise TypeError('expected PORT or STBD orientation, not {}'.format(orientation))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clamp(self, value):
        '''
        Return the clamp lambda function.
        '''
        return self._clamp(value)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _reset_slew_rate(self):
        '''
        Halts any automated deceleration.
        '''
        for _motor in self._all_motors:
            _motor.slew_limiter.reset()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_slew_rate(self, slew_rate):
        '''
        Set the slew rate for both motors to the argument.
        '''
        for _motor in self._all_motors:
            _motor.slew_limiter.slew_rate = slew_rate

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def brake(self):
        '''
        Slowly coasts both motors to a stop.
        '''
        self._log.info(Fore.MAGENTA + Style.BRIGHT + 'B: BRAKE')
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def attach_halt_lambda(self, value):
        self._log.warning('🍅 attach halt lambda... value: {}'.format(value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def halt(self):
        '''
        Quickly (but not immediately) stops both motors.
        '''
        self._log.info(Fore.MAGENTA + Style.BRIGHT + 'A: HALT')
        if self.is_stopped:
            self._log.warning('already halted.')
            return
        elif self._is_stopping:
            self._log.warning('already halting.')
            return
        else:
            self._log.info('halting…')
        self._is_stopping = True
        if self._use_ext_clock or self._loop_enabled:
            if self._slew_limiter_enabled:
                self._log.info('halting soft…')
                # use slew limiter for halting if available
                self._set_slew_rate(self._halt_slew_rate)
                for _motor in self._all_motors:
                    _motor.add_velocity_multiplier(MotorController.HALT_LAMBDA_NAME, self._halting_lambda)
#                   _motor.target_velocity = 0.0
                self.set_velocity(Orientation.PORT, 0.0)
                self.set_velocity(Orientation.STBD, 0.0)
#               for _motor in self._all_motors:
#                   _motor.target_velocity = 0.0
            else:
                self._log.info('halting hard…')
                for _motor in self._all_motors:
                    _motor.target_velocity = 0.0
        else:
            self._log.info('halting very hard…')
            self.emergency_stop()
        self._log.info('halted.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stops both motors immediately, with no slewing.

        This differs from both halt() and brake() in that it also suppresses
        all behaviours.
        '''
        self._log.info(Fore.MAGENTA + Style.BRIGHT + 'Y: STOP')
        if self.is_stopped:
            self._log.warning('already stopped.')
            # but we do it anyway...
        else:
            self._log.info('stopping...')
        # suppress any behaviours
#       self._suppress_behaviours()
        if self.loop_is_running:
            print('🍋 loop_is_running')
            if self._slew_limiter_enabled:
                print('🍋 slew enabled: stopping SOFT')
                self._log.info('stopping soft...')
                try:
                    # temporarily suppress slew
                    for _motor in self._all_motors:
                        _slew_limiter_suppressed = _motor.slew_limiter.suppressed
                        _motor.target_velocity = 0.0
                finally:
                    for _motor in self._all_motors:
                        _motor.slew_limiter.release
            else:
                print('🍋 slew not enabled: stopping HARD')
                # the last two blocks aren't currently very different from each other
                self._log.info('stopping hard...')
                # set velocity but don't wait, just call stop
                for _motor in self._all_motors:
                    _motor.target_velocity = 0.0
                self._reset_slew_rate() # go back to default slew rate
                for _motor in self._all_motors:
                    # we rely on this ultimately
                    _motor.target_velocity = 0.0
                    _motor.stop()
        else:
            print('💔 loop is not running')
            self._log.warning('loop is not running.')
            self.emergency_stop()

#       self._log.warning('STOPPING NOW...')
#       for _motor in self._all_motors:
#           # we rely on this ultimately
#           _motor.stop()
#       self._log.info('stopped.')


    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def emergency_stop(self):
        self._log.info('emergency stop…')
        for _motor in self._all_motors:
            # we rely on this ultimately
            _motor.target_velocity = 0.0
            _motor.stop()
        self._log.info('stopped.')
        self._log.info('emergency stopped.')

#EOF

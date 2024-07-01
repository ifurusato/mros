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
import time
import statistics
import itertools
from math import isclose
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
from hardware.slew_rate import SlewRate

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorController(Component):
    '''
    The controller for 4-6 motors:

        pfwd: Port-Forward        sfwd: Starboard-Forward
        pmid: Port-Mid            smid: Starboard-Mid
        paft: Port-Aft            saft: Starboard-Aft

    This permits speed change lambda functions to be added to the motors
    to alter their behaviour, such as coming to a halt.

    The contract that speed change lambdas have is that they pass a lambda
    function and the target speed as arguments, and return an altered target
    speed OR the name of the originating lambda, in the case where the lambda
    has completed and should no longer be processed (i.e., it should be removed).
    The passed lambda returns True only when all motors are clear of speed
    change lambdas.

    By default, lambdas alter the target speed only for that 20Hz cycle, unless
    the lambda has 'accum' in its name, which causes changes to accumulate. The
    latter are used for stopping modes.

    :param config:            the YAML based application configuration
    :param external_clock     the optional external clock (in lieu of a thread loop)
    :param suppressed         if True the controller is suppressed
    :param enabled            if True the controller is enabled upon instantiation
    :param level:             the logging Level
    '''
    def __init__(self, config, external_clock=None, suppressed=False, enabled=False, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._log = Logger("motor-ctrl", level)
        Component.__init__(self, self._log, suppressed, enabled)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['mros'].get('motor_controller')
        _i2c_scanner = I2CScanner(config, level)
        self._external_clock = external_clock
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._verbose        = _cfg.get('verbose')
        self._loop_freq_hz   = _cfg.get('loop_freq_hz') # main loop frequency
        self._loop_delay_sec = 1 / self._loop_freq_hz
        self._rate           = Rate(self._loop_freq_hz, Level.ERROR)
        self._log.info('loop frequency:\t{}Hz ({:4.2f}s)'.format(self._loop_freq_hz, self._loop_delay_sec))
        self._halt_slew_rate = SlewRate.from_string(_cfg.get('halt_rate'))
        self._log.info('halt rate:\t{}'.format(self._halt_slew_rate.name))
        # slew limiters are on motors, not here
        self._slew_limiter_enabled = config['mros'].get('motor').get('enable_slew_limiter')
        # motor controller ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _motor_configurer    = MotorConfigurer(config, _i2c_scanner, motors_enabled=True, level=level)
        self._pfwd_motor     = _motor_configurer.get_motor(Orientation.PFWD)
        self._sfwd_motor     = _motor_configurer.get_motor(Orientation.SFWD)
        self._pmid_motor     = _motor_configurer.get_motor(Orientation.PMID)
        self._smid_motor     = _motor_configurer.get_motor(Orientation.SMID)
        self._paft_motor     = _motor_configurer.get_motor(Orientation.PAFT)
        self._saft_motor     = _motor_configurer.get_motor(Orientation.SAFT)
        self._all_motors     = self._get_motors()
        self._is_daemon      = True
        self._loop_thread    = None
        self._loop_enabled   = False
        self._event_counter  = itertools.count()
        # speed and changes to speed ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._theta          = 0.0
        self._stbd_speed     = 0.0
        self._port_speed     = 0.0
        self._speed_scale_factor = _cfg.get('scale_factor') # effectively sets max power to motors
        _max_speed           = _cfg.get('max_speed') # max speed of motors (0-100)
        _min_speed           = -1 * _max_speed
        self._log.info('motor speed clamped at {} to {}.'.format(_min_speed, _max_speed))
        self._clamp          = lambda n: max(min(_max_speed, n), _min_speed)
        self._print_info_done = False
        # finish up…
        self._log.info('ready with {} motors.'.format(len(self._all_motors)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_mean_speed(self, orientation):
        '''
        Return the mean speed of the motors as characterised by the
        orientation argument, PORT, STBD, or CNTR as a proxy for all
        motors.
        '''
        _speeds = []
        if orientation is Orientation.CNTR:
            for _motor in self._get_motors():
                self._log.info('AVERAGING with speed of {} motor speed: modified={:.2f}; speed={:.2f}; target={:.2f}'.format(_motor.orientation.name,
                        _motor.modified_speed, _motor.speed, _motor.target_speed))
                _speeds.append(_motor.speed)
#               _speeds.append(_motor.modified_speed)

        elif orientation is Orientation.PORT:
            _speeds.append(self._pfwd_motor.modified_speed)
#           _speeds.append(self._pmid_motor.modified_speed)
            _speeds.append(self._paft_motor.modified_speed)
        elif orientation is Orientation.STBD:
            _speeds.append(self._sfwd_motor.modified_speed)
#           _speeds.append(self._smid_motor.modified_speed)
            _speeds.append(self._saft_motor.modified_speed)
        return statistics.fmean(_speeds)

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
            if self._external_clock:
                self._external_clock.add_callback(self.external_callback_method)
                for _motor in self._all_motors:
                    _motor.enable()
#               self.set_speed(Orientation.PORT, 0.0)
#               self.set_speed(Orientation.STBD, 0.0)
            elif not self.loop_is_running:
                self._start_loop()
            self._log.info('enabled.')

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
            if self._external_clock:
                raise Exception('cannot use thread-based loop: external clock enabled.')
            self._loop_enabled = True
            self._loop_thread = Thread(name='motor_loop', target=MotorController._motor_loop, args=[self, lambda: self._loop_enabled], daemon=self._is_daemon)
            self._loop_thread.start()
            self._log.info('loop enabled.')
        else:
            raise Exception('cannot enable loop: thread already exists.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _stop_loop(self):
        '''
        Stop the motor control loop.
        '''
        if self.loop_is_running:
            self._loop_enabled = False
            self._loop_thread  = None
            self._log.info(Style.BRIGHT + 'stopped motor control loop.')
        else:
            self._log.warning('motor control loop already disabled.')

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
                    self._log.info(Fore.GREEN + 'updating {} motor…'.format(_motor.orientation.name))
                    _motor.update_target_speed()
                # add execute any callbacks here…
                if self._verbose: # print stats
                    _count = next(self._event_counter)
                    if _count % 20 == 0:
                        self.print_info(_count)
                self._rate.wait()
        except Exception as e:
            self._log.error('error in loop: {}\n{}'.format(e, traceback.format_exc()))
        finally:
            self._log.info(Fore.GREEN + 'exited motor control loop.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def external_callback_method(self):
        '''
        The callback called by the external clock as an alternative to the
        asyncio _loop() method.
        '''
        if self.enabled:
            for _motor in self._all_motors:
                if _motor.enabled:
                    _motor.update_target_speed()
            if self._verbose: # print stats
                _count = next(self._event_counter)
                if _count % 10 == 0:
                    self.print_info(_count)
        else:
            self._log.warning('not enabled: external callback ignored.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_stopping(self):
        _lambda_count = 0
        for _motor in self._all_motors:
            _lambda_count += _motor.speed_multiplier_count
        return _lambda_count > 0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_stopping(self):
        # then we've reached a stop, so remove any stopping features
        self._log.info(Fore.GREEN + 'reset stopping.')
        for _motor in self._all_motors:
            if _motor.has_speed_multiplier(MotorController.STOP_LAMBDA_NAME):
                _motor.remove_speed_multiplier(MotorController.STOP_LAMBDA_NAME)
            if _motor.has_speed_multiplier(MotorController.HALT_LAMBDA_NAME):
                _motor.remove_speed_multiplier(MotorController.HALT_LAMBDA_NAME)
            if _motor.has_speed_multiplier(MotorController.BRAKE_LAMBDA_NAME):
                _motor.remove_speed_multiplier(MotorController.BRAKE_LAMBDA_NAME)
        self._reset_slew_rate()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_stopped(self):
        '''
        Returns true if the speed of all motors is zero.
        '''
        _not_in_motion = not self.is_in_motion
#       if self.is_stopping() and _not_in_motion:
#           self.reset_stopping()
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
    def set_speed(self, orientation, value):
        '''
        Sets the speed of all motors associated with the port or
        starboard orientation.
        '''
        _color = None
        _port_style = Style.NORMAL
        _stbd_style = Style.NORMAL
        if orientation is Orientation.PORT:
            _color = Fore.RED
            _port_style = Style.BRIGHT
            self._port_speed = self._clamp(value)
            self.set_motor_speed(Orientation.PFWD, self._port_speed)
#           self.set_motor_speed(Orientation.PMID, self._port_speed)
            self.set_motor_speed(Orientation.PAFT, self._port_speed)
        elif orientation is Orientation.STBD:
            _color = Fore.GREEN
            _stbd_style = Style.BRIGHT
            self._stbd_speed = self._clamp(value)
            self.set_motor_speed(Orientation.SFWD, self._stbd_speed)
#           self.set_motor_speed(Orientation.SMID, self._stbd_speed)
            self.set_motor_speed(Orientation.SAFT, self._stbd_speed)

        else:
            raise Exception('unsupported orientation {}'.format(orientation.name))
#       self._theta = ( self._port_speed / 100.0 ) - ( self._stbd_speed / 100.0 )
#       _display_theta = int( 100.0 * self._theta )
#       self._log.info(_color + "add value {:.2f}; speed: ".format(value)
#               + _port_style + " port={:.2f}".format(self._port_speed)
#               + _stbd_style + " stbd={:.2f}".format(self._stbd_speed))
#               + Fore.WHITE + Style.BRIGHT + ' theta: {:d}'.format(_display_theta))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_motor_speed(self, orientation, target_speed):
        '''
        A convenience method that sets the target speed and motor power of
        the specified motor, as identified by Orientation. Accepts either
        ints or floats between -100 and 100.

        When the motor controller is disabled any calls to this method will
        override the target speed argument and set it to zero.
        '''
        if not self.enabled:
            self._log.error('motor controller not enabled.')
            target_speed = 0.0
        if isinstance(target_speed, int):
            raise ValueError('expected target speed as float not int: {:d}'.format(target_speed))
        if not isinstance(target_speed, float):
            raise ValueError('expected float, not {}'.format(type(target_speed)))
        _argument = target_speed
        target_speed *= self._speed_scale_factor
#       self._log.info('set {} argument: {:5.2f}; motor speed: {:5.2f}'.format(orientation.name, _argument, target_speed))
        if orientation is Orientation.PFWD and self._pfwd_motor.enabled:
            self._pfwd_motor.target_speed = target_speed
#           self._log.info('set PFWD motor speed ' + Fore.RED   + 'PORT: {:5.2f}'.format(target_speed))
        if orientation is Orientation.SFWD and self._sfwd_motor.enabled:
            self._sfwd_motor.target_speed = target_speed
#           self._log.info('set SFWD motor speed ' + Fore.GREEN + 'STBD: {:5.2f}'.format(target_speed))
#       if orientation is Orientation.PMID:
#           self._pmid_motor.target_speed = target_speed
#           self._log.info('set PMID motor speed ' + Fore.RED   + 'PORT: {:5.2f}'.format(target_speed))
#       if orientation is Orientation.SMID:
#           self._smid_motor.target_speed = target_speed
#           self._log.info('set SMID motor speed ' + Fore.GREEN + 'STBD: {:5.2f}'.format(target_speed))
        if orientation is Orientation.PAFT and self._paft_motor.enabled:
            self._paft_motor.target_speed = target_speed
#           self._log.info('set PAFT motor speed ' + Fore.RED   + 'PORT: {:5.2f}'.format(target_speed))
        if orientation is Orientation.SAFT and self._saft_motor.enabled:
            self._saft_motor.target_speed = target_speed
#           self._log.info('set SAFT motor speed ' + Fore.GREEN + 'STBD: {:5.2f}'.format(target_speed))
#       else:
#           raise TypeError('expected a motor orientation, not {}'.format(orientation))

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
        Set the slew rate for all motors to the argument.
        '''
        for _motor in self._all_motors:
            _motor.slew_limiter.slew_rate = slew_rate

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def all_motors_are_stopped(self):
        '''
        Returns True when all motors are stopped.
        '''
        for _motor in self._all_motors:
            if not _motor.is_stopped:
                return False
        return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear_speed_multipliers(self):
        '''
        Clear all motor speed control lambda functions from all motors.
        '''
        self._log.info(Fore.MAGENTA + Style.BRIGHT + 'A. clear lambdas: xxxxxx xxxxxx xxxxxx xxxxx xxxxxxx xxxxxx ')
        for _motor in self._all_motors:
            if _motor:
                _motor.clear_speed_multipliers()
            else:
                raise Exception('null motor in clear list.')
        self._log.info(Fore.MAGENTA + Style.BRIGHT + 'B. clear lambdas: xxxxxx xxxxxx xxxxxx xxxxx xxxxxxx xxxxxx ')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def list_speed_multipliers(self):
        '''
        List the motor speed control lambda functions from all motors.
        '''
        self._log.info(Fore.GREEN + 'listing speed multipliers for {:d} motors:'.format( len(self._all_motors)))
        for _motor in self._all_motors:
            if _motor:
                _motor.list_speed_multipliers()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _braking_function(self, target_speed):
        '''
        This is a lambda function that will slow the motors to zero speed
        at a rather slow rate.
        '''
        target_speed = target_speed * self._brake_ratio
        if self.all_motors_are_stopped:
            # return lambda name indicating we're done
            return MotorController.BRAKE_LAMBDA_NAME
        elif isclose(target_speed, 0.0, abs_tol=1e-2):
            return 0.0
        else:
            return target_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _halting_function(self, target_speed):
        '''
        This is a lambda function that returns the target speed diminished
        by the halt ratio, until either it reaches near zero (and then returns
        zero), or at such time when there are no lambdas operating on this or
        any other motor, in which case it returns the name of the lambda as a
        signal that the robot has stopped.
        '''
        target_speed = target_speed * self._halt_ratio
        if self.all_motors_are_stopped:
            # return lambda name indicating we're done
            return MotorController.HALT_LAMBDA_NAME
        elif isclose(target_speed, 0.0, abs_tol=1e-2):
            return 0.0
        else:
            return target_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _stopping_function(self, target_speed):
        '''
        This is a lambda function that will slow the motors to zero speed
        very quickly. This additionally directly calls stop() on all motors.
        '''
        target_speed = target_speed * self._stop_ratio
        if self.all_motors_are_stopped:
            self._log.info('full stop now…')
            for _motor in self._all_motors:
                # we rely on this ultimately
                _motor.stop()
            self._log.info('stopped.')
            # return lambda name indicating we're done
            return MotorController.STOP_LAMBDA_NAME
        elif isclose(target_speed, 0.0, abs_tol=1e-2):
            return 0.0
        else:
            return target_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Stops all motors immediately, with no slewing.

        This differs from both halt() and brake() in that it also suppresses
        all behaviours. TODO
        '''
        self._log.info(Fore.MAGENTA + Style.BRIGHT + 'Y: STOP')
        if self.is_stopped:
            # just in case a motor is still moving
            for _motor in self._all_motors:
                _motor.stop()
            self._log.warning('already stopped.')
            return
        elif self.is_stopping():
            self._log.warning('already stopping.')
            return
        else:
            self._log.info('stopping…')
        if self._external_clock or self._loop_enabled:
            if self._slew_limiter_enabled:
                self._log.info('stopping soft…')
                # use slew limiter for stopping if available
#               self._set_slew_rate(self._stop_slew_rate)
                for _motor in self._all_motors:
                    _motor.add_speed_multiplier(MotorController.STOP_LAMBDA_NAME, self._stopping_function)
            else:
                self._log.info('stopping hard…')
                for _motor in self._all_motors:
                    _motor.stop()
        else:
            self._log.info('stopping very hard…')
            self.emergency_stop()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def emergency_stop(self):
        '''
        We try to avoid this as it's hard on the gears.
        '''
        self._log.info('emergency stop…')
        for _motor in self._all_motors:
            # we rely on this ultimately
            _motor.stop()
        self._log.info('emergency stopped.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the motors, halting first if in motion.
        '''
        if self.enabled:
            self._log.info('disabling…')
            if self._external_clock:
                self._external_clock.remove_callback(self.external_callback_method)
            else:
                self._stop_loop() # stop loop thread if we're using it
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.debug('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Closes the motor controller.
        '''
        if not self.closed:
            Component.close(self) # calls disable
            self._log.info('motor controller closed.')
        else:
            self._log.warning('motor controller already closed.')

    # reporting ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_info(self, count):
        if self.is_stopped:
            if not self._print_info_done:
                self._log.info(('[{:04d}] '.format(count) if count else '') + 'speed: stopped.')
            self._print_info_done = True
        else:
            self._print_info_done = False
            self._log.info(('[{:04d}] '.format(count) if count else '')
                    + 'speed: '
                    # fwd ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                    + Fore.RED   + 'pfwd: {:<4.2f} / {:<4.2f}'.format(
                            self._pfwd_motor.speed, self._pfwd_motor.modified_speed)
                    + Fore.CYAN  + ' :: '
                    + Fore.GREEN + 'sfwd: {:<4.2f} / {:<4.2f}'.format(
                            self._sfwd_motor.speed, self._sfwd_motor.modified_speed)
#                   # mid ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#                   + Fore.RED   + 'pmid: {:<4.2f} / {:<4.2f}'.format(
#                           self._pmid_motor.speed, self._pmid_motor.modified_speed)
#                   + Fore.CYAN  + ' :: '
#                   + Fore.GREEN + 'smid: {:<4.2f} / {:<4.2f}'.format(
#                           self._smid_motor.speed, self._smid_motor.modified_speed)
#                   + Fore.CYAN + ' :: movement: {}'.format(self._characterise_movement())
                    # aft ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                    + ' :: '
                    + Fore.RED   + 'paft: {:<4.2f} / {:<4.2f}'.format(
                            self._paft_motor.speed, self._paft_motor.modified_speed)
                    + Fore.CYAN  + ' :: '
                    + Fore.GREEN + 'saft: {:<4.2f} / {:<4.2f}'.format(
                            self._saft_motor.speed, self._saft_motor.modified_speed)
                    + Fore.CYAN + ' :: movement: {}'.format(self._characterise_movement()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _characterise_movement(self):
        '''
        Return a string characterising the robot's movement based on the
        direction of the port and starboard motors.
        '''
        return self._get_movement_description(self.get_mean_speed(Orientation.PORT), self.get_mean_speed(Orientation.STBD))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_movement_description(self, port_velocity, stbd_velocity):
        _avg_velocity = ( port_velocity + stbd_velocity ) / 2.0
        # FIXME use existing methods!
        if isclose(port_velocity, 0.0, abs_tol=0.5) and isclose(stbd_velocity, 0.0, abs_tol=0.5):
            # close to stopped
            return 'stopped'
        elif isclose(port_velocity, stbd_velocity, abs_tol=0.02):
            if port_velocity > 0.0:
                return 'straight ahead'
            else:
                return 'straight astern'
        elif isclose(_avg_velocity, 0.0, abs_tol=0.5):
            if port_velocity > stbd_velocity:
                return 'rotate to starboard'
            elif port_velocity < stbd_velocity:
                return 'rotate to port'
            else:
                return 'indeterminate (0)'
        elif _avg_velocity > 0.0:
            if port_velocity > stbd_velocity:
                return 'turn ahead to starboard'
            elif port_velocity < stbd_velocity:
                return 'turn ahead to port'
            else:
                return 'ahead indeterminate (1)'
        elif _avg_velocity < 0.0:
            if port_velocity > stbd_velocity:
                return 'turn astern to starboard'
            elif port_velocity < stbd_velocity:
                return 'turn astern to port'
            else:
                return 'astern indeterminate (2)'


#EOF

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-06-24
# modified: 2024-06-24
#

import traceback # TEMP
import time
import types
from datetime import datetime as dt
import itertools
from math import isclose
from colorama import init, Fore, Style
init(autoreset=True)

from core.logger import Logger, Level
from core.event import Event, Group
from core.component import Component

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class StopHandler(Component):
    CLASS_NAME = 'stop'
    BRAKE_LAMBDA_NAME  = "__brake_accum" 
    HALT_LAMBDA_NAME   = "__halt_accum" 
    STOP_LAMBDA_NAME   = "__stop_accum" 
    '''
    Handles Stop group events.

        Brake:           Slowly coasts all motors to a stop.
        Halt:            Quickly (but not immediately) stops all motors.
        Stop:            Stops all motors almost immediately, with no slewing.
        Emergency Stop:  Sends cuts power to the motors, stopping them
                         immediately. This is very hard on the gears. 

    :param config:            the application configuration
    :param motor_controller:  the robot's motor controller 
    :param level:             the logging level
    '''
    def __init__(self, config, motor_controller, level=Level.INFO):
        self._log = Logger('stop', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._motor_controller = motor_controller
        self._all_motors = self._motor_controller.get_motors()
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        # slew limiters are on motors, not here
        self._slew_limiter_enabled = config['mros'].get('motor').get('enable_slew_limiter')
        _cfg = config['mros'].get('stop_handler')
        self._brake_ratio = _cfg.get('brake_ratio')
        self._halt_ratio  = _cfg.get('halt_ratio')
        self._stop_ratio  = _cfg.get('stop_ratio')
        self._log.info('ratios: brake={:4.2f}; halt={:4.2f}; stop={:4.2f}'.format(self._brake_ratio, self._halt_ratio, self._stop_ratio))
        self._last_event  = None
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def process_message(self, message):
        '''
        Process the message.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        _event = message.event
        self._log.info('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
        _value = message.value
        if _event.num != self._last_event:
            if _event.num == Event.BRAKE.num:
                self.brake()
            elif _event.num == Event.HALT.num:
                self.halt()
            elif _event.num == Event.STOP.num:
                self.stop()
            elif _event.num == Event.EMERGENCY_STOP.num:
                self._emergency_stop()
            else:
                raise Exception('unrecognised stop event: {}'.format(_event.name))
        self._last_event = _event.num
        # TEMP
        self._last_event = None
    
        self._log.info('post-processing message {}'.format(message.name))

        # Brake: Slowly coasts all motors to a stop.

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def brake(self, callback=None):
        '''
        Slowly coasts all motors to a stop.
        Executes the callback once the robot is no longer moving.
        '''
        self._log.info('brake.')
        if callback is None:
            callback = lambda: self._log.info('brake complete.')
        self._stopping_function(lambda_name=StopHandler.BRAKE_LAMBDA_NAME, lambda_function=self._braking_lambda, timeout_ms=6000, callback=callback)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def halt(self, callback=None):
        '''
        Quickly (but not immediately) stops all motors.
        Executes the callback once the robot is no longer moving.
        '''
        self._log.info('halt.')
        if callback is None:
            callback = lambda: self._log.info('halt complete.')
        self._stopping_function(lambda_name=StopHandler.HALT_LAMBDA_NAME, lambda_function=self._halting_lambda, timeout_ms=2500, callback=callback)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self, callback=None):
        '''
        Stops all motors very quickly.
        Executes the callback once the robot is no longer moving.
        '''
        self._log.info('stop.')
        if callback is None:
            callback = lambda: self._log.info('stop complete.')
        self._stopping_function(lambda_name=StopHandler.STOP_LAMBDA_NAME, lambda_function=self._stopping_lambda, timeout_ms=1000, callback=callback)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _stopping_function(self, lambda_name=None, lambda_function=None, timeout_ms=5000, callback=None):
        '''
        Halts the robot. Executes the callback once the robot has stopped.
        '''
        if callback is not None and not isinstance(callback, types.FunctionType):
            raise ValueError('expected an callback argument, not {}'.format(type(callback)))
        self._log.info('stopping function: {} with timeout of {:d}ms.'.format(lambda_name, timeout_ms))
        if self._motor_controller.is_stopped:
            # just in case a motor is still moving
            for _motor in self._all_motors:
                _motor.stop()
            self._log.warning('already halted.')
        elif self._motor_controller.is_stopping():
            self._log.warning('already halting.')
        else:
            self._log.info('halting…')
            if self._slew_limiter_enabled:
                self._log.info('soft halt…')
                try:
                    for _motor in self._all_motors:
                        _motor.add_speed_multiplier(lambda_name, lambda_function)
                    if callback:
                        _count = 0
                        _counter = itertools.count()
                        _start_time = dt.now()
                        _elapsed_ms = 0
                        while not self._motor_controller.is_stopped and _elapsed_ms < timeout_ms:
                            _count = next(_counter)
                            _elapsed_ms = int((dt.now() - _start_time).total_seconds() * 1000)
                            self._log.info(Style.DIM + '[{:02d}] waiting til stopped; {:4d}ms elapsed.'.format(_count, _elapsed_ms))
                            time.sleep(0.1)
                        self._log.info('stopped.')
                        # TODO check if timed out and still moving? If so, emergency stop...
                finally:
                    self._log.info('removing motor lambdas…')
                    for _motor in self._all_motors:
                        _motor.remove_speed_multiplier(lambda_name)
            else:   
                self._log.info('hard halt…')
                for _motor in self._all_motors:
                    _motor.stop()
        # done .....................
        if callback:
            try:
                self._log.info('executing callback of type: {}…'.format(type(callback)))
                callback()
            except Exception as e:
                self._log.error('{} thrown executing callback: {}\n{}'.format(type(e), e, traceback.format_exc()))

        print('🍐 z. halt complete.')

    # lambda functions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    '''
    These are the lambda functions that return the target speed diminished
    by the brake/halt/stop ratio, until either it reaches near zero (and 
    then returns zero), or at such time when there are no lambdas operating 
    on this or any other motor, in which case it returns the name of 
    the lambda as a signal that the robot has halted.
    '''

    def _braking_lambda(self, target_speed):
        '''
        The brake lambda.
        '''
        target_speed = target_speed * self._brake_ratio
        if self._motor_controller.all_motors_are_stopped:
            return StopHandler.BRAKE_LAMBDA_NAME
        elif isclose(target_speed, 0.0, abs_tol=1e-2):
            return 0.0
        else:
            return target_speed

    def _halting_lambda(self, target_speed):
        '''
        The halt lambda.
        '''
        target_speed = target_speed * self._halt_ratio
        if self._motor_controller.all_motors_are_stopped:
            return StopHandler.HALT_LAMBDA_NAME
        elif isclose(target_speed, 0.0, abs_tol=1e-2):
            return 0.0
        else:
            return target_speed

    def _stopping_lambda(self, target_speed):
        '''
        The stop lambda.
        '''
        target_speed = target_speed * self._stop_ratio
        if self._motor_controller.all_motors_are_stopped:
            return StopHandler.STOP_LAMBDA_NAME
        elif isclose(target_speed, 0.0, abs_tol=1e-2):
            return 0.0
        else:
            return target_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _emergency_stop(self):
        ''' 
        We try to avoid this as it's really hard on the gears.
        '''
        self._log.info('emergency stop…')
        for _motor in self._all_motors:
            # we rely on this ultimately
            _motor.stop()
        self._log.info('emergency stopped.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            Component.disable(self)
        else:
            self._log.warning('stop component already disabled.')

#EOF

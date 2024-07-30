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

from math import isclose
from colorama import init, Fore, Style
init(autoreset=True)

from core.logger import Logger, Level
from core.event import Event, Group
from core.component import Component

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class StopHandler(Component):
    CLASS_NAME = 'stop'
    STOP_LAMBDA_NAME  = "__stop_accum" 
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
            if _event.num == Event.EMERGENCY_STOP.num:
                self._emergency_stop()
            else:
                self._stop(_event)
        self._last_event = _event.num
        # TEMP
        self._last_event = None
    
        self._log.info('post-processing message {}'.format(message.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _stopping_lambda(self, target_speed):
        '''
        This is a lambda function that returns the target speed diminished
        by the stopping ratio, until either it reaches near zero (and then 
        returns zero), or at such time when there are no lambdas operating 
        on this or any other motor, in which case it returns the name of 
        the lambda as a signal that the robot has stopped.
        '''
        target_speed = target_speed * self._brake_ratio
        if self._motor_controller.all_motors_are_stopped:
            # return lambda name indicating we're done
            return StopHandler.STOP_LAMBDA_NAME
        elif isclose(target_speed, 0.0, abs_tol=1e-2):
            return 0.0
        else:
            return target_speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        '''
        Quickly stops the robot.
        '''
        self._stop(Event.STOP)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _stop(self, event):
        '''
        Slowly coasts all motors to a stop.
        '''
        self._log.info(Fore.GREEN + Style.BRIGHT + "STOP event: '{}'".format(event.name))
#       if self.is_stopped:
#           self._log.warning('already braked.')
#           return
#       elif self.is_stopping():
#           self._log.warning('already braking.')
#           return
#       else:
#           self._log.info('braking…')
        if self._slew_limiter_enabled:
            self._log.info(Fore.YELLOW + 'stopping motors…') # stopping soft
            for _motor in self._all_motors:
#               self._log.info(Fore.YELLOW + 'adding stopping lambda to {} motor…'.format(_motor.orientation.name))
                _motor.clear_speed_multipliers()
                _motor.add_speed_multiplier(StopHandler.STOP_LAMBDA_NAME, self._stopping_lambda)
        else:   
            self._log.info('stopping hard…')
            for _motor in self._all_motors:
                _motor.target_speed = 0.0
#       self._log.info('braking very hard…')
#       self.emergency_stop()

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

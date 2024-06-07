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

from datetime import datetime as dt
from math import isclose
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.rate import Rate
from core.event import Event, Group
from core.orientation import Orientation
from core.subscriber import Subscriber
from core.logger import Logger, Level
from hardware.servo_controller import ServoController, SteeringMode
from hardware.headlight import Headlight
from hardware.servo import Servo
from hardware.motor_controller import MotorController

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotionController(Subscriber):
    CLASS_NAME= 'motion'
    '''
    The MotionController combines motor and servo control as a subscriber
    to various types of events.

    :param config:            the YAML based application configuration
    :param message_bus:       the asynchronous message bus
    :param level:             the logging Level
    '''
    def __init__(self, config, message_bus, suppressed=False, enabled=False, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._log = Logger("motion-ctrl", level)
        Subscriber.__init__(self, MotionController.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        if config is None:
            raise ValueError('no configuration provided.')
        self._mros = globals.get('mros')
        self._config = config['mros'].get('hardware').get('motion_control')
        # servo controller # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._servo_controller = ServoController(config, level=level)
        self._all_servos = self._servo_controller.get_servos()
        self._log.info('created servo controller with {} servos.'.format(len(self._all_servos)))
        # motor controller # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _use_external_clock = config['mros'].get('use_external_clock')
        self._motor_controller = MotorController(config, use_external_clock=_use_external_clock, level=level)
        self._all_motors = self._motor_controller.get_motors()
        self._log.info('created motor controller with {} motors.'.format(len(self._all_motors)))
        # subscribe to event groups ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self.add_events([
                Group.GAMEPAD,
                Group.STOP,
                Group.VELOCITY,
            ])
        self._log.info(Fore.WHITE + 'registered {} events.'.format(len(self._events)) + Style.RESET_ALL)
        
        # headlight ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._headlight = Headlight(Orientation.STBD)
        # finish up…
        globals.put('motion-ctrl', self)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def servo_controller(self):
        return self._servo_controller

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def motor_controller(self):
        return self._motor_controller

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
#       if message.gcd:
#           raise GarbageCollectedError('cannot process message: message has been garbage collected.')
#       _event = message.event
#       self._log.info('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.label))
#       await Subscriber.process_message(self, message)
#       self._log.info('post-processing message {}'.format(message.name))
#       if message.gcd:
#           raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        _event = message.event
        self._log.debug('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.label))
        _value = message.value
        self._repeat_event = False
        _event_num = _event.num

        # alternatives to port/stbd velocities (would be stepped):
#           VELOCITY               = ( 200, "velocity",               100,   Group.VELOCITY ) # with value
#           PORT_VELOCITY          = ( 201, "port velocity",          100,   Group.VELOCITY ) # with value
#           STBD_VELOCITY          = ( 202, "stbd velocity",          100,   Group.VELOCITY ) # with value
#           DECREASE_VELOCITY      = ( 203, "decrease velocity",      100,   Group.VELOCITY ) # step change
#           INCREASE_VELOCITY      = ( 204, "increase velocity",      100,   Group.VELOCITY ) # step change
#           DECREASE_PORT_VELOCITY = ( 205, "decrease port velocity", 100,   Group.VELOCITY ) # step change
#           INCREASE_PORT_VELOCITY = ( 206, "increase port velocity", 100,   Group.VELOCITY ) # step change
#           DECREASE_STBD_VELOCITY = ( 207, "decrease stbd velocity", 100,   Group.VELOCITY ) # step change
#           INCREASE_STBD_VELOCITY = ( 208, "increase stbd velocity", 100,   Group.VELOCITY ) # step change

        if _event_num == Event.A_BUTTON.num: # A_BUTTON                 = ( 41, "a-cross",    10, Group.GAMEPAD)
            self._log.warning('should not be receiving A_BUTTON; value: {}'.format(_value))
        elif _event_num == Event.HALT.num: # HALT
            self._log.info('♑ handling A_BUTTON; value: {}'.format(_value))
            self._motor_controller.halt()

        elif _event_num == Event.B_BUTTON.num: # B_BUTTON               = ( 42, "b-circle",   10, Group.GAMEPAD)
            self._handle_gamepad_B_BUTTON(_value)

        elif _event_num == Event.X_BUTTON.num: # X_BUTTON               = ( 43, "x-triangle", 10, Group.GAMEPAD)
            self._log.warning('should not be receiving A_BUTTON; value: {}'.format(_value))
        elif _event_num == Event.STOP.num: # STOP
            self._log.info('♑ handling X_BUTTON; value: {}'.format(_value))
            self._motor_controller.stop()

        elif _event_num == Event.Y_BUTTON.num: # Y_BUTTON               = ( 44, "y-square",   10, Group.GAMEPAD)
            self._handle_gamepad_Y_BUTTON(_value)

        elif _event_num == Event.L1_BUTTON.num: # L1_BUTTON              = ( 45, "l1",         10, Group.GAMEPAD)
            self._handle_gamepad_L1_BUTTON(_value)
        elif _event_num == Event.EMERGENCY_STOP.num: # EMERGENCY_STOP
#       elif _event_num == Event.L2_BUTTON.num: # EMERGENCY_STOP
            self._motor_controller.emergency_stop()

        elif _event_num == Event.R1_BUTTON.num: # R1_BUTTON              = ( 47, "r1",         10, Group.GAMEPAD)
#           self._handle_gamepad_R1_BUTTON(_value)
            self._motor_controller.attach_halt_lambda(_value) # TEMP

        elif _event_num == Event.R2_BUTTON.num: # R2_BUTTON              = ( 48, "r2",         10, Group.GAMEPAD)
#           self._handle_gamepad_R2_BUTTON(_value)
            if self._headlight:
                self._headlight.toggle()

        elif _event_num == Event.START_BUTTON.num: # START_BUTTON        = ( 49, "start",      10, Group.GAMEPAD)
            self._handle_gamepad_START_BUTTON(_value)
        elif _event_num == Event.SELECT_BUTTON.num: # SELECT_BUTTON      = ( 50, "select",     10, Group.GAMEPAD)
            self._handle_gamepad_SELECT_BUTTON(_value)
        elif _event_num == Event.HOME_BUTTON.num: # HOME_BUTTON          = ( 51, "home",       10, Group.GAMEPAD)
            self._handle_gamepad_HOME_BUTTON(_value)
        elif _event_num == Event.DPAD_HORIZONTAL.num: # DPAD_HORIZONTAL  = ( 52, "dpad-h",     10, Group.GAMEPAD)
            self._handle_gamepad_DPAD_HORIZONTAL(_value)
        elif _event_num == Event.DPAD_VERTICAL.num: # DPAD_VERTICAL      = ( 53, "dpad-v",     10, Group.GAMEPAD)
            self._handle_gamepad_DPAD_VERTICAL(_value)

        elif _event_num == Event.L3_VERTICAL.num: # L3_VERTICAL          = ( 54, "l3-vert",    10, Group.GAMEPAD)
#           self._log.debug('handling L3_VERTICAL; value: {}'.format(type(_value)))
            self._motor_controller.set_velocity(Orientation.PORT, -1 * (_value - 127))
        elif _event_num == Event.L3_HORIZONTAL.num: # L3_HORIZONTAL      = ( 55, "l3-horz",    10, Group.GAMEPAD)
            # These messages should be suppressed as we don't pay attention to the
            # horizontal aspect of the joysticks.
            self._log.warning('should not be receiving L3_HORIZONTAL; value: {}'.format(_value))
        elif _event_num == Event.R3_VERTICAL.num: # R3_VERTICAL          = ( 56, "r3-vert",    10, Group.GAMEPAD)
#           self._log.debug('handling R3_VERTICAL; value: {}'.format(type(_value)))
            self._motor_controller.set_velocity(Orientation.STBD, -1 * (_value - 127))
        elif _event_num == Event.R3_HORIZONTAL.num: # R3_HORIZONTAL      = ( 57, "r3-horz",    10, Group.GAMEPAD)
            # These messages should be suppressed as we don't pay attention to the
            # horizontal aspect of the joysticks.
            self._log.warning('should not be receiving R3_HORIZONTAL; value: {}'.format(_value))

        # direct mappings .................................................
        else:
            self._log.warning('unrecognised event on message {}'.format(message.name) + ''.format(message.event.label))
        await Subscriber.process_message(self, message)
        self._log.debug('post-processing message {}'.format(message.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_B_BUTTON(self, value):
        self._log.info('♑ handling B_BUTTON; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_Y_BUTTON(self, value):
        self._log.warning(Style.BRIGHT + '♑ SHOULD NOT BE handling Y_BUTTON; value: {}'.format(value))
#       self._shutdown()
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_L1_BUTTON(self, value):
        self._log.info('♑ handling L1_BUTTON; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_R1_BUTTON(self, value):
        self._log.info('♑ handling R1_BUTTON; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_R2_BUTTON(self, value):
        self._log.info('♑ handling R2_BUTTON; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_START_BUTTON(self, value):
        self._log.info('♑ handling START_BUTTON; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_SELECT_BUTTON(self, value):
        self._log.info('♑ handling SELECT_BUTTON; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_HOME_BUTTON(self, value):
        self._log.info('♑ handling HOME_BUTTON; value: {}'.format(value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_DPAD_HORIZONTAL(self, value):
        self._log.info('♑ handling DPAD_HORIZONTAL; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_DPAD_VERTICAL(self, value):
        self._log.info('♑ handling DPAD_VERTICAL; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'motion-ctrl'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_steering_mode(self, mode):
        self._servo_controller.set_mode(mode)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def command(self, remote):
        '''
        Responds to a remote command.

        Note that the X and Y buttons are swapped between the Explorer
        shield and the 8BitDo N30 Pro gamepad.
        '''
        self._log.info('command: {}'.format(remote.name))
        if remote   is Event.REMOTE_A:
            self._log.info(Fore.MAGENTA + Style.BRIGHT + 'A: START')
            self.enable()
        elif remote is Event.REMOTE_B:
            self._motor_controller.brake()
        elif remote is Event.REMOTE_Y:
            self._motor_controller.stop()
        elif remote is Event.REMOTE_X:
            self._log.info(Fore.MAGENTA + Style.BRIGHT + 'X: SHUTDOWN ')
            self._shutdown()
        elif remote is Event.REMOTE_D:
            self._log.info(Fore.MAGENTA + Style.BRIGHT + 'D: SLOW DOWN')
        elif remote is Event.REMOTE_R:
            self._log.info(Fore.MAGENTA + Style.BRIGHT + 'R: SLOW STARBOARD MOTOR')
        elif remote is Event.REMOTE_U:
            self._log.info(Fore.MAGENTA + Style.BRIGHT + 'U: SPEED UP')
        elif remote is Event.REMOTE_L:
            self._log.info(Fore.MAGENTA + Style.BRIGHT + 'L: SLOW PORT MOTOR')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _shutdown(self):
        self._log.info(Fore.CYAN + Style.BRIGHT + '😧 SHUTTING DOWN!')
        self.disable()
        self.close()
        if self._mros:
            self._mros.shutdown()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enables the motors. This issues a warning if already enabled, but no
        harm is done in calling it repeatedly.
        '''
        if not self.closed:
            if not self.enabled:
                self._log.info('enabling motion controller…')
                for _motor in self._all_motors:
                    if _motor:
                        _motor.enable()
                # starts motor loop thread
                self._motor_controller.enable()
                for _servo in self._all_servos:
                    if _servo:
                        _servo.enable()
                self._servo_controller.enable()
                self._log.info('motion controller enabled.')
            else:
                self._log.warning('motion controller already enabled.')
        else:
            self._log.warning('motion controller closed.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disables both the motors and servos.
        '''
        if not self.closed:
            if self.enabled:
                self._log.info('disabling motion controller…')
                self._motor_controller.disable()
                for _motor in self._all_motors:
                    if _motor:
                        _motor.disable()
                self._servo_controller.recenter()
                self._servo_controller.disable()
                for _servo in self._all_servos:
                    if _servo:
                        _servo.disable()
                self._log.info('motion controller disabled.')
            else:
                self._log.warning('motion controller already disabled.')
        else:
            self._log.warning('motion controller closed.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Closes both the motors and servos.
        '''
        if not self.closed:
            Subscriber.close(self) # calls disable
            self._log.info('motion controller closed.')
        else:
            self._log.warning('motion controller already closed.')

#EOF

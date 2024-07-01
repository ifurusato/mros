#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# author:   Jon Hylands for the calculate_afrs_steering() function (thank you!)
# created:  2020-10-05
# modified: 2024-06-21
#

import sys, traceback
from datetime import datetime as dt
import time
from math import isclose
import math
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.rate import Rate
from core.util import Util
from core.chadburn import Chadburn
from core.steering_angle import SteeringAngle
from core.event import Event, Group
from core.orientation import Orientation
from core.subscriber import Subscriber
from core.logger import Logger, Level
from hardware.stop_handler import StopHandler
from hardware.analog_pot import AnalogPotentiometer
from hardware.digital_pot import DigitalPotentiometer
from hardware.servo_controller import ServoController, SteeringMode
from hardware.headlight import Headlight
from hardware.servo import Servo
from hardware.sensor_array import SensorData
from hardware.motor_controller import MotorController

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotionController(Subscriber):
    CLASS_NAME= 'motion'
    PORT_AFRS_STEERING_LAMBDA_NAME  = "__port_afrs_steering"
    STBD_AFRS_STEERING_LAMBDA_NAME  = "__stbd_afrs_steering"
    '''
    The MotionController combines motor and servo control as a subscriber
    to various types of events.

    :param config:            the YAML based application configuration
    :param message_bus:       the asynchronous message bus
    :param external_clock:    the optional external clock (used by the motor controller)
    :param level:             the logging Level
    '''
    def __init__(self, config, message_bus, external_clock=None, suppressed=False, enabled=False, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._log = Logger("motion-ctrl", level)
        Subscriber.__init__(self, MotionController.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        if config is None:
            raise ValueError('no configuration provided.')
        self._mros = globals.get('mros')
        # geometry # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._half_length  = config.get('mros').get('geometry').get('wheel_base') / 2
        self._half_width   = config.get('mros').get('geometry').get('wheel_track') / 2
        self._wheel_offset = config.get('mros').get('geometry').get('wheel_offset')
        # motion controller # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['mros'].get('motion_controller')
        self._afrs_max_ratio = _cfg.get('afrs_max_ratio') # ratio at maximum turn
        _max_angle       = _cfg.get('afrs_max_angle')     # maximum AFRS inner turn angle
        self._afrs_clamp = lambda n: max(min(_max_angle, n), -1.0 * _max_angle)
        self._incr_clamp = lambda n: max(min(7, n), -7)
        # servo controller # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._servo_controller = ServoController(config, level=level)
        self._all_servos = self._servo_controller.get_servos()
        self._log.info('created servo controller with {} servos.'.format(len(self._all_servos)))
        self._afrs_angle = 0 # inner angle set by R3 Horiz
        self._steering_angle_index = 0
        self._steering_angle = SteeringAngle.STRAIGHT_AHEAD # enumerated by DPad
        self._steering_mode = None
        # lambdas to alter speed for steering ┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._port_motor_ratio = 1.0
        self._stbd_motor_ratio = 1.0
        self._port_steering_lambda = lambda target_speed: target_speed * self._port_motor_ratio
        self._stbd_steering_lambda = lambda target_speed: target_speed * self._stbd_motor_ratio
        # motor controller # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _use_external_clock = config['mros'].get('use_external_clock')
        self._motor_controller = MotorController(config, external_clock=external_clock, level=level)
        self._all_motors = self._motor_controller.get_motors()
        self._log.info('created motor controller with {} motors.'.format(len(self._all_motors)))
        self._speed_value = 0.0 # set by L3 Vertical
        # stop handler # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._stop_handler = StopHandler(config, self._motor_controller, level)
        # subscribe to event groups ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self.add_events(Event.by_groups([Group.GAMEPAD, Group.BUMPER, Group.STOP, Group.VELOCITY]))
        self._log.info(Fore.WHITE + 'registered {} events.'.format(len(self._events)) + Style.RESET_ALL)
        # manual velocity control ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        USE_ANALOG_POT = False
        if USE_ANALOG_POT:
            self._speed_pot = AnalogPotentiometer(config, level=level)
        else:
            self._speed_pot = DigitalPotentiometer(config, level=level)
            self._speed_pot.set_output_range(0.0, 1.0)
        # headlight ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._headlight = Headlight(Orientation.STBD)
        # chadburn events ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._chadburn_index = 0
        self._chadburn = Chadburn.STOP
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
#       self._log.info('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
#       await Subscriber.process_message(self, message)
#       self._log.info('post-processing message {}'.format(message.name))
#       if message.gcd:
#           raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        _event = message.event
#       self._log.debug('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
        _value = message.value
        self._repeat_event = False
        _event_num = _event.num

        if _event.group is Group.STOP:
            if _value == 0:
                self._log.info('♑ handling STOP; value: {}'.format(_value))
                self._stop_handler.process_message(message)

        elif _event.group is Group.BUMPER:
            self._log.info('♍ handling BUMPER; value: {}'.format(type(_value)))
            _sensor_data = _value
            # from hardware.sensor_array import SensorData
            _events = _sensor_data.events
            _fop_cm = _sensor_data.fop_cm
            _fos_cm = _sensor_data.fos_cm
            self._log.info('♍ handling BUMPER; events: {}; fop {:d}cm; fos: {:d}cm'.format(_events, _fop_cm, _fos_cm))
            self._stop_handler.process_message(message)

        elif _event_num == Event.L1_BUTTON.num:
            if _value == 0:
                self._log.info('♑ handling L1_BUTTON; value: {}'.format(_value))
                self._servo_controller.recenter()

        elif _event_num == Event.L2_BUTTON.num:
            if _value == 0:
                self._log.info('♑ handling L2_BUTTON; value: {}'.format(_value))

        elif _event_num == Event.SHUTDOWN: # SHUTDOWN (Y)
            if _value == 0:
                self._shutdown()

        elif _event_num == Event.A_BUTTON.num: # A_BUTTON = ( 42, "b-circle",   10, Group.GAMEPAD)
            if _value == 0:
                self._handle_gamepad_A_BUTTON(_value)

        elif _event_num == Event.B_BUTTON.num: # B_BUTTON = ( 42, "b-circle",   10, Group.GAMEPAD)
            if _value == 0:
                self._handle_gamepad_B_BUTTON(_value)

        elif _event_num == Event.R1_BUTTON.num: # R1_BUTTON = ( 47, "r1",         10, Group.GAMEPAD)
            if _value == 0:
                self.set_manual_speed()

        elif _event_num == Event.R2_BUTTON.num: # R2_BUTTON = ( 48, "r2",         10, Group.GAMEPAD)
#           self._handle_gamepad_R2_BUTTON(_value)
            if _value == 0 and self._headlight:
                self._headlight.toggle()

        elif _event_num == Event.START_BUTTON.num: # START_BUTTON        = ( 49, "start",      10, Group.GAMEPAD)
            self._handle_gamepad_START_BUTTON(_value)
        elif _event_num == Event.SELECT_BUTTON.num: # SELECT_BUTTON      = ( 50, "select",     10, Group.GAMEPAD)
            self._handle_gamepad_SELECT_BUTTON(_value)
        elif _event_num == Event.HOME_BUTTON.num: # HOME_BUTTON          = ( 51, "home",       10, Group.GAMEPAD)
            if _value == 0:
                self._handle_gamepad_HOME_BUTTON(_value)

        elif _event_num == Event.DPAD_LEFT.num or _event_num == Event.DPAD_RIGHT.num:
            self.increment_steering_angle(_value)

        elif _event_num == Event.DPAD_UP.num or _event_num == Event.DPAD_DOWN.num:
            self.increment_speed(_value)

        elif _event_num == Event.L3_VERTICAL.num: # L3_VERTICAL          = ( 54, "l3-vert",    10, Group.GAMEPAD)
            if _value != self._speed_value: # only pay attention to changes
                self.change_speed(_value)
                self._speed_value = _value

        elif _event_num == Event.L3_HORIZONTAL.num: # L3_HORIZONTAL      = ( 55, "l3-horz",    10, Group.GAMEPAD)
            # These messages should be suppressed as we don't pay attention to the
            # horizontal aspect of the left joystick.
            self._log.warning('should not be receiving L3_HORIZONTAL; value: {}'.format(_value))

        elif _event_num == Event.R3_VERTICAL.num: # R3_VERTICAL          = ( 56, "r3-vert",    10, Group.GAMEPAD)
            self._log.debug(Style.DIM + 'ignoring R3_VERTICAL; value: {}'.format(type(_value)))
            pass

        elif _event_num == Event.R3_HORIZONTAL.num: # R3_HORIZONTAL      = ( 57, "r3-horz",    10, Group.GAMEPAD)
            self.set_afrs_steering_angle(_value)

        # direct mappings ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        else:
            self._log.warning('unrecognised event on message {}'.format(message.name) + ''.format(message.event.name))

        self._log.info('almost-post-processing message {}'.format(message.name))
        await Subscriber.process_message(self, message)
        self._log.info('post-processing message {}'.format(message.name))

   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def calculate_afrs_steering(self, inside_steering_angle):
        '''
        author: Jon Hylands
        source: https://discord.com/channels/835614845129719828/835618978096218112/1243244829866197042
        '''
#       self._log.info(Fore.BLACK + 'set AFRS servo angle to {:.2f}°'.format(inside_steering_angle) + Style.RESET_ALL)
        if inside_steering_angle == 0:
            return 0

        # First, inside radius
        inside_steering_rads = math.radians(inside_steering_angle)
        inside_distance = self._half_length / (math.tan(inside_steering_rads))
        inside_hyp = math.sqrt(self._half_length ** 2 + inside_distance ** 2)
        inside_radius = inside_hyp - self._wheel_offset

        # Second, outside radius
        outside_distance = inside_distance + (self._half_width * 2)
        outside_hyp = math.sqrt(outside_distance ** 2 + self._half_length ** 2)
        outside_radius = outside_hyp + self._wheel_offset

        # Finally, outside angle
        outside_steering_rads = math.atan2(self._half_length, outside_distance)
        outside_steering_angle = math.degrees(outside_steering_rads)

#       return (inside_radius, outside_steering_angle, outside_radius)
        return outside_steering_angle

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def steering_translation(self, value):
        '''
        A simple range translator with an input range of 0-45°, an output
        range between 1.0 and 0.36016, a constant determined by the geometry
        of the robot.
        '''
        return 1.0 + (float(value) / 45.0 * (self._afrs_max_ratio - 1.0))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def increment_steering_angle(self, value):
        '''
        Increment or decrement the steering angle via enumerated values.
        This spans between COUNTER_CLOCKWISE_45 and CLOCKWISE_45.
        values.
        '''

#       elif abs(self._afrs_angle) >= 45.0 and abs(inner_angle) >= 45.0: # if we're already at 45° then spin

        if value == -1: # decrement
            if self._steering_angle_index > SteeringAngle.COUNTER_CLOCKWISE_45.num:
                self._steering_angle_index -= 1
                self._steering_angle = SteeringAngle.from_index(self._steering_angle_index)
                self._log.info(Style.NORMAL + '🥑 A1. decrement_steering_angle: {}; value: {}'.format(self._steering_angle, self._steering_angle.value))
                self._set_afrs_steering_angle_from_inner_angle(self._steering_angle.value)
            else:
                self._log.info(Style.BRIGHT + '🥑 A2. IGNORE call to decrement_steering_angle: {}; value: {}'.format(self._steering_angle, self._steering_angle.value))
        elif value == 1: # increment
            if self._steering_angle_index < SteeringAngle.CLOCKWISE_45.num:
                self._steering_angle_index += 1
                self._steering_angle = SteeringAngle.from_index(self._steering_angle_index)
                self._log.info(Style.NORMAL + '🥑 B1. increment_steering_angle: {}; value: {}'.format(self._steering_angle, self._steering_angle.value))
                self._set_afrs_steering_angle_from_inner_angle(self._steering_angle.value)
            else:
                self._log.info(Style.BRIGHT + '🥑 B2. IGNORE call to increment_steering_angle: {}; value: {}'.format(self._steering_angle, self._steering_angle.value))

        self._log.info('steering angle: ' + Style.BRIGHT + '{}'.format(self._steering_angle.name) + Style.NORMAL + '; angle: {:.2f}°'.format(self._steering_angle.value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_afrs_steering_angle(self, value):
        '''
        Sets both the AFRS steering angle and alters the lambdas on the
        motors so that the inner motors are going proportionately slower
        than the outer motors.

        When we are turning the inner wheels have a tighter turning radius
        and proportionately slower speed than the outer wheels.

        We clamp the inner angle to -45°/45°.
        '''
        _inner_angle = int(Util.remap_range(value, 0, 255, -45, 45))
        self._set_afrs_steering_angle_from_inner_angle(_inner_angle)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_afrs_steering_angle_from_inner_angle(self, inner_angle):
        '''
        Gets the outer angle based on the inner angle and calls the
        servo controller to perform the move.
        '''
        if inner_angle != self._afrs_angle: # only pay attention to changes
            self.set_steering_mode(SteeringMode.AFRS)
#           self._servo_controller.set_mode(SteeringMode.AFRS)
            inner_angle = self._afrs_clamp(inner_angle)
            _outer_angle = int(self.calculate_afrs_steering(abs(inner_angle)))

            self._log.info(Style.BRIGHT + 'set_afrs_steering_angle: inner: {}; outer: {}'.format(inner_angle, _outer_angle))

            _style = Style.NORMAL
#           self._afrs_max_ratio = _cfg.get('afrs_max_ratio') # ratio at maximum turn
            if inner_angle == 0: # no rotation
                _style = Style.BRIGHT
                _direction = 'STRAIGHT AHEAD'
                _port_angle = 0.0
                _stbd_angle = 0.0
                self._port_motor_ratio = 1.0
                self._stbd_motor_ratio = 1.0

            elif inner_angle < 0: # counter-clockwise (port is inner)
                _style = Fore.RED
                _direction = 'counter-clockwise'
                self._port_motor_ratio = self.steering_translation(-1 * inner_angle)
                self._stbd_motor_ratio = 1.0
                _port_angle = inner_angle
                _stbd_angle = _outer_angle

            else: # clockwise (stbd is inner)
                _style = Fore.GREEN
                _direction = 'clockwise'
                self._port_motor_ratio = 1.0
                self._stbd_motor_ratio = self.steering_translation(inner_angle)
                _port_angle = _outer_angle
                _stbd_angle = inner_angle

            self._servo_controller.set_afrs_angle(_port_angle, _stbd_angle)

            self._log.info('turning: ' + _style + '{} '.format(_direction) + Fore.CYAN + Style.NORMAL + 'inner angle: {}°; '.format(inner_angle)
                    + Fore.RED + 'port: {}°; ratio: {:.2f}; '.format(_port_angle, self._port_motor_ratio)
                    + Fore.GREEN + 'stbd: {}°; ratio: {:.2f}'.format(_stbd_angle, self._stbd_motor_ratio) + Style.RESET_ALL)

            self._afrs_angle = inner_angle

        else:
            self._port_motor_ratio = 1.0
            self._stbd_motor_ratio = 1.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def change_speed(self, value):
        '''
        Change the current speed (for all motors) by a scaled increment
        based on the left joystick's value.
        '''
        _scale_factor = 0.01428 / 5.0
        _vel = 127 - value
        _color = Fore.GREEN
        if _vel < 0:
            _color = Fore.MAGENTA
        _increment = self._incr_clamp(int(_vel / 16.0))
        _speed_delta = _increment * _scale_factor
        # now apply delta to all motors
#       _mean_speed = self._motor_controller.get_mean_speed(Orientation.CNTR)
        if not isclose(_speed_delta, 0.0, abs_tol=1e-2):
            for _motor in self._motor_controller.get_motors():
                _target_speed = _motor.speed + _speed_delta
                self._log.info(Fore.GREEN + 'handling L3_VERTICAL for {} motor with current speed {:.2f}; delta: {:7.4f}; target speed: {:4.2f}; '.format(
                        _motor.orientation.name, _motor.speed, _speed_delta, _target_speed))
                self._motor_controller.set_motor_speed(_motor.orientation, _target_speed)
        else:
            self._log.info(Fore.WHITE + 'handling L3_VERTICAL; delta: {:7.4f}; vel: {}; value: {}; '.format(_speed_delta, _vel, value) + _color + ' bar: {}'.format(self.scale(abs(_increment))))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def increment_speed(self, value):
        '''
        Increment or decrement the Chadburn speed via enumerated values.
        This spans between FULL_ASTERN and FULL_AHEAD, not using the MAXIMUM
        values.
        '''
        CHADBURN_MATCH = True
        if CHADBURN_MATCH:
            _mean_speed = self._motor_controller.get_mean_speed(Orientation.CNTR)
            _chadburn = Chadburn.get_closest_value(_mean_speed)
            self._log.info('closest chadburn: ' + Style.BRIGHT + '{}'.format(_chadburn.name) + Style.NORMAL + '; speed: {:.2f}'.format(_chadburn.speed))
            # alter existing to match:
            self._chadburn = _chadburn
            self._chadburn_index = self._chadburn.num
        if value == -1: # increment
            if self._chadburn_index < Chadburn.FULL_AHEAD.num:
                self._chadburn_index += 1
                self._chadburn = Chadburn.from_index(self._chadburn_index)
        else: # decrement
            if self._chadburn_index > Chadburn.FULL_ASTERN.num:
                self._chadburn_index -= 1
                self._chadburn = Chadburn.from_index(self._chadburn_index)
        self._log.info('chadburn: ' + Style.BRIGHT + '{}'.format(self._chadburn.name) + Style.NORMAL + '; speed: {:.2f}'.format(self._chadburn.speed))
        self._motor_controller.set_speed(Orientation.PORT, self._chadburn.speed)
        self._motor_controller.set_speed(Orientation.STBD, self._chadburn.speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_manual_speed(self):
        '''
        Sets the speed via a manually-adjusted potentiometer value.
        '''
        _target_speed = self._speed_pot.get_scaled_value(True)
        if isclose(_target_speed, 0.0, abs_tol=1e-2):
            self._motor_controller.set_speed(Orientation.PORT, _target_speed)
            self._motor_controller.set_speed(Orientation.STBD, _target_speed)
        else:
            self._motor_controller.set_speed(Orientation.PORT, _target_speed)
            self._motor_controller.set_speed(Orientation.STBD, _target_speed)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def scale(self, value):
        return Util.repeat('▒', value)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_A_BUTTON(self, value):
        self._log.info('♑ handling A_BUTTON; value: {}'.format(value))
        self.set_steering_mode(SteeringMode.AFRS)
#       self._servo_controller.report()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_B_BUTTON(self, value):
        self._log.info('♑ a. handling B_BUTTON; value: {}'.format(value))
#       self._servo_controller.report()
        self.set_steering_mode(SteeringMode.ROTATE)
#       self.travel_by_ticks(650)
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def travel_by_ticks(self, ticks):
        self._log.warning(Style.BRIGHT + '♑ travel by {} ticks.'.format(ticks))
        _sfwd_motor = self._motor_controller.get_motor(Orientation.SFWD)
        raise Exception('unimplemented')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_Y_BUTTON(self, value):
        self._log.warning(Style.BRIGHT + '♑ SHOULD NOT BE handling Y_BUTTON; value: {}'.format(value))
#       self._shutdown()
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
        self._motor_controller.list_speed_multipliers()
        self._motor_controller.clear_speed_multipliers()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'motion-ctrl'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_steering_mode(self, steering_mode):
        '''
        Set the steering mode of the motors and servos to the argument.
        '''
        if steering_mode is None:
            raise ValueError('null steering mode argument.')
        if self._steering_mode is steering_mode:
            self._log.info(Fore.GREEN + 'no change: already in steering mode {}'.format(steering_mode))
            return
        # NOTE: remove any lambdas not part of the current mode
        self._log.info(Fore.GREEN + Style.BRIGHT + 'removing existing steering mode lambdas…')
        for _motor in self._motor_controller.get_motors():
            _motor.remove_speed_multiplier('steering') # remove any other steering mode lambdas
        # set mode for motors
        if steering_mode is SteeringMode.AFRS:
            print('set AFRS mode....')
            self._steering_mode = SteeringMode.AFRS
            self._servo_controller.set_mode(steering_mode)
            # add steering lambdas if they're not already added
            for _motor in self._motor_controller.get_motors():
                if _motor.orientation.side is Orientation.PORT:
                    if not _motor.has_speed_multiplier(MotionController.PORT_AFRS_STEERING_LAMBDA_NAME):
                        _motor.add_speed_multiplier(MotionController.PORT_AFRS_STEERING_LAMBDA_NAME, self._port_steering_lambda)
                elif _motor.orientation.side is Orientation.STBD:
                    if not _motor.has_speed_multiplier(MotionController.STBD_AFRS_STEERING_LAMBDA_NAME):
                        _motor.add_speed_multiplier(MotionController.STBD_AFRS_STEERING_LAMBDA_NAME, self._stbd_steering_lambda)
                else:
                    raise Exception('unrecognised orientation {}'.format(_motor.orientation.name))
        elif steering_mode is SteeringMode.ROTATE:
            self._steering_mode = SteeringMode.ROTATE
            self._servo_controller.set_mode(steering_mode)
            # TODO
            pass
        elif steering_mode is SteeringMode.SKID:
            self._steering_mode = SteeringMode.SKID
            self._servo_controller.set_mode(steering_mode)
            # TODO
            pass
        else:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))

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
#           self.enable() # TODO enable default motion/behaviour
        elif remote is Event.REMOTE_B:
            self._log.info(Fore.MAGENTA + Style.BRIGHT + 'B: BUTTON ')
        elif remote is Event.REMOTE_Y:
            self._log.info(Fore.MAGENTA + Style.BRIGHT + 'Y: STOP ')
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
        self._log.info(Fore.CYAN + Style.BRIGHT + 'shutting down!')
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
                if not self._motor_controller.enabled:
                    self._motor_controller.enable()
                self._servo_controller.enable()
                Subscriber.enable(self)
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
                self._servo_controller.disable()
                Subscriber.disable(self)
                self._log.info('motion controller disabled.')
            else:
                self._log.warning('motion controller already disabled.')
        else:
            self._log.warning('motion controller already closed.')

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

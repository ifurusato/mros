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

import sys, signal, traceback
from datetime import datetime as dt
import time
import itertools
import math, statistics
from math import isclose
from math import pi as PI
from threading import Thread
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.rate import Rate
from core.speed import Speed
from core.convert import Convert
from core.cardinal import Cardinal
from core.rotation import Rotation
from core.util import Util
from core.chadburn import Chadburn
from core.steering_angle import SteeringAngle
from core.event import Event, Group
from core.orientation import Orientation
from core.subscriber import Subscriber
from core.logger import Logger, Level
from hardware.task_selector import TaskSelector
from hardware.stop_handler import StopHandler
from hardware.servo_controller import ServoController, SteeringMode
from hardware.headlight import Headlight
from hardware.servo import Servo
from hardware.sensor_array import SensorData
from hardware.motor_controller import MotorController
from hardware.motion_coordinator import MotionCoordinator
from hardware.sound import Sound, Player

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotionController(Subscriber):
    CLASS_NAME= 'motion'
    PORT_AFRS_STEERING_LAMBDA_NAME  = "__port_afrs_steering"
    STBD_AFRS_STEERING_LAMBDA_NAME  = "__stbd_afrs_steering"
    '''
    The MotionController combines motor and servo control as a subscriber
    to various types of events.

    Most movements of the steering servos and motors are independent upon
    each other. When the servos and motors need to coordinate their movements,
    such as when recentering or moving the steering servos to a new position
    while stopped, the MotionCoordinator class is used.

    :param config:            the YAML based application configuration
    :param message_bus:       the asynchronous message bus
    :param external_clock:    the optional external clock (used by the motor controller)
    :param level:             the logging Level
    '''
    def __init__(self, config, message_bus, external_clock=None, suppressed=False, enabled=False, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        self._log = Logger("motion-ctrl", level)
        Subscriber.__init__(self, MotionController.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        if config is None:
            raise ValueError('no configuration provided.')
        self._config = config
        self._component_registry = globals.get('component-registry')
#       self._component_registry = self._mros.get_component_registry()
        if self._component_registry:
            self._sensor_array = self._component_registry.get('pub:sensors')
            self._log.info('using sensor array.')
            self._digital_pot  = self._component_registry.get('digital-pot-0x0E')
            self._digital_pot.set_output_range(0.0, 1.0)
            self._log.info('using digital potentiometer.')
            self._monitor = self._component_registry.get('monitor')
            self._screen  = self._component_registry.get('screen')
        else:
            self._monitor = None
            self._screen  = None
        # geometry # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._half_length  = config.get('mros').get('geometry').get('wheel_base') / 2
        self._half_width   = config.get('mros').get('geometry').get('wheel_track') / 2
        self._wheel_offset = config.get('mros').get('geometry').get('wheel_offset')
        _wheel_diameter = config.get('mros').get('geometry').get('wheel_diameter')
        # motion controller # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['mros'].get('motion_controller')
        self._play_sound = _cfg.get('play_sound')
        if self._play_sound:
            self._player = Player.instance()
        self._imu        = None
        self._afrs_max_ratio = _cfg.get('afrs_max_ratio') # ratio at maximum turn
        _max_angle       = _cfg.get('afrs_max_angle')     # maximum AFRS inner turn angle
        self._afrs_clamp = lambda n: max(min(_max_angle, n), -1.0 * _max_angle)
        self._incr_clamp = lambda n: max(min(7, n), -7)
        self._pot        = None # used for manual control
        self._default_manual_speed = _cfg.get('default_manual_speed')
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
        self._motor_controller.add_state_change_callback(self._state_changed)
        self._all_motors = self._motor_controller.get_motors()
        self._log.info('created motor controller with {} motors.'.format(len(self._all_motors)))
        self._speed_value = 0.0 # set by L3 Vertical
        # stop handler # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._stop_handler = StopHandler(config, self._motor_controller, level)
        # subscribe to event groups ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self.add_events(Event.by_groups([Group.GAMEPAD, Group.BUMPER, Group.IMU, Group.STOP, Group.VELOCITY]))
        self._log.info(Fore.WHITE + 'registered {} events.'.format(len(self._events)))
        # headlight ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._headlight = Headlight(Orientation.STBD)
        # chadburn events ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._chadburn_index = 0
        self._chadburn = Chadburn.STOP
        # steering coordinator # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._steps_per_rotation = config.get('mros').get('geometry').get('steps_per_rotation')
        self._wind_down_ratio = 0.90
        self._minimum_speed = 0.20
        _ticks_per_rotation = 2774.64
        _wheel_circumference = 2 * _wheel_diameter * PI # 364.42
        # finish up…
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motion_coordinator(self, orientation):
        _motor = self._motor_controller.get_motor(orientation)
        _servo = self._servo_controller.get_servo(orientation)
        _moco = MotionCoordinator(self._config, _motor, _servo, self._level)
        return _moco

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_potentiometer(self, potentiometer):
        '''
        Set the optional potentiometer, used for manual speed control.
        '''
        self._pot = potentiometer

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
                if self._motor_controller.is_stopped:
                    self._log.info('robot already stopped.')
                else:
                    self._log.info('handling STOP; value: {}'.format(_value))
                    self._stop_handler.process_message(message)

        elif _event.group is Group.BUMPER:
#           self._log.info('handling BUMPER; value: {}'.format(type(_value)))
            _sensor_data = _value
            # from hardware.sensor_array import SensorData
            _events = _sensor_data.events
            _fop_cm = _sensor_data.fop_cm
            _fos_cm = _sensor_data.fos_cm
            self._log.info('handling bumper events: {}; fop {:d}cm; fos: {:d}cm'.format(_events, _fop_cm, _fos_cm))
            self._stop_handler.process_message(message)

        elif _event.group is Group.IMU:
            if self._motor_controller.is_stopped:
                # we ignore IMU events if stopped (the robot is likely being transported)
                self._log.info('robot already stopped.')
            else:
                self._handle_imu_event(message)

        elif _event_num == Event.SHUTDOWN: # SHUTDOWN (Y)
            if _value == 0:
                self._shutdown()

        elif _event_num == Event.A_BUTTON.num: # A_BUTTON = ( 42, "b-circle",   10, Group.GAMEPAD)
            if _value == 0:
#               self._handle_gamepad_A_BUTTON(_value)
                self.execute_task()

        elif _event_num == Event.B_BUTTON.num: # B_BUTTON = ( 42, "b-circle",   10, Group.GAMEPAD)
            if _value == 0:
                self._handle_gamepad_B_BUTTON(_value)

        elif _event_num == Event.L1_BUTTON.num:
            if _value == 0:
                self._log.info('handling L1_BUTTON; value: {} rotation: counter-clockwise'.format(_value))
                self._motor_controller.rotate(Rotation.COUNTER_CLOCKWISE)

        elif _event_num == Event.R1_BUTTON.num: # R1_BUTTON = ( 47, "r1",         10, Group.GAMEPAD)
            if _value == 0:
                self._log.info('handling R1_BUTTON; value: {} rotation: clockwise'.format(_value))
                self._motor_controller.rotate(Rotation.CLOCKWISE)
#               self.set_manual_speed()

        elif _event_num == Event.L2_BUTTON.num:
            if _value == 0:
                self._log.info('handling L2_BUTTON; value: {} servos: recenter'.format(_value))
                self._servo_controller.recenter()

        elif _event_num == Event.R2_BUTTON.num: # R2_BUTTON = ( 48, "r2",         10, Group.GAMEPAD)
#           self._handle_gamepad_R2_BUTTON(_value)
            if _value == 0 and self._headlight:
                self._log.info('handling R2_BUTTON; value: {} headlight toggle'.format(_value))
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
#       self._log.info(Fore.BLACK + 'set AFRS servo angle to {:.2f}°'.format(inside_steering_angle))
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
        self.set_steering_mode(SteeringMode.AFRS)
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
                    + Fore.GREEN + 'stbd: {}°; ratio: {:.2f}'.format(_stbd_angle, self._stbd_motor_ratio))

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
        self._prepare_to_move()
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
                _target_speed = _motor.target_speed + _speed_delta
                self._log.info(Fore.GREEN + 'handling L3_VERTICAL for {} motor with current speed {:.2f}; delta: {:7.4f}; target speed: {:4.2f}; '.format(
                        _motor.orientation.name, _motor.target_speed, _speed_delta, _target_speed))
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
        self._prepare_to_move()
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
        Sets the speed via a manually-adjusted potentiometer value. If no
        potentiometer is available a warning is issued and a default value
        is used.
        '''
        self._prepare_to_move()
        if self._pot is None:
            self._log.warning('no potentiometer available.')
            _target_speed = self._default_manual_speed
        else:
            _target_speed = self._pot.get_scaled_value(True)
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
    def _handle_imu_event(self, message):
        self._log.info('handling IMU event; value: {}'.format(message))
        self._stop_handler.stop()
        Player.instance().play(Sound.CHATTER_3)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_A_BUTTON(self, value):
        self._log.info('handling A_BUTTON; value: {}'.format(value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_B_BUTTON(self, value):
        self._log.info('a. handling B_BUTTON; value: {}'.format(value))
#       self._servo_controller.report()
#       self.set_steering_mode(SteeringMode.AFRS)
#       self.set_steering_mode(SteeringMode.ROTATE)
#       self.travel_by_ticks(650)
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def travel_by_ticks(self, ticks):
        self._log.warning(Style.BRIGHT + 'travel by {} ticks.'.format(ticks))
        _sfwd_motor = self._motor_controller.get_motor(Orientation.SFWD)
        raise Exception('unimplemented')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_Y_BUTTON(self, value):
        self._log.warning(Style.BRIGHT + 'SHOULD NOT BE handling Y_BUTTON; value: {}'.format(value))
#       self._shutdown()
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_R2_BUTTON(self, value):
        self._log.info('handling R2_BUTTON; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_START_BUTTON(self, value):
        self._log.info('handling START_BUTTON; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_SELECT_BUTTON(self, value):
        self._log.info('handling SELECT_BUTTON; value: {}'.format(value))
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handle_gamepad_HOME_BUTTON(self, value):
        self._log.info('handling HOME_BUTTON; value: {}'.format(value))
        self._motor_controller.list_speed_multipliers()
        self._motor_controller.clear_speed_multipliers()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'motion-ctrl'

    # tasks ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def assign_tasks(self, task_selector):
        '''
        Load tasks from configuration.
        '''
        self._task_selector = task_selector
        self._task_selector.add(self.task1, 'voice',     'announcement') # red
        self._task_selector.add(self.task2, 'task list', 'list available tasks') # orange
        self._task_selector.add(self.task3, 'north',     'align north') # yellow
        self._task_selector.add(self.task4, 'east',      'align east') # yellow-green
        self._task_selector.add(self.task5, 'south',     'align south') # green
        self._task_selector.add(self.task6, 'west',      'align west') # turquoise
        self._task_selector.add(self.task7, 'task 6',    'description of task 7') # cyan
        self._task_selector.add(self.task8, 'task 8',    'description of task 8') # sky-blue
        self._task_selector.add(self.task9, 'task 9',    'description of task 9') # blue
        self._task_selector.add(self.task10, 'task 10',  'description of task 10') # blue-violet
        self._task_selector.add(self.task11, 'task 11',  'description of task 11') # purple
        self._task_selector.add(self.task12, 'task 12',  'description of task 12') # magenta
        self._task_selector.add(self.task13, 'task 13',  'description of task 13') # fuchsia
        self._task_selector.add(self.task14, 'task 14',  'description of task 14') # pink
        self._task_selector.add(self.task15, 'task 15',  'description of task 15') # grey
        self._task_selector.add(self.task16, 'task 16',  'description of task 16') # white
        self._task_selector.print_tasks()
        self._task_selector.update()

    def execute_task(self):
        '''
        Execute the currently selected task.
        '''
        _selection = self._task_selector.selection
        self._task_selector.set_color(self._task_selector.color)
        self._log.info('executing task: ' + Fore.YELLOW + '{}…'.format(_selection.name))
        _selection.task()
        self._task_selector.set_color(None)

    def task1(self):
#       self._log.info(Fore.GREEN + 'processing task 1…')
        Player.instance().play(Sound.KLAXON)

    def task2(self):
#       self._log.info(Fore.GREEN + 'processing task 2…')
        self._task_selector.print_tasks()

    def task3(self):
#       self._log.info(Fore.GREEN + 'processing task 3…')
        self._log.info(Fore.GREEN + 'heading north…')
        self.set_heading(Rotation.CLOCKWISE, Cardinal.NORTH)

    def task4(self):
#       self._log.info(Fore.GREEN + 'processing task 4…')
        self._log.info(Fore.GREEN + 'heading east…')
        self.set_heading(Rotation.CLOCKWISE, Cardinal.EAST)

    def task5(self):
#       self._log.info(Fore.GREEN + 'processing task 5…')
        self._log.info(Fore.GREEN + 'heading south…')
        self.set_heading(Rotation.CLOCKWISE, Cardinal.SOUTH)

    def task6(self):
        self._log.info(Fore.GREEN + 'processing task 6…')
        self._log.info(Fore.GREEN + 'heading west…')
        self.set_heading(Rotation.CLOCKWISE, Cardinal.WEST)

    def task7(self):
        self._log.info(Fore.GREEN + 'processing task 7…')
        Player.instance().play(Sound.CHATTER_5)

    def task8(self):
        self._log.info(Fore.GREEN + 'processing task 8…')
        Player.instance().play(Sound.CHATTER_5)

    def task9(self):
        self._log.info(Fore.GREEN + 'processing task 9…')
        Player.instance().play(Sound.CHATTER_5)

    def task10(self):
        self._log.info(Fore.GREEN + 'processing task 10…')
        Player.instance().play(Sound.CHATTER_5)

    def task11(self):
        self._log.info(Fore.GREEN + 'processing task 11…')
        Player.instance().play(Sound.CHATTER_5)

    def task12(self):
        self._log.info(Fore.GREEN + 'processing task 12…')
        Player.instance().play(Sound.CHATTER_5)

    def task13(self):
        self._log.info(Fore.GREEN + 'processing task 13…')
        Player.instance().play(Sound.CHATTER_5)

    def task14(self):
        self._log.info(Fore.GREEN + 'processing task 14…')
        Player.instance().play(Sound.CHATTER_5)

    def task15(self):
        self._log.info(Fore.GREEN + 'processing task 15…')
        Player.instance().play(Sound.CHATTER_5)

    def task16(self):
        self._log.info(Fore.GREEN + 'processing task 16…')
        Player.instance().play(Sound.CHATTER_5)

    # ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈ ┈┈┈┈
    def rotate_sigint_handler(self, signal, frame):
        self._log.info(Fore.WHITE + '\nCtrl-C caught by signal; exiting…')
        self._motor_controller.stop()
#       self._motor_controller.set_speed(Orientation.PORT, 0.0)
#       self._motor_controller.set_speed(Orientation.STBD, 0.0)
        self._shutdown()

    def set_heading(self, rotation, cardinal):
        '''
        Rotate the robot so that it is heading in the specified cardinal direction.
        '''
        if not self._digital_pot:
            raise Exception('cannot proceed: no digital potentiometer available.')
        self._prepare_to_move()
        # this is somehow necessary as Ctrl-C isn't normally getting caught in this loop.
        signal.signal(signal.SIGINT, self.rotate_sigint_handler)

        _ACTIVATE_MOTION = True
        self._log.info('turning ' + Fore.YELLOW + '{}'.format(rotation.label) + Fore.CYAN + ' to heading ' + Fore.YELLOW + '{}…'.format(cardinal.label))
        if self._imu is None:
            raise Exception('cannot calibrate: IMU has not been set.')
        _icm20948 = self._imu.icm20948
        if not _icm20948.enabled:
            _icm20948.enable()
        _starting_heading = _icm20948.heading
        _target_heading = cardinal.degrees
        _diff_deg = Convert.difference_in_degrees(_starting_heading, _target_heading)
        self._log.info(Style.BRIGHT + 'starting heading: {:5.2f}°; target heading: {:5.2f}𝛑;'.format(_starting_heading, _target_heading)
                + Fore.YELLOW + ' diff: {}°'.format(_diff_deg))

#       _rotation_speed = self._config['mros'].get('motor_controller').get('rotation_speed')
#       _target_speed = _rotation_speed
        _target_speed = self._digital_pot.get_scaled_value() # values 0.0-1.0

        # change to rotate mode ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if _ACTIVATE_MOTION:
            self.set_steering_mode(SteeringMode.ROTATE)
            time.sleep(3) # wait until the servos are clearly finished moving
            # add rotation lambdas
            self._motor_controller.rotate(Rotation.CLOCKWISE)

        _min_speed_clamp = lambda n: max(min(Chadburn.DEAD_SLOW_AHEAD.speed, n), Chadburn.DEAD_SLOW_ASTERN.speed)
        _max_speed_clamp = lambda n: max(min(Chadburn.HALF_AHEAD.speed, n), Chadburn.HALF_ASTERN.speed)
        try:
            _hz = 20
            _rate = Rate(_hz, Level.ERROR)

            while not isclose(_diff_deg, 0.0, abs_tol=1.0):

                _target_speed = self._digital_pot.get_scaled_value() # values 0.0-1.0
                _icm20948.poll()
                _current_heading = _icm20948.heading
                _diff_deg = Convert.difference_in_degrees(_current_heading, _target_heading)
                _multiplier = _diff_deg / 180.0
                _target_speed *= _multiplier

                if isclose(_target_speed, 0.0, abs_tol=1e-4):
#                   _multiplier = 0.0
#                   self._motor_controller.rotation_speed_multiplier = _multiplier
                    self._motor_controller.set_speed(Orientation.PORT, 0.0)
                    self._motor_controller.set_speed(Orientation.STBD, 0.0)
                    self._log.info(Fore.BLACK + 'target speed: {:.2f}; '.format(_target_speed)
                            + Fore.YELLOW + 'heading: {:4.2f}; target: {:4.2f}; diff: {:4.2f}; '.format(_current_heading, _target_heading, _diff_deg)
                            + Fore.GREEN + 'multiplier: {:4.2f}'.format(_multiplier))
                else:
                    _clamped_speed = _max_speed_clamp(_target_speed) # we never want to go faster than ONE_THIRD
                    _clamped_speed = _min_speed_clamp(_clamped_speed) # we never want to go slower than DEAD_SLOW
#                   self._motor_controller.rotation_speed_multiplier = _multiplier
                    self._motor_controller.set_speed(Orientation.PORT, _clamped_speed)
                    self._motor_controller.set_speed(Orientation.STBD, _clamped_speed)
                    self._log.info(Fore.CYAN + 'target/clamped speed: {:.2f}/{:4.2f}; '.format(_target_speed, _clamped_speed)
                            + Fore.YELLOW + 'heading: {:4.2f}; target: {:4.2f}; diff: {:4.2f}; '.format(_current_heading, _target_heading, _diff_deg)
                            + Fore.GREEN + 'multiplier: {:4.2f}'.format(_multiplier))
                _rate.wait()
#               time.sleep(0.2)

        except KeyboardInterrupt:
            self_log.info('Ctrl-C caught; exiting…')
        finally:
            Player.instance().play(Sound.CHIRP_1)

        if _ACTIVATE_MOTION:
            self._motor_controller.rotate(Rotation.STOPPED)
            self.set_steering_mode(SteeringMode.AFRS)
            time.sleep(3) # wait until the servos are clearly finished moving

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def rotate(self, rotation):
        '''
        In rotate mode, each wheel must travel through 1243mm for a complete
        circle with a wheel diameter of 72mm, circumference is 452.39mm, or
        about 2.75 rotations. At 2774.64 ticks per 452.49mm that's 6.138584
        ticks/mm. 1242mm is therefore 7627 ticks. But the number of steps is
        not the same as the number of motor ticks, but a multiple of that.
        '''
        self._prepare_to_move()
        if rotation is Rotation.STOPPED:
            self._motor_controller.rotate(Rotation.STOPPED)
            if self._sensor_array:
                self._sensor_array.suppress_events(None)
        else:
            if self._sensor_array:
                # suppress bumpers while rotating
                self._sensor_array.suppress_events([
                    Event.BUMPER_PFWD,
                    Event.BUMPER_PAFT,
                    Event.BUMPER_SFWD,
                    Event.BUMPER_SAFT,
                    Event.BUMPER_FOBP,
                    Event.BUMPER_FOBS ])
            self._motor_controller.rotate(rotation)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _calibrate_loop(self):
        '''
        Performs the physical movement of rotating the robot clockwise through
        a complete circle, mimicking the accumulation of statistics to determine
        following the rotation if the values have approached stability.
        '''
        self._log.info(Fore.GREEN + 'begin calibration loop...')
        if self._imu is None:
            raise Exception('cannot calibrate: IMU has not been set.')
        _icm20948 = self._imu.icm20948

        self.set_steering_mode(SteeringMode.ROTATE)
        time.sleep(3) # wait until the servos are clearly finished moving

        if not _icm20948.enabled:
            _icm20948.enable()
        self._log.info(Fore.GREEN + 'ICM20948 enabled: {}'.format(_icm20948.enabled))

        self.rotate(Rotation.CLOCKWISE)

        i = 0
        _hz = 20
        _rate = Rate(_hz, Level.ERROR)

        _max_delta = 380.0
        _diff = 0.0
        _delta = 0.0
        _initial_heading = _icm20948.uncalibrated_heading

        _counter = itertools.count()
        _count = 0

        # choose a motor
        _motor = self._motor_controller.get_motor(Orientation.SAFT)
        _step_count = 0
        _steps = _motor.steps
        self._log.info(Fore.GREEN + 'INITIAL heading value: {:4.2f}°; {} steps.'.format(_initial_heading, _steps))
        _step_limit = 400000 # roughly one rotation

        while _step_count < _step_limit:
            _count = next(_counter)
            # when rotating clockwise, the heading value should only increase...
            _steps = _motor.steps
            _step_count += abs(_steps)
            _heading = _icm20948.uncalibrated_heading

            # add to queue
            if _icm20948.calibration_check(_heading):
                self._log.info(Fore.GREEN + Style.BRIGHT + 'IMU was calibrated while moving.')
#               break
            self._log.info(Fore.GREEN + '[{:d}] heading: {:4.2f}°; {} steps; count: {}/{} steps;'.format(i, _heading, _steps, _step_count, _step_limit)
                    + Fore.YELLOW + ' calibrated? {}'.format(_icm20948.is_calibrated))
            i += 1
            _rate.wait()
            # end loop ......................................................

        self._log.info('exited loop.')

        self.rotate(Rotation.STOPPED)
        # in theory we should be re-enabling bumpers after 'taking a break'

        # recenter before proceeding...
        self._log.info('recentering…')
        self._servo_controller.recenter()
        self._log.info('recentered.')

        if not self._motor_controller.all_motors_are_stopped:
            self._motor_controller.stop()

        self._log.info('taking a break…')
        time.sleep(3)

        if _icm20948.is_calibrated:
            self._log.info(Fore.GREEN + 'IMU has been calibrating.')
            if self._play_sound:
                self._player.play(Sound.CHATTER_4)
            # enable monitor
            if self._monitor:
                self._monitor.set_callback(_icm20948._formatted_heading)
        else:
            self._log.warning(Fore.GREEN + 'unable to calibrate IMU.')

        self._log.info(Fore.GREEN + 'calibrate loop end. ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈ ')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_imu(self, imu):
        '''
        Set the IMU used by the MotionController.
        '''
        self._imu = imu

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def calibrate_imu(self):
        '''
        Perform a 360° rotation to calibrate the ICM20948.
        '''
        if self._play_sound:
            self._player.play(Sound.CHATTER_2)
        self._log.info(Style.BRIGHT + 'calibrating IMU…')
        self._calibrate_loop_thread = Thread(name='calibrate_thread', target=MotionController._calibrate_loop, args=[self], daemon=True)
        self._calibrate_loop_thread.start()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_steering_mode(self, steering_mode):
        '''
        Set the steering mode of the motors and servos to the argument.
        '''
        if steering_mode is None:
            raise ValueError('null steering mode argument.')
        if self._steering_mode is steering_mode:
            self._log.info(Style.DIM + 'no change: already in steering mode {}'.format(self._steering_mode))
            return
        # NOTE: remove any lambdas not part of the current mode
        for _motor in self._motor_controller.get_motors():
            if _motor.has_speed_multiplier('steering'):
                self._log.info(Fore.GREEN + Style.BRIGHT + 'removing existing steering mode or stop lambda from {} motor…'.format(_motor.orientation.name))
                _motor.remove_speed_multiplier('steering') # remove any other steering mode lambdas
                _motor.remove_speed_multiplier('stop')     # remove any stop lambdas
        # set mode for motors
        if steering_mode is SteeringMode.AFRS:
            self._servo_controller.recenter() # recenter before proceeding...

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
            self._log.info(Fore.MAGENTA + '🍉 setting rotate mode {}'.format(steering_mode))
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
    def _prepare_to_move(self):
        '''
        Make any preparations prior to moving the robot.
        '''
        self._log.info(Fore.WHITE + 'preparing to move…')
        if self._monitor:
            self._monitor.disable()
        if self._screen:
            self._screen.disable()
        # make a noise?

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _state_changed(self):
        self._log.info('ready.')
        _currently_is_stopped = self._motor_controller.is_stopped
        if _currently_is_stopped:
            self._log.info('state: stopped.')
        else:
            self._log.info('state: moving.')
        if self._monitor:
            if _currently_is_stopped:
                self._monitor.enable()
            elif self._monitor.enabled:
                self._monitor.disable()
        if self._screen:
            if _currently_is_stopped:
                self._screen.enable()
            else:
                self._screen.disable()

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

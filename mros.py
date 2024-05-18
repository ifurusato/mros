#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2024-05-18
#
# The MR01 Robot Operating System (MROS), including its command line
# interface (CLI).
#
#        1         2         3         4         5         6         7         8         9         C
#234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#

import os, sys, signal, time, threading, traceback
import argparse, psutil
from pathlib import Path
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.logger import Logger, Level
from core.event import Event, Group
from core.component import Component
from core.fsm import FiniteStateMachine
from core.util import Util
from core.message import Message, Payload
from core.message_bus import MessageBus
from core.message_factory import MessageFactory
from core.config_loader import ConfigLoader
from core.controller import Controller

from core.publisher import Publisher
from core.queue_publisher import QueuePublisher
from core.macro_publisher import MacroPublisher

from core.subscriber import Subscriber, GarbageCollector
from core.system_subscriber import SystemSubscriber
from core.macro_subscriber import MacroSubscriber
from core.omni_subscriber import OmniSubscriber
#from core.kr01_macrolibrary import KR01MacroLibrary

from hardware.system import System
#from hardware.ifs_publisher import IfsPublisher
#from hardware.remote_ctrl_publisher import RemoteControlPublisher
#from hardware.gpio_bmp_publisher import GpioBumperPublisher
#from hardware.mcu_bmp_publisher import McuBumperPublisher
#from hardware.ext_bmp_publisher import ExternalBumperPublisher

#from hardware.motor_subscriber import MotorSubscriber
#from hardware.rgb_subscriber import RgbSubscriber
#from hardware.remote_ctrl_subscriber import RemoteControlSubscriber
#from hardware.bumper_subscriber import BumperSubscriber
#from hardware.infrared_subscriber import InfraredSubscriber

from hardware.i2c_scanner import I2CScanner
#from hardware.battery import BatteryCheck
#from hardware.color import Color
#from hardware.external_clock import ExternalClock
#from hardware.irq_clock import IrqClock
#from hardware.killswitch import KillSwitch
from hardware.motor_configurer import MotorConfigurer
#from hardware.motor_controller import MotorController
#from hardware.status import Status

#from mock.event_publisher import EventPublisher
#from mock.external_clock import MockExternalClock
#from mock.velocity_publisher import VelocityPublisher
#from mock.mock_pot_publisher import MockPotPublisher
#from hardware.gamepad_controller import GamepadController
#from hardware.gamepad_publisher import GamepadPublisher # lazily-imported

#from behave.behaviour_manager import BehaviourManager
#from behave.avoid import Avoid
#from behave.roam import Roam
#from behave.swerve import Swerve
#from behave.moth import Moth
#from behave.sniff import Sniff
#from behave.idle import Idle

from experimental.experiment_mgr import ExperimentManager

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MROS(Component, FiniteStateMachine):
    '''
    Extends Component and Finite State Machine (FSM) as a basis of a K-Series
    Robot Operating System (MROS) or behaviour-based system (BBS), including
    spawning the various tasks and starting up the Subsumption Architecture,
    used for communication between Components over a common message bus.

    The MessageBus receives Event-containing messages from sensors and other
    message sources, which are passed on to the Arbitrator, whose job it is
    to determine the highest priority action to execute for that task cycle,
    by passing it on to the Controller.

    There is also a mrosd linux daemon, which can be used to start, enable and
    disable mros.
    '''
    def __init__(self, level=Level.INFO):
        '''
        This initialises MROS and calls the YAML configurer.
        '''
        _name = 'mros'
        self._level = level
        self._log = Logger(_name, self._level)
        self._print_banner()
        self._log.info('…')
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        FiniteStateMachine.__init__(self, self._log, _name)
        self._system             = System(self, level)
        self._system.set_nice()
        globals.put('mros', self)
        # configuration…
        self._config                = None
        self._message_bus           = None
        self._behaviour_mgr         = None
        self._queue_publisher       = None
        self._macro_publisher       = None
        self._ifs_publisher         = None
        self._remote_ctrl_publisher = None
        self._experiment_mgr        = None
        self._arbitrator            = None
        self._controller            = None
        self._gamepad_publisher     = None
        self._gamepad_controller    = None
        self._external_clock        = None # a Publisher used for accessory timing
        self._irq_clock             = None # used for motor control timing
        self._motor_ctrl            = None
        self._ifs                   = None
        self._killswitch            = None
        self._status_light          = None
        self._disable_leds          = False
        self._closing               = False
        self._log.info('oid: {}'.format(id(self)))
        self._log.info('initialised.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def configure(self, arguments):
        '''
        Provided with a set of configuration arguments, configures MROS based on
        both KD01/KR01 standard hardware as well as optional features, the
        latter based on devices showing up (by address) on the I²C bus. Optional
        devices are only enabled at startup time via registration of their feature
        availability.
        '''
        self._log.heading('configuration', 'configuring mros…',
            '[1/2]' if arguments.start else '[1/1]')
        self._log.info('application log level: {}'.format(self._log.level.name))

        # read YAML configuration ..............................................
        _loader = ConfigLoader(self._level)
        _config_filename = arguments.config_file
        _filename = _config_filename if _config_filename is not None else 'config.yaml'
        self._config = _loader.configure(_filename)
        self._is_raspberry_pi = self._system.is_raspberry_pi()

        # configuration from command line arguments ............................

        _args = self._config['mros'].get('arguments')
        # copy argument-based configuration over to _config (changing the names!)

#       self._log.info('argument gamepad:     {}'.format(arguments.gamepad))
#       _args['gamepad_enabled'] = arguments.gamepad and self._is_raspberry_pi
#       self._log.info('gamepad enabled:      {}'.format(_args['gamepad_enabled']))
#       _args['video_enabled']   = arguments.video
#       self._log.info('video enabled:        {}'.format(_args['video_enabled']))
        _args['motors_enabled']  = not arguments.no_motors
        self._log.info('motors enabled:       {}'.format(_args['motors_enabled']))
#       _args['mock_enabled']    = arguments.mock
#       self._log.info('mock enabled:         {}'.format(_args['mock_enabled']))
        _args['experimental_enabled'] = arguments.experimental
        self._log.info('experiment enabled:   {}'.format(_args['experimental_enabled']))
        _args['log_enabled']    = arguments.log
        self._log.info('write log enabled:    {}'.format(_args['log_enabled']))

        # print remaining arguments
        self._log.info('argument config-file: {}'.format(arguments.config_file))
        self._log.info('argument level:       {}'.format(arguments.level))

        # scan I2C bus .........................................................
        _i2c_scanner = I2CScanner(self._config, self._log.level)
        _i2c_scanner.print_device_list()
        self._addresses = _i2c_scanner.get_int_addresses()

        # check for availability of pigpio .....................................

        try:
            import pigpio
            _pigpio_available = True
            self._log.info('pigpio library available.')
        except Exception:
            _pigpio_available = False
            self._log.warning('pigpio library not available; will attempt to use mocks where available.')

        # establish basic subsumption components ..............................

        self._log.info('configure subsumption components…')

        self._message_bus = MessageBus(self._config, self._level)
        self._message_factory = MessageFactory(self._message_bus, self._level)

        self._controller = Controller(self._message_bus, self._level)

        self._use_external_clock = self._config['mros'].get('use_external_clock')
        if self._use_external_clock and _pigpio_available:
            self._log.info('configuring external clock callback…')
#           self._irq_clock      = IrqClock(self._config, level=self._level)
#           self._external_clock = ExternalClock(self._config, self._message_bus, self._message_factory, self._level)
        else:
            self._irq_clock      = None # MockExternalClock(self._config, freq_hz=20, callback=None, level=self._level)
#           self._external_clock = MockExternalClock(self._config, freq_hz=20, callback=None, level=self._level)
            # TODO only if mocks permitted?
            self._use_external_clock = True

        # add motor controller ................................................
        self._log.info('configure motor controller…')
        _motor_configurer = MotorConfigurer(self._config, self._message_bus, _i2c_scanner, level=self._level)
        self._motor_ctrl = MotorController(self._config, self._message_bus, _motor_configurer, self._irq_clock != None, self._level)
        if self._use_external_clock:
#           self._irq_clock.add_callback(self._motor_ctrl._external_callback_method)
#           self._external_clock.add_callback(self._motor_ctrl._ext_callback_method, True)
#           self._external_clock.add_callback(self._rgbmatrix.random_update)
            pass

        # create components ....................................................

        _cfg = self._config['mros'].get('component')

        # create publishers ................................

        _pubs = arguments.pubs if arguments.pubs else ''

        if _cfg.get('enable_queue_publisher') or 'q' in _pubs:
            self._queue_publisher = QueuePublisher(self._config, self._message_bus, self._message_factory, self._level)
        if _cfg.get('enable_macro_publisher') or 'm' in _pubs:
            _callback = None
            self._macro_publisher = MacroPublisher(self._config, self._message_bus, self._message_factory, self._queue_publisher, _callback, self._level)
#           _library = KR01MacroLibrary(self._macro_publisher)
#           self._macro_publisher.set_macro_library(_library)

        _enable_ifs_publisher = _cfg.get('enable_ifs_publisher') or 'i' in _pubs
        if _enable_ifs_publisher:
            self._ifs_publisher = IfsPublisher(self._config, self._message_bus, self._message_factory, level=self._level)

        _enable_remote_ctrl_publisher = _cfg.get('enable_remote_ctrl_publisher') or 'i' in _pubs
        if _enable_remote_ctrl_publisher:
            self._remote_ctrl_publisher = RemoteControlPublisher(self._config, self._message_bus, self._message_factory, level=self._level)

        # first one wins!
        if _cfg.get('enable_mcu_bumper_publisher') or 'b' in _pubs:
            # bumpers connected via UART to an MCU
            self._bumper_publisher = McuBumperPublisher(self._config, self._message_bus, self._message_factory, level=self._level)
        elif _cfg.get('enable_ext_bumper_publisher') or 'b' in _pubs:
            # bumpers connected via GPIO pins using bespoke comms
            self._bumper_publisher = ExternalBumperPublisher(self._config, self._message_bus, self._message_factory, level=self._level)
        elif _cfg.get('enable_gpio_bumper_publisher') or 'g' in _pubs:
            # bumpers connected directly to GPIO pins
            self._bumper_publisher = GpioBumperPublisher(self._config, self._message_bus, self._message_factory, level=self._level)

        _enable_event_publisher = _cfg.get('enable_event_publisher') or 'e' in _pubs
        if _enable_event_publisher:
            self._event_publisher = EventPublisher(self._config, self._message_bus, self._message_factory, self._motor_ctrl,
                    self._system, level=self._level)
            if _cfg.get('enable_velocity_publisher'):
                self._log.warning('key event and potentiometer publishers both enabled; using only key events.')
        if not _enable_event_publisher: # we only enable potentiometer publishers if event publisher isn't available
            if _cfg.get('enable_velocity_publisher') or 'v' in _pubs:
                self._pot_publisher = VelocityPublisher(self._config, self._message_bus, self._message_factory, level=self._level)
#           else:
#               self._pot_publisher = MockPotPublisher(self._config, self._message_bus, self._message_factory, level=self._level)

        # add battery check publisher
        if _cfg.get('enable_battery_publisher') or 'p' in _pubs:
            self._battery = BatteryCheck(self._config, self._message_bus, self._message_factory, self._level)

        _enable_killswitch= _cfg.get('enable_killswitch') or 'k' in _pubs
        if _enable_killswitch and _pigpio_available:
            self._killswitch = KillSwitch(self._config, self, level=self._level)

        # create subscribers ...............................

        _subs = arguments.subs if arguments.subs else ''
        if _cfg.get('enable_rgb_subscriber') or 'r' in _subs:
            self._rgb_subscriber = RgbSubscriber(self._config, self._message_bus, level=self._level)
        if _cfg.get('enable_remote_ctrl_subscriber') or 'r' in _subs:
            self._remote_ctrl_subscriber = RemoteControlSubscriber(self._config, self._message_bus, level=self._level)
        if _cfg.get('enable_system_subscriber') or 's' in _subs:
            self._system_subscriber = SystemSubscriber(self._config, self, self._message_bus, level=self._level)
        if _cfg.get('enable_motor_subscriber') or 'm' in _subs:
            self._motor_subscriber = MotorSubscriber(self._config, self._message_bus, self._motor_ctrl, level=self._level)
        if _cfg.get('enable_bumper_subscriber') or 'b' in _subs:
            self._bumper_subscriber = BumperSubscriber(self._config, self._message_bus, self._message_factory, self._motor_ctrl, level=self._level)
        if _cfg.get('enable_infrared_subscriber') or 'i' in _subs:
            self._infrared_subscriber = InfraredSubscriber(self._config, self._message_bus, self._motor_ctrl, level=self._level) # reacts to IR sensors
        if _cfg.get('enable_macro_subscriber'):
            if not self._macro_publisher:
                raise ConfigurationError('macro subscriber requires macro publisher.')
            self._macro_subscriber = MacroSubscriber(self._config, self._message_bus, self._message_factory, self._macro_publisher, self._level)
        if _cfg.get('enable_omni_subscriber') or 'o' in _subs:
            self._omni_subscriber = OmniSubscriber(self._config, self._message_bus, level=self._level) # reacts to IR sensors
        # and finally, the garbage collector:
        self._garbage_collector = GarbageCollector(self._config, self._message_bus, level=self._level)

        _use_experiment_manager = self._config['mros'].get('component').get('enable_experimental') or Util.is_true(arguments.experimental)
        if _use_experiment_manager:
            self._experiment_mgr = ExperimentManager(self._config, level=self._level)
            self._log.info(Fore.YELLOW + 'enabled experiment manager.')

        # create behaviours ................................

        _enable_behaviours = _cfg.get('enable_behaviours') or Util.is_true(arguments.behave)
        if _enable_behaviours:
            self._behaviour_mgr = BehaviourManager(self._config, self._message_bus, self._level) # a specialised subscriber
            self._log.info(Style.BRIGHT + 'behaviour manager enabled.')

            _bcfg = self._config['mros'].get('behaviour')
            # create and register behaviours (listed in priority order)
            if _bcfg.get('enable_avoid_behaviour'):
                self._avoid  = Avoid(self._config, self._message_bus, self._message_factory, self._motor_ctrl,
                        external_clock=self._external_clock, level=self._level)
            if _bcfg.get('enable_roam_behaviour'):
                self._roam   = Roam(self._config, self._message_bus, self._message_factory, self._motor_ctrl,
                        external_clock=self._external_clock, level=self._level)
            if _bcfg.get('enable_swerve_behaviour'):
                self._swerve = Swerve(self._config, self._message_bus, self._message_factory, self._motor_ctrl,
                        self._external_clock, level=self._level)
            if _bcfg.get('enable_moth_behaviour'):
                self._moth   = Moth(self._config, self._message_bus, self._message_factory, self._motor_ctrl, self._level)
            if _bcfg.get('enable_sniff_behaviour'):
                self._sniff  = Sniff(self._config, self._message_bus, self._message_factory, self._motor_ctrl, self._level)
            if _bcfg.get('enable_idle_behaviour'):
                self._idle   = Idle(self._config, self._message_bus, self._message_factory, self._level)

        if _args['gamepad_enabled'] or _cfg.get('enable_gamepad_publisher') or 'g' in _pubs:
            try:
                from hardware.gamepad_publisher import GamepadPublisher
                self._gamepad_publisher = GamepadPublisher(self._config, self._message_bus, self._message_factory, True, self._level)
                try:
                    from hardware.gamepad_controller import GamepadController
                    self._gamepad_controller = GamepadController(self._message_bus, self._level)
                except Exception as e:
                    self._log.error('unable to import GamepadeController: {}'.format(e))
            except Exception as e:
                self._log.error('unable to import GamepadPublisher: {}'.format(e))

        self._export_config = False
        if self._export_config:
            self.export_config()
        self._log.info('configured.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        '''
        This first disables the Pi's status LEDs, establishes the message bus,
        arbitrator, controller, enables the set of features, then starts the main
        OS loop.
        '''
        self._log.heading('starting', 'starting k-series robot operating system (mros)…', '[2/2]' )
        FiniteStateMachine.start(self)

        self._status_light = Status(self._config, Level.INFO)
        self._status_light.enable()

        self._disable_leds = self._config['pi'].get('disable_leds')
        if self._disable_leds:
            # disable Pi LEDs since they may be distracting
            self._set_pi_leds(False)

        if self._killswitch:
            self._killswitch.enable()

        if self._experiment_mgr:
            self._experiment_mgr.enable()

        # begin main loop ..................................

        self._log.notice('Press Ctrl-C to exit.')
        self._log.info('begin main os loop.\r')

        if self._motor_ctrl:
            self._motor_ctrl.enable()

        # enable arbitrator tasks (normal functioning of robot)
#       self._arbitrator.start()

        # we enable ourself if we get this far successfully
        Component.enable(self)
        FiniteStateMachine.enable(self)

        self._log.info('enabling external clock…')
        if self._irq_clock:
            self._irq_clock.enable()

        # print registry of components
#       _component_registry = globals.get('component-registry')
#       _component_registry.print_registry()

        # now in main application loop until quit or Ctrl-C…
        self._log.info('enabling message bus…')
        self._message_bus.enable()
        # that blocks so we never get here until the end…
        self._log.info('main loop closed.')

        # end main loop ....................................

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_config(self):
        '''
        Returns the application configuration.
        '''
        return self._config

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_logger(self):
        '''
        Returns the application-level logger.
        '''
        return self._log

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_level(self):
        '''
        Returns the log level of the application.
        '''
        return self._level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_message_bus(self):
        '''
        Returns the MessageBus.
        '''
        return self._message_bus

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_message_factory(self):
        '''
        Returns the MessageFactory.
        '''
        return self._message_factory

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_behaviour_manager(self):
        '''
        Returns the BehaviourManager, None if not used.
        '''
        return self._behaviour_mgr

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_macro_publisher(self):
        '''
        Returns the MacroPublisher, None if not used.
        '''
        return self._macro_publisher

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_gamepad_publisher(self):
        '''
        Returns the GamepadPublisher, None if not used.
        '''
        return self._gamepad_publisher

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_queue_publisher(self):
        '''
        Returns the QueuePublisher, None if not used.
        '''
        return self._queue_publisher

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_ifs_publisher(self):
        '''
        Returns the IfsPublisher, None if not used.
        '''
        return self._ifs_publisher

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motor_ctrl(self):
        '''
        Returns the motor controller.
        '''
        return self._motor_ctrl

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_experiment_manager(self):
        '''
        Returns the ExperimentManager, None if not used.
        '''
        return self._experiment_mgr

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_external_clock(self):
        '''
        Returns the ExternalClock, None if not used.
        '''
        return self._external_clock

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_irq_clock(self):
        '''
        Returns the IRQ clock used for the motor controller, None if not used.
        '''
        return self._irq_clock

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_pi_leds(self, enable):
        '''
        Enables or disables the Raspberry Pi's board LEDs.
        '''
        sudo_name = self._config['pi'].get('sudo_name')
        _led_0_path = self._config['pi'].get('led_0_path')
        _led_0 = Path(_led_0_path)
        _led_1_path = self._config['pi'].get('led_1_path')
        _led_1 = Path(_led_1_path)
        if _led_0.is_file() and _led_0.is_file():
            if enable:
                self._log.info('re-enabling LEDs…')
                os.system('echo 1 | {} tee {}'.format(sudo_name,_led_0_path))
                os.system('echo 1 | {} tee {}'.format(sudo_name,_led_1_path))
            else:
                self._log.debug('disabling LEDs…')
                os.system('echo 0 | {} tee {}'.format(sudo_name,_led_0_path))
                os.system('echo 0 | {} tee {}'.format(sudo_name,_led_1_path))
        else:
            self._log.warning('could not change state of LEDs: does not appear to be a Raspberry Pi.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def shutdown(self):
        '''
        This halts any motor activity, demands a sudden halt of all tasks,
        then shuts down the OS.
        '''
        self._log.info(Fore.MAGENTA + '👾 shutdown: ' + Style.BRIGHT + 'kill! kill! kill! kill!')
        self.close()
        # we never get here if we shut down properly

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        This permanently disables the MROS.
        '''
        if self.closed:
            self._log.warning('already closed.')
        elif self._closing:
            self._log.warning('already closing.')
        elif self.enabled:
            self._log.info('disabling…')
            if self._experiment_mgr:
                self._experiment_mgr.disable()
            while not Component.disable(self):
                self._log.info('disabling component…')
            if self._motor_ctrl:
                self._motor_ctrl.disable()
            if self._external_clock and not self._external_clock.disabled:
                self._external_clock.disable()
            if self._irq_clock and not self._irq_clock.disabled:
                self._irq_clock.disable()
            FiniteStateMachine.disable(self)
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')
        return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def closing(self):
        return self._closing

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        This closes MROS and sets the robot to a passive, stable state
        following a session.
        '''
        if self.closed:
            self._log.warning('already closed.')
        elif self.closing:
            self._log.warning('already closing.')
        else:
            self._log.info('closing…')
            self._closing = True
            _component_registry = globals.get('component-registry')
            _registry = _component_registry.get_registry()
            # closes all components that are not a publisher, subscriber, the message bus or mros itself…
            while len(_registry) > 0:
                _name, _component = _registry.popitem(last=True)
                if not isinstance(_component, Publisher) and not isinstance(_component, Subscriber) \
                        and _component != self and _component != self._message_bus:
                    self._log.info('closing component \'{}\' ({})…'.format(_name, _component.classname))
                    _component.close()
            time.sleep(0.1)
            if self._message_bus and not self._message_bus.closed:
                self._log.info('closing message bus from mros…')
                self._message_bus.close()
                self._log.info('closed message bus.')
            while not Component.close(self): # will call disable()
                self._log.info('closing component…')
            if self._disable_leds: # restore normal function of Pi LEDs
                self._set_pi_leds(True)
            FiniteStateMachine.close(self)
            if self._status_light:
                self._status_light.close()
            self._closing = False
            self._log.info('application closed.')
            self._log.close()
#           sys.exit(0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def export_config(self):
        '''
        Exports the current configuration to a YAML file named ".config.yaml".
        '''
        self._log.info('exporting configuration to file…')
        _loader.export(self._config, comments=[ \
            '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈', \
            '      YAML configuration for K-Series Robot Operating System (MROS)           ', \
            '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈', \
            '', \
            'exported: {}'.format(Util.get_timestamp()) ])

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _print_banner(self):
        '''
        Display banner on console.
        '''
        self._log.info(' ')
        self._log.info(' ')
        self._log.info('      ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓ ')
        self._log.info('      ┃                                                        ┃ ')
        self._log.info('      ┃    █▒▒      █▒▒  █▒▒▒▒▒▒▒    █▒▒▒▒▒▒    █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒▒▒   █▒▒▒  █▒▒   █▒▒  █▒▒   █▒▒  █▒▒        █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒█▒▒█▒▒▒▒▒  █▒▒▒▒▒▒▒   █▒▒   █▒▒   █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃    █▒▒  █▒▒ █▒▒  █▒▒   █▒▒  █▒▒   █▒▒        █▒▒       ┃ ')
        self._log.info('      ┃    █▒▒      █▒▒  █▒▒    █▒▒  █▒▒▒▒▒▒    █▒▒▒▒▒▒   █▒▒  ┃ ')
        self._log.info('      ┃                                                        ┃ ')
        self._log.info('      ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛ ')
        self._log.info(' ')
        self._log.info(' ')

    # end of MROS class  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━


# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def print_documentation(console=True):
    '''
    Print the extended documentation as imported from the help.txt file. If
    'console' is false, just return its contents.
    '''
    _help_file = Path("help.txt")
    if _help_file.is_file():
        with open(_help_file) as f:
            _content = f.read()
            if console:
                return _content
            else:
                print(_content)
    else:
        if console:
            return 'help file not found.'
        else:
            print('{} not found.'.format(_help_file))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def parse_args():
    '''
    Parses the command line arguments and return the resulting args object.
    Help is available via '--help', '-h', or '--docs', '-d' (for extended help),
    or calling the script with no arguments.
    '''
    _log = Logger('parse-args', Level.INFO)
    _log.debug('parsing…')
#   formatter = lambda prog: argparse.HelpFormatter(prog,max_help_position=60)
    formatter = lambda prog: argparse.RawTextHelpFormatter(prog)
    parser = argparse.ArgumentParser(formatter_class=formatter,
            description='Provides command line control of the K-Series Robot OS application.',
            epilog='This script may be executed by mrosd (mros daemon) or run directly from the command line.')

    parser.add_argument('--docs',         '-d', action='store_true', help='show the documentation message and exit')
    parser.add_argument('--configure',    '-c', action='store_true', help='run configuration (included by -s)')
    parser.add_argument('--start',        '-s', action='store_true', help='start mros')
    parser.add_argument('--experimental', '-x', action='store_true', help='enable experiment manager')
    parser.add_argument('--no-motors',    '-n', action='store_true', help='disable motors (uses mock)')
    parser.add_argument('--gamepad',      '-g', action='store_true', help='enable bluetooth gamepad control')
    parser.add_argument('--video',        '-v', action='store_true', help='enable video if installed')
    parser.add_argument('--pubs',         '-P', help='enable publishers as identified by first character')
    parser.add_argument('--subs',         '-S', help='enable subscribers as identified by first character')
    parser.add_argument('--behave',       '-B', help='override behaviour configuration (1, y, yes or true, otherwise false)')
    parser.add_argument('--mock',         '-m', action='store_true', help='permit mocked libraries (e.g., when not on a Pi)')
    parser.add_argument('--config-file',  '-f', help='use alternative configuration file')
    parser.add_argument('--log',          '-L', action='store_true', help='write log to timestamped file')
    parser.add_argument('--level',        '-l', help='specify logging level \'DEBUG\'|\'INFO\'|\'WARN\'|\'ERROR\' (default: \'INFO\')')

    try:
        print('')
        args = parser.parse_args()
        print("-- ARGS type: {}".format(type(args)))
        if args.docs:
            print(Fore.CYAN + '{}\n{}'.format(parser.format_help(), print_documentation(True)) + Style.RESET_ALL)
            return -1
        elif not args.configure and not args.start:
            print(Fore.CYAN + '{}'.format(parser.format_help()) + Style.RESET_ALL)
            return -1
        else:
            globals.put('log-to-file', args.log)
            return args
    except NotImplementedError as nie:
        _log.error('unrecognised log level \'{}\': {}'.format(args.level, nie))
        _log.error('exit on error.')
        sys.exit(1)
    except Exception as e:
        _log.error('error parsing command line arguments: {}\n{}'.format(e, traceback.format_exc()))
        _log.error('exit on error.')
        sys.exit(1)

# execution handler ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def signal_handler(signal, frame):
    global _mros
    print('\nsignal handler    :' + Fore.MAGENTA + Style.BRIGHT + ' INFO  : Ctrl-C caught: exiting…' + Style.RESET_ALL)
    if _mros and not ( _mros.closing or _mros.closed ):
        _mros.close()
    print(Fore.MAGENTA + 'exit.' + Style.RESET_ALL)
#   sys.stderr = DevNull()
    sys.exit(0)

# main ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

_mros = None

def main(argv):
    global _mros
    signal.signal(signal.SIGINT, signal_handler)
    _suppress = False
    _log = Logger("main", Level.INFO)
    try:
        _args = parse_args()
        if _args == None:
            print('')
            _log.info('arguments: no action.')
        elif _args == -1:
            _suppress = True # help or docs
        else:
            # write log_to_file to global symbol table
            _level = Level.from_string(_args.level) if _args.level != None else Level.INFO
            _log.level = _level
            _log.debug('arguments: {}'.format(_args))
            _mros = MROS(level=_level)
            if _args.configure or _args.start:
                _mros.configure(_args)
                if not _args.start:
                    _log.info('configure only: ' + Fore.YELLOW + 'specify the -s argument to start mros.')
            if _args.start:
                _mros.start()
            # mros is now running…
    except KeyboardInterrupt:
        print(Style.BRIGHT + 'caught Ctrl-C; exiting…' + Style.RESET_ALL)
    except Exception:
        print(Fore.RED + Style.BRIGHT + 'error starting mros: {}'.format(traceback.format_exc()) + Style.RESET_ALL)
    finally:
        if not _suppress:
            _log.info('mros exit.')
        if _mros and not ( _mros.closing or _mros.closed ):
            _log.info('finally calling close…')
            _mros.close()

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
if __name__== "__main__":
    main(sys.argv[1:])

# prevent Python script from exiting abruptly
#signal.pause()

#EOF

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-04-08
# modified: 2024-04-08
#

import asyncio
from colorama import init, Fore, Style
init(autoreset=True)

import core.globals as globals
globals.init()

from core.logger import Logger, Level
from core.event import Event, Group
from core.orientation import Orientation
from core.subscriber import Subscriber
from hardware.color import Color
from hardware.sound import Player, Sound
from hardware.i2c_scanner import I2CScanner
from rgbmatrix5x5 import RGBMatrix5x5

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class RemoteControlSubscriber(Subscriber):
    CLASS_NAME = 'rcsub'
    '''
    A subscriber to handle remote control events.

    :param config:       the application configuration
    :param message_bus:  the message bus
    :param level:        the logging level
    '''
    def __init__(self, config, message_bus, level=Level.INFO):
        Subscriber.__init__(self, RemoteControlSubscriber.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        _i2c_scanner = I2CScanner(config, Level.WARN)
        if _i2c_scanner.has_address([0x74]):
            self._stbd_rgbmatrix = RGBMatrix5x5(address=0x74)
            self._stbd_rgbmatrix.set_brightness(0.8)
            self._stbd_rgbmatrix.set_clear_on_exit()
#           self._height = self._stbd_rgbmatrix.height
#           self._width  = self._stbd_rgbmatrix.width
            self._log.info('using rgbmatrix display.')
        else:
            self._log.warning('no rgbmatrix display found.')
            self._stbd_rgbmatrix = None
#           self._height = 5
#           self._width  = 5
        self.add_events(Event.by_groups([Group.REMOTE]))
        # configuration ................
        _cfg = config['mros'].get('subscriber').get('remote')
        self._play_sound = _cfg.get('play_sound')
        self._mros = globals.get('mros')
        if self._mros is None:
            raise Exception('mros not set in globals.')
        self._last_event = None
        self._player = Player.instance()
        self._clear()
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _arbitrate_message(self, message):
        '''
        Pass the message on to the Arbitrator and acknowledge that it has been
        sent (by setting a flag in the message).
        '''
        await self._message_bus.arbitrate(message.payload)
        # increment sent acknowledgement count
#       self._log.debug('acknowledging message {}; with payload value: {}'.format(message.name, message.payload.value))
        _value = message.value
        message.acknowledge_sent()
        self._log.info('arbitrated message ' + Fore.WHITE + '{} '.format(message.name)
                + Fore.CYAN + 'for event \'{}\' with value type: '.format(message.event.name)
                + Fore.YELLOW + '{}'.format(type(_value)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message.

        Mapping:

            Event.REMOTE_A    Sound.CHATTER_2
            Event.REMOTE_B    Sound.CHATTER_3
            Event.REMOTE_Y    Sound.CHATTER_4
            Event.REMOTE_X    Sound.TELEMETRY
            Event.REMOTE_D    Sound.CHIRP_1
            Event.REMOTE_R    Sound.CHATTER_1
            Event.REMOTE_U    Sound.CHIRP_2
            Event.REMOTE_L    Sound.CHATTER_5

        :param message:  the message to process.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        _event = message.event
        self._log.debug('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
        _value = message.value
        if _event.num != self._last_event:
            if _event.num == Event.REMOTE_A.num:
                self._remote_A(_value)
                self.play_sound(Sound.CHATTER_2)
            elif _event.num == Event.REMOTE_B.num:
                self._remote_B(_value)
                self.play_sound(Sound.CHATTER_3)
            elif _event.num == Event.REMOTE_Y.num:
                self._remote_Y(_value)
                self.play_sound(Sound.CHATTER_4)
            elif _event.num == Event.REMOTE_X.num:
                self._remote_X(_value)
                self.play_sound(Sound.TELEMETRY)
            elif _event.num == Event.REMOTE_D.num:
                self._remote_D(_value)
                self.play_sound(Sound.CHIRP_1)
            elif _event.num == Event.REMOTE_R.num:
                self._remote_R(_value)
                self.play_sound(Sound.CHATTER_1)
            elif _event.num == Event.REMOTE_U.num:
                self._remote_U(_value)
                self.play_sound(Sound.CHIRP_2)
            elif _event.num == Event.REMOTE_L.num:
                self._remote_L(_value)
                self.play_sound(Sound.CHATTER_5)
            else:
                self._log.warning('unrecognised RGB event on message {}'.format(message.name) + ''.format(message.event.name))
        self._last_event = _event.num
        await Subscriber.process_message(self, message)
        self._log.debug('post-processing message {}'.format(message.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def play_sound(self, sound):
        '''
        Plays the associated sound.
        '''
        if sound:
            print(Fore.MAGENTA + "sound: '{}'".format(sound.name))
            self._player.play(sound)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _remote_A(self, value):
        '''
        Reacts to the A button.
        '''
        print(Fore.MAGENTA + Style.DIM + "0. A message: '{}'".format(value))
        self._set_color(Color.BLACK)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _remote_B(self, value):
        '''
        Reacts to the B button.
        '''
        print(Fore.MAGENTA + Style.DIM + "1. B message: '{}'".format(value))
        self._set_color(Color.MAGENTA)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _remote_Y(self, value):
        '''
        Reacts to the Y button.
        '''
        print(Fore.MAGENTA + Style.BRIGHT + "2. Y message: '{}'".format(value))
        self._set_color(Color.ORANGE)
        self.startup()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _remote_X(self, value):
        '''
        Reacts to the X button.
        '''
        print(Fore.MAGENTA + Style.BRIGHT + "3. X message: '{}'".format(value))
        self._set_color(Color.CYAN)
        # TODO send Event.SHUTDOWN
        self._mros.shutdown()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _remote_D(self, value):
        '''
        Reacts to the Down arrow button.
        '''
        print(Fore.MAGENTA + Style.DIM + "4. DOWN message: '{}'".format(value))
        self._set_color(Color.YELLOW)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _remote_R(self, value):
        '''
        Reacts to the Right arrow button.
        '''
        print(Fore.MAGENTA + Style.DIM + "5. RIGHT message: '{}'".format(value))
        self._set_color(Color.GREEN)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _remote_U(self, value):
        '''
        Reacts to the Up arrow button.
        '''
        print(Fore.MAGENTA + Style.BRIGHT + "6. UP message: '{}'".format(value))
        self._set_color(Color.BLUE)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _remote_L(self, value):
        '''
        Reacts to the Left arrow button.
        '''
        print(Fore.MAGENTA + Style.DIM + "7. LEFT message: '{}'".format(value))
        self._set_color(Color.RED)

    # actions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
   
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def startup(self):
        _motor_configurer = self._mros.get_motor_configurer()
        _motor_configurer.set_thunderborg_leds(False)
        _screen = self._mros.get_screen()
        if _screen:
            _screen.disable() 
        _status_light = self._mros.get_status_light()
        if _status_light:
            _status_light.enable()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _clear(self, show=True):
        '''
        Clears the RGB Matrix by setting its color to black.
        '''
        if self._stbd_rgbmatrix:
            self._set_color(Color.BLACK, show)
        else:
            self._log.info('no rgb matrix available.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_color(self, color, show=True):
        if self._stbd_rgbmatrix:
            self._stbd_rgbmatrix.set_all(color.red, color.green, color.blue)
            if show:
                self._stbd_rgbmatrix.show()
        else:
            self._log.info('no rgb matrix available.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            Subscriber.disable(self)
        else:
            self._log.warning('rgb subscriber already disabled.')

#EOF

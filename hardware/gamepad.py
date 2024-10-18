#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-08-05
# modified: 2024-05-30
#
# This class interprets the signals arriving from the 8BitDo N30 Pro gamepad,
# a paired Bluetooth device.
#
# GamepadScan at bottom.
#

import os, sys
from pathlib import Path
import datetime as dt
from enum import Enum
from evdev import InputDevice, ecodes
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.message_bus import MessageBus
from core.message_factory import MessageFactory
from core.component import Component
from core.event import Event
from core.rate import Rate
from hardware.gamepad_mapping import GamepadMapping

'''
    Pairing and using a bluetooth gamepad device:

    1. prior to pairing your gamepad, list the current devices using:

       % ls /dev/input

    2. connect and pair the gamepad, then repeat the previous command. You'll
       notice a new device, e.g., "/dev/input/event6". This is likely your
       gamepad. You may need to check which of the devices was most recently
       changed to determine this, it isn't always the highest number.
    3. set the value of gamepad:device_path in the config.yaml file to the
       value of your gamepad device.
    4. be sure your gamepad is paired prior to starting mros.

    If everything seems all wired up but you're not getting a response from
    your gamepad, you may have configured a connection to the wrong device.

    How to Pair with a Raspberry Pi

    The first thing we need to do is connect our Bluetooth device to the Pi.
    Whilst you can do this using the Bluetooth GUI found on the taskbar, I
    find it’s easier to work in the terminal as everything will be easier to
    see, and it will help to understand what’s going on.

    First, let’s launch the Bluetooth control application, and make sure that
    it’s enabled (it may already be enabled, but just in case):

        sudo bluetoothctl

        power on

        agent on

        default-agent

    This by default does a scan for available devices. Now, let’s run a scan of
    available devices with the gamepad turned off:

        scan on

    Now, turn your gamepad on, and re scan. If your Pi is still scanning, it’ll
    let you know and start a new scan when the previous one is finished. Now you
    can find the new device which should be your gamepad. Make a note of the
    device address which is formatted in pairs of alphanumeric digits.

    Now that you’ve got the device ID, pair with it using the following formatting:

        pair XX:XX:XX:XX:XX:XX

    e.g.,

        pair E4:17:D8:37:32:77

    Wait for pairing to complete, then click ‘Ok’ on the dialogue box, then close
    the terminal.

    If the SystemSubscriber configuration has 'exit_on_dire_event' set True, this
    will shut down MROS if the Gamepad is not available or disconnected. This is
    so that the robot will stop moving if somehow communications with the Gamepad
    are cut off.

    This class based on information found at:

        https://core-electronics.com.au/tutorials/using-usb-and-bluetooth-controllers-with-python.html

'''

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Gamepad(Component):

    _exit_on_y_btn = False
    _NOT_AVAILABLE_ERROR = 'gamepad device not found (not configured, paired, powered or otherwise available)'

    def __init__(self, config, message_bus, message_factory, suppressed=False, enabled=True, level=Level.INFO):
        if not isinstance(suppressed, bool):
            raise ValueError('wrong type for suppressed argument: {}'.format(type(suppressed)))
        if not isinstance(enabled, bool):
            raise ValueError('wrong type for enabled argument: {}'.format(type(enabled)))
        '''
        Parameters:

           message_bus:      the message bus to receive messages from this task
           message_factory:  the factory for creating messages
        '''
        self._level = level
        self._log = Logger("gamepad", level)
        Component.__init__(self, self._log, suppressed=suppressed, enabled=enabled)
        if config is None:
            raise ValueError('no configuration provided.')
        if message_bus is None:
            raise ValueError('null message bus argument.')
        elif not isinstance(message_bus, MessageBus):
            raise ValueError('unrecognised message bus argument: {}'.format(type(message_bus)))
        self._message_bus     = message_bus
        if message_factory is None:
            raise ValueError('null message factory argument.')
        elif not isinstance(message_factory, MessageFactory):
            raise ValueError('unrecognised message factory argument: {}'.format(type(message_bus)))
        self._message_factory = message_factory
        self._log.info('initialising…')
        _cfg = config['mros'].get('hardware').get('gamepad')
        _loop_freq_hz         = _cfg.get('loop_freq_hz')
#       _loop_freq_hz = 20
        self._rate = Rate(_loop_freq_hz)
        self._device_path     = _cfg.get('device_path')
        self._log.info('device path:        {}'.format(self._device_path))
        self._suppress_horiz_events = _cfg.get('suppress_horiz_events')
        self._log.info('suppress horizontal events: {}'.format(self._device_path))
        self._gamepad_closed = False
        self._thread         = None
        self._gamepad        = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def connect(self):
        '''
        Scan for likely gamepad device, and if found, connect.
        Otherwise we raise an OSError.
        '''
        self._log.info('connecting at device path {}…'.format(self._device_path))
        _scan = GamepadScan(self._device_path, self._level)
        if not _scan.check_gamepad_device():
            self._log.warning('connection warning: gamepad is not the most recent device (configured at: {}).'.format(self._device_path))
            raise ConnectionError('no gamepad device found.')
        self._connect()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def set_exit_on_Y_BUTTON():
        Gamepad._exit_on_y_btn = True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def device_exists(self):
        '''
        Returns True if the gamepad device exists.
        '''
        _device = Path(self._device_path)
        return _device.exists()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def has_connection(self):
        return self._gamepad != None and self.device_exists()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _connect(self):
        self._log.info('connecting gamepad…')
        try:
            if not self.device_exists():
                raise Exception("gamepad device '{}' does not exist.".format(self._device_path))
            self._gamepad = InputDevice(self._device_path)
            # display device info
            self._log.info(Fore.YELLOW + 'gamepad: {} at path: {}'.format(self._gamepad, self._device_path))
            self._log.info('connected.')
        except Exception as e:
            Component.disable(self)
            self._gamepad = None
            raise ConnectionError('unable to connect to input device path {}: {}'.format(self._device_path, e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
#       if not self.enabled and not self.closed:
        if not self.enabled:
            if not self.in_loop():
                if self._gamepad == None:
                    self.connect()
                    Component.enable(self)
                    self._log.info('enabled gamepad.')
            else:
                self._log.warning('already started gamepad.')
        elif self.closed:
            self._log.warning('cannot enable gamepad: already closed.')
            Component.disable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def in_loop(self):
        '''
        Returns true if the main loop is active (the thread is alive).
        '''
        return self._thread != None and self._thread.is_alive()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def convert_range(value):
        return ( (value - 127.0) / 255.0 ) * -2.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _gamepad_loop(self, callback, f_is_enabled):
        self._log.info('starting event loop…')
        __enabled = True
        while __enabled and f_is_enabled():
            self._log.info('gamepad enabled.')
            try:
                if self._gamepad is None:
                    raise Exception(Gamepad._NOT_AVAILABLE_ERROR + ' [gamepad no longer available]')
                # loop and filter by event code and print the mapped label
                async for _event in self._gamepad.async_read_loop():
                    _message = self._handleEvent(_event)
                    if callback and _message:
                        await callback(_message)
                    if not f_is_enabled():
                        self._log.debug('breaking from event loop.')
                        break
                self._log.info('exit gamepad loop.')
            except KeyboardInterrupt:
                self._log.info('caught Ctrl-C, exiting…')
                __enabled = False
            except Exception as e:
                self._log.error('gamepad device error: {}'.format(e))
                __enabled = False
            except OSError as e:
                self._log.error(Gamepad._NOT_AVAILABLE_ERROR + ' [lost connection to gamepad]')
                __enabled = False
            finally:
                '''
                Note that closing the InputDevice is a bit tricky, and we're currently
                masking tn exception that's always thrown. As there is no data loss on
                a gamepad event loop being closed suddenly this is not an issue.
                '''
                try:
                    self._log.info('closing gamepad device…')
                    self._gamepad.close()
                    self._log.info(Fore.YELLOW + 'gamepad device closed.')
                except Exception as e:
                    self._log.info('error closing gamepad device: {}'.format(e))
                finally:
                    __enabled = False
                    self.disable()
                    self._gamepad_closed = True

            self._rate.wait()
        self._log.info('exited event loop.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _handleEvent(self, event):
        '''
        Handles the incoming event by filtering on event type and code.
        There's possibly a more elegant way of doing this but for now this
        works just fine.
        '''
        _message = None
        _control = None
#       self._log.debug("event type: EV_KEY; event: {}; value: {}".format(event.code, event.value))
        if event.type == ecodes.EV_KEY:
            _control = GamepadMapping.get_by_code(self, event)
            if event.value == 1:
                if event.code == GamepadMapping.A_BUTTON.code:
                    self._log.info(Fore.RED + "A Button")
                    _control = GamepadMapping.A_BUTTON
                elif event.code == GamepadMapping.B_BUTTON.code:
                    self._log.info(Fore.RED + "B Button")
                    _control = GamepadMapping.B_BUTTON
                elif event.code == GamepadMapping.X_BUTTON.code:
                    self._log.info(Fore.RED + "X Button")
                    _control = GamepadMapping.X_BUTTON
                elif event.code == GamepadMapping.Y_BUTTON.code:
                    self._log.info(Fore.RED + "Y Button: exit? {}".format(Gamepad._exit_on_y_btn))
                    _control = GamepadMapping.Y_BUTTON
                    if Gamepad._exit_on_y_btn:
                        self._log.info(Style.BRIGHT + 'exit on Y Button…')
                        sys.exit(0)
                elif event.code == GamepadMapping.L1_BUTTON.code:
                    self._log.info(Fore.YELLOW + "L1 Button")
                    _control = GamepadMapping.L1_BUTTON
                elif event.code == GamepadMapping.L2_BUTTON.code:
                    self._log.info(Fore.YELLOW + "L2 Button")
                    _control = GamepadMapping.L2_BUTTON
                elif event.code == GamepadMapping.R1_BUTTON.code:
                    self._log.info(Fore.YELLOW + "R1 Button")
                    _control = GamepadMapping.R1_BUTTON
                elif event.code == GamepadMapping.R2_BUTTON.code:
                    self._log.info(Fore.YELLOW + "R2 Button")
                    _control = GamepadMapping.R2_BUTTON
                elif event.code == GamepadMapping.START_BUTTON.code:
                    self._log.info(Fore.GREEN + "Start Button")
                    _control = GamepadMapping.START_BUTTON
                elif event.code == GamepadMapping.SELECT_BUTTON.code:
                    self._log.info(Fore.GREEN + "Select Button")
                    _control = GamepadMapping.SELECT_BUTTON
                elif event.code == GamepadMapping.HOME_BUTTON.code:
                    self._log.info(Fore.MAGENTA + "Home Button")
                    _control = GamepadMapping.HOME_BUTTON
                else:
                    self._log.warning("unexpected event type: EV_KEY; event: {}; value: {}".format(event.code, event.value))
                pass
        elif event.type == ecodes.EV_ABS:
            _control = GamepadMapping.get_by_code(self, event)
            if _control == GamepadMapping.DPAD_HORIZONTAL:
                self._log.warning("D-Pad Horizontal(N) {}".format(event.value))
                return None
            elif _control == GamepadMapping.DPAD_LEFT:
                self._log.info("D-Pad LEFT {}".format(event.value))
            elif _control == GamepadMapping.DPAD_RIGHT:
                self._log.info("D-Pad RIGHT {}".format(event.value))
            elif _control == GamepadMapping.DPAD_VERTICAL:
                self._log.warning("D-Pad Vertical(N) {}".format(event.value))
                return None
            elif _control == GamepadMapping.DPAD_UP:
                self._log.info("D-Pad UP {}".format(event.value))
            elif _control == GamepadMapping.DPAD_DOWN:
                self._log.info("D-Pad DOWN {}".format(event.value))
            elif event.code == GamepadMapping.L3_VERTICAL.code:
                self._log.debug(Fore.MAGENTA + "L3 Vertical {}".format(event.value))
#               _control = GamepadMapping.R3_VERTICAL
            elif event.code == GamepadMapping.L3_HORIZONTAL.code:
                self._log.info(Fore.YELLOW + "L3 Horizontal {}".format(event.value))
                if self._suppress_horiz_events:
                    _control = None
            elif event.code == GamepadMapping.R3_VERTICAL.code:
                self._log.debug(Fore.GREEN + "R3 Vertical {}".format(event.value))
#               _control = GamepadMapping.R3_VERTICAL
            elif event.code == GamepadMapping.R3_HORIZONTAL.code:
#               self._log.info(Fore.GREEN + "R3 Horizontal {}".format(event.value))
#               if self._suppress_horiz_events:
#                   _control = None
                pass
            else:
                pass
        else:
            pass
        if _control != None:
            _message = self._message_factory.create_message(_control.event, event.value)
            return _message
        return None

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GamepadScan(object):
    '''
    Returns the device with the most recently changed status from /dev/input/event{n}
    This can help you figure out which device is your gamepad, if if was connected
    after everything else in the system had settled.
    '''
    def __init__(self, device_path, level):
        self._log = Logger("gamepad-scan", level)
        self._device_path = device_path
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_ctime(self, path):
        try:
            _device_stat = os.stat(path)
            return _device_stat.st_ctime
        except OSError:
            return -1.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_latest_device(self):
        '''
        Build a dictionary of available devices, return the one with the
        most recent status change.
        '''
        _dict = {}
        for i in range(10):
            _path = '/dev/input/event{}'.format(i)
            try:
                _device_stat = os.stat(_path)
                _ctime = _device_stat.st_ctime
            except OSError:
                break
            self._log.debug('device path:        ' + Fore.YELLOW + '{}\t'.format(_path) + Fore.CYAN + '  status changed: ' + Fore.YELLOW + '{}'.format(dt.datetime.fromtimestamp(_ctime)))
            _dict[_path] = _ctime
        # find most recent by sorting the dictionary on ctime
        _sorted = sorted(_dict.items(), key=lambda x:x[1])
        if len(_sorted) == 0:
            return None
        _latest_devices = _sorted[len(_sorted)-1]
        _latest_device  = _latest_devices[0]
        self._log.debug('device path config: ' + Fore.YELLOW + '{}'.format(self._device_path))
        self._log.debug('most recent device: ' + Fore.YELLOW + '{}'.format(_latest_device))
        return _latest_device

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def check_gamepad_device(self):
        '''
        Checks that the configured device matches the device with the most
        recently changed status, returning True if matched.
        '''
        _latest_device = self.get_latest_device()
        if _latest_device is None:
            self._log.warning('no devices found.')
            return False
        elif self._device_path == _latest_device:
            self._log.info('device matches:     {}'.format(self._device_path))
            return True
        else:
            self._log.warning('does not match:     {}'.format(_latest_device))
            return False

# EOF

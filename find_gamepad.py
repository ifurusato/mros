#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-08-05
# modified: 2024-05-06
#
# This class interprets the signals arriving from the 8BitDo N30 Pro gamepad,
# a paired Bluetooth device.
#
#   pair E4:17:D8:37:05:72
#

import os, sys, time, asyncio, traceback
import datetime as dt
from enum import Enum
from evdev import InputDevice, ecodes
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader

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

        pair E4:17:D8:37:05:72

    Once it has paired, you then trust and connect:

        trust E4:17:D8:37:05:72
        connect E4:17:D8:37:05:72

    This class based on information found at:

        https://core-electronics.com.au/tutorials/using-usb-and-bluetooth-controllers-with-python.html

    If you have difficulty connecting, edit:

        /lib/systemd/system/bthelper@.service

    as according to::

        https://raspberrypi.stackexchange.com/a/123914
'''

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GamepadFinder(object):

    _NOT_AVAILABLE_ERROR = 'gamepad device not found (not configured, paired, powered or otherwise available)'

    def __init__(self, config, level=Level.INFO):
        self._level = level
        self._log = Logger("gamepad-finder", level)
        if config is None:
            raise ValueError('no configuration provided.')
        self._config = config
        self._log.info('initialising...')
        _config = self._config['mros'].get('hardware').get('gamepad')
        _loop_freq_hz = _config.get('loop_freq_hz')
#       _loop_freq_hz = 20
        self._device_path     = _config.get('device_path')
        self._log.info('device path:        {}\t'.format(self._device_path))
#       self._device_path     = '/dev/input/event5' # the path to the bluetooth gamepad on the pi (see find_gamepad.py)
        self._gamepad_closed  = False
        self._thread          = None
        self._gamepad         = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def connect(self):
        '''
        Scan for likely gamepad device, and if found, connect.
        Otherwise we raise an OSError.
        '''
        self._log.info(Fore.YELLOW + 'connecting at device path {}…'.format(self._device_path))
        _scan = GamepadScan(self._device_path, self._level)
        if not _scan.check_gamepad_device():
            self._log.warning('connection warning: gamepad is not the most recent device (configured at: {}).'.format(self._device_path))
#           raise ConnectionError('no gamepad device found.')
        self._connect()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def has_connection(self):
        return self._gamepad != None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _connect(self):
        self._log.info('scanning for gamepad…')
        try:
            self._gamepad = InputDevice(self._device_path)
            # display device info
            self._log.info(Fore.GREEN + "gamepad: {}".format(self._gamepad))
            self._log.info('connected.')
        except Exception as e:
            self._gamepad = None
            raise ConnectionError('unable to connect to input device path {}: {}'.format(self._device_path, e))

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
                _input_device = InputDevice(_path)
                # display device info
                self._log.info(Fore.GREEN + Style.BRIGHT + "device info: {}".format(_input_device))
            except Exception as e:
                self._log.warning('unable to connect to input device path {}: {}'.format(self._device_path, e))
            except OSError:
                break
            self._log.info('device path:        ' + Fore.YELLOW + '{}\t'.format(_path) + Fore.CYAN + '  status changed: ' + Fore.YELLOW + '{}'.format(dt.datetime.fromtimestamp(_ctime)))
            _dict[_path] = _ctime
        # find most recent by sorting the dictionary on ctime
        _sorted = sorted(_dict.items(), key=lambda x:x[1])
        if len(_sorted) == 0:
            return None
        _latest_devices = _sorted[len(_sorted)-1]
        _latest_device = _latest_devices[0]
        self._log.info('device path config: ' + Fore.YELLOW + '{}'.format(self._device_path))
        self._log.info('most recent device: ' + Fore.YELLOW + '{}'.format(_latest_device))
        return _latest_device

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def check_gamepad_device(self):
        '''
        Checks that the configured device matches the device with the most
        recently changed status, returning True if matched.
        '''
        _latest_device = self.get_latest_device()
        if _latest_device is None:
            self._log.warning(Style.BRIGHT + 'no devices found.')
            return False
        elif self._device_path == _latest_device:
            self._log.info(Style.BRIGHT + 'matches:            {}'.format(self._device_path))
            return True
        else:
            self._log.warning(Style.BRIGHT + 'does not match:     {}'.format(_latest_device))
            return False


# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    try:
        # read YAML configuration
        _level = Level.INFO
        _config = ConfigLoader(Level.INFO).configure()

        _gamepad = GamepadFinder(_config, level=Level.INFO)
        _gamepad.connect()

    except Exception as e:
        print(Fore.RED + 'error in motor test: {}'.format(e) + Style.RESET_ALL)
        traceback.print_exc(file=sys.stdout)
    finally:
        pass

if __name__== "__main__":
    main()


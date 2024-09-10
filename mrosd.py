#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-08-01
# modified: 2024-09-04
#
# MR01 Robot Operating System Daemon (mrosd). This also uses the mrosd.service.
#
# see: https://dpbl.wordpress.com/2017/02/12/a-tutorial-on-python-daemon/
#
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import os
import sys, traceback
from pathlib import Path
import time
import itertools
import subprocess
import signal

try:
    import RPi.GPIO as GPIO
except Exception:
    sys.exit('This script requires the RPi.GPIO module.\nInstall with: sudo pip3 install RPi.GPIO')
try:
    import daemon
    from daemon import pidfile
except Exception:
    sys.exit("This script requires the python-daemon module.\nInstall with: pip3 install --user python-daemon")

from core.util import Util
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from mros import MROS
from mros import parse_args
from hardware.player import Player
from hardware.sound import Sound

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

WORK_DIR = '/home/pi/workspaces/workspace-mros/mros/'
PID_FILE = WORK_DIR + '.mrosd.pid'

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def shutdown(signum, frame):  # signum and frame are mandatory
    print('mrosd.shutdown')
    sys.exit(0)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class PButton(object):
    PIN = 8
    def __init__(self, level=Level.INFO):
        self._log = Logger('pbtn', level)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PButton.PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def pushed(self):
        _value = not GPIO.input(PButton.PIN)
        time.sleep(0.1)
        return _value

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class MrosDaemon(object):
    '''
    Monitors a push switch connected to a GPIO pin.

    This state is used to enable or disable the MROS.
    '''
    def __init__(self, level):
        self._level = level
        self._log = Logger("mrosd", self._level)
        self._log.info(Fore.WHITE + 'initialising mrosd…')
        self._use_subprocess = True # if True, start MROS as subprocess, otherwise call directly
        self._pushbutton = PButton(level=Level.INFO)
        self._counter   = itertools.count()
        self._old_state = False
        self._mros      = None
        _rosd_mask      = os.umask(0)
        os.umask(_rosd_mask)
        self._log.info(Fore.WHITE + 'mask: {}'.format(_rosd_mask))
        self._log.info(Fore.WHITE + 'uid:  {}'.format(os.getuid()))
        self._log.info(Fore.WHITE + 'gid:  {}'.format(os.getgid()))
        self._log.info(Fore.WHITE + 'cwd:  {}'.format(os.getcwd()))
        self._log.info(Fore.WHITE + 'pid file: {}'.format(PID_FILE))
        self._log.info(Fore.WHITE + 'mrosd ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_timestamp(self):
        return dt.utcfromtimestamp(dt.utcnow().timestamp()).isoformat()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def read_state(self):
        '''
        Reads the state of the push button switch then returns the value.
        This only calls enable() or disable() if the value has changed
        since last reading.
        '''
        if next(self._counter) % 30 == 0: # call every second, log every 30 seconds
#           self._log.info(Fore.WHITE + 'mros daemon waiting…')
            pass
        self._state = self._pushbutton.pushed()
        self._log.debug(Fore.WHITE + 'read state: {}'.format(self._state))
        if self._state is not self._old_state:
            if self._state:
                self._log.info(Fore.WHITE + 'enabling MROS from state: {}'.format(self._state))
                self.enable()
            else:
                self._log.info(Fore.WHITE + 'disabling MROS from state: {}'.format(self._state))
                self.disable()
            self._old_state = self._state

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._enable_mros()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self._disable_mros()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _enable_mros(self):
        if Util.already_running('mros.py'):
            self._log.info(Fore.WHITE + 'mros already running…')
        else:
            Player.instance().play(Sound.SONIC_BAT)
            self._log.info(Fore.WHITE + 'starting mros at {}…'.format(self._get_timestamp()))
            if self._use_subprocess:
                _result = subprocess.check_output(['mros.py', '-s', '-g'])
                _lines = _result.splitlines()
                for _bytes in _lines:
                    _line = _bytes.decode('utf-8').strip() # convert byte array to string
                    if len(_line) > 0:
                        self._log.info(Fore.WHITE + Style.DIM + "line: '{}'".format(_line))
            else:
                self._mros = MROS(level=Level.INFO)
                self._mros.configure(parse_args(['-s', '-g']))
                self._mros.start()
                pass
            self._log.info(Fore.WHITE + 'mros started.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _disable_mros(self):
        if self._mros is not None:
            self._log.info(Fore.WHITE + 'mros state disabled at: {}'.format(self._get_timestamp()))
#           self._log.info(Fore.WHITE + 'suppressing mros arbitrator…')
#           _arbitrator = self._mros.get_arbitrator()
#           _arbitrator.set_suppressed(True)
            self._mros.shutdown()
            self._log.info(Fore.WHITE + 'mros arbitrator suppressed.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self._log.info(Fore.WHITE + 'closing mrosd…')
        if self._mros is not None:
            self._log.info(Fore.WHITE + 'closing mros…')
            self._mros.close()
            self._log.info(Fore.WHITE + 'mros closed.')
        else:
            self._log.warning('mros thread was null.')
        self._log.info(Fore.WHITE + 'mrosd closed.')

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    _daemon = None
    try:
        _daemon = MrosDaemon(Level.INFO)
        while True:
            _daemon.read_state()
            time.sleep(0.2)
    except Exception:
        print(Fore.WHITE + 'error starting mros daemon: {}'.format(traceback.format_exc()) + Style.RESET_ALL)
    finally:
        if _daemon:
            try:
                _daemon.close()
            except Exception:
                print(Fore.RED + 'error closing mros daemon.' + Style.RESET_ALL)
        print(Fore.WHITE + 'mrosd complete.' + Style.RESET_ALL)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if Path(PID_FILE).is_file() and not Util.already_running('mros.py'):
    os.remove(PID_FILE)
    print(Fore.WHITE + 'deleted previous pid file.' + Style.RESET_ALL)

with daemon.DaemonContext(
    stdout=sys.stdout,
    stderr=sys.stderr,
    working_directory=WORK_DIR,
    umask=0o002,
#   pidfile=pidfile.TimeoutPIDLockFile(PID_FILE)) as context:
    pidfile=pidfile.TimeoutPIDLockFile(PID_FILE),
    signal_map={ signal.SIGTERM: shutdown, signal.SIGTSTP: shutdown }) as context:
    main()

#EOF

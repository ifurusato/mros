#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-25
# modified: 2024-05-25
#

import sys, time, traceback
from threading import Thread
from colorama import init, Fore, Style
init()

import ioexpander as io

from core.component import Component
from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Indicator(Component):
    '''
    A simple thread-based LED blinker that uses a pin on the IO Expander.

    To be clear: the constructor argument is not a Raspberry Pi GPIO pin
    number, it is the pin on the IO Expander.

    It can also be used as a simple pin controller.
    '''
    IOE_I2C_ADDRESS = 0x18

    def __init__(self, pin, level=Level.DEBUG):
        self._log = Logger('indicator', level)
        self._pin = pin
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._ioe = io.IOE(i2c_addr=Indicator.IOE_I2C_ADDRESS)
        self._ioe.set_mode(self._pin, io.OUT)
        self._ioe.output(self._pin, io.LOW)
        self._loop_enabled = True
        self._loop_thread  = None
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self.set(True)
        Component.enable(self)
 
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self.set(False)
        Component.disable(self)
 
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set(self, enable):
        '''
        Turns the LED on or off.
        '''
        if enable:
            self._ioe.output(self._pin, io.HIGH)
        else:
            self._ioe.output(self._pin, io.LOW)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def loop_is_running(self):
        '''
        Returns true if using an external clock or if the loop thread is alive.
        '''
        return ( self._loop_enabled and self._loop_thread != None and self._loop_thread.is_alive() )

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def blink(self, delay_sec):
        '''
        Blink the LED at the provided rate.
        '''
        self._log.info('start blinking…')
        if self.loop_is_running:
            self._log.warning('loop already running.')
        elif self._loop_thread is None:
            self._loop_enabled = True
            _is_daemon = False
            self._loop_thread = Thread(name='blink_loop', target=Indicator._loop, args=[self, delay_sec, lambda: self._loop_enabled], daemon=_is_daemon)
            self._loop_thread.start()
            self._log.info('loop enabled.')
        else:
            raise Exception('cannot enable loop: thread already exists.')

   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _loop(self, delay_sec, f_is_enabled):
        '''
        The blink loop.
        '''
        try:
            while f_is_enabled():
                print(Fore.GREEN + 'ON' + Style.RESET_ALL)
                self.set(True)
                time.sleep(delay_sec)
                print(Fore.RED   + 'OFF' + Style.RESET_ALL)
                self.set(False)
                time.sleep(delay_sec)
        except KeyboardInterrupt:
            self._log.info('Ctrl-C caught; exiting…')
        except Exception as e:
            self._log.error('error in loop: {}\n{}'.format(e, traceback.format_exc()))
        finally:
            self._log.info('exited loop.')
            self.close()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _stop_loop(self):
        '''
        Stop the loop.
        '''
        if self.loop_is_running:
            self._loop_enabled = False
            self._loop_thread  = None
            self._log.info('loop disabled.')
        else:
            self._log.warning('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Stop the loop if running, then close the Indicator.
        '''
        self.set(False)
        self._stop_loop()
        Component.close(self) # calls disable

#EOF

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2020-01-18
# modified: 2024-05-25
#
# An integrated sensor array, including front bumper, oblique (front-facing)
# IR distance sensors, and wheel-based IR proximity sensors. Rather than
# maintain the artificial distinction between event source and publisher (as
# in the past), this class is also directly publishes events onto the message
# bus.
#
# Wraps the functionality of a Pimoroni IO Expander Breakout board, providing
# access to the values of the board's pins, which outputs 0-255 values for
# analog pins, and a 0 or 1 for digital pins.
#

import sys
import asyncio
from datetime import datetime as dt
import RPi.GPIO as GPIO
from colorama import init, Fore, Style
init()

import ioexpander as io

from core.logger import Logger, Level
from core.event import Event, Group
from core.stringbuilder import StringBuilder
from core.convert import Convert
from core.util import Util
from core.publisher import Publisher
from hardware.i2c_scanner import I2CScanner

# SensorData class at bottom

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SensorArray(Publisher):
    CLASS_NAME = 'sensors'
    _LISTENER_LOOP_NAME = '__sensor_array_listener_loop'
    '''
    Wraps an IO Expander board as input for an integrated sensor array
    of bumper switches, infrared distance and proximity sensors.

    This polls the forward IO Expander for bumper and infrared data,
    posting a single message wth event type BUMPER_ANY for each loop,
    if any sensor has been triggered, with the message value containing
    a list of corresponding triggered events. Events remain triggered
    for a delayed period of time.

        bumper events ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        BUMPER_ANY   = ( 110, "any bumper",            4, Group.BUMPER )
        BUMPER_MAST  = ( 111, "mast bumper",           4, Group.BUMPER )
        BUMPER_PORT  = ( 112, "port bumper",           4, Group.BUMPER )
        BUMPER_CNTR  = ( 113, "center bumper",         4, Group.BUMPER )
        BUMPER_STBD  = ( 114, "starboard bumper",      4, Group.BUMPER )
        BUMPER_PFWD  = ( 115, "port fwd bumper",       4, Group.BUMPER )
        BUMPER_PAFT  = ( 116, "port aft bumper",       4, Group.BUMPER )
        BUMPER_SFWD  = ( 117, "starboard fwd bumper",  4, Group.BUMPER )
        BUMPER_SAFT  = ( 118, "starboard aft bumper",  4, Group.BUMPER )
        BUMPER_FOBP  = ( 119, "fwd oblique port",      4, Group.BUMPER )
        BUMPER_FOBS  = ( 120, "fwd oblique starboard", 4, Group.BUMPER )

    TODO: don't trigger IRs when angle suggests the robot is seeing itself.

    :param config:            the application configuration
    :param message_bus:       the asynchronous message bus (unused, required for Publisher API)
    :param message_factory:   the factory for creating messages
    :param level:             the log level
    '''
    def __init__(self, config, message_bus, message_factory, level=Level.INFO):
        Publisher.__init__(self, SensorArray.CLASS_NAME, config, message_bus, message_factory, level=level)
        if config is None:
            raise ValueError('no configuration provided.')
#       self._queue_publisher = queue_publisher
        _cfg = config['mros'].get('publisher').get('sensor_array')
        self._fwd_i2c_address = _cfg.get('fwd_i2c_address')
        self._aft_i2c_address = _cfg.get('aft_i2c_address')
        # fwd IOE pins ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._bsb_pin = _cfg.get('bsb_pin') # 1 bumper starboard bottom
        self._bst_pin = _cfg.get('bst_pin') # 2 bumper starboard top
        self._bpb_pin = _cfg.get('bpb_pin') # 3 bumper port bottom
        self._bpt_pin = _cfg.get('bpt_pin') # 4 bumper port top
        self._log.info('bumper pin assignments:         '
                + Fore.RED   + ' port bottom={:d}; top={:d};'.format(self._bpb_pin, self._bpt_pin)
                + Fore.GREEN + ' stbd bottom={:d}; top={:d}'.format(self._bsb_pin, self._bst_pin))
        self._fop_pin = _cfg.get('fop_pin') # 9 was 7 oblique fore port
        self._fos_pin = _cfg.get('fos_pin') # 8 oblique fore starboard
        self._log.info('oblique sensor pin assignments: '
                + Fore.RED   + ' port={:d};'.format(self._fop_pin)
                + Fore.GREEN + '               stbd={:d}'.format(self._fos_pin))
        self._wsa_pin = _cfg.get('wsa_pin') # 11 wheel starboard aft
        self._wsf_pin = _cfg.get('wsf_pin') # 12 wheel starboard fore
        self._wpa_pin = _cfg.get('wpa_pin') # 13 wheel port aft
        self._wpf_pin = _cfg.get('wpf_pin') # 14 wheel starboard fore
        self._log.info('wheel sensor pin assignments:   ' \
                + Fore.RED + ' port fore={:d}; aft={:d};'.format(self._wpf_pin, self._wpa_pin) \
                + Fore.GREEN + ' stbd fore={:d}; aft={:d}'.format(self._wsf_pin, self._wsa_pin))
        # aft IOE pins ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._mast_pin = _cfg.get('mast_pin') # mast IR sensor pin on aft IOE
        # et cetera ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _loop_freq_hz     = _cfg.get('loop_freq_hz')
        self._publish_delay_sec = 1.0 / _loop_freq_hz
        self._counter_limit = 100 # how many 20Hz cycles? (5s)
        self._oblique_trigger_cm = _cfg.get('oblique_trigger_cm') # min distance trigger oblique IR
        self.reset_triggers(0)
        self._ts_limit = 1.0 # how long a timestamp (e.g., '1716596193.904905') lasts
        self._page = Util.repeat('\n', 50)
        self._fop_cm   = 0
        self._fos_cm   = 0
        self._last_sensor_data = SensorData(0,0)
        # thread support
#       self._thread = None
#       self._thread_enabled = False
        self._suppressed = []
        self._verbose  = False

        # configure board
        _i2c_scanner = I2CScanner(config, bus_number=1, level=level)

        try:
            if _i2c_scanner.has_address([self._fwd_i2c_address]):
                self._log.info('found forward IO Expander at address 0x{:02X}, configuring…'.format(self._fwd_i2c_address))
                self._fwd_ioe = io.IOE(i2c_addr=self._fwd_i2c_address)
                # configure digital pins ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                self._fwd_ioe.set_mode(self._bsb_pin, io.IN_PU) # bumper starboard bottom
                self._fwd_ioe.set_mode(self._bst_pin, io.IN_PU) # bumper starboard top
                self._fwd_ioe.set_mode(self._bpb_pin, io.IN_PU) # bumper port bottom
                self._fwd_ioe.set_mode(self._bpt_pin, io.IN_PU) # bumper port top
                self._fwd_ioe.set_mode(self._wsa_pin, io.IN_PU) # wheel starboard aft
                self._fwd_ioe.set_mode(self._wsf_pin, io.IN_PU) # wheel starboard fore
                self._fwd_ioe.set_mode(self._wpa_pin, io.IN_PU) # wheel port aft
                self._fwd_ioe.set_mode(self._wpf_pin, io.IN_PU) # wheel starboard fore
                # configure analog pins ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                self._fwd_ioe.set_mode(self._fop_pin, io.ADC)   # oblique fore port IR distance
                self._fwd_ioe.set_mode(self._fos_pin, io.ADC)   # oblique fore starboard IR distance
                self._fwd_ioe.set_adc_vref(3.3)  # input voltage of IO Expander, this is 3.3 on Breakout Garden
                self._log.info('forward IOE ready.')
            else:
                raise Exception('no forward IO Expander found.')

            if _i2c_scanner.has_address([self._aft_i2c_address]):
                self._log.info('found aft IO Expander at address 0x{:02X}, configuring…'.format(self._aft_i2c_address))
                self._aft_ioe = io.IOE(i2c_addr=self._aft_i2c_address)
                # configure digital pins ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                self._aft_ioe.set_mode(self._mast_pin, io.IN_PU) # bumper starboard bottom
                self._aft_ioe.set_adc_vref(3.3)  # input voltage of IO Expander, this is 3.3 on Breakout Garden
                self._log.info('aft IOE ready.')
            else:
                raise Exception('no aft IO Expander found.')
        except ImportError:
            raise Exception('This script requires the pimoroni-ioexpander module\nInstall with: pip3 install --user pimoroni-ioexpander')

   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'sensors'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_verbose(self):
        self._verbose = True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Publisher.enable(self)
        if self.enabled:
            if self._message_bus.get_task_by_name(SensorArray._LISTENER_LOOP_NAME):
                self._log.warning('already enabled.')
            else:
                self._log.info('creating task for sensor array monitor loop…')
                self._message_bus.loop.create_task(self._monitor_loop(lambda: self.enabled), name=SensorArray._LISTENER_LOOP_NAME)
            self._log.info('enabled.')
        else:
            self._log.warning('failed to enable publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def toggle(self):
        if self.suppressed:
            self.release()
        else:
            self.suppress()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def release(self):
        '''
        Releases (un-suppresses) this Publisher.
        '''
        if not self.enabled:
            self._log.warning('sensor array not enabled.')
        else:
            Publisher.release(self)
            self._log.info('sensor array released.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def suppress(self):
        '''
        Suppresses this Publisher.
        '''
        if not self.enabled:
            self._log.warning('sensor array not enabled.')
        else:
            Publisher.suppress(self)
            self._log.info('sensor array suppressed.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def suppress_events(self, events):
        '''
        Suppresses the set of Events. Calling this method with a None argument
        will clear the list.
        '''
        if events is None:
            self._suppressed.clear()
        else:
            for _event in events:
                self._suppressed.append(_event)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable this publisher.
        '''
        Publisher.disable(self)
        self._log.info('disabled publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def any_triggered(self):
        '''
        Returns True if anything has been triggered.
        '''
        return ( self._port_bmp_triggered + self._stbd_bmp_triggered + self._mast_triggered
                + self._paft_triggered + self._saft_triggered
                + self._pfwd_triggered + self._sfwd_triggered
                + self._fobp_triggered + self._fobs_triggered ) > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def mast_triggered(self):
        '''
        Returns True if the mast IR sensor has been triggered.
        '''
        return self._mast_triggered > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def port_bumper_triggered(self):
        '''
        Returns True if the port bumper has been triggered.
        '''
        return self._port_bmp_triggered > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def stbd_bumper_triggered(self):
        '''
        Returns True if the starboard bumper has been triggered.
        '''
        return self._stbd_bmp_triggered > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def wheel_port_aft_triggered(self):
        '''
        Returns True if the port aft wheel sensor has been triggered.
        '''
        return self._paft_triggered > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def wheel_stbd_aft_triggered(self):
        '''
        Returns True if the starboard aft wheel sensor has been triggered.
        '''
        return self._saft_triggered > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def wheel_port_fwd_triggered(self):
        '''
        Returns True if the port forward wheel sensor has been triggered.
        '''
        return self._pfwd_triggered > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def wheel_stbd_fwd_triggered(self):
        '''
        Returns True if the starboard forward wheel sensor has been triggered.
        '''
        return self._sfwd_triggered > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def port_oblique_triggered(self):
        '''
        Returns True if the port oblique IR sensor has been triggered.
        '''
        return self._fobp_triggered > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def stbd_oblique_triggered(self):
        '''
        Returns True if the starboard oblique IR sensor has been triggered.
        '''
        return self._fobs_triggered > 0.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def fwd_oblique_port_cm(self):
        '''
        Returns the last-read value of the fore oblique starboard IR sensor
        in centimeters, regardless of being triggered.
        '''
        return self._fop_cm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def fwd_oblique_port_cm(self):
        '''
        Returns the last-read value of the fore oblique port IR sensor in
        centimeters, regardless of being triggered.
        '''
        return self._fos_cm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset_triggers(self, now_ts):
        '''
        Resets the loop counter and any bumper or wheel sensor counts.
        '''
        if now_ts == 0:
            self._port_bmp_triggered = 0.0
            self._stbd_bmp_triggered = 0.0
            self._mast_triggered = 0.0
            self._paft_triggered = 0.0
            self._saft_triggered = 0.0
            self._pfwd_triggered = 0.0
            self._sfwd_triggered = 0.0
            self._fobp_triggered = 0.0
            self._fobs_triggered = 0.0
        else:
            if now_ts - self._mast_triggered > self._ts_limit:
                self._mast_triggered = 0.0
            if now_ts - self._port_bmp_triggered > self._ts_limit:
                self._port_bmp_triggered = 0.0
            if now_ts - self._stbd_bmp_triggered > self._ts_limit:
                self._stbd_bmp_triggered = 0.0
            if now_ts - self._paft_triggered > self._ts_limit:
                self._paft_triggered = 0.0
            if now_ts - self._saft_triggered > self._ts_limit:
                self._saft_triggered = 0.0
            if now_ts - self._pfwd_triggered > self._ts_limit:
                self._pfwd_triggered = 0.0
            if now_ts - self._sfwd_triggered > self._ts_limit:
                self._sfwd_triggered = 0.0
            if now_ts - self._fobp_triggered > self._ts_limit:
                self._fobp_triggered = 0.0
            if now_ts - self._fobs_triggered > self._ts_limit:
                self._fobs_triggered = 0.0
        self._counter = 0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _monitor_loop(self, f_is_enabled):

        self._log.info('monitoring IO Expander pins…')

        while f_is_enabled():

#           self._log.info(Fore.BLUE + 'monitor loop begin…')

            if self._verbose:
                print(self._page)
    
            _now_ts = dt.timestamp(dt.now())
    
            # mast sensor trigger ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            if Event.BUMPER_MAST not in self._suppressed and self._aft_ioe.input(self._mast_pin) == 0: # mast IR
                self._mast_triggered = _now_ts
            # set bumper triggers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            if Event.BUMPER_PORT not in self._suppressed and self._fwd_ioe.input(self._bpb_pin) == 0 or self._fwd_ioe.input(self._bpt_pin) == 0:
                self._port_bmp_triggered = _now_ts
            if Event.BUMPER_STBD not in self._suppressed and self._fwd_ioe.input(self._bsb_pin) == 0 or self._fwd_ioe.input(self._bst_pin) == 0:
                self._stbd_bmp_triggered = _now_ts
            # set wheel sensor triggers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            if Event.BUMPER_SAFT not in self._suppressed and self._fwd_ioe.input(self._wsa_pin) == 0: # wheel starboard aft
                self._saft_triggered = _now_ts
            if Event.BUMPER_SFWD not in self._suppressed and self._fwd_ioe.input(self._wsf_pin) == 0: # wheel starboard fwd
                self._sfwd_triggered = _now_ts
            if Event.BUMPER_PAFT not in self._suppressed and self._fwd_ioe.input(self._wpa_pin) == 0: # wheel port aft
                self._paft_triggered = _now_ts
            if Event.BUMPER_PFWD not in self._suppressed and self._fwd_ioe.input(self._wpf_pin) == 0: # wheel port fwd
                self._pfwd_triggered = _now_ts
    
            if self._verbose:
                # bumper status ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                if self._mast_triggered > 0:
                    _mast_msg = Fore.WHITE + Style.BRIGHT + 'mast: {}'.format(self._mast_triggered)
                else:
                    _mast_msg = Fore.WHITE + Style.NORMAL + 'mast:           no'
                if self._port_bmp_triggered > 0:
                    _bp_msg = Fore.RED   + Style.BRIGHT + 'port bumper:    {}'.format(self._port_bmp_triggered)
                else:
                    _bp_msg = Fore.RED   + Style.NORMAL + 'port bumper:    no'
                if self._stbd_bmp_triggered > 0:
                    _bs_msg = Fore.GREEN + Style.BRIGHT + 'stbd bumper:    {}'.format(self._stbd_bmp_triggered)
                else:
                    _bs_msg = Fore.GREEN + Style.NORMAL + 'stbd bumper:    no'
                if self._paft_triggered > 0:
                    _wpa_msg = Fore.RED   + Style.BRIGHT + 'wheel port aft: {}'.format(self._paft_triggered)
                else:
                    _wpa_msg = Fore.RED   + Style.NORMAL + 'wheel port aft: no'
                if self._saft_triggered > 0:
                    _wsa_msg = Fore.GREEN + Style.BRIGHT + 'wheel stbd aft: {}'.format(self._saft_triggered)
                else:
                    _wsa_msg = Fore.GREEN + Style.NORMAL + 'wheel stbd aft: no'
                if self._pfwd_triggered > 0:
                    _wpf_msg = Fore.RED   + Style.BRIGHT + 'wheel port fwd: {}'.format(self._pfwd_triggered)
                else:
                    _wpf_msg = Fore.RED   + Style.NORMAL + 'wheel port fwd: no'
                if self._sfwd_triggered > 0:
                    _wsf_msg = Fore.GREEN + Style.BRIGHT + 'wheel stbd fwd: {}'.format(self._sfwd_triggered)
                else:
                    _wsf_msg = Fore.GREEN + Style.NORMAL + 'wheel stbd fwd: no'
    
                self._log.info(Fore.WHITE + '\nstatus:\n    {}\n    {}\n    {}\n    {}\n    {}\n    {}\n    {}'.format(
                        _mast_msg, _bp_msg, _bs_msg, _wsa_msg, _wsf_msg, _wpa_msg, _wpf_msg))
    
            # analog pins ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._fop_cm = int(Convert.convert_to_distance(self.get_fop_value())) # oblique fore port IR distance
            if Event.BUMPER_FOBP not in self._suppressed and self._fop_cm < self._oblique_trigger_cm: # 20cm
                self._fobp_triggered = _now_ts
            self._fos_cm = int(Convert.convert_to_distance(self.get_fos_value())) # oblique fore starboard IR distance
            if Event.BUMPER_FOBS not in self._suppressed and self._fos_cm < self._oblique_trigger_cm: # 20cm
                self._fobs_triggered = _now_ts
    
            # display ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            if self._verbose:
                _port_style = Style.NORMAL
                _stbd_style = Style.NORMAL
                if self._fobp_triggered:
                    _port_style = Style.BRIGHT
                if self._fobs_triggered:
                    _stbd_style = Style.BRIGHT
                self._log.info(Fore.WHITE + 'ir:      '
                        + Fore.RED + _port_style + '{:d}cm'.format(self._fop_cm)
                        + Fore.WHITE + Style.NORMAL + '  ::  '
                        + Fore.GREEN + _stbd_style + '{:d}cm'.format(self._fos_cm))
    
            # every now and then we consider retiring our triggers
            if self._counter > self._counter_limit:
                self.reset_triggers(_now_ts)
            self._counter += 1

            if self.any_triggered:
#               self._log.info('any triggered.')
                _sensor_data = self.get_sensor_data(self._fop_cm, self._fos_cm)
                if _sensor_data.signature != self._last_sensor_data.signature:
                    self._last_sensor_data = _sensor_data
                    _message = self.message_factory.create_message(Event.BUMPER_ANY, _sensor_data)
#                   if self._verbose:
                    self._log.info('🦋 sensor data:\n{}'.format(_sensor_data))
                    # publish message
                    self._log.info('🦋 publishing sensor data: ' + Fore.WHITE + '{}'.format(_sensor_data.signature))
    #               self._queue_publisher.put(_message) 
                    await Publisher.publish(self, _message)
                    self._log.info('🦋 published sensor data.')
    
                else:
                    if self._verbose:
                        self._log.info(Fore.WHITE + Style.DIM + 'sensor data:\n{}'.format(self._last_sensor_data))
                    pass
            else:
    #           self._log.info(Fore.WHITE + 'no triggers.')
                pass

#           self._log.info(Fore.BLUE + 'asyncio.sleep…')
            await asyncio.sleep(self._publish_delay_sec)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def external_callback_method(self):

#       self._log.info('monitoring IO Expander pins…')
        if self._verbose:
            print(self._page)

        _now_ts = dt.timestamp(dt.now())

        # set bumper triggers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._fwd_ioe.input(self._bpb_pin) == 0 or self._fwd_ioe.input(self._bpt_pin) == 0:
            self._port_bmp_triggered = _now_ts
        if self._fwd_ioe.input(self._bsb_pin) == 0 or self._fwd_ioe.input(self._bst_pin) == 0:
            self._stbd_bmp_triggered = _now_ts
        # set wheel sensor triggers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._fwd_ioe.input(self._wsa_pin) == 0: # wheel starboard aft
            self._saft_triggered = _now_ts
        if self._fwd_ioe.input(self._wsf_pin) == 0: # wheel starboard fwd
            self._sfwd_triggered = _now_ts
        if self._fwd_ioe.input(self._wpa_pin) == 0: # wheel port aft
            self._paft_triggered = _now_ts
        if self._fwd_ioe.input(self._wpf_pin) == 0: # wheel port fwd
            self._pfwd_triggered = _now_ts

        if self._verbose:
            # bumper status ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            if self._port_bmp_triggered > 0:
                _bp_msg = Fore.RED   + Style.BRIGHT + 'port bumper:    {}'.format(self._port_bmp_triggered)
            else:
                _bp_msg = Fore.RED   + Style.NORMAL + 'port bumper:    no'
            if self._stbd_bmp_triggered > 0:
                _bs_msg = Fore.GREEN + Style.BRIGHT + 'stbd bumper:    {}'.format(self._stbd_bmp_triggered)
            else:
                _bs_msg = Fore.GREEN + Style.NORMAL + 'stbd bumper:    no'
            if self._paft_triggered > 0:
                _wpa_msg = Fore.RED   + Style.BRIGHT + 'wheel port aft: {}'.format(self._paft_triggered)
            else:
                _wpa_msg = Fore.RED   + Style.NORMAL + 'wheel port aft: no'
            if self._saft_triggered > 0:
                _wsa_msg = Fore.GREEN + Style.BRIGHT + 'wheel stbd aft: {}'.format(self._saft_triggered)
            else:
                _wsa_msg = Fore.GREEN + Style.NORMAL + 'wheel stbd aft: no'
            if self._pfwd_triggered > 0:
                _wpf_msg = Fore.RED   + Style.BRIGHT + 'wheel port fwd: {}'.format(self._pfwd_triggered)
            else:
                _wpf_msg = Fore.RED   + Style.NORMAL + 'wheel port fwd: no'
            if self._sfwd_triggered > 0:
                _wsf_msg = Fore.GREEN + Style.BRIGHT + 'wheel stbd fwd: {}'.format(self._sfwd_triggered)
            else:
                _wsf_msg = Fore.GREEN + Style.NORMAL + 'wheel stbd fwd: no'

            self._log.info(Fore.WHITE + '\nstatus:\n    {}\n    {}\n    {}\n    {}\n    {}\n    {}'.format(
                    _bp_msg, _bs_msg, _wsa_msg, _wsf_msg, _wpa_msg, _wpf_msg))

        # analog pins ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._fop_cm = int(Convert.convert_to_distance(self.get_fop_value())) # oblique fore port IR distance
        if self._fop_cm < self._oblique_trigger_cm: # 20cm
            self._fobp_triggered = _now_ts
        self._fos_cm = int(Convert.convert_to_distance(self.get_fos_value())) # oblique fore starboard IR distance
        if self._fos_cm < self._oblique_trigger_cm: # 20cm
            self._fobs_triggered = _now_ts

        # display ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._verbose:
            _port_style = Style.NORMAL
            _stbd_style = Style.NORMAL
            if self._fobp_triggered:
                _port_style = Style.BRIGHT
            if self._fobs_triggered:
                _stbd_style = Style.BRIGHT
            self._log.info(Fore.WHITE + 'ir:      '
                    + Fore.RED + _port_style + '{:d}cm'.format(self._fop_cm)
                    + Fore.WHITE + Style.NORMAL + '  ::  '
                    + Fore.GREEN + _stbd_style + '{:d}cm'.format(self._fos_cm))

        # every now and then we consider retiring our triggers
        if self._counter > self._counter_limit:
            self.reset_triggers(_now_ts)
        self._counter += 1

        if self.any_triggered:
            _sensor_data = self.get_sensor_data(self._fop_cm, self._fos_cm)
            if _sensor_data.signature != self._last_sensor_data.signature:
                self._last_sensor_data = _sensor_data
                _message = self.message_factory.create_message(Event.BUMPER_ANY, _sensor_data)
                if self._verbose:
                    self._log.info('sensor data:\n{}'.format(_sensor_data))
                # publish message
                self._log.info('publishing sensor data:\n' + Fore.WHITE + '{}'.format(_sensor_data))
#               self._queue_publisher.put(_message) 
#               await Publisher.publish(self, _message)

            else:
                if self._verbose:
                    self._log.info(Fore.WHITE + Style.DIM + 'sensor data:\n{}'.format(self._last_sensor_data))
                pass
        else:
#           self._log.info(Fore.WHITE + 'no triggers.')
            pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_sensor_data(self, fop_cm, fos_cm):
        '''
        Returns a SensorData package containing an event list of all
        triggered sensors, and the values in cm for the oblique IRs,
        regardless of being triggered. Note that we return CNTR if
        both PORT and STBD bumpers are triggered.
        '''
        _sensor_data = SensorData(fop_cm, fos_cm)
        # mast IR bumper ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._mast_triggered:
            _sensor_data.add_event(Event.BUMPER_MAST)
        # mechanical fore bumpers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._port_bmp_triggered and self._stbd_bmp_triggered:
            _sensor_data.add_event(Event.BUMPER_CNTR)
        elif self._port_bmp_triggered:
            _sensor_data.add_event(Event.BUMPER_PORT)
        elif self._stbd_bmp_triggered:
            _sensor_data.add_event(Event.BUMPER_STBD)
        # wheel-based IR bumpers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._pfwd_triggered:
            _sensor_data.add_event(Event.BUMPER_PFWD)
        if self._sfwd_triggered:
            _sensor_data.add_event(Event.BUMPER_SFWD)
        if self._paft_triggered:
            _sensor_data.add_event(Event.BUMPER_PAFT)
        if self._saft_triggered:
            _sensor_data.add_event(Event.BUMPER_SAFT)
        # fore oblique IR sensors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if self._fobp_triggered:
            _sensor_data.add_event(Event.BUMPER_FOBP)
        if self._fobs_triggered:
            _sensor_data.add_event(Event.BUMPER_FOBS)
        return _sensor_data

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   def _callback_method(self, *argv):
#       self._log.info(Fore.YELLOW + Style.BRIGHT + 'triggering callback method…; interrupt: {:d}'.format(self._fwd_ioe.get_interrupt()))
#       self._callback(argv)
#       self._fwd_ioe.clear_interrupt()
#       self._log.info(Fore.YELLOW + Style.BRIGHT + 'triggered callback method; interrupt: {:d}'.format(self._fwd_ioe.get_interrupt()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_ioe(self):
        return self._fwd_ioe

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_active(self):
        return self._fwd_ioe != None

    # infrared sensors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_fop_value(self):
        '''
        Return the current (live) analog value of the fore oblique port distance sensor.
        '''
        return int(round(self._fwd_ioe.input(self._fop_pin) * 100.0))

    def get_fos_value(self):
        '''
        Return the current (live) analog value of the fore oblique starboard distance sensor.
        '''
        return int(round(self._fwd_ioe.input(self._fos_pin) * 100.0))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # raw values are unprocessed values from the IO Expander (used for testing)

    # raw infrared sensors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_raw_ofp_ir_value(self):
        return self._fwd_ioe.input(self._fop_pin)

    def get_raw_ofs_ir_value(self):
        return self._fwd_ioe.input(self._fos_pin)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        Publisher.disable(self)
        self._log.info('disabled publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        Publisher.close(self)
        self._log.info('closed publisher.')

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SensorData(object):
    SENSOR_ID = [ 'mast', 'port', 'cntr', 'stbd', 'pfwd', 'paft', 'sfwd', 'saft', 'fobp', 'fobs' ]
    '''
    A simple container for sensor data.
    '''
    def __init__(self, fop_cm, fos_cm):
        self._events = []
        self._fop_cm = fop_cm
        self._fos_cm = fos_cm
        self._signature = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_event(self, event):
        self._events.append(event)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def events(self):
        '''
        Return the list of triggered events.
        '''
        return self._events

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def fop_cm(self):
        '''
        Returns the last read value of the forward oblique port IR sensor,
        in centimeters.
        '''
        return self._fop_cm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def fos_cm(self):
        '''
        Returns the last read value of the forward oblique starboard IR
        sensor, in centimeters.
        '''
        return self._fos_cm

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def signature(self):
        '''
        Returns a signature indicating which events have been triggered,
        ignoring the distance values of the IR sensors.
        '''
        if self._signature is None:
            _sb = StringBuilder('sig', indent=0, delim='+')
            for _event in Event.by_group(Group.BUMPER):
                if _event in self._events:
                    _index = _event.num - Event.BUMPER_ANY.num - 1
                    _sb.append(SensorData.SENSOR_ID[_index])
            self._signature = _sb.to_string()
        return self._signature

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        _sb = StringBuilder('SensorData[', indent=4, delim='\n')
        _sb.append('id={}'.format(id(self)))
        _sb.append('hash={}'.format(hash(self)))
        if Event.BUMPER_MAST in self._events: _sb.append('mast=triggered')
        if Event.BUMPER_CNTR in self._events: _sb.append('cntr=triggered')
        if Event.BUMPER_PORT in self._events: _sb.append('port=triggered')
        if Event.BUMPER_STBD in self._events: _sb.append('stbd=triggered')
        if Event.BUMPER_PFWD in self._events: _sb.append('pfwd=triggered')
        if Event.BUMPER_SFWD in self._events: _sb.append('sfwd=triggered')
        if Event.BUMPER_PAFT in self._events: _sb.append('paft=triggered')
        if Event.BUMPER_SAFT in self._events: _sb.append('saft=triggered')
        if Event.BUMPER_FOBP in self._events: _sb.append('fobp={:d}cm'.format(self._fop_cm))
        if Event.BUMPER_FOBS in self._events: _sb.append('fobs={:d}cm'.format(self._fos_cm))
        _sb.append('sign={}'.format(self.signature))
        _sb.append(']', indent=2, delim=StringBuilder.NONE)
        return _sb.to_string()

# EOF

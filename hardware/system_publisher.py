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
from colorama import init, Fore, Style
init()

import ioexpander as io

from core.logger import Logger, Level
from core.event import Event, Group
from core.stringbuilder import StringBuilder
from core.util import Util
from core.publisher import Publisher
from hardware.i2c_scanner import I2CScanner
from ads1015 import ADS1015
from hardware.ina260_sensor import Ina260

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SystemPublisher(Publisher):
    CLASS_NAME = 'sys'
    _LISTENER_LOOP_NAME = '__system_publisher_loop'
    '''
    Captures system current, battery voltage and Raspberry Pi temperature
    as potential system level events.

    :param config:            the application configuration
    :param message_bus:       the asynchronous message bus
    :param message_factory:   the factory for creating messages
    :param level:             the log level
    '''
    def __init__(self, config, message_bus, message_factory, system, level=Level.INFO):
        Publisher.__init__(self, SystemPublisher.CLASS_NAME, config, message_bus, message_factory, level=level)
        if config is None:
            raise ValueError('no configuration provided.')
        if system is None:
            raise ValueError('no system sensor provided.')
        self._system = system
        _cfg = config['mros'].get('publisher').get('system')
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._publish_delay_sec = _cfg.get('publish_delay_sec')
        self._current_threshold = _cfg.get('current_threshold')
        self._battery_threshold = _cfg.get('battery_threshold')
        self._regulator_5v_threshold = _cfg.get('regulator_5v_threshold')
        self._regulator_3v3_threshold = _cfg.get('regulator_3v3_threshold')
        self._temperature_threshold = _cfg.get('temperature_threshold')
        # dire event test values ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#       self._current_threshold = 0.4  # TEST
#       self._battery_threshold = 20.0 # TEST
#       self._regulator_5v_threshold = 6.0
#       self._regulator_3v3_threshold = 4.0
#       self._temperature_threshold = 30.0 # TEST
        self._log.info('thresholds for current: {:3.2f}A; voltage: {:3.2f}V.'.format(self._current_threshold, self._battery_threshold))
        # INA260 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._ina260 = Ina260(config, level=self._level)
        # ADS1015 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._ads1015 = ADS1015()
        chip_type = self._ads1015.detect_chip_type()
        self._log.info('ADS1015 chip type: {}'.format(chip_type))
        self._ads1015.set_mode("single")
        self._ads1015.set_programmable_gain(2.048)
        if chip_type == 'ADS1015':
            self._ads1015.set_sample_rate(1600)
        else:
            self._ads1015.set_sample_rate(860)
        self._reference = self._ads1015.get_reference_voltage()
        self._log.info('reference voltage: {:6.3f}v'.format(self._reference))
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_battery(self):
        return self._ads1015.get_compensated_voltage(channel='in0/ref', reference_voltage=self._reference)
            
    def get_5v_regulator(self):
        return self._ads1015.get_compensated_voltage(channel='in1/ref', reference_voltage=self._reference)
        
    def get_3v3(self):
        return self._ads1015.get_compensated_voltage(channel='in2/ref', reference_voltage=self._reference)

   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'system-pub'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Publisher.enable(self)
        if self.enabled:
            if self._message_bus.get_task_by_name(SystemPublisher._LISTENER_LOOP_NAME):
                self._log.warning('already enabled.')
            else:
                self._log.info('creating task for system monitor loop…')
                self._message_bus.loop.create_task(self._monitor_loop(lambda: self.enabled), name=SystemPublisher._LISTENER_LOOP_NAME)
            self._log.info('enabled.')
        else:
            self._log.warning('failed to enable publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable this publisher.
        '''
        Publisher.disable(self)
        self._log.info('disabled publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _monitor_loop(self, f_is_enabled):

        self._log.info('monitoring IO Expander pins…')

        while f_is_enabled():

            self._log.debug('system monitor loop…')
            _undervoltage_ads_batt = False
            _undervoltage_ina_batt = False
            _undervoltage_5v   = False
            _undervoltage_3v3  = False
            _overcurrent       = False
   
            # read values ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            _ina_battery = self._ina260.voltage
            _ads_battery = self.get_battery()
            _current = self._ina260.current
            _power   = self._ina260.power
            _5vReg   = self.get_5v_regulator()
            _3v3Reg  = self.get_3v3()
            _temp    = self._system.read_cpu_temperature()

            # battery ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            # we get a second battery reading from the ADS1015, so why not?
            _batt_color = Fore.YELLOW
            if _ina_battery < self._battery_threshold:
                _undervoltage_ina_batt = True
                _batt_color = Fore.RED + Style.BRIGHT
            if _ads_battery < self._battery_threshold:
                _undervoltage_ads_batt = True
                _batt_color = Fore.RED + Style.BRIGHT
            if _5vReg < self._regulator_5v_threshold:
                _undervoltage_5v = True
                _batt_color = Fore.RED + Style.BRIGHT
            if _3v3Reg < self._regulator_3v3_threshold:
                _undervoltage_3v3 = True
                _batt_color = Fore.RED + Style.BRIGHT

            # current ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            _curr_color = Fore.MAGENTA
            if _current > self._current_threshold:
                _overcurrent = True
                _curr_color = Fore.RED + Style.BRIGHT
            if _current < 1.0:
                _current_display = '{:d}mA; '.format(int(_current * 1000))
            else:
                _current_display = '{:4.2f}A; '.format(_current)

            # temperature ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            _temp_color = Fore.WHITE
            if _temp > self._temperature_threshold:
                _temp_color = Fore.RED + Style.BRIGHT

            # generate event/message ┈┈┈┈┈┈┈┈┈┈┈┈┈
            # we prioritise and only publish one message
            _message = None
            if _overcurrent:
                _oc_msg = 'over-current: ' + _current_display
                self._log.warning(_oc_msg)
                _message = self.message_factory.create_message(Event.OVER_CURRENT, _oc_msg)
            elif _temp > self._temperature_threshold:
                _ht_msg = 'high temperature: {:4.2f}C'.format(_temp)
                self._log.warning(_ht_msg)
                _message = self.message_factory.create_message(Event.HIGH_TEMPERATURE, _ht_msg)
            elif _undervoltage_ina_batt and _undervoltage_ads_batt:
                _ub_msg = 'under-voltage on battery: {:4.2f}V (INA) / {:4.2f}V (ADS)'.format(_ina_battery, _ads_battery)
                self._log.warning(_ub_msg)
                _message = self.message_factory.create_message(Event.BATTERY_LOW, _ub_msg)
            elif _undervoltage_5v:
                _uv5_msg = 'under-voltage on 5V regulator: {:4.2f}V'.format(_5vReg)
                self._log.warning(_uv5_msg)
                _message = self.message_factory.create_message(Event.REGULATOR_5V_LOW, _uv5_msg)
            elif _undervoltage_3v3:
                _uv3_msg = 'under-voltage on 3V3 regulator: {:4.2f}V'.format(_3v3Reg)
                self._log.warning(_uv3_msg)
                _message = self.message_factory.create_message(Event.REGULATOR_3V3_LOW, _uv3_msg)
            # display ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._log.info('' + _batt_color + 'battery: {:4.2f}V (INA)/{:4.2f}V (ADS); '.format(_ina_battery, _ads_battery)
                    + 'regulators: {:4.2f}V (5V); {:4.2f}V (3v3); '.format(_5vReg, _3v3Reg) 
                    + Style.NORMAL + _curr_color + _current_display
                    + Style.NORMAL + Fore.GREEN + 'power: {:4.2f}W; '.format(_power)
                    + Style.NORMAL + _temp_color + 'temperature: {:4.2f}C'.format(_temp))
            # publish event ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            if _message:
                await Publisher.publish(self, _message)
            await asyncio.sleep(self._publish_delay_sec)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        Publisher.disable(self)
        self._log.info('disabled publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        Publisher.close(self)
        self._log.info('closed publisher.')

# EOF

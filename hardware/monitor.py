#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-07-04
# modified: 2024-07-07
#
# Derived in part from the sys_info_extended.py example file, part of
# Luna OLED library, Copyright (c) 2023, Richard Hull and contributors
#
# Display detailed system information in graph format, including CPU,
# memory, disk utilization, temperature, IP address, system Uptime,
# battery, regulator and Pi 3V3 voltages.
#
# Dependencies: psutil:
#
#   $ sudo apt-get install python-dev
#   $ sudo -H pip install psutil
#

import os
from pathlib import Path
from datetime import datetime
from threading import Thread
from collections import OrderedDict
import psutil
import subprocess as sp
import socket
from colorama import init, Fore, Style
init()

from luma.core.render import canvas
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1327
from luma.core.error import DeviceNotFoundError
from PIL import ImageFont
from ads1015 import ADS1015

import core.globals as globals
globals.init()

from core.rate import Rate
from core.logger import Logger, Level
from core.component import Component
from hardware.irq_clock import IrqClock

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Monitor(Component):
    '''
    Displays multiple lines of system monitoring data on an OLED display.

    This sets up its own IRQ "slow clock" for more independence and less
    potential for interference with the rest of the system.

    The Monitor is disabled by default. If the device cannot be found the
    class will log an error but not otherwise function differently, i.e.,
    the callback on the IRQ clock won't be set and update() will therefore
    not be called.
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger('monitor', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if config is None:
            raise ValueError('no configuration provided.')
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['mros'].get('hardware').get('monitor')
        self._permit_callback = _cfg.get('permit_callback') # setting True introduces message timing issues
        self._batt_max  = 21.5 # theoretical 18v LiOn battery max
        self._pi_max    = 5.25 # Pi operating range: 4.75 to 5.25V
        self._logic_max = 3.5 # 3v3 logic max
        self._network_interface_name = None
        self._bar_width       = 52
        self._bar_width_full  = 95
        self._bar_height      = 8
        self._bar_margin_top  = 3
        self._margin_x_bar    = 31
        self._margin_x_figure = 83
        self._margin_y_line   = [0, 12, 24, 36, 48, 60, 72, 84, 96, 108]
        _font_size            = _cfg.get('font_size')
        _font_size_full       = _cfg.get('font_size_full')
        _font_name            = _cfg.get('font_name')
        _font_file            = os.path.join(os.path.dirname(__file__), 'fonts/' + _font_name)
        self._font_default    = ImageFont.truetype(_font_file, _font_size)
        self._font_full       = ImageFont.truetype(_font_file, _font_size_full)
        self.__callback = None
        # get device ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._device = None
        try:
            self._device = ssd1327(i2c(port=0, address=0x3C))
        except DeviceNotFoundError:
            self._log.error('no monitor available: display not found.')
        # ADS1015 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._ads1015 = ADS1015()
        chip_type = self._ads1015.detect_chip_type()
        self._log.info('found chip type: {}'.format(chip_type))
        self._ads1015.set_mode("single")
        self._ads1015.set_programmable_gain(2.048)
        if chip_type == 'ADS1015':
            self._ads1015.set_sample_rate(1600)
        else:
            self._ads1015.set_sample_rate(860)
        self._reference = self._ads1015.get_reference_voltage()
        self._log.info('Reference voltage: {:6.3f}v'.format(self._reference))
        _use_thread = True
        if _use_thread:
            # update thread ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._update_loop_thread = Thread(name='update_loop_thread', target=Monitor._update_loop, args=[self], daemon=True)
            self._update_loop_thread.start()
            self._irq_clock = None
            _hz = 2
            self._rate = Rate(_hz, level=Level.INFO)
            self.enable()
        else:
            # IRQ Clock ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            _component_registry = globals.get('component-registry')
            self._irq_clock = _component_registry.get('irq-clock-slo')
            if self._irq_clock is None:
                CLOCK_PIN = 23
                self._irq_clock = IrqClock(config, pin=CLOCK_PIN, level=Level.INFO)
            self._irq_clock.add_callback(self.update)
        # add to global object registry ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        globals.put('monitor', self)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _update_loop(self):
        while self.enabled:
            self.update()
            self._rate.wait()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def permit_callback(self):
        return self._permit_callback

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_callback(self, callback):
        '''
        Set a callback for a substitution on the last line, which should
        be a lambda function.
        '''
        self.__callback = callback

    def get_callback_value(self):
        return self.__callback()

    # data ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_temp(self):
        temp = float(sp.getoutput("vcgencmd measure_temp").split("=")[1].split("'")[0])
        return temp

    def get_cpu(self):
        return psutil.cpu_percent()

    def get_mem(self):
        return psutil.virtual_memory().percent

    def get_disk_usage(self):
        usage = psutil.disk_usage("/")
        return usage.used / usage.total * 100

    def get_uptime(self):
        uptime = ("%s" % (datetime.now() - datetime.fromtimestamp(psutil.boot_time()))).split(".")[0]
        return "UpTime: {}".format(uptime)

    def get_timestamp(self):
        _now = datetime.now()
        return 'TS: {}'.format(_now.strftime('%Y-%m-%dT%H:%M:%S'))

    def find_single_ipv4_address(self, addrs):
        for addr in addrs:
            if addr.family == socket.AddressFamily.AF_INET:  # IPv4
                return addr.address

    def get_ipv4_address(self, interface_name=None):
        if_addrs = psutil.net_if_addrs()

        if isinstance(interface_name, str) and interface_name in if_addrs:
            addrs = if_addrs.get(interface_name)
            address = self.find_single_ipv4_address(addrs)
            return address if isinstance(address, str) else ""
        else:
            if_stats = psutil.net_if_stats()
            # remove loopback
            if_stats_filtered = {key: if_stats[key] for key, stat in if_stats.items() if "loopback" not in stat.flags}
            # sort interfaces by
            # 1. Up/Down
            # 2. Duplex mode (full: 2, half: 1, unknown: 0)
            if_names_sorted = [stat[0] for stat in sorted(if_stats_filtered.items(), key=lambda x: (x[1].isup, x[1].duplex), reverse=True)]
            if_addrs_sorted = OrderedDict((key, if_addrs[key]) for key in if_names_sorted if key in if_addrs)

            for _, addrs in if_addrs_sorted.items():
                address = self.find_single_ipv4_address(addrs)
                if isinstance(address, str):
                    return address
            return ""

    def get_ip(self, network_interface_name):
        '''
        None : find suitable IPv4 address among all network interfaces
        or specify the desired interface name as string.
        '''
        self._network_interface_name = network_interface_name
        return "IP: {}".format(self.get_ipv4_address(self._network_interface_name))

    def get_battery(self):
        return self._ads1015.get_compensated_voltage(channel='in0/ref', reference_voltage=self._reference)

    def get_5v_regulator(self):
        return self._ads1015.get_compensated_voltage(channel='in1/ref', reference_voltage=self._reference)

    def get_3v3(self):
        return self._ads1015.get_compensated_voltage(channel='in2/ref', reference_voltage=self._reference)

    # drawing ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _draw_text(self, draw, margin_x, line_num, text):
        draw.text((margin_x, self._margin_y_line[line_num]), text, font=self._font_default, fill="white")

    def _draw_bar(self, draw, line_num, percent):
        top_left_y = self._margin_y_line[line_num] + self._bar_margin_top
        draw.rectangle((self._margin_x_bar, top_left_y, self._margin_x_bar + self._bar_width, top_left_y + self._bar_height), outline="white")
        draw.rectangle((self._margin_x_bar, top_left_y, self._margin_x_bar + self._bar_width * percent / 100, top_left_y + self._bar_height), fill="white")

    def _draw_bar_full(self, draw, line_num):
        top_left_y = self._margin_y_line[line_num] + self._bar_margin_top
        draw.rectangle((self._margin_x_bar, top_left_y, self._margin_x_bar + self._bar_width_full, top_left_y + self._bar_height), fill="white")
        draw.text((65, top_left_y - 2), "100 %", font=self._font_full, fill="black")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear(self):
        if self._device:
            self._device.clear()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def update(self):
        if self._device and self.enabled:
            with canvas(self._device) as draw:
                # line 0 : temperature ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                _temp = self.get_temp()
                self._draw_text(draw, 0, 0, "Temp")
                self._draw_text(draw, self._margin_x_figure, 0, " {:5.1f}C".format(_temp))
    
                # line 1 : cpu ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                _cpu = self.get_cpu()
                self._draw_text(draw, 0, 1, "CPU")
                if _cpu < 100:
                    self._draw_text(draw, self._margin_x_figure, 1, " {:5.2f}%".format(_cpu))
                    self._draw_bar(draw, 1, _cpu)
                else:
                    self._draw_bar_full(draw, 1)
    
                # line 2 : memory ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                _mem = self.get_mem()
                self._draw_text(draw, 0, 2, "Mem")
                if _mem < 100:
                    self._draw_text(draw, self._margin_x_figure, 2, " {:5.1f}%".format(_mem))
                    self._draw_bar(draw, 2, _mem)
                else:
                    self._draw_bar_full(draw, 2)
    
                # line 3 : disk usage ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                _disk = self.get_disk_usage()
                self._draw_text(draw, 0, 3, "Disk")
                if _disk < 100:
                    self._draw_text(draw, self._margin_x_figure, 3, " {:5.1f}%".format(_disk))
                    self._draw_bar(draw, 3, _disk)
                else:
                    self._draw_bar_full(draw, 3)
    
                # line 4 : battery ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                _battery = self.get_battery()
                self._draw_text(draw, 0, 4, "Batt")
                if _battery < self._batt_max:
                    self._draw_text(draw, self._margin_x_figure, 4, " {:5.1f}v".format(_battery))
                    self._draw_bar(draw, 4, ( _battery / self._batt_max * 100.0) )
                else:
                    self._draw_bar_full(draw, 4)
    
                # line 5 : 5V regulator ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                _5v_reg = self.get_5v_regulator()
                self._draw_text(draw, 0, 5, "5vRg")
                if _5v_reg < self._pi_max:
                    self._draw_text(draw, self._margin_x_figure, 5, " {:5.2f}v".format(_5v_reg))
                    self._draw_bar(draw, 5, ( _5v_reg / self._pi_max * 100.0) )
                else:
                    self._draw_bar_full(draw, 5)
    
                # line 6 : 3V3 logic ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                _3v3_reg = self.get_3v3()
                self._draw_text(draw, 0, 6, "3v3")
                if _3v3_reg < self._logic_max:
                    self._draw_text(draw, self._margin_x_figure, 6, " {:5.2f}v".format(_3v3_reg))
                    self._draw_bar(draw, 6, ( _3v3_reg / self._logic_max * 100.0) )
                else:
                    self._draw_bar_full(draw, 6)
    
                # line 7 : IP address  ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                self._draw_text(draw, 0, 7, self.get_ip(self._network_interface_name))
    
                # line 8 : uptime ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                self._draw_text(draw, 0, 8, self.get_uptime())
    
                # line 9 : timestamp or callback ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                if self._permit_callback and self.__callback is not None:
                    self._draw_text(draw, 0, 9, self.get_callback_value())
                else:
                    self._draw_text(draw, 0, 9, self.get_timestamp())

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self.enabled:
            Component.enable(self)
            if self._device is None:
                self._log.info('monitor not enabled (no device).')
            else:
                self._log.info('enabled monitor.')
        elif self.closed:
            self._log.warning('cannot enable monitor: already closed.')
            Component.disable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        Component.disable(self)
        self.clear()
        self._log.info('disabled monitor.')

#EOF

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-04-30
# modified: 2020-05-24
#
# Sets the I2C address of an IO Expander (whose default is 0x0E) to a new address.
#
# See: https://github.com/pimoroni/ioe-python/blob/master/REFERENCE.md#function-reference
#

from colorama import init, Fore, Style
init(autoreset=True)

import ioexpander as io

_ioe = io.IOE(i2c_addr=0x18, interrupt_pin=4)

print('instantiated IO Expander.')

_i2c_addr = 0x19 # new I2C address
_ioe.set_i2c_addr(_i2c_addr)

print(Fore.GREEN + 'changed I2C address of IO Expander to 0x{:02X}.'.format(_i2c_addr) + Style.RESET_ALL)

print('complete.')

#EOF

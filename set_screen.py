#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2020-2021 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-19
# modified: 2024-05-19
#

import sys, time, traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.screen import Screen

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if len(sys.argv) != 2:
    print(Fore.RED + "\n  ERROR: expected 1 command line argument: 'on' or 'off'." + Style.RESET_ALL)
    sys.exit(1)

setting = sys.argv[1]
print(Fore.CYAN + Style.DIM + '-- setting \"{}\"...'.format(setting) + Style.RESET_ALL)

if setting == 'on' or setting == 'off':
    print(Fore.GREEN + '-- setting \"{}\"...'.format(setting) + Style.RESET_ALL)

    _config = ConfigLoader(Level.INFO).configure()
    _screen = Screen(_config, Level.INFO)
  
    if setting == 'on':
        _screen.enable()
    else:
        _screen.disable()

else:
    print(Fore.RED + "ERROR: expected 'on' or 'off'." + Style.RESET_ALL)

#EOF

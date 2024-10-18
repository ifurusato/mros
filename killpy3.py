#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-31
# modified: 2024-05-31
#
# Kills any and all 'python3' processes. Beware!
#

import os, subprocess
from subprocess import run
import sys, traceback
from colorama import init, Fore, Style
init()

try:
    _pid = os.getpid()
    _cmd = ['ps', '-A']
    proc = subprocess.Popen(_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    o, e = proc.communicate()
    _count = 0
    _proc_num = -1
    _lines = o.decode('ascii').split('\n')
    for _line in _lines:
        if 'python3' in _line:
            _proc_num = int(_line.split()[0])
            if _proc_num != _pid:
                _kill_command = 'kill -9 {}'.format(_proc_num)
                print(Fore.GREEN + '-- killing python process {}…'.format(_proc_num) + Style.RESET_ALL)
                os.system(_kill_command)
                _count += 1
    if _count == 1:
        print(Fore.GREEN + '-- killed 1 python3 process.' + Style.RESET_ALL)
    elif _count > 1:
        print(Fore.GREEN + '-- killed {} python3 processes.'.format(_count) + Style.RESET_ALL)
    else:
        print(Fore.GREEN + '-- no python3 processes found.' + Style.RESET_ALL)
except Exception as e:
    print(Fore.RED + 'error killing python3 processes: {}'.format(e) + Style.RESET_ALL)
    traceback.print_exc(file=sys.stdout)

#EOF

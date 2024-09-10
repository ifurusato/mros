#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-28
# modified: 2024-09-06
#
# Sends 0 to both channels of the TinyFX, turning all LEDs on.
#

import sys, time, traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.tinyfx_controller import TinyFxController

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    _tinyfx = None
    _log = Logger('test', Level.INFO)

    try:

        # read YAML configuration
        _level = Level.INFO
        _config = ConfigLoader(Level.INFO).configure()
        _tinyfx = TinyFxController(_config, level=Level.INFO)
        _log.info('starting test…')

        _tinyfx.channel_on(TinyFxController.DOWNLIGHT)

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting…')
    except Exception as e:
        _log.error('error in motor test: {}'.format(e))
        traceback.print_exc(file=sys.stdout)
    finally:
        if _tinyfx:
#           _tinyfx.close()
            pass

if __name__== "__main__":
    main()

#EOF

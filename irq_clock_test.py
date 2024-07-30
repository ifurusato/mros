#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-10-10
# modified: 2024-05-18
#

import itertools
from datetime import datetime as dt
import time
import statistics
from colorama import init, Fore, Style
init(autoreset=True)

from core.logger import Logger, Level
from core.dequeue import DeQueue
from core.config_loader import ConfigLoader
from hardware.irq_clock import IrqClock

_log = Logger('irq-clock-test', Level.INFO)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def callback_method():
    global _counter, _timestamp, _queue
    _elapsed_ms = (dt.now() - _timestamp).total_seconds() * 1000.0
    if _queue.full:
        _queue.poll()
    _queue.push(_elapsed_ms)
    _mean_ms = statistics.median(_queue.queue)
    _hz = 1000 * ( 1 / _mean_ms )
    _log.info('[{:04d}] callback: '.format(next(_counter))
            + Fore.YELLOW  + '{:6.3f}ms elapsed; '.format(_elapsed_ms)
            + Fore.MAGENTA + '{:6.3f}ms average; '.format(_mean_ms)
            + Fore.GREEN   + '{:6.3f}Hz average'.format(_hz))
    _timestamp = dt.now()

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():
    global _counter, _timestamp, _queue

    _irq_clock = None
    _log.info('starting test...')

    _counter   = itertools.count()
    _timestamp = dt.now()

    try:

        # read YAML configuration
        _level = Level.INFO
        _config = ConfigLoader(Level.INFO).configure() 

        _queue = DeQueue(maxsize=20, mode=DeQueue.FIFO)

        _irq_clock = IrqClock(_config, level=Level.INFO)
        _irq_clock.add_callback(callback_method)

        _log.info('starting IRQ clock...')
        _irq_clock.enable()

        while True:
            _log.info(Fore.BLACK + 'waiting for clock toggle.')
            time.sleep(5.0)

    except KeyboardInterrupt:
        _log.info(Fore.RED + "caught Ctrl-C.")
    finally:
        if _irq_clock:
            _log.info("closing external clock...")
            if _irq_clock:
                _irq_clock.close()

if __name__== "__main__":
    main()

#EOF

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. 
# Released without license, this code is in the public domain.
#
# author:   Murray Altheim
# created:  2024-05-18
# modified: 2024-05-18
#
# A handy script for setting a GPIO pin high or low.
#

import sys
try:
    import RPi.GPIO as GPIO
except Exception:
    print('This script requires the RPi.GPIO module.\nInstall with: sudo pip3 install RPi.GPIO')
    sys.exit(1)

_pin = 25

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(_pin, GPIO.OUT)

GPIO.output(_pin, GPIO.LOW)

#GPIO.cleanup(_pin)

#EOF

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-07-30
# modified: 2024-07-31
#
# Derived from the 'play_wav_from_flash_blocking.py' example found at:
#
#   https://github.com/miketeachman/micropython-i2s-examples
#
# The MIT License (MIT)
# Copyright (c) 2022 Mike Teachman
# https://opensource.org/licenses/MIT
#
# Description:
#
#   This plays a WAV file from the flash memory of an Unexpected Maker 
#   I2S Audio Shield. The constructor defaults are for a mono 16 bit
#   WAV file with a 16kHz sample rate.
#
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import uos
from machine import I2S
from machine import Pin

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SoundPlayer(object):
    PLAY   = 0
    PAUSE  = 1
    RESUME = 2
    STOP   = 3
    '''
    Accepts a filename argument, playing a WAV file to its end and then
    exiting. This provides no SD card buffering so any long audio files
    will end up getting chopped up into pieces as the file is read from
    the SD card, but for short audio clips this works fine. The default
    path is ignored: for wav_file you must provide a full path argument.

    The pin assignments are for use with the Unexpected Maker I2S Audio
    Shield. If you're using different hardware you'll have to alter the
    configuration.

    :param wav_file          the WAV file name as it appears on the SD card
    :param sample_size_bits  the WAV sample size in bits, default is 16
    :param sample_rate_hz    the WAV sample rate in Hz, default is 16k
    '''
    def __init__(self, wav_file, sample_size_bits=16, sample_rate_hz=16000):
        # get file info ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._wav_file = wav_file
        _info = uos.stat(self._wav_file)
        self._file_size = _info[6]
        # create I2S device ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _BUFFER_LENGTH_IN_BYTES = 5000
        self._audio_out = I2S(
            0,           # I2S ID
            sck=Pin(27), # BCLK
            ws=Pin(26),  # LRCLK
            sd=Pin(25),  # D IN
            mode=I2S.TX,
            bits=sample_size_bits,
            format=I2S.MONO, # device is mono
            rate=sample_rate_hz,
            ibuf=_BUFFER_LENGTH_IN_BYTES,
        )

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def play(self):
        print("-- playing file '{}' with size {} bytes…".format(self._wav_file, self._file_size))
        wav = open(self._wav_file, "rb")
        pos = wav.seek(44)  # advance to first byte of Data section in WAV file
        # allocate sample array
        wav_samples = bytearray(1000)
        # memoryview used to reduce heap allocation
        wav_samples_mv = memoryview(wav_samples)
        # read audio samples from the WAV file and write them to an I2S DAC
        try:
            _playing = True
            while _playing:
                num_read = wav.readinto(wav_samples_mv)
                # end of WAV file?
                if num_read == 0:
                    # end-of-file, advance to first byte of Data section
                    _ = wav.seek(44)
                    _playing = False
                else:
                    _ = self._audio_out.write(wav_samples_mv[:num_read])
        except (KeyboardInterrupt, Exception) as e:
            print("caught exception {} {}".format(type(e).__name__, e))
        finally:
            # cleanup ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            wav.close()
            self._audio_out.deinit()
            print("done.")

#EOF

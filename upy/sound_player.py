#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-07-30
# modified: 2024-07-30
#
# Derived from the 'play_wav_from_sdcard_non_blocking.py' example found at:
#
#   https://github.com/miketeachman/micropython-i2s-examples
#
# The MIT License (MIT)
# Copyright (c) 2022 Mike Teachman
# https://opensource.org/licenses/MIT
#
# Description:
#
#   In non-blocking fashion, this plays a WAV file from the SD card of
#   an Unexpected Maker I2S Audio Shield. The constructor defaults are
#   for a mono 16 bit WAV file with a 16kHz sample rate.
#
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import os, uos
import micropython
from machine import I2S
from machine import Pin
from machine import SDCard

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SoundPlayer(object):
    PLAY = 0
    PAUSE = 1
    RESUME = 2
    STOP = 3
    '''
    Accepts a filename argument, playing a WAV file to its end and then
    exiting. This provides no SD card buffering so any long audio files
    will end up getting chopped up into pieces as the file is read from
    the SD card, but for short audio clips this works fine.

    The pin assignments are for use with the Unexpected Maker I2S Audio
    Shield. If you're using different hardware you'll have to alter the
    configuration.

    :param wav_file          the WAV file name as it appears on the SD card
    :param sample_size_bits  the WAV sample size in bits, default is 16
    :param sample_rate_hz    the WAV sample rate in Hz, default is 16k
    '''
    def __init__(self, wav_file, sample_size_bits=16, sample_rate_hz=16000):
        # mount SD card
        self._sd = SDCard(slot=2)  # sck=18, mosi=23, miso=19, cs=5
        os.mount(self._sd, "/sd")
        # get file info
        self._wav_file = wav_file
        self._filepath = '/sd/' + wav_file
        _info = uos.stat(self._filepath)
        self._file_size = _info[6]
        self._wav = None
        # create I2S device
        self._audio_out = I2S(
            0, # I2S ID
            sck=Pin(27), # BCLK
            ws=Pin(26),  # LRCLK
            sd=Pin(25),  # D IN
            mode=I2S.TX,
            bits=sample_size_bits,
            format=I2S.MONO,
            rate=sample_rate_hz,
            ibuf=self._file_size,
        )
        # allocate a small array of blank samples
        self._silence = bytearray(1000)
        # allocate sample array buffer
        wav_samples = bytearray(10000)
        self._wav_samples_mv = memoryview(wav_samples)
        self._audio_out.irq(self._i2s_callback)
        self._state = SoundPlayer.PAUSE

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def play(self):
        print("-- playing file '{}' with size {} bytes…".format(self._filepath, self._file_size))
        _ = self._audio_out.write(self._silence)
        self._wav = open("/sd/{}".format(self._wav_file), "rb")
        _ = self._wav.seek(44)  # advance to first byte of Data section in WAV file
        self._state = SoundPlayer.PLAY

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def pause(self):
        self._state = SoundPlayer.PAUSE

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def resume(self):
        self._state = SoundPlayer.RESUME

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        self._state = SoundPlayer.STOP

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _eof_callback(self, arg):
        self._state = SoundPlayer.STOP

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _i2s_callback(self, arg):
        if self._state == SoundPlayer.PLAY:
            num_read = self._wav.readinto(self._wav_samples_mv)
            # end of WAV file?
            if num_read == 0:
                # end-of-file, advance to first byte of Data section
                pos = self._wav.seek(44)
                _ = self._audio_out.write(self._silence)
                micropython.schedule(self._eof_callback, None)
            else:
                _ = self._audio_out.write(self._wav_samples_mv[:num_read])
        elif self._state == SoundPlayer.RESUME:
            self._state = SoundPlayer.PLAY
            _ = self._audio_out.write(self._silence)
        elif self._state == SoundPlayer.PAUSE:
            _ = self._audio_out.write(self._silence)
        elif self._state == SoundPlayer.STOP:
            self._close()
        else:
            raise Exception("invalid state.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _close(self):
        '''
        Closes all resources.
        '''
        self._wav.close()
        os.umount("/sd")
        self._sd.deinit()
        self._audio_out.deinit()

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

#WAV_FILE = "008-Telemetry.wav"
#WAV_SAMPLE_SIZE_IN_BITS = 16
#SAMPLE_RATE_IN_HZ = 16000
#player = SoundPlayer(WAV_FILE, WAV_SAMPLE_SIZE_IN_BITS, SAMPLE_RATE_IN_HZ)
#player.play()

#EOF

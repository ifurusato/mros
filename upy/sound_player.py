# This is a work in progress, based on 'play_wav_from_sdcard_non_blocking.py'
# by Mike Teachman.
#
# The MIT License (MIT)
# Copyright (c) 2022 Mike Teachman
# https://opensource.org/licenses/MIT
#

import sys
import os
import time
import micropython
from machine import I2S
from machine import Pin
from machine import SDCard

# ======= I2S CONFIGURATION =======
SCK_PIN = 27 # BCLK
WS_PIN  = 26 # LRCLK
SD_PIN  = 25 # D IN
I2S_ID  = 0
BUFFER_LENGTH_IN_BYTES = 16250 # set to file length
# ======= I2S CONFIGURATION =======

# ======= AUDIO CONFIGURATION =======
#WAV_FILE = "music-16k-16bits-mono.wav"
WAV_FILE = "008-Telemetry.wav"
WAV_SAMPLE_SIZE_IN_BITS = 16
FORMAT = I2S.MONO
SAMPLE_RATE_IN_HZ = 16000
# ======= AUDIO CONFIGURATION =======


# WAV player support ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
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

    This is hard-wire configured for use with the Unexpected Maker I2S
    Audio Shield. If you're using different hardware you'll have to alter
    the pin assignments.
    '''
    def __init__(self):
        # mount SD card
        self._sd = SDCard(slot=2)  # sck=18, mosi=23, miso=19, cs=5
        os.mount(self._sd, "/sd")
        # create I2S device
        self._audio_out = I2S(
            I2S_ID,
            sck=Pin(SCK_PIN),
            ws=Pin(WS_PIN),
            sd=Pin(SD_PIN),
            mode=I2S.TX,
            bits=WAV_SAMPLE_SIZE_IN_BITS,
            format=FORMAT,
            rate=SAMPLE_RATE_IN_HZ,
            ibuf=BUFFER_LENGTH_IN_BYTES,
        )
        self._wav = None
        # allocate a small array of blank samples
        self._silence = bytearray(1000)
        # allocate sample array buffer
        wav_samples = bytearray(10000)
        self._wav_samples_mv = memoryview(wav_samples)
        self._audio_out.irq(self._i2s_callback)
        self._state = SoundPlayer.PAUSE
        print('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def play(self, wav_file):
        print("PLAY() a.")
        _ = self._audio_out.write(self._silence)
        print("PLAY() b.")
        self._wav = open("/sd/{}".format(wav_file), "rb")
        print("PLAY() c.")
        _ = self._wav.seek(44)  # advance to first byte of Data section in WAV file
        print("PLAY() d.")
        self._state = SoundPlayer.PLAY
        print("PLAY() e.")

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
        print("end of audio file")
        self._state = SoundPlayer.STOP  # uncomment to stop looping playback
    
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _i2s_callback(self, arg):
        if self._state == SoundPlayer.PLAY:
            print("PLAY.")
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
            print("RESUME.")
            self._state = SoundPlayer.PLAY
            _ = self._audio_out.write(self._silence)
        elif self._state == SoundPlayer.PAUSE:
            print("PAUSE.")
            _ = self._audio_out.write(self._silence)
        elif self._state == SoundPlayer.STOP:
            print("STOP.")
            self._close()
            print("STOPPED.")
        else:
            print("invalid state.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _close(self):
        '''
        Closes all resources.
        '''
        print("CLOSING...")
        self._wav.close()
        os.umount("/sd")
        self._sd.deinit()
        self._audio_out.deinit()
        sys.exit(0)
        print("CLOSED.")

# main ......................

player = SoundPlayer()

print("starting playback...")

player.play(WAV_FILE)

print("completed playback.")

#EOF

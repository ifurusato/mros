#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-14
# modified: 2024-09-18
#

import machine
from machine import Pin
import utime
from RP2040_Slave import i2c_slave

import itertools
from colors import*
from stringbuilder import StringBuilder

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class I2CSlave(object):
    '''
    This class supports using an RP2040 as an I2C slave device. Default
    configuration is pin 24 for SDA, pin 25 for SCL. Constructor arguments
    can be specified to suit any RP2040 board.

    The receive mode permits strings of up to 32 characters, composed of
    ASCII characters between SPACE (20) and '~' (126).

    I2C requests return an enumerated response code in the form of a
    single byte, similar to an HTTP Status Code.

    This also provides an optional callback method that calls status()
    with an RGB value and optional message. If self._visual is True the
    status is accompanied by a change to the RGB LED display. The
    process_payload() method can be overridden by subclasses to further
    process the message and perform a specific function.

    :param: i2c_id        the I2C bus identifier; default is 0
    :param: sda           the SDA pin; default is 24
    :param: scl           the SCL pin; default is 25
    :param: i2c_address   the I2C address of the device; default is 0x44
    :param: blink         if True, will periodically call status()
                          to indicate the loop is operating.
    :param: callback      the optional callback method
    '''
    # default constants:
    I2C_ID      = 0
    SDA_PIN     = 24
    SCL_PIN     = 25
    I2C_ADDRESS = 0x44
    MAX_CHARS   = 32
    LED_PIN     = 11

    # response codes: (note: extension values <= 0x20 are considered 'okay')
    INIT              = 0x10
    OKAY              = 0x20
    BAD_REQUEST       = 0x40
    BAD_ADDRESS       = 0x41
    OUT_OF_SYNC       = 0x42
    INVALID_CHAR      = 0x43
    SOURCE_TOO_LARGE  = 0x44
    UNVALIDATED       = 0x45
    EMPTY_PAYLOAD     = 0x46
    PAYLOAD_TOO_LARGE = 0x47
    UNKNOWN_ERROR     = 0x50

    def __init__(self, i2c_id=I2C_ID, sda=SDA_PIN, scl=SCL_PIN, i2c_address=I2C_ADDRESS, blink=True, callback=None):
        super().__init__()
        self._blink = blink
        self._callback = callback
        self._visual  = False
        self._verbose = False
        self._enabled = False
        self._counter = itertools.count()
        self.message("starting I2C slave…")
        self.s_i2c = i2c_slave(self.I2C_ID, sda=self.SDA_PIN, scl=self.SCL_PIN, slaveAddress=i2c_address)
        # red LED
        self._led = Pin(self.LED_PIN, Pin.OUT)
        # initial conditions
        self._index = 0
        self._payload = ''
        self._response = self.INIT
        self._currentTransaction = self.s_i2c.I2CTransaction(0x00, [])
        self._state = self.s_i2c.I2CStateMachine.I2C_START
        # indicate startup…
        if self._visual:
            for i in range(3):
                self.status(None, COLOR['CYAN'])
                utime.sleep_ms(50)
                self.status(None, COLOR['BLACK'])
                utime.sleep_ms(50)
        else:
            for i in range(3):
                self._led.value(True)
                utime.sleep_ms(50)
                self._led.value(False)
                utime.sleep_ms(50)

        utime.sleep_ms(333)
        self.message("ready.")

    def is_visual(self):
        return self._visual

    def set_visual(self, visual):
        self._visual = visual

    def message(self, msg):
        if self._verbose:
            print(msg)

    def enable(self):
        if self._enabled:
            self.message("already enabled.")
            return
        self._enabled = True
        self._loop()

    def disable(self):
        if not self._enabled:
            self.message("already disabled.")
            return
        self._enabled = False

    def _loop(self):
        self.message("starting loop…")
        while self._enabled:
            try:
                self._state = self.s_i2c.handle_event()
                if self._state == None:
                    pass
                elif self._state == self.s_i2c.I2CStateMachine.I2C_START:
                    self.status('start', COLOR['MAGENTA'])
                elif self._state == self.s_i2c.I2CStateMachine.I2C_RECEIVE:
                    self.status('rx', COLOR['SKY_BLUE'])
                    '''
                    Receive data from the master. The first byte is 0x00, followed by a byte
                    indicating the count of bytes in the payload, then the data bytes followed
                    by 0x01 to validate, then 0xff to finish.
                    '''
                    if self._currentTransaction.address == 0x00:
                        # first byte received is the register address
                        _register_address = self.s_i2c.Read_Data_Received()
                        self._currentTransaction.address = _register_address

                    _valid = False
                    self._index = 0
                    _sb = StringBuilder()
                    _expected_length = 0

                    # read all data byte received until Rx FIFO is empty
                    while self.s_i2c.Available():
                        _data_rx = self.s_i2c.Read_Data_Received()
                        _int_value = int(_data_rx)
                        if _data_rx == 0x00:
                            pass
                        elif _data_rx == 0x01:
                            _valid = True
                        elif _data_rx == 0xFF:
                            self.status('eor', COLOR['DARK_TURQUOISE'])
                            break
                        else:
                            if self._index == 0:
                                _expected_length = _int_value
                                if _expected_length > self.MAX_CHARS:
                                    self.status('error', COLOR['ORANGE'])
                                    raise I2CSlaveError(I2CSlave.SOURCE_TOO_LARGE, "WARNING: packet failed with {:d} chars, exceeded maximum length of {:d}.".format(
                                            _expected_length, self.MAX_CHARS))
                            elif _sb.length() < _expected_length:
                                if ( _int_value >= 32 ) and ( _int_value < 127 ):
                                    _sb.append(chr(_data_rx))
                                else:
                                    self.status('error', COLOR['RED'])
                                    raise I2CSlaveError(I2CSlave.INVALID_CHAR, "invalid character received: '0x{:02X}' (int: '{:d}'); buf length: {:d}; sb: '{}'".format(
                                            _data_rx, _int_value, _sb.length(), _sb.to_string()))
                            elif _sb.length() == _expected_length:
                                pass
                            else:
                                self.status('error', COLOR['FUCHSIA'])
                                raise I2CSlaveError(I2CSlave.OUT_OF_SYNC, "out of sync: '0x{:02X}' (int: '{:d}'); buf length: {:d}; expected length: {:d}".format(
                                        _data_rx, _int_value, _sb.length(), _expected_length))
                        self._index = self._index + 1
                        self._currentTransaction.data_byte.append(_data_rx)
                    # end of receive loop
                    if _sb.length() > 0:
                        if _valid:
                            if _expected_length != _sb.length():
                                self.status('error', COLOR['TANGERINE'])
                                raise I2CSlaveError(I2CSlave.PAYLOAD_TOO_LARGE, "package failed with expected length: {:d}; actual length: {:d}.".format(
                                        _expected_length, _sb.length()))
                            else:
                                self._payload = self.process_buffer(_sb)
                        else:
                            self.status('error', COLOR['BROWN'])
                            raise I2CSlaveError(I2CSlave.UNVALIDATED, "unvalidated buffer: '{}'".format(_sb.to_string()))

                    self.status('rxd', COLOR['YELLOW_GREEN'])
                    # clear for next transaction
                    self.reset()

                elif self._state == self.s_i2c.I2CStateMachine.I2C_REQUEST:
                    if len(self._payload) > 0:
                        self.status('okay', COLOR['GREEN'])
                        self._response = self.OKAY
                    else:
                        self.status('nop', COLOR['DARK_ORANGE'])
                        self._response = self.EMPTY_PAYLOAD
                    # otherwise use existing response
                    while (self.s_i2c.is_Master_Req_Read()):
                        self.write_response(self._response)
                elif self._state == self.s_i2c.I2CStateMachine.I2C_FINISH:
                    self.reset()

                if self._blink: # is alive indicator
                    if next(self._counter) % 1000 == 0:
                        if self._visual:
                            self.status(None, COLOR['DARK_CYAN'])
                            utime.sleep_ms(4)
                            self.status(None, COLOR['BLACK'])
                        else:
                            self._led.value(True)
                            utime.sleep_ms(4)
                            self._led.value(False)

            except KeyboardInterrupt:
                break
            except I2CSlaveError as se:
                _msg = 'I2C slave error 0x{:02X} on transaction: {}/{}'.format(se.code, str(se),type(se.code))
                self.status(_msg, COLOR['DARK_RED'])
                self.message(_msg)
                # empty buffer
                self.message('emptying buffer…')
                while self.s_i2c.Available():
                    _data_rx = self.s_i2c.Read_Data_Received()
                self.reset()
                while (self.s_i2c.is_Master_Req_Read()):
                    self.message("sending error response: {}".format(str(se)))
                    self.write_response(se.code)
            except Exception as e:
                self.message('Exception raised: {}'.format(e))

    def write_response(self, response):
        '''
        Writes the single byte response to the I2C bus.
        '''
        self.s_i2c.Slave_Write_Data(response)

    def status(self, message, color):
        '''
        If the callback has been provided, it is called with a message (which may
        be None) and an RGB color tuple to display the color on the NeoPixel.
        '''
        if self._callback:
            self._callback(message, color)
            if self._visual:
                self.message("message: '{}'; color: '{}'".format(message, color))
                if not self._blink:
                    # callback but no blink so we need to un-blink the LED
                    utime.sleep_ms(10)
                    self._callback(None, BLACK)

    def process_buffer(self, buffer):
        '''
        Receives the packet sent by the master, returning the contents as a string.
        This can be expanded to further process the value.
        '''
        __payload = buffer.to_string()
        return self.process_payload(__payload)

    def process_payload(self, payload):
        '''
        Override this method in a subclass to process the payload value.
        '''
        return payload

    def reset(self):
        self._index = 0
        self._payload = ''
        self._response = self.INIT
        self._currentTransaction.reset()
        self._state = self.s_i2c.I2CStateMachine.I2C_START
        self.message('reset.')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class I2CSlaveError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self._code = code

    @property
    def code(self):
        return self._code

#EOF

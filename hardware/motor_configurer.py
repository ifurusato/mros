#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2021-04-22
#

import sys, traceback
from fractions import Fraction
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.orientation import Orientation
from hardware.i2c_scanner import I2CScanner
from hardware.motor import Motor
from hardware.decoder import Decoder

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorConfigurer():
    '''
    Configures either a ThunderBorg motor controller for a pair of motors.

    :param config:          the application configuration
    :param i2c_scanner:     the I²C bus scanner
    :param motors_enabled:  an optional flag to enable motors (default false)
    :param level:           the logging level
    '''
    def __init__(self, config, i2c_scanner, motors_enabled=False, level=Level.INFO):
        self._log = Logger("motor-config", level)
        if config is None:
            raise ValueError('null configuration argument.')
        self._config = config
        if not isinstance(i2c_scanner, I2CScanner):
            raise ValueError('expected I2CScanner, not {}.'.format(type(i2c_scanner)))
        self._i2c_scanner = i2c_scanner
        self._log.debug('getting battery reading…')
        # configure from command line argument properties
        _args = self._config['mros'].get('arguments')
        self._motors_enabled = _args.get('motors_enabled') or motors_enabled
        self._log.info('motors enabled?\t{}'.format(self._motors_enabled))
        self._max_power_ratio = None
        # Import the ThunderBorg library, then configure and return the motors
        self._fwd_tb = self._import_thunderborg(Orientation.FWD)
        self._log.info('configured FWD ThunderBorg at I2C address: 0x{:02X}'.format(self._fwd_tb.I2cAddress))
        self._mid_tb  = None # self._import_thunderborg(Orientation.MID)
#       self._log.info('configured MID ThunderBorg at I2C address: 0x{:02X}'.format(self._mid_tb.I2cAddress))
        self._aft_tb  = self._import_thunderborg(Orientation.AFT)
        self._log.info('configured AFT ThunderBorg at I2C address: 0x{:02X}'.format(self._aft_tb.I2cAddress))
        if self._max_power_ratio is None: # this should have been set by the ThunderBorg code.
            raise ValueError('max_power_ratio not set.')
        self._enable_mid_motors = False
        # now import motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        try:
            self._log.info('configuring motors…')
            # PFWD "port-pwd" ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._pfwd_motor = Motor(self._config, self._fwd_tb, Orientation.PFWD, level)
            self._pfwd_motor.max_power_ratio = self._max_power_ratio
            # SFWD "starboard-fwd" ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._sfwd_motor = Motor(self._config, self._fwd_tb, Orientation.SFWD, level)
            self._sfwd_motor.max_power_ratio = self._max_power_ratio
            if self._enable_mid_motors:
                # PMID "port-mid" ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                self._pmid_motor = Motor(self._config, self._mid_tb, Orientation.PMID, level)
                self._pmid_motor.max_power_ratio = self._max_power_ratio
                # SMID "starboard-mid" ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                self._smid_motor = Motor(self._config, self._mid_tb, Orientation.SMID, level)
                self._smid_motor.max_power_ratio = self._max_power_ratio
            else:
                self._pmid_motor = None
                self._smid_motor = None
            # PAFT "port-aft" ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._paft_motor = Motor(self._config, self._aft_tb, Orientation.PAFT, level)
            self._paft_motor.max_power_ratio = self._max_power_ratio
            # SAFT "starboard-aft" ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._saft_motor = Motor(self._config, self._aft_tb, Orientation.SAFT, level)
            self._saft_motor.max_power_ratio = self._max_power_ratio

        except OSError as oe:
            self._log.error('failed to configure motors: {}'.format(oe))
            self._pfwd_motor = None
            self._sfwd_motor = None
            self._pmid_motor = None
            self._smid_motor = None
            self._paft_motor = None
            self._saft_motor = None
            raise Exception('unable to instantiate ThunderBorg [1].')

        _odo_cfg = self._config['mros'].get('motor').get('odometry')
        _enable_odometry = _odo_cfg.get('enable_odometry')
        if _enable_odometry: # motor odometry configuration ┈┈┈┈┈┈┈┈┈┈
            self._reverse_encoder_orientation = _odo_cfg.get('reverse_encoder_orientation')
            self._log.info('reverse encoder orientation: {}'.format(self._reverse_encoder_orientation))
            # in case you wire something up backwards (we need this prior to the logger)
            self._reverse_motor_orientation   = _odo_cfg.get('reverse_motor_orientation')
            self._log.info('reverse motor orientation:   {}'.format(self._reverse_motor_orientation))
            # GPIO pins configured for A and B channels for each encoder
            self._motor_encoder_sfwd_a    = _odo_cfg.get('motor_encoder_sfwd_a')
            self._log.info('motor encoder sfwd A: {:d}'.format(self._motor_encoder_sfwd_a))
            self._motor_encoder_sfwd_b    = _odo_cfg.get('motor_encoder_sfwd_b')
            self._log.info('motor encoder sfwd B: {:d}'.format(self._motor_encoder_sfwd_b))
            self._motor_encoder_pfwd_a    = _odo_cfg.get('motor_encoder_pfwd_a')
            self._log.info('motor encoder pfwd A: {:d}'.format(self._motor_encoder_pfwd_a))
            self._motor_encoder_pfwd_b    = _odo_cfg.get('motor_encoder_pfwd_b')
            self._log.info('motor encoder pfwd B: {:d}'.format(self._motor_encoder_pfwd_b))
            if self._enable_mid_motors:
                self._motor_encoder_smid_a    = _odo_cfg.get('motor_encoder_smid_a')
                self._log.info('motor encoder smid A: {:d}'.format(self._motor_encoder_smid_a))
                self._motor_encoder_smid_b    = _odo_cfg.get('motor_encoder_smid_b')
                self._log.info('motor encoder smid B: {:d}'.format(self._motor_encoder_smid_b))
                self._motor_encoder_pmid_a    = _odo_cfg.get('motor_encoder_pmid_a')
                self._log.info('motor encoder pmid A: {:d}'.format(self._motor_encoder_pmid_a))
                self._motor_encoder_pmid_b    = _odo_cfg.get('motor_encoder_pmid_b')
                self._log.info('motor encoder pmid B: {:d}'.format(self._motor_encoder_pmid_b))
            self._motor_encoder_saft_a    = _odo_cfg.get('motor_encoder_saft_a')
            self._log.info('motor encoder saft A: {:d}'.format(self._motor_encoder_saft_a))
            self._motor_encoder_saft_b    = _odo_cfg.get('motor_encoder_saft_b')
            self._log.info('motor encoder saft B: {:d}'.format(self._motor_encoder_saft_b))
            self._motor_encoder_paft_a    = _odo_cfg.get('motor_encoder_paft_a')
            self._log.info('motor encoder paft A: {:d}'.format(self._motor_encoder_paft_a))
            self._motor_encoder_paft_b    = _odo_cfg.get('motor_encoder_paft_b')
            self._log.info('motor encoder paft B: {:d}'.format(self._motor_encoder_paft_b))

            # configure motor encoders… ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._reverse_encoder_sfwd    = _odo_cfg.get('reverse_encoder_sfwd')
            self._log.info('sfwd motor encoder reversed? {}'.format(self._reverse_encoder_sfwd))
            self._reverse_encoder_pfwd    = _odo_cfg.get('reverse_encoder_pfwd')
            self._log.info('pfwd motor encoder reversed? {}'.format(self._reverse_encoder_pfwd))
            self._reverse_encoder_smid    = _odo_cfg.get('reverse_encoder_smid')
            self._log.info('smid motor encoder reversed? {}'.format(self._reverse_encoder_smid))
            self._reverse_encoder_pmid    = _odo_cfg.get('reverse_encoder_pmid')
            self._log.info('pmid motor encoder reversed? {}'.format(self._reverse_encoder_pmid))
            self._reverse_encoder_saft    = _odo_cfg.get('reverse_encoder_saft')
            self._log.info('saft motor encoder reversed? {}'.format(self._reverse_encoder_saft))
            self._reverse_encoder_paft    = _odo_cfg.get('reverse_encoder_paft')
            self._log.info('paft motor encoder reversed? {}'.format(self._reverse_encoder_paft))
            self._log.info('configuring motor encoders…')
            self._configure_encoder(self._pfwd_motor, Orientation.PFWD)
            self._configure_encoder(self._sfwd_motor, Orientation.SFWD)
            if self._enable_mid_motors:
                self._configure_encoder(self._pmid_motor, Orientation.PMID)
                self._configure_encoder(self._smid_motor, Orientation.SMID)
            self._configure_encoder(self._paft_motor, Orientation.PAFT)
            self._configure_encoder(self._saft_motor, Orientation.SAFT)

        # end odometry configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _configure_encoder(self, motor, orientation):
        '''
        Configure the encoder for the specified motor.
        '''
        if self._reverse_encoder_orientation:
            pass # unsupported: swap port for starboard
        if orientation is Orientation.SFWD:
            _reversed  = self._reverse_encoder_sfwd
            _encoder_a = self._motor_encoder_sfwd_a
            _encoder_b = self._motor_encoder_sfwd_b
        elif orientation is Orientation.PFWD:
            _reversed  = self._reverse_encoder_pfwd
            _encoder_a = self._motor_encoder_pfwd_a
            _encoder_b = self._motor_encoder_pfwd_b
        elif orientation is Orientation.SMID:
            _reversed  = self._reverse_encoder_smid
            _encoder_a = self._motor_encoder_smid_a
            _encoder_b = self._motor_encoder_smid_b
        elif orientation is Orientation.PMID:
            _reversed  = self._reverse_encoder_pmid
            _encoder_a = self._motor_encoder_pmid_a
            _encoder_b = self._motor_encoder_pmid_b
        elif orientation is Orientation.SAFT:
            _reversed  = self._reverse_encoder_saft
            _encoder_a = self._motor_encoder_saft_a
            _encoder_b = self._motor_encoder_saft_b
        elif orientation is Orientation.PAFT:
            _reversed  = self._reverse_encoder_paft
            _encoder_a = self._motor_encoder_paft_a
            _encoder_b = self._motor_encoder_paft_b
        else:
            raise ValueError("unrecognised value for orientation.")
        motor.decoder = Decoder(motor.orientation, _encoder_a, _encoder_b, motor._callback_step_count, self._log.level)
        if _reversed:
            motor.decoder.set_reversed()
        self._log.info('configured {} motor encoder on pin {} and {} (reversed? {}).'.format(motor.orientation.name, _encoder_a, _encoder_b, _reversed))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _import_thunderborg(self, orientation):
        if self._motors_enabled:
            if orientation is Orientation.FWD:
                self._log.info('configure thunderborg & motors for {} orientation…')
                _thunderborg_address = self._config['mros'].get('motor').get('thunderborg_fore_address')
            elif orientation is Orientation.MID:
                _thunderborg_address = self._config['mros'].get('motor').get('thunderborg_mid_address')
            elif orientation is Orientation.AFT:
                _thunderborg_address = self._config['mros'].get('motor').get('thunderborg_aft_address')
            else:
                raise Exception('expected FWD or AFT orientation.')
            self._log.info(Fore.MAGENTA + 'importing ThunderBorg for orientation {} at address 0x{:02X}…'.format(orientation.name, _thunderborg_address) + Style.RESET_ALL)
            try:
                if self._i2c_scanner.has_address([_thunderborg_address]):
                    self._log.info('importing ThunderBorg at address 0x{:02X}…'.format(_thunderborg_address))
                    import hardware.ThunderBorg3 as ThunderBorg
                    self._log.info('successfully imported ThunderBorg.')
                    self._log.info('instantiating thunderborg…')
                    _tb = ThunderBorg.ThunderBorg(Level.INFO)  # create a new ThunderBorg object
                    _tb.i2cAddress = _thunderborg_address
                else:
                    raise Exception('unable to instantiate ThunderBorg [2].')
                _tb.Init() # set the board up (checks the board is connected)
                self._log.info('successfully instantiated ThunderBorg.')
                self._log.info(Fore.MAGENTA + 'successfully instantiated ThunderBorg for orientation {} at address 0x{:02X}…'.format(
                        orientation.name, _thunderborg_address) + Style.RESET_ALL)
                if not _tb.foundChip:
                    boards = ThunderBorg.ScanForThunderBorg()
                    if len(boards) == 0:
                        self._log.error('No ThunderBorg found, check you are attached :)')
                    else:
                        self._log.error('No ThunderBorg at address %02X, but we did find boards:' % (_tb.i2cAddress))
                        for board in boards:
                            self._log.info('    %02X (%d)' % (board, board))
                        self._log.error('If you need to change the I²C address change the setup line so it is correct, e.g. TB.i2cAddress = 0x{}'.format(
                                boards[0]))
                    raise Exception('unable to instantiate ThunderBorg [3].')
                _tb.SetLedShowBattery(True)
                # initialise ThunderBorg ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                self._log.info('getting battery reading…')
                # get battery voltage to determine max motor power
                # could be: Makita 12V or 18V power tool battery, 12V line supply
                voltage_in = _tb.GetBatteryReading()
                self._log.info('battery reading: {}'.format(voltage_in))
                if voltage_in is None:
                    raise OSError('cannot continue: cannot read battery voltage.')
                self._log.info('voltage in: {:>5.2f}V'.format(voltage_in))
        #       voltage_in = 20.5
                # maximum motor voltage
#               _motor_voltage = 9.0
                _motor_voltage = self._config['mros'].get('motor').get('motor_voltage')
                self._log.info('voltage out: {:>5.2f}V'.format(_motor_voltage))
                if voltage_in < _motor_voltage:
                    raise OSError('cannot continue: battery voltage too low ({:>5.2f}V).'.format(voltage_in))
                # set the power limits
                if _motor_voltage > voltage_in:
                    self._max_power_ratio = 1.0
                else:
                    self._max_power_ratio = _motor_voltage / float(voltage_in)
                    self._log.info(Fore.WHITE + Style.BRIGHT + 'voltage in: {:.2f}; motor voltage: {:.2f}; max_power_ratio: {:.2f}'.format(
                            voltage_in, _motor_voltage, self._max_power_ratio))
                # convert float to ratio format
                self._log.info(Style.BRIGHT + 'battery level: {:>5.2f}V; motor voltage: {:>5.2f}V; maximum power ratio: {}'.format(voltage_in, _motor_voltage, \
                        str(Fraction(self._max_power_ratio).limit_denominator(max_denominator=20)).replace('/',':')))
                return _tb
            except OSError as e:
                    raise Exception('unable to instantiate ThunderBorg [4].')
            except Exception as e:
                    raise Exception('unable to instantiate ThunderBorg [5].')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_thunderborg_leds(self, enable):
        '''
        Turns the motor controller LEDs on or off.
        '''
        _tb_fwd = self.get_thunderborg(Orientation.FWD)
        _tb_mid = self.get_thunderborg(Orientation.MID)
        _tb_aft = self.get_thunderborg(Orientation.AFT)
        if _tb_fwd:
            _tb_fwd.SetLedShowBattery(enable)
            if not enable:
                _tb_fwd.SetLeds(0.0, 0.0, 0.0) # black
        if _tb_mid:
            _tb_mid.SetLedShowBattery(enable)
            if not enable:
                _tb_mid.SetLeds(0.0, 0.0, 0.0) # black
        if _tb_aft:
            _tb_aft.SetLedShowBattery(enable)
            if not enable:
                _tb_aft.SetLeds(0.0, 0.0, 0.0) # black

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_thunderborg(self, orientation):
        '''
        Temporary: do not use this brain.
        '''
        if orientation is Orientation.FWD:
            return self._fwd_tb
        elif orientation is Orientation.MID:
            return self._mid_tb
        elif orientation is Orientation.AFT:
            return self._aft_tb

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motor(self, orientation):
        if orientation is Orientation.PFWD:
            return self._pfwd_motor
        elif orientation is Orientation.SFWD:
            return self._sfwd_motor
        elif orientation is Orientation.PMID:
            return self._pmid_motor
        elif orientation is Orientation.SMID:
            return self._smid_motor
        elif orientation is Orientation.PAFT:
            return self._paft_motor
        elif orientation is Orientation.SAFT:
            return self._saft_motor

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self.set_thunderborg_leds(True)
        # anything else?

#EOF

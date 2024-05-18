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
from core.speed import Speed
from hardware.i2c_scanner import I2CScanner
from hardware.motor import Motor
from hardware.decoder import Decoder

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MotorConfigurer():
    '''
    Configures either a ThunderBorg motor controller for a pair of motors.

    :param config:          the application configuration
    :param message_bus:     the message bus handling event-laden messages
    :param i2c_scanner:     the I²C bus scanner
    :param motors_enabled:  an optional flag to enable motors (default false)
    :param level:           the logging level
    '''
    def __init__(self, config, message_bus, i2c_scanner, motors_enabled=False, level=Level.INFO):
        self._log = Logger("motor-config", level)
        if config is None:
            raise ValueError('null configuration argument.')
        self._config = config
        if Speed.FULL.ahead == 0.0:
            self._log.info('importing speed enum values from configuration…')
            Speed.configure(self._config)
            Speed.print_configuration(self._log)
        self._message_bus = message_bus
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
        self._fore_tb = self._import_thunderborg(Orientation.FORE)
#       self._mid_tb  = self._import_thunderborg(Orientation.MID)
        self._aft_tb  = self._import_thunderborg(Orientation.AFT)
        if self._max_power_ratio is None: # this should have been set by the ThunderBorg code.
            raise ValueError('max_power_ratio not set.')

        self._enable_mid_motors = False

        # now import motors
        try:
            self._log.info('configuring motors…')
            # PFOR "port-fore" .............................
            self._pfor_motor = Motor(self._config, self._fore_tb, self._message_bus, Orientation.PFOR, level)
            self._pfor_motor.max_power_ratio = self._max_power_ratio
            # SFOR "starboard-fore" ........................
            self._sfor_motor = Motor(self._config, self._fore_tb, self._message_bus, Orientation.SFOR, level)
            self._sfor_motor.max_power_ratio = self._max_power_ratio
            if self._enable_mid_motors:
                # PMID "port-mid" ..............................
                self._pmid_motor = Motor(self._config, self._mid_tb, self._message_bus, Orientation.PMID, level)
                self._pmid_motor.max_power_ratio = self._max_power_ratio
                # SMID "starboard-mid" .........................
                self._smid_motor = Motor(self._config, self._mid_tb, self._message_bus, Orientation.SMID, level)
                self._smid_motor.max_power_ratio = self._max_power_ratio
            else:
                self._pmid_motor = None
                self._smid_motor = None
            # PAFT "port-aft" ..............................
            self._paft_motor = Motor(self._config, self._aft_tb, self._message_bus, Orientation.PAFT, level)
            self._paft_motor.max_power_ratio = self._max_power_ratio
            # SAFT "starboard-aft" .........................
            self._saft_motor = Motor(self._config, self._aft_tb, self._message_bus, Orientation.SAFT, level)
            self._saft_motor.max_power_ratio = self._max_power_ratio

        except OSError as oe:
            self._log.error('failed to configure motors: {}'.format(oe))
            self._pfor_motor = None
            self._sfor_motor = None
            self._pmid_motor = None
            self._smid_motor = None
            self._paft_motor = None
            self._saft_motor = None
            raise Exception('unable to instantiate ThunderBorg [1].')

        _odo_cfg = self._config['mros'].get('motor').get('odometry')
        _enable_odometry = _odo_cfg.get('enable_odometry')
        if _enable_odometry: # motor odometry configuration ....................
            self._reverse_encoder_orientation = _odo_cfg.get('reverse_encoder_orientation')
            self._log.info('reverse encoder orientation: {}'.format(self._reverse_encoder_orientation))
            # in case you wire something up backwards (we need this prior to the logger)
            self._reverse_motor_orientation   = _odo_cfg.get('reverse_motor_orientation')
            self._log.info('reverse motor orientation:   {}'.format(self._reverse_motor_orientation))
            # GPIO pins configured for A and B channels for each encoder
            self._motor_encoder_sfor_a    = _odo_cfg.get('motor_encoder_sfor_a')
            self._log.info('motor encoder sfor A: {:d}'.format(self._motor_encoder_sfor_a))
            self._motor_encoder_sfor_b    = _odo_cfg.get('motor_encoder_sfor_b')
            self._log.info('motor encoder sfor B: {:d}'.format(self._motor_encoder_sfor_b))
            self._motor_encoder_pfor_a    = _odo_cfg.get('motor_encoder_pfor_a')
            self._log.info('motor encoder pfor A: {:d}'.format(self._motor_encoder_pfor_a))
            self._motor_encoder_pfor_b    = _odo_cfg.get('motor_encoder_pfor_b')
            self._log.info('motor encoder pfor B: {:d}'.format(self._motor_encoder_pfor_b))
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
            # configure motor encoders…
            self._log.info('configuring motor encoders…')
            self._configure_encoder(self._pfor_motor, Orientation.PFOR)
            self._configure_encoder(self._sfor_motor, Orientation.SFOR)
            if self._enable_mid_motors:
                self._configure_encoder(self._pmid_motor, Orientation.PMID)
                self._configure_encoder(self._smid_motor, Orientation.SMID)
            self._configure_encoder(self._paft_motor, Orientation.PAFT)
            self._configure_encoder(self._saft_motor, Orientation.SAFT)

        # end odometry configuration ...........................................
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _configure_encoder(self, motor, orientation):
        if self._reverse_encoder_orientation:
            pass # unsupported: swap port for starboard
        if orientation is Orientation.PFOR:
            _encoder_a = self._motor_encoder_pfor_a
            _encoder_b = self._motor_encoder_pfor_b
        elif orientation is Orientation.SFOR:
            _encoder_a = self._motor_encoder_sfor_a
            _encoder_b = self._motor_encoder_sfor_b
        elif orientation is Orientation.PMID:
            _encoder_a = self._motor_encoder_pmid_a
            _encoder_b = self._motor_encoder_pmid_b
        elif orientation is Orientation.SMID:
            _encoder_a = self._motor_encoder_smid_a
            _encoder_b = self._motor_encoder_smid_b
        elif orientation is Orientation.PAFT:
            _encoder_a = self._motor_encoder_paft_a
            _encoder_b = self._motor_encoder_paft_b
        elif orientation is Orientation.SAFT:
            _encoder_a = self._motor_encoder_saft_a
            _encoder_b = self._motor_encoder_saft_b
        else:
            raise ValueError("unrecognised value for orientation.")
        motor.decoder = Decoder(motor.orientation, _encoder_a, _encoder_b, motor._callback_step_count, self._log.level)
        if self._reverse_encoder_orientation:
            motor.decoder.set_reversed()
        self._log.info('configured {} motor encoder on pin {} and {}.'.format(motor.orientation.name, _encoder_a, _encoder_b))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _import_thunderborg(self, orientation):
        if self._motors_enabled:
            if orientation is Orientation.FORE:
                self._log.info('configure thunderborg & motors for {} orientation…')
                _thunderborg_address = self._config['mros'].get('motor').get('thunderborg_fore_address')
            elif orientation is Orientation.MID:
                _thunderborg_address = self._config['mros'].get('motor').get('thunderborg_mid_address')
            elif orientation is Orientation.AFT:
                _thunderborg_address = self._config['mros'].get('motor').get('thunderborg_aft_address')
            else:
                raise Exception('expected FORE or AFT orientation.')
            try:
                if self._i2c_scanner.has_address([_thunderborg_address]):
                    self._log.info('importing ThunderBorg at address 0x[:02X]…'.format(_thunderborg_address))
                    import hardware.ThunderBorg3 as ThunderBorg
                    self._log.info('successfully imported ThunderBorg.')
                    self._log.info('instantiating thunderborg…')
                    _tb = ThunderBorg.ThunderBorg(Level.INFO)  # create a new ThunderBorg object
                else:
                    raise Exception('unable to instantiate ThunderBorg [2].')
                _tb.Init()                       # set the board up (checks the board is connected)
                self._log.info('successfully instantiated ThunderBorg.')
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
                # initialise ThunderBorg ...........................
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
                voltage_out = 9.0
                self._log.info('voltage out: {:>5.2f}V'.format(voltage_out))
                if voltage_in < voltage_out:
                    raise OSError('cannot continue: battery voltage too low ({:>5.2f}V).'.format(voltage_in))
                # set the power limits
                if voltage_out > voltage_in:
                    self._max_power_ratio = 1.0
                else:
                    self._max_power_ratio = voltage_out / float(voltage_in)
                # convert float to ratio format
                self._log.info('battery level: {:>5.2f}V; motor voltage: {:>5.2f}V; maximum power ratio: {}'.format(voltage_in, voltage_out, \
                        str(Fraction(self._max_power_ratio).limit_denominator(max_denominator=20)).replace('/',':')))
                return _tb

            except OSError as e:
                    raise Exception('unable to instantiate ThunderBorg [4].')
            except Exception as e:
                    raise Exception('unable to instantiate ThunderBorg [5].')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_thunderborg(self, orientation):
        '''
        Temporary: do not use this brain.
        '''
        if orientation is Orientation.FORE:
            return self._fore_tb
        elif orientation is Orientation.MID:
            return self._mid_tb
        elif orientation is Orientation.AFT:
            return self._aft_tb

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_motor(self, orientation):
        if orientation is Orientation.PFOR:
            return self._pfor_motor 
        elif orientation is Orientation.SFOR:
            return self._sfor_motor
        elif orientation is Orientation.PMID:
            return self._pmid_motor
        elif orientation is Orientation.SMID:
            return self._smid_motor
        elif orientation is Orientation.PAFT:
            return self._paft_motor
        elif orientation is Orientation.SAFT:
            return self._saft_motor

#EOF

#!/usr/bin/env python3 # -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-02-21
# modified: 2024-02-25
#

from enum import Enum

from core.direction import Direction
from core.speed import Speed

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Group(Enum):
    #             num   name
    NONE       = (  0, "none" )
    SYSTEM     = (  1, "system" )
    MACRO      = (  2, "macro" )
    GAMEPAD    = (  3, "gamepad" )
    STOP       = (  4, "stop" )
    BUMPER     = (  5, "bumper" )
    INFRARED   = (  6, "infrared" )
    IMU        = (  7, "imu" )
    VELOCITY   = (  8, "velocity" )
    THETA      = (  9, "theta" )
    CHADBURN   = ( 10, "chadburn" )
    BEHAVIOUR  = ( 11, "behaviour" )
    CLOCK      = ( 12, "clock" )
    EXPERIMENT = ( 13, "experiment" )
    REMOTE     = ( 14, "remote" )
    OTHER      = ( 15, "other" )

    def __new__(cls, *args, **kwds):
        obj = object.__new__(cls)
        obj._value_ = args[0]
        return obj

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name):
        self._num  = num
        self._name = name

    # properties ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def num(self):
        return self._num

    @property
    def name(self):
        return self._name

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Event(Enum):
    '''
    Events are used as part of a message Payload, which includes the Event
    as a type. The Payload may as well as contain a value.

    TODO: define priority as an Enum rather than an int.

    Messages are prioritised by their Event type, where the priority operates
    in reverse-order: the smaller the number the higher the priority.
    '''
    # name                     n   name                   priority   group
    # misc events ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    NOOP                   = ( 0, "no operation",            1000,   Group.NONE )

    # system events ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    SHUTDOWN               = ( 10, "shutdown",                  1,   Group.SYSTEM )
    BATTERY_LOW            = ( 11, "battery low",               1,   Group.SYSTEM )
    REGULATOR_5V_LOW       = ( 12, "regulator 5v low",          1,   Group.SYSTEM )
    REGULATOR_3V3_LOW      = ( 13, "regulator 3.3v low",        1,   Group.SYSTEM )
    HIGH_TEMPERATURE       = ( 14, "high temperature",          1,   Group.SYSTEM )
    OVER_CURRENT           = ( 15, "over current",              1,   Group.SYSTEM )
    NO_CONNECTION          = ( 16, "no gamepad connection",     1,   Group.SYSTEM )
    DISCONNECTED           = ( 17, "gamepad disconnected",      1,   Group.SYSTEM )

    # lambda events ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    MACRO                  = ( 20, "macro script",              5,   Group.MACRO ) # with script ID as value
    LAMBDA                 = ( 21, "lambda function",           5,   Group.MACRO ) # with lambda as value

    # gamepad events ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    GAMEPAD                = ( 30, "gamepad",                  10,   Group.GAMEPAD )

    A_BUTTON               = ( 31, "a-cross",                  10,   Group.GAMEPAD)
    B_BUTTON               = ( 32, "b-circle",                 10,   Group.GAMEPAD)
    X_BUTTON               = ( 33, "x-triangle",               10,   Group.GAMEPAD)
    Y_BUTTON               = ( 34, "y-square",                 10,   Group.GAMEPAD)

    L1_BUTTON              = ( 35, "l1",                       10,   Group.GAMEPAD)
    L2_BUTTON              = ( 36, "l2",                       10,   Group.GAMEPAD) # unassigned
    R1_BUTTON              = ( 37, "r1",                       10,   Group.GAMEPAD) # unassigned
    R2_BUTTON              = ( 38, "r2",                       10,   Group.GAMEPAD)

    START_BUTTON           = ( 39, "start",                    10,   Group.GAMEPAD)
    SELECT_BUTTON          = ( 40, "select",                   10,   Group.GAMEPAD)
    HOME_BUTTON            = ( 41, "home",                     10,   Group.GAMEPAD)

    DPAD_HORIZONTAL        = ( 42, "dpad-horizontal",          10,   Group.GAMEPAD)
    DPAD_LEFT              = ( 43, "dpad-left",                10,   Group.GAMEPAD)
    DPAD_RIGHT             = ( 44, "dpad-right",               10,   Group.GAMEPAD)

    DPAD_VERTICAL          = ( 45, "dpad-vertical",            10,   Group.GAMEPAD)
    DPAD_UP                = ( 46, "dpad-up",                  10,   Group.GAMEPAD)
    DPAD_DOWN              = ( 47, "dpad-down",                10,   Group.GAMEPAD)

    L3_VERTICAL            = ( 48, "l3-vert",                  10,   Group.GAMEPAD)
    L3_HORIZONTAL          = ( 49, "l3-horz",                  10,   Group.GAMEPAD)
    R3_VERTICAL            = ( 50, "r3-vert",                  10,   Group.GAMEPAD)
    R3_HORIZONTAL          = ( 51, "r3-horz",                  10,   Group.GAMEPAD)

    # stopping and halting ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    EMERGENCY_STOP         = ( 60, "emergency-stop",            3,   Group.STOP )
    COLLISION_STOP         = ( 61, "collision-stop",            3,   Group.STOP )
    STOP                   = ( 62, "stop",                     12,   Group.STOP )
    HALT                   = ( 63, "halt",                     13,   Group.STOP )
    BRAKE                  = ( 64, "brake",                    14,   Group.STOP )
    STANDBY                = ( 65, "standby",                  15,   Group.STOP )

    # remote ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    REMOTE_A               = ( 70, "remote A",                 10,   Group.REMOTE )
    REMOTE_B               = ( 71, "remote B",                 10,   Group.REMOTE )
    REMOTE_Y               = ( 72, "remote Y",                 10,   Group.REMOTE )
    REMOTE_X               = ( 73, "remote X",                 10,   Group.REMOTE )
    REMOTE_D               = ( 74, "remote D",                 10,   Group.REMOTE )
    REMOTE_R               = ( 75, "remote R",                 10,   Group.REMOTE )
    REMOTE_U               = ( 76, "remote U",                 10,   Group.REMOTE )
    REMOTE_L               = ( 77, "remote L",                 10,   Group.REMOTE )

    # bumper ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    BUMPER_ANY             = ( 110, "any bumper",               4,   Group.BUMPER )
    BUMPER_MAST            = ( 111, "mast bumper",              4,   Group.BUMPER )
    BUMPER_PORT            = ( 112, "port bumper",              4,   Group.BUMPER )
    BUMPER_CNTR            = ( 113, "center bumper",            4,   Group.BUMPER )
    BUMPER_STBD            = ( 114, "starboard bumper",         4,   Group.BUMPER )
    BUMPER_PFWD            = ( 115, "port fwd bumper",          4,   Group.BUMPER )
    BUMPER_PAFT            = ( 116, "port aft bumper",          4,   Group.BUMPER )
    BUMPER_SFWD            = ( 117, "starboard fwd bumper",     4,   Group.BUMPER )
    BUMPER_SAFT            = ( 118, "starboard aft bumper",     4,   Group.BUMPER )
    BUMPER_FOBP            = ( 119, "fwd oblique port",        10,   Group.BUMPER )
    BUMPER_FOBS            = ( 120, "fwd oblique starboard",   10,   Group.BUMPER )

    # infrared ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    INFRARED_PORT          = ( 130, "infrared port",           51,   Group.INFRARED )
    INFRARED_CNTR          = ( 131, "infrared cntr",           50,   Group.INFRARED )
    INFRARED_STBD          = ( 132, "infrared stbd",           51,   Group.INFRARED )

    # imu ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    IMU_OVER_PITCH          = ( 140, "imu over-pitch",         70,   Group.IMU )
    IMU_OVER_ROLL           = ( 141, "imu over-roll",          70,   Group.IMU )

    # velocity directives  ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    VELOCITY               = ( 200, "velocity",               100,   Group.VELOCITY ) # with value
    PORT_VELOCITY          = ( 201, "port velocity",          100,   Group.VELOCITY ) # with value
    STBD_VELOCITY          = ( 202, "stbd velocity",          100,   Group.VELOCITY ) # with value
    DECREASE_VELOCITY      = ( 203, "decrease velocity",      100,   Group.VELOCITY ) # step change
    INCREASE_VELOCITY      = ( 204, "increase velocity",      100,   Group.VELOCITY ) # step change
    DECREASE_PORT_VELOCITY = ( 205, "decrease port velocity", 100,   Group.VELOCITY ) # step change
    INCREASE_PORT_VELOCITY = ( 206, "increase port velocity", 100,   Group.VELOCITY ) # step change
    DECREASE_STBD_VELOCITY = ( 207, "decrease stbd velocity", 100,   Group.VELOCITY ) # step change
    INCREASE_STBD_VELOCITY = ( 208, "increase stbd velocity", 100,   Group.VELOCITY ) # step change

    # theta directives  ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    THETA                  = ( 300, "theta",                  100,   Group.THETA ) # with value
    PORT_THETA             = ( 301, "port theta",             100,   Group.THETA )
    STBD_THETA             = ( 302, "stbd theta",             100,   Group.THETA )
    EVEN                   = ( 303, "even",                   100,   Group.THETA )
    INCREASE_PORT_THETA    = ( 304, "increase port theta",    100,   Group.THETA )
    DECREASE_PORT_THETA    = ( 305, "decrease port theta",    100,   Group.THETA )
    INCREASE_STBD_THETA    = ( 306, "increase stbd theta",    100,   Group.THETA )
    DECREASE_STBD_THETA    = ( 307, "decrease stbd theta",    100,   Group.THETA )
    # port turns
    TURN_AHEAD_PORT        = ( 310, "turn ahead port",        100,   Group.THETA )
    TURN_TO_PORT           = ( 311, "turn to port",           100,   Group.THETA ) # based on current avg direction
    TURN_ASTERN_PORT       = ( 312, "turn astern port",       100,   Group.THETA )
    SPIN_PORT              = ( 313, "spin port",              100,   Group.THETA )
    # starboard turns
    SPIN_STBD              = ( 320, "spin stbd",              100,   Group.THETA )
    TURN_ASTERN_STBD       = ( 321, "turn astern stbd",       100,   Group.THETA )
    TURN_TO_STBD           = ( 322, "turn to stbd",           100,   Group.THETA ) # based on current avg direction
    TURN_AHEAD_STBD        = ( 323, "turn ahead stbd",        100,   Group.THETA )

    # chadburn events ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # the num values here are fixed, and used in ./hardware/motors
    # astern
#   ASTERN                 = ( 400, "astern",                 100,   Group.CHADBURN, Direction.ASTERN ) # with value
    FULL_ASTERN            = ( 401, "full astern",            100,   Group.CHADBURN, Direction.ASTERN, Speed.FULL )
    TWO_THIRDS_ASTERN      = ( 402, "two thids astern",       100,   Group.CHADBURN, Direction.ASTERN, Speed.TWO_THIRDS )
    HALF_ASTERN            = ( 403, "half astern",            100,   Group.CHADBURN, Direction.ASTERN, Speed.HALF )
    ONE_THIRD_ASTERN       = ( 404, "one third astern",       100,   Group.CHADBURN, Direction.ASTERN, Speed.ONE_THIRD )
    SLOW_ASTERN            = ( 405, "slow astern",            100,   Group.CHADBURN, Direction.ASTERN, Speed.SLOW )
    DEAD_SLOW_ASTERN       = ( 406, "dead slow astern",       100,   Group.CHADBURN, Direction.ASTERN, Speed.DEAD_SLOW )
    # stopped
    STOPPED                = ( 409, "stop",                   100,   Group.CHADBURN, Direction.STOPPED, Speed.STOP )
    # ahead
#   AHEAD                  = ( 410, "ahead",                  100,   Group.CHADBURN, Direction.AHEAD ) # with value
    DEAD_SLOW_AHEAD        = ( 411, "dead slow ahead",        100,   Group.CHADBURN, Direction.AHEAD, Speed.DEAD_SLOW )
    SLOW_AHEAD             = ( 412, "slow ahead",             100,   Group.CHADBURN, Direction.AHEAD, Speed.SLOW )
    ONE_THIRD_AHEAD        = ( 413, "one third ahead",        100,   Group.CHADBURN, Direction.AHEAD, Speed.ONE_THIRD )
    HALF_AHEAD             = ( 414, "half ahead",             100,   Group.CHADBURN, Direction.AHEAD, Speed.HALF )
    TWO_THIRDS_AHEAD       = ( 415, "two thirds ahead",       100,   Group.CHADBURN, Direction.AHEAD, Speed.TWO_THIRDS )
    FULL_AHEAD             = ( 416, "full ahead",             100,   Group.CHADBURN, Direction.AHEAD, Speed.FULL )

    # high level behaviours ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    AVOID                  = ( 500, "avoid",                  150,   Group.BEHAVIOUR )
    MOTION_DETECT          = ( 501, "motion detect",          151,   Group.BEHAVIOUR )
    ROAM                   = ( 502, "roam",                   160,   Group.BEHAVIOUR )
    MOTH                   = ( 503, "moth",                   161,   Group.BEHAVIOUR )
    SWERVE                 = ( 504, "swerve",                 162,   Group.BEHAVIOUR )
    SNIFF                  = ( 505, "sniff",                  162,   Group.BEHAVIOUR ) # A Button
    EVENT_L2               = ( 506, "L2",                     163,   Group.BEHAVIOUR ) # L2 Button
    EVENT_R1               = ( 507, "cruise",                 164,   Group.BEHAVIOUR ) # R1 Button
    LIGHTS                 = ( 508, "lights",                 165,   Group.BEHAVIOUR ) # R2 Button
    VIDEO                  = ( 509, "video",                  175,   Group.BEHAVIOUR ) # L1 Button
    IDLE                   = ( 510, "idle",                   180,   Group.BEHAVIOUR ) # A Button

    # clock (> 700) ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    TICK                   = ( 701, "tick",                   700,   Group.CLOCK )

    # experiments (> 800) ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    EXPERIMENT_1           = ( 801, "experiment 1",           800,   Group.EXPERIMENT )
    EXPERIMENT_2           = ( 802, "experiment 2",           800,   Group.EXPERIMENT )
    EXPERIMENT_3           = ( 803, "experiment 3",           800,   Group.EXPERIMENT )
    EXPERIMENT_4           = ( 804, "experiment 4",           800,   Group.EXPERIMENT )
    EXPERIMENT_5           = ( 805, "experiment 5",           800,   Group.EXPERIMENT )
    EXPERIMENT_6           = ( 806, "experiment 6",           800,   Group.EXPERIMENT )
    EXPERIMENT_7           = ( 807, "experiment 7",           800,   Group.EXPERIMENT )

    # other events (> 900) ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    NO_ACTION              = ( 900, "no action",              998,   Group.OTHER )
    RGB                    = ( 909, "rgb",                    999,   Group.OTHER )
    ANY                    = ( 1000, "any",                  1000,   Group.OTHER )

    def __new__(cls, *args, **kwds):
        obj = object.__new__(cls)
        obj._value_ = args[0]
        return obj

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, priority, group, direction=None, speed=None):
        self._num       = num
        self._name      = name
        self._priority  = priority
        self._group     = group
        self._direction = direction
        self._speed     = speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_number(value):
        for e in Event:
            if value == e._num:
                return e
        raise NotImplementedError

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def is_motor_event(event):
        '''
        A convenience method to determine if the event is directly
        related to motor control. This includes stopping, velocity,
        theta (turning), and Chadburn (engine order telegraph) events.
        '''
        return ( event.group is Group.STOP ) \
                or ( event.group is Group.VELOCITY ) \
                or ( event.group is Group.THETA ) \
                or ( event.group is Group.CHADBURN )

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def is_system_event(event):
        return event.group is Group.SYSTEM

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def is_clock_event(event):
        return event.group is Group.CLOCK

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def is_bumper_event(event):
        return event.group is Group.BUMPER

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def is_infrared_event(event):
        return event.group is Group.INFRARED

    # properties ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def num(self):
        return self._num

    @property
    def name(self):
        return self._name

    @property
    def priority(self):
        return self._priority

    @property
    def group(self):
        return self._group

    @property
    def direction(self):
        return self._direction

    @property
    def speed(self):
        return self._speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def by_group(gid):
        '''
        Return all Events belonging to the requested Group.
        '''
        _list = []
        for _event in Event:
            if _event.group is gid:
                _list.append(_event)
        return _list

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def by_groups(gids):
        '''
        Return the accumulated Events belonging to all the requested Groups.
        '''
        _list = []
        for _gid in gids:
            _events = Event.by_group(_gid)
            _list.append(_events)
        return _list

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def compare_to_priority_of(self, event):
        '''
        Returns 1 if the Event of the argument is a higher priority
        (lower number) than this Event; a -1 if the Event of the
        argument is a lower priority (higher number) than this Event;
        and 0 if they have the same priority.
        '''
        if not isinstance(event, Event):
            raise ValueError('expected event argument, not {}'.format(type(event)))
        elif self._priority < event.priority:
            return 1
        elif self._priority > event.priority:
            return -1
        else:
            return 0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        '''
        Return the string value returned for an enum.
        '''
        return self.name

    def __lt__(self, other):
        return self.__hash__() < other.__hash__()

    def __hash__(self):
        return hash(self._num)

    def __eq__(self, other):
        return isinstance(other, Event) and self.__hash__() is other.__hash__()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_string(value):
        for e in Event:
            if value.upper() == e.name:
                return e
        raise NotImplementedError

#EOF

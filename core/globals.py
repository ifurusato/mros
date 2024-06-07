#!/usr/bin/env python3.7
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-09-03
# modified: 2021-09-03
#

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def init():
    '''
    Creates an application-global dictionary as a way to share objects across
    all modules. This is only suitable for application-level singletons.

    Typical usage:

        import core.globals as globals
        globals.init()

        globals.put('variable-name', _value)
        _value = globals.get('variable-name')

    You can call your variables anything you like, but by convention, both
    for consistency and to clearly differentiate between actual variable
    names we are using dashes as delimiters within variable names.

    Note that you should only need to call 'globals.init()' on your first
    module access or your main() method.
    '''
    global gvars
    try:
        if gvars:
            pass
    except Exception as e:
        gvars = {}

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def has(key):
    global gvars
    if gvars:
        return gvars.get(key) != None
    else:
        return False

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def put(key, value):
    global gvars
    if not gvars:
        init()
    gvars[key] = value

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def get(key):
    global gvars
    if not gvars:
        return None
    return gvars.get(key)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GlobalsError(Exception):
    '''
    This exception is thrown when any error occurs while referencing or
    otherwise processing global variables.
    '''
    pass

#EOF

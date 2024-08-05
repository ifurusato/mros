"""
Operator Interface

This module exports a set of functions corresponding to the intrinsic
operators of Python.  For example, operator.add(x, y) is equivalent
to the expression x+y.  The function names are those used for special
methods; variants without leading and trailing '__' are also provided
for convenience.

This is a subset of the CPython implementation of the module.
"""

def attrgetter(attr):
    assert "." not in attr

    def _attrgetter(obj):
        return getattr(obj, attr)
    return _attrgetter

# Comparison Operations *******************************************************#

def lt(a, b):
    "Same as a < b."
    return a < b

def le(a, b):
    "Same as a <= b."
    return a <= b

def eq(a, b):
    "Same as a == b."
    return a == b

def ne(a, b):
    "Same as a != b."
    return a != b

def ge(a, b):
    "Same as a >= b."
    return a >= b

def gt(a, b):
    "Same as a > b."
    return a > b

# Mathematical/Bitwise Operations *********************************************#

def mod(a, b):
    "Same as a % b."
    return a % b

def mul(a, b):
    "Same as a * b."
    return a * b

def matmul(a, b):
    "Same as a @ b."
    return a @ b

def truediv(a, b):
    "Same as a / b."
    return a / b

def ifloordiv(a, b):
    "Same as a //= b."
    a //= b
    return a


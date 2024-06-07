# tests the ability to instantiate (fake) enums

from core.orientation import Orientation
from core.speed import Speed
from core.direction import Direction
from hardware.slew_rate import SlewRate
from hardware.steering_mode import SteeringMode

_orientation = Orientation.PORT
_half = Speed.HALF
_astern = Direction.ASTERN
_slew_rate = SlewRate.SLOW
_afrs = SteeringMode.AFRS


# tests the ability to instantiate (fake) enums

from core.orientation import Orientation
from core.speed import Speed
from core.direction import Direction
from core.steering_mode import SteeringMode
from hardware.slew_rate import SlewRate

_orientation = Orientation.PORT
_half = Speed.HALF
_astern = Direction.ASTERN
_slew_rate = SlewRate.SLOW
_afrs = SteeringMode.AFRS


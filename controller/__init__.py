"""Hexapod Controller Package"""

from .model.controller import HexapodController
from .model.controller import State as HexapodState

from .model.gaits import *
from .hardware.HDLC import *
from .hardware.kernel import *
from .utils.calibration import *

# What gets imported with "from controller import *"
__all__ = ['HexapodController', 'HexapodState']

__version__ = '0.1.1'
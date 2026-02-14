"""Hexapod Controller Package"""

from .model.controller import HexapodController
from .model.gaits import *
from .hardware.kernel import *
from .hardware.HDLC import *
from .utils.calibration import *

# What gets imported with "from controller import *"
__all__ = ['HexapodController']

__version__ = '0.1.0'
"""Hexapod Controller Package"""

from .model.controller import HexapodController
from .model.controller import State as HexapodState
from .model.interface import Interface as HexapodInterface
from .hardware.kernel import Kernel as HexapodKernel

from .model.gaits import *
from .hardware.HDLC import *

# What gets imported with "from controller import *"
__all__ = [
    'HexapodController',
    'HexapodState',
    'HexapodKernel',
    'HexapodInterface'
]

__version__ = '0.1.4'
"""Pi-side controller for the Hexapod firmware.

Mirrors the wire protocol in Hexapod-Firmware/src/protocol.hpp and streams
high-level setpoints to the board, which runs the whole control loop.
"""
from .client import ConfigError, HexapodClient, HexapodError, Velocity, connect
from .framing import FrameParser, crc16, encode_frame
from .protocol import (
    BodyPose,
    GaitId,
    LedMode,
    Opcode,
    State,
    Status,
    Telemetry,
)
from .transport import SerialTransport, Transport

__all__ = [
    "HexapodClient",
    "HexapodError",
    "ConfigError",
    "connect",
    "Transport",
    "SerialTransport",
    "FrameParser",
    "encode_frame",
    "crc16",
    "Opcode",
    "State",
    "GaitId",
    "LedMode",
    "Status",
    "Telemetry",
    "BodyPose",
    "Velocity",
]

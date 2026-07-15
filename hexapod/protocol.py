"""
Wire protocol for the Hexapod firmware.

Keep the two in lockstep: every payload here uses the exact `struct` format
that is annotated as `// py:` next to the C++ struct. Enum values travel on the
wire, so they must match too.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

# Transport constants (config.hpp)
SOF = 0xAA
MAX_PAYLOAD = 128


class State(IntEnum):
    SETUP = 0     # RISING: stand-up animation running
    IDLE = 1      # standing, zero velocity
    WALK = 2      # executing a gait
    SHUTDOWN = 3  # LOWERING: sit-down animation running
    FAULT = 4     # emergency-stopped, de-energized
    OFF = 5       # de-energized standby: booted or fully sat down, awaiting Enable


class GaitId(IntEnum):
    TRIPOD = 0
    WAVE = 1
    RIPPLE = 2


class LedMode(IntEnum):
    SOLID = 0
    BLINK = 1


class Status(IntEnum):
    REJECTED = 0x00
    OK = 0x01
    UNREACHABLE = 0x02
    BAD_OPCODE = 0x03
    BAD_LENGTH = 0x04


class Opcode(IntEnum):
    # low-level debug (request/reply, ack)
    JOG_SERVO = 0x01
    # provisioning / runtime config (request/reply, ack)
    PROVISION_BODY = 0x10
    PROVISION_MOUNTS = 0x11
    PROVISION_DIRECTION = 0x12
    PROVISION_TRIM = 0x13
    PROVISION_RANGES = 0x14
    PROVISION_GAITS = 0x15
    PROVISION_LIMITS = 0x16
    PROVISION_BODY_POSE = 0x17
    PROVISION_SERVO_CAL = 0x18
    PROVISION_PINS = 0x19
    # fire-and-forget (no reply; pets the watchdog)
    SET_VELOCITY = 0x30
    SET_BODY_POSE = 0x31
    SET_GAIT = 0x32
    ENABLE = 0x33
    SHUTDOWN = 0x34
    STOP = 0x35
    SET_LED = 0x36
    HEARTBEAT = 0x37
    # request/reply (board answers with the same opcode)
    GET_TELEMETRY = 0x40
    GET_VOLTAGE = 0x41
    GET_CURRENT = 0x42
    GET_JOINTS = 0x43
    GET_BODY_POSE = 0x44
    # board-initiated
    ERROR = 0xEE


_FIRE_AND_FORGET = frozenset({
    Opcode.SET_VELOCITY, Opcode.SET_BODY_POSE, Opcode.SET_GAIT, Opcode.ENABLE,
    Opcode.SHUTDOWN, Opcode.STOP, Opcode.SET_LED, Opcode.HEARTBEAT,
})


def is_fire_and_forget(opcode: int) -> bool:
    return Opcode(opcode) in _FIRE_AND_FORGET


# Payload formats (little-endian; mirror the `// py:` annotations)
FMT_SET_VELOCITY = "<fff"       # vx (mm/s), vy (mm/s), wz (deg/s)
FMT_SET_BODY_POSE = "<ffffff"   # x, y, z (mm); roll, pitch, yaw (deg)
FMT_SET_GAIT = "<B"             # gait_id
FMT_SET_LED = "<BBBBf"          # mode, r, g, b, freq_hz
FMT_TELEMETRY = "<Bfffff"       # state, odom_x, odom_y, odom_yaw(deg), voltage, current
FMT_VOLTAGE = "<f"
FMT_CURRENT = "<f"
FMT_JOINTS = "<18f"             # servo-space deg, leg-major (leg*3 + joint)
FMT_BODY_POSE = "<ffffff"       # x, y, z (mm, z rel. standing); roll, pitch, yaw (deg)
FMT_ERROR = "<B"                # status
FMT_ACK = "<B"                  # status (OK on success) — jog / provisioning reply

# Jog / provisioning (see protocol.hpp). Arrays are leg-major / joint-major.
FMT_JOG_SERVO = "<BH"           # channel, pulse_us (0 = release)
FMT_PROVISION_BODY = "<ffffff"  # coxa,femur,tibia len; standing_h; stance_r; cycle_t
FMT_PROVISION_MOUNTS = "<18f"   # per leg: x, y, yaw_deg
FMT_PROVISION_DIRECTION = "<18f"  # per servo (leg-major): direction (+/-1)
FMT_PROVISION_TRIM = "<18f"     # per servo (leg-major): trim_deg
FMT_PROVISION_RANGES = "<6f"    # per joint type: min_deg, max_deg
FMT_PROVISION_GAITS = "<12f"    # per gait: duty, step_height, max_stride, overlap
FMT_PROVISION_LIMITS = "<ffffffff"  # lin/ang vel max, smooth tau, body lin/ang, joint vel, current_max, voltage_min
FMT_PROVISION_BODY_POSE = "<12f"  # per axis min,max: x, y, z (mm); roll, pitch, yaw (deg)
FMT_PROVISION_SERVO_CAL = "<54H"  # per servo (leg-major): min_us, mid_us, max_us
FMT_PROVISION_PINS = "<18B"     # physical pin per logical channel


@dataclass
class Telemetry:
    state: State
    odom_x: float    # mm
    odom_y: float    # mm
    odom_yaw: float  # deg
    voltage: float   # V
    current: float   # A


def unpack_telemetry(payload: bytes) -> Telemetry:
    st, ox, oy, oyaw, v, c = struct.unpack(FMT_TELEMETRY, payload)
    return Telemetry(State(st), ox, oy, oyaw, v, c)


@dataclass
class BodyPose:
    x: float      # mm, body shift +x (forward)
    y: float      # mm, body shift +y (left)
    z: float      # mm, height offset relative to standing height
    roll: float   # deg
    pitch: float  # deg
    yaw: float    # deg

    @property
    def height(self) -> float:
        """Body-height offset from standing (mm). Same reference 
        as set_body_pose's z (0 == standing height)."""
        return self.z


def unpack_body_pose(payload: bytes) -> BodyPose:
    return BodyPose(*struct.unpack(FMT_BODY_POSE, payload))

"""
Pi-side client for the on-board controller.

The board runs the whole control loop (gait, IK, FSM); the Pi streams high-level
setpoints, reads telemetry, and provisions per-robot config at connect. Three
message kinds mirror the firmware:
  fire-and-forget: setpoints, lifecycle, LED, heartbeat, no reply. Each one pets
    the board's command watchdog.
  request/reply: telemetry queries; the board answers with one frame and we
    block for it (up to `timeout`).
  ack'd: jog and provisioning; the board replies with a single Status byte.

Typical use:

    from hexapod.client import connect
    bot = connect()          # opens the link and provisions from config.yml
    bot.enable()

Canonical orders are fixed by the firmware and must not be reordered.
"""

from __future__ import annotations

import struct
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Union

import yaml

from . import protocol as P
from .framing import FrameParser, encode_frame
from .protocol import BodyPose, GaitId, LedMode, Opcode, Telemetry
from .transport import Transport

# cfg::Leg / cfg::Joint / proto::GaitId order on the wire (leg-major flattening)
LEGS = (
    "front_right",
    "middle_right",
    "rear_right",
    "rear_left",
    "middle_left",
    "front_left",
)
JOINTS = ("coxa", "femur", "tibia")
GAITS = ("tripod", "wave", "ripple")

# config.yml ships inside the package (hexapod/config/config.yml) so it is present
# whether the package is run from a checkout or pip-installed
DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "config.yml"

ConfigLike = Union[Mapping, str, Path, None]


@dataclass
class Velocity:
    vx: float  # mm/s, body +x (forward)
    vy: float  # mm/s, body +y (left)
    wz: float  # deg/s, yaw (+ CCW)


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class HexapodError(RuntimeError):
    """The board replied with an Error frame."""


class ConfigError(ValueError):
    """The config does not match the structure provision() expects. The message
    names the offending path so a hand-edited config.yml is easy to fix."""


class HexapodClient:

    def __init__(self, transport: Transport, timeout: float = 0.2) -> None:
        self._t = transport
        self._timeout = timeout
        self._parser = FrameParser()
        # Bytes pulled from the transport but not yet parsed. Reading a port is
        # destructive, so anything a request over-reads has to live here until
        # the next one consumes it, or the frame behind it is lost
        self._rx = bytearray()
        # Host-side buffers for open-loop state the board adopts as commanded, so
        # get_velocity()/get_gait() need no round-trip. The velocity clamps are
        # learned at provision() so the buffer mirrors the board's own clamp.
        self._vx = self._vy = self._wz = 0.0
        self._gait = GaitId.TRIPOD  # firmware default (gait::GaitId TRIPOD)
        self._lin_vel_max: Optional[float] = None
        self._ang_vel_max: Optional[float] = None

    def close(self) -> None:
        self._t.close()

    # Provisioning

    def provision(self, config: ConfigLike = None) -> None:
        """Validate a whole config and push every board section, in wire order.

        `config` may be a mapping already parsed, a path (str/Path) to a YAML
        file, or None for the packaged default. The config is validated in full
        first: any missing key or wrong shape raises ConfigError naming the
        path, and nothing is sent, so the board is never left half provisioned.
        `pins` and `pulses` are optional overrides; if a section is absent the
        board keeps its baked default, if present it is validated and sent.
        Sections apply only while the board is de-energized and it acks each.
        """
        if isinstance(config, Mapping):
            cfg = config
        else:
            path = Path(config) if config is not None else DEFAULT_CONFIG
            with open(path) as f:
                cfg = yaml.safe_load(f)
        if not isinstance(cfg, Mapping):
            raise ConfigError(f"config root must be a mapping, got {type(cfg).__name__}")

        # Accessors that raise ConfigError with the dotted path of any mismatch
        def node(dotted: str):
            cur = cfg
            for key in dotted.split("."):
                if not isinstance(cur, Mapping) or key not in cur:
                    raise ConfigError(f"missing '{dotted}'")
                cur = cur[key]
            return cur

        def numbers(value, n: int, where: str) -> List[float]:
            if not isinstance(value, (list, tuple)) or len(value) != n:
                raise ConfigError(f"'{where}' must be {n} numbers, got {value!r}")
            out = []
            for i, x in enumerate(value):
                if isinstance(x, bool) or not isinstance(x, (int, float)):
                    raise ConfigError(f"'{where}[{i}]' must be a number, got {x!r}")
                out.append(float(x))
            return out

        def number(dotted: str) -> float:
            x = node(dotted)
            if isinstance(x, bool) or not isinstance(x, (int, float)):
                raise ConfigError(f"'{dotted}' must be a number, got {x!r}")
            return float(x)

        def vec(dotted: str, n: int) -> List[float]:
            return numbers(node(dotted), n, dotted)

        def per_leg(dotted: str, n: int) -> List[float]:
            return [v for leg in LEGS for v in vec(f"{dotted}.{leg}", n)]

        # Build and validate every section before a single byte goes out
        body = [
            number("kinematics.legs.coxa"),
            number("kinematics.legs.femur"),
            number("kinematics.legs.tibia"),
            number("kinematics.standing_height"),
            number("kinematics.stance_radius"),
            number("gaits.cycle_time"),
        ]

        mounts: List[float] = []
        for leg in LEGS:
            pos = vec(f"kinematics.mounts.{leg}.position", 3)
            ori = vec(f"kinematics.mounts.{leg}.orientation", 3)
            mounts += [pos[0], pos[1], ori[2]]  # planar robot, only x, y, yaw ship

        direction = per_leg("hardware.direction", 3)
        trim = per_leg("hardware.trim", 3)

        ranges = [v for j in JOINTS for v in vec(f"safety.{j}_range", 2)]

        gaits: List[float] = []
        for name in GAITS:
            gaits += [
                number(f"gaits.{name}.duty_factor"),
                number(f"gaits.{name}.step_height"),
                number(f"gaits.{name}.max_stride"),
                number(f"gaits.{name}.overlap"),
            ]

        limits = [
            number("safety.lin_vel_max"),
            number("safety.ang_vel_max"),
            number("safety.vel_smooth_tau"),
            number("safety.body_lin_vel_max"),
            number("safety.body_ang_vel_max"),
            number("safety.joint_vel_max"),
            number("safety.current_max"),
            number("safety.voltage_min"),
        ]
        # Cache the velocity clamps so set_velocity can mirror the board's clamp
        # when buffering the commanded value (see get_velocity)
        self._lin_vel_max = limits[0]
        self._ang_vel_max = limits[1]

        body_pose = [
            v
            for axis in ("x", "y", "z", "roll", "pitch", "yaw")
            for v in vec(f"safety.{axis}_range", 2)
        ]

        hw = node("hardware")
        pins = None
        if "pins" in hw:
            pins = [int(v) for leg in LEGS for v in vec(f"hardware.pins.{leg}", 3)]
        pulses = None
        if "pulses" in hw:
            pulses = []
            for leg in LEGS:
                triples = node(f"hardware.pulses.{leg}")
                if not isinstance(triples, (list, tuple)) or len(triples) != len(JOINTS):
                    raise ConfigError(
                        f"'hardware.pulses.{leg}' must be {len(JOINTS)} triples, "
                        f"got {triples!r}"
                    )
                for j, triple in enumerate(triples):
                    pulses += [
                        int(x) for x in numbers(triple, 3, f"hardware.pulses.{leg}[{j}]")
                    ]

        # Everything validated, now push it. Pins go before calibration so the
        # HAL's logical to physical map is set when the pulses are applied
        self._ack(Opcode.PROVISION_BODY, struct.pack(P.FMT_PROVISION_BODY, *body))
        self._ack(Opcode.PROVISION_MOUNTS, struct.pack(P.FMT_PROVISION_MOUNTS, *mounts))
        self._ack(Opcode.PROVISION_DIRECTION, struct.pack(P.FMT_PROVISION_DIRECTION, *direction))
        self._ack(Opcode.PROVISION_TRIM, struct.pack(P.FMT_PROVISION_TRIM, *trim))
        self._ack(Opcode.PROVISION_RANGES, struct.pack(P.FMT_PROVISION_RANGES, *ranges))
        self._ack(Opcode.PROVISION_GAITS, struct.pack(P.FMT_PROVISION_GAITS, *gaits))
        self._ack(Opcode.PROVISION_LIMITS, struct.pack(P.FMT_PROVISION_LIMITS, *limits))
        self._ack(Opcode.PROVISION_BODY_POSE, struct.pack(P.FMT_PROVISION_BODY_POSE, *body_pose))
        if pins is not None:
            self._ack(Opcode.PROVISION_PINS, struct.pack(P.FMT_PROVISION_PINS, *pins))
        if pulses is not None:
            self._ack(Opcode.PROVISION_SERVO_CAL, struct.pack(P.FMT_PROVISION_SERVO_CAL, *pulses))

    # Low-level primitives, also used by the offline calibration tool

    def set_pins(self, pins) -> None:
        """Set the 18 physical outputs directly from a leg-major list, without a
        full provision. Used by calibrate.py before jogging"""
        self._ack(Opcode.PROVISION_PINS,
                  struct.pack(P.FMT_PROVISION_PINS, *[int(p) for p in pins]))

    def jog_servo(self, channel: int, pulse_us: int) -> None:
        """Drive one servo (logical channel) to a raw pulse in us; 0 releases it.
        Used by the offline calibration tool. Works only in the OFF state"""
        self._ack(Opcode.JOG_SERVO,
                  struct.pack(P.FMT_JOG_SERVO, channel, pulse_us))

    # Fire-and-forget

    def enable(self) -> None:
        self._vx = self._vy = self._wz = 0.0  # board resets motion on enable
        self._send(Opcode.ENABLE)

    def shutdown(self) -> None:
        self._vx = self._vy = self._wz = 0.0  # board zeros velocity on sit-down
        self._send(Opcode.SHUTDOWN)

    def stop(self) -> None:
        self._vx = self._vy = self._wz = 0.0
        self._send(Opcode.STOP)

    def heartbeat(self) -> None:
        self._send(Opcode.HEARTBEAT)

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        """vx, vy in mm/s (body frame, +x fwd / +y left); wz in deg/s (+ CCW).
        Clamped to the provisioned limits and buffered so get_velocity() reads it
        back without a round-trip (open-loop: the command is the setpoint)."""
        if self._lin_vel_max is not None:
            vx = _clamp(vx, -self._lin_vel_max, self._lin_vel_max)
            vy = _clamp(vy, -self._lin_vel_max, self._lin_vel_max)
        if self._ang_vel_max is not None:
            wz = _clamp(wz, -self._ang_vel_max, self._ang_vel_max)
        self._vx, self._vy, self._wz = vx, vy, wz
        self._send(Opcode.SET_VELOCITY, struct.pack(P.FMT_SET_VELOCITY, vx, vy, wz))

    def set_body_pose(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        """Body-pose offsets from standing: x/y/z mm (z relative to standing
        height), roll/pitch/yaw deg"""
        self._send(
            Opcode.SET_BODY_POSE,
            struct.pack(P.FMT_SET_BODY_POSE, x, y, z, roll, pitch, yaw),
        )

    def set_gait(self, gait: GaitId) -> None:
        self._gait = GaitId(gait)  # buffered; board adopts it as commanded
        self._send(Opcode.SET_GAIT, struct.pack(P.FMT_SET_GAIT, int(gait)))

    def set_led(
        self, mode: LedMode, r: int, g: int, b: int, freq_hz: float = 0.0
    ) -> None:
        self._send(
            Opcode.SET_LED, struct.pack(P.FMT_SET_LED, int(mode), r, g, b, freq_hz)
        )

    # Request/reply

    def get_telemetry(self) -> Telemetry:
        return P.unpack_telemetry(self._request(Opcode.GET_TELEMETRY))

    def get_voltage(self) -> float:
        return struct.unpack(P.FMT_VOLTAGE, self._request(Opcode.GET_VOLTAGE))[0]

    def get_current(self) -> float:
        return struct.unpack(P.FMT_CURRENT, self._request(Opcode.GET_CURRENT))[0]

    def get_joints(self) -> List[float]:
        return list(struct.unpack(P.FMT_JOINTS, self._request(Opcode.GET_JOINTS)))

    def get_body_pose(self) -> BodyPose:
        """Query the board's LIVE body pose: the interpolated value as it slews
        toward the last set_body_pose target, not the target itself. Same units
        and reference as set_body_pose (x/y/z mm, z relative to standing height;
        roll/pitch/yaw deg). Read .height for the z offset."""
        return P.unpack_body_pose(self._request(Opcode.GET_BODY_POSE))

    # Host-buffered state (no round-trip). The board obeys these commands
    # instantly and open-loop, so the last command IS the current setpoint.

    def get_velocity(self) -> Velocity:
        """Last commanded velocity (mm/s, mm/s, deg/s), clamped to the provisioned
        limits. Does NOT reflect an autonomous board-side stop (comms watchdog or
        a fault) — read get_telemetry().state if you need to confirm the board is
        still energized and walking."""
        return Velocity(self._vx, self._vy, self._wz)

    def get_gait(self) -> GaitId:
        """Last commanded gait (buffered on the host)."""
        return self._gait

    # Internals

    def _ack(self, opcode: Opcode, payload: bytes = b"") -> None:
        status = self._request(opcode, payload)  # raises HexapodError on Error
        code = status[0] if status else int(P.Status.REJECTED)
        if code != int(P.Status.OK):
            raise HexapodError(f"{Opcode(opcode).name} rejected: {P.Status(code).name}")

    def _send(self, opcode: Opcode, payload: bytes = b"") -> None:
        self._t.write(encode_frame(int(opcode), payload))

    def _request(self, opcode: Opcode, payload: bytes = b"") -> bytes:
        self._t.write(encode_frame(int(opcode), payload))
        deadline = time.monotonic() + self._timeout
        want = int(opcode)

        while True:
            # Parse what is already buffered before going back to the port, so a
            # frame that shared a read with an earlier reply still gets seen
            i = 0
            try:
                while i < len(self._rx):
                    b = self._rx[i]
                    i += 1
                    if not self._parser.feed(b):
                        continue
                    op, data = self._parser.opcode, self._parser.payload
                    if op == int(Opcode.ERROR):
                        status = P.Status(data[0]) if data else P.Status.REJECTED
                        raise HexapodError(f"board error: {status.name}")
                    if op == want:
                        return data
                    # A frame we did not ask for, usually a late reply to an
                    # earlier query: drop it and keep parsing
            finally:
                # Runs on every exit, so the bytes behind a returned or failed
                # frame are kept rather than dropped with the stack frame
                del self._rx[:i]

            if time.monotonic() >= deadline:
                raise TimeoutError(f"no reply to {Opcode(opcode).name}")
            # 256 covers two maximum-size frames, so a burst arrives in one call
            self._rx += self._t.read(256)


def connect(
    config: ConfigLike = None, port: Optional[str] = None, baud: int = 921600,
    provision: bool = True, timeout: float = 0.2
) -> HexapodClient:
    """Open the serial link and provision the board from the config.

    config is a mapping, a path to config.yml, or None for the packaged
    default; port overrides serial.port. `serial` is host-only and never sent.
    timeout is the per-reply wait (s): a lost reply costs this much before the
    query gives up, so keep it small relative to your polling period.
    """
    from .transport import SerialTransport

    if isinstance(config, Mapping):
        cfg = config
    else:
        path = Path(config) if config is not None else DEFAULT_CONFIG
        with open(path) as f:
            cfg = yaml.safe_load(f)
    serial = cfg.get("serial", {}) if isinstance(cfg, Mapping) else {}
    port = port or serial.get("port")
    baud = baud or int(serial.get("baud", 921600))
    client = HexapodClient(SerialTransport(port, baud, timeout=timeout), timeout=timeout)
    if provision:
        client.provision(cfg)
    return client

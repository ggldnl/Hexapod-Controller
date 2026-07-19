"""Client tests against a fake firmware that speaks the real wire protocol."""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hexapod import (  # noqa: E402
    GaitId, HexapodClient, HexapodError, LedMode, Opcode, State, Status,
    Velocity,
)
from hexapod.framing import FrameParser, encode_frame  # noqa: E402
from hexapod.transport import Transport  # noqa: E402


class FakeFirmware(Transport):
    """Decodes what the client writes; answers requests with canned replies."""

    def __init__(self):
        self._rx = FrameParser()
        self._out = bytearray()
        self.last = {}       # opcode -> last payload seen (fire-and-forget)
        self.fail = False    # if set, requests answer with an Error frame

    def write(self, data: bytes) -> None:
        for b in data:
            if self._rx.feed(b):
                self._handle(self._rx.opcode, self._rx.payload)

    def read(self, n: int) -> bytes:
        chunk = bytes(self._out[:n])
        del self._out[:n]
        return chunk

    def _reply(self, opcode, payload=b""):
        self._out.extend(encode_frame(int(opcode), payload))

    def _handle(self, opcode, payload):
        if self.fail and opcode >= 0x40:
            self._reply(Opcode.ERROR, bytes([0x03]))  # BAD_OPCODE
            return
        if opcode == Opcode.GET_VOLTAGE:
            self._reply(opcode, struct.pack("<f", 7.4))
        elif opcode == Opcode.GET_CURRENT:
            self._reply(opcode, struct.pack("<f", 1.25))
        elif opcode == Opcode.GET_TELEMETRY:
            self._reply(opcode, struct.pack("<Bfffff", int(State.WALK),
                                            12.5, -3.0, 45.0, 8.1, 2.2))
        elif opcode == Opcode.GET_JOINTS:
            self._reply(opcode, struct.pack("<18f", *([0.0] * 18)))
        elif opcode == Opcode.GET_BODY_POSE:
            self._reply(opcode, struct.pack("<ffffff", 1.0, 2.0, -5.0, 3.0, 4.0, 6.0))
        elif opcode == Opcode.JOG_SERVO or 0x10 <= opcode <= 0x19:
            self.last[opcode] = payload
            self._reply(opcode, bytes([int(Status.OK)]))  # ack provisioning / jog
        else:
            self.last[opcode] = payload  # fire-and-forget


def test_fire_and_forget_frames():
    fw = FakeFirmware()
    c = HexapodClient(fw)
    c.enable()
    c.set_velocity(1.0, -2.0, 0.5)
    c.set_gait(GaitId.RIPPLE)
    c.set_led(LedMode.BLINK, 10, 20, 30, 2.0)
    assert fw.last[Opcode.ENABLE] == b""
    assert struct.unpack("<fff", fw.last[Opcode.SET_VELOCITY]) == (1.0, -2.0, 0.5)
    assert fw.last[Opcode.SET_GAIT] == bytes([int(GaitId.RIPPLE)])
    assert struct.unpack("<BBBBf", fw.last[Opcode.SET_LED]) == (int(LedMode.BLINK), 10, 20, 30, 2.0)


def test_scalar_queries():
    c = HexapodClient(FakeFirmware())
    assert abs(c.get_voltage() - 7.4) < 1e-4
    assert abs(c.get_current() - 1.25) < 1e-4


def test_telemetry_query():
    c = HexapodClient(FakeFirmware())
    t = c.get_telemetry()
    assert t.state == State.WALK
    assert abs(t.odom_x - 12.5) < 1e-4 and abs(t.odom_yaw - 45.0) < 1e-4
    assert abs(t.voltage - 8.1) < 1e-4 and abs(t.current - 2.2) < 1e-4


def test_joints_query():
    c = HexapodClient(FakeFirmware())
    j = c.get_joints()
    assert len(j) == 18


def test_body_pose_query():
    c = HexapodClient(FakeFirmware())
    p = c.get_body_pose()
    assert abs(p.z - (-5.0)) < 1e-4
    assert abs(p.height - (-5.0)) < 1e-4  # .height convenience == z offset
    assert abs(p.roll - 3.0) < 1e-4 and abs(p.yaw - 6.0) < 1e-4


def test_velocity_buffered_no_roundtrip():
    fw = FakeFirmware()
    c = HexapodClient(fw)
    c.set_velocity(10.0, -20.0, 5.0)
    assert c.get_velocity() == Velocity(10.0, -20.0, 5.0)
    c.stop()  # board zeros velocity -> buffer follows
    assert c.get_velocity() == Velocity(0.0, 0.0, 0.0)


def test_velocity_clamped_to_provisioned_limits():
    c = HexapodClient(FakeFirmware())
    c.provision()  # packaged default config supplies the clamps
    c.set_velocity(1e6, 0.0, 1e6)
    v = c.get_velocity()
    assert abs(v.vx - c._lin_vel_max) < 1e-4  # clamped to lin_vel_max
    assert abs(v.wz - c._ang_vel_max) < 1e-4  # clamped to ang_vel_max


def test_gait_buffered():
    c = HexapodClient(FakeFirmware())
    assert c.get_gait() == GaitId.TRIPOD  # firmware default
    c.set_gait(GaitId.WAVE)
    assert c.get_gait() == GaitId.WAVE


def test_error_reply_raises():
    fw = FakeFirmware()
    fw.fail = True
    c = HexapodClient(fw)
    try:
        c.get_voltage()
    except HexapodError as e:
        assert "BAD_OPCODE" in str(e)
    else:
        assert False, "expected HexapodError"


def test_timeout_when_silent():
    class Silent(Transport):
        def write(self, data): pass
        def read(self, n): return b""
    c = HexapodClient(Silent(), timeout=0.05)
    try:
        c.get_voltage()
    except TimeoutError:
        pass
    else:
        assert False, "expected TimeoutError"


class Preloaded(Transport):
    """A port with bytes already waiting. `chunk` caps how many come back per
    read, so a frame can be made to straddle two reads."""

    def __init__(self, data: bytes, chunk: int = 256):
        self._out = bytearray(data)
        self._chunk = chunk

    def write(self, data: bytes) -> None:
        pass

    def read(self, n: int) -> bytes:
        out = bytes(self._out[:min(n, self._chunk)])
        del self._out[:len(out)]
        return out


def _telemetry_frame(state=State.WALK, voltage=8.1):
    return encode_frame(int(Opcode.GET_TELEMETRY),
                        struct.pack("<Bfffff", int(state), 12.5, -3.0, 45.0, voltage, 2.2))


def _joints_frame(values):
    return encode_frame(int(Opcode.GET_JOINTS), struct.pack("<18f", *values))


def test_frame_sharing_a_read_is_not_lost():
    # Two replies sitting in the port together, as happens when one arrives late
    # and the next query goes out before it has been consumed. Reading is
    # destructive, so the joints frame is only ever seen by this first read
    values = [float(i) for i in range(18)]
    c = HexapodClient(Preloaded(_telemetry_frame() + _joints_frame(values)), timeout=0.05)
    assert c.get_telemetry().state == State.WALK
    assert c.get_joints() == values


def test_unwanted_frame_ahead_of_the_reply_is_skipped():
    stale = encode_frame(int(Opcode.GET_VOLTAGE), struct.pack("<f", 6.0))
    c = HexapodClient(Preloaded(stale + _telemetry_frame(voltage=7.7)), timeout=0.05)
    t = c.get_telemetry()
    assert abs(t.voltage - 7.7) < 1e-4  # the telemetry frame, not the stale one


def test_frame_split_across_reads():
    values = [float(i) for i in range(18)]
    c = HexapodClient(Preloaded(_joints_frame(values), chunk=7), timeout=0.5)
    assert c.get_joints() == values  # 77 bytes reassembled from 7-byte reads


def test_error_frame_keeps_the_bytes_behind_it():
    err = encode_frame(int(Opcode.ERROR), bytes([int(Status.BAD_OPCODE)]))
    c = HexapodClient(Preloaded(err + _telemetry_frame()), timeout=0.05)
    try:
        c.get_telemetry()
    except HexapodError:
        pass
    else:
        assert False, "expected HexapodError"
    assert c.get_telemetry().state == State.WALK  # survived the raise


def _run():
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"  ok    {name}")
            except AssertionError as e:
                fails += 1
                print(f"  FAIL  {name}  {e}")
    print(f"\n[client] {'PASSED' if not fails else 'FAILED'} — {fails} failure(s)")
    return fails


if __name__ == "__main__":
    sys.exit(1 if _run() else 0)

"""Client tests against a fake firmware that speaks the real wire protocol."""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hexapod import (  # noqa: E402
    GaitId, HexapodClient, HexapodError, LedMode, Opcode, State,
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

"""Framing tests, including golden frames captured from the C++ encoder so the
Python mirror is proven byte-for-byte compatible with the firmware."""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hexapod import Opcode, encode_frame, crc16  # noqa: E402
from hexapod.framing import FrameParser  # noqa: E402

# Golden frames emitted by Hexapod-Firmware's proto::encode_frame (host build).
GOLDEN = {
    "set_velocity": "AA 0C 30 00 00 80 3F 00 00 00 C0 00 00 00 3F 1E F8",
    "enable":       "AA 00 33 3F 1B",
    "framey":       "AA 05 40 01 02 AA 55 CD B6 26",
}


def _hex(s):
    return bytes.fromhex(s.replace(" ", ""))


def test_golden_encode_matches_cpp():
    assert encode_frame(Opcode.SET_VELOCITY, struct.pack("<fff", 1.0, -2.0, 0.5)) == _hex(GOLDEN["set_velocity"])
    assert encode_frame(Opcode.ENABLE) == _hex(GOLDEN["enable"])
    assert encode_frame(Opcode.GET_TELEMETRY, bytes([0x01, 0x02, 0xAA, 0x55, 0xCD])) == _hex(GOLDEN["framey"])


def _parse(data):
    p = FrameParser()
    done = None
    for b in data:
        if p.feed(b):
            done = p
    return done


def test_golden_parse_round_trip():
    p = _parse(_hex(GOLDEN["set_velocity"]))
    assert p is not None
    assert p.opcode == Opcode.SET_VELOCITY
    assert struct.unpack("<fff", p.payload) == (1.0, -2.0, 0.5)


def test_zero_length_frame():
    p = _parse(_hex(GOLDEN["enable"]))
    assert p is not None and p.opcode == Opcode.ENABLE and p.payload == b""


def test_payload_with_framing_bytes():
    # LEN-governed framing must not trip over 0xAA / 0x55 inside the payload.
    p = _parse(_hex(GOLDEN["framey"]))
    assert p is not None and p.payload == bytes([0x01, 0x02, 0xAA, 0x55, 0xCD])


def test_resync_after_garbage():
    stream = bytes([0x00, 0xAA, 0xFF]) + encode_frame(Opcode.SET_GAIT, bytes([2]))
    p = _parse(stream)
    assert p is not None and p.opcode == Opcode.SET_GAIT and p.payload == bytes([2])


def test_crc_rejects_corruption():
    frame = bytearray(encode_frame(Opcode.GET_JOINTS, bytes([10, 20, 30, 40])))
    frame[3] ^= 0xFF  # flip a payload byte
    assert _parse(bytes(frame)) is None


def test_crc16_known_value():
    # CRC is computed over LEN + opcode + payload; check the enable frame's value.
    assert crc16(bytes([0x00, 0x33])) == 0x1B3F


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
    print(f"\n[framing] {'PASSED' if not fails else 'FAILED'} — {fails} failure(s)")
    return fails


if __name__ == "__main__":
    sys.exit(1 if _run() else 0)

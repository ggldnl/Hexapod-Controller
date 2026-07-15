"""
HDLC-style framing.

Frame:  SOF(0xAA) | LEN | opcode | payload[LEN] | CRC16-lo | CRC16-hi
  - LEN counts the payload only (the opcode is separate); no end-of-frame byte.
  - CRC16-CCITT (poly 0x1021, init 0xFFFF) over LEN + opcode + payload.
"""
from __future__ import annotations

from .protocol import MAX_PAYLOAD, SOF

CRC_POLY = 0x1021
CRC_INIT = 0xFFFF


def _crc_update(crc: int, byte: int) -> int:
    crc ^= byte << 8
    for _ in range(8):
        crc = ((crc << 1) ^ CRC_POLY) if (crc & 0x8000) else (crc << 1)
        crc &= 0xFFFF
    return crc


def crc16(data: bytes) -> int:
    crc = CRC_INIT
    for b in data:
        crc = _crc_update(crc, b)
    return crc


def encode_frame(opcode: int, payload: bytes = b"") -> bytes:
    """Build a complete frame for one opcode + payload."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} > MAX_PAYLOAD {MAX_PAYLOAD}")
    length = len(payload)
    body = bytes([length, opcode]) + payload
    crc = crc16(body)
    return bytes([SOF]) + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class FrameParser:
    """Byte-at-a-time receiver. `feed(b)` returns True on the byte that completes
    a CRC-valid frame; `opcode`/`payload` are then valid until the next feed.
    On a bad length or CRC it silently resyncs on the next SOF."""

    _SOF, _LEN, _OP, _PAYLOAD, _CRCLO, _CRCHI = range(6)

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._phase = self._SOF
        self._len = 0
        self._crc = CRC_INIT
        self._rx_crc = 0
        self.opcode = 0
        self.payload = b""
        self._buf = bytearray()

    def feed(self, b: int) -> bool:
        b &= 0xFF
        p = self._phase
        if p == self._SOF:
            if b == SOF:
                self._phase = self._LEN
            return False
        if p == self._LEN:
            if b > MAX_PAYLOAD:  # implausible length -> resync
                self._phase = self._LEN if b == SOF else self._SOF
                return False
            self._len = b
            self._crc = _crc_update(CRC_INIT, b)
            self._buf = bytearray()
            self._phase = self._OP
            return False
        if p == self._OP:
            self.opcode = b
            self._crc = _crc_update(self._crc, b)
            self._phase = self._CRCLO if self._len == 0 else self._PAYLOAD
            return False
        if p == self._PAYLOAD:
            self._buf.append(b)
            self._crc = _crc_update(self._crc, b)
            if len(self._buf) >= self._len:
                self._phase = self._CRCLO
            return False
        if p == self._CRCLO:
            self._rx_crc = b
            self._phase = self._CRCHI
            return False
        if p == self._CRCHI:
            self._rx_crc |= b << 8
            self._phase = self._SOF
            if self._rx_crc == self._crc:
                self.payload = bytes(self._buf)
                return True
            return False
        return False

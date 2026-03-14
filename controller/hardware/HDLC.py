from typing import Optional, Tuple
import serial
import time


SOF = 0xAA
EOF = 0x55

CRC_POLY = 0x1021
CRC_INIT = 0xFFFF

class HDLC:
    def __init__(self, port: str, baud: int, timeout: float = 0.1):
        self.ser = serial.Serial(port, baud, timeout=timeout)

    def close(self):
        self.ser.close()

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = CRC_INIT
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ CRC_POLY
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def send_frame(self, opcode: int, payload: bytes):
        body = bytes([opcode]) + payload
        length = len(body)

        frame = bytearray()
        frame.append(SOF)
        frame.append(length)
        frame.extend(body)

        crc = self.crc16(bytes([length]) + body)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)

        frame.append(EOF)

        self.ser.write(frame)

    def read_frame(self, timeout: float = 0.1) -> Optional[Tuple[int, bytes]]:
        deadline = time.monotonic() + timeout
        original_timeout = self.ser.timeout

        try:

            # SOF never arrived — Pico didn't respond at all, or wrong baud rate
            while time.monotonic() < deadline:
                self.ser.timeout = max(0.001, deadline - time.monotonic())
                b = self.ser.read(1)
                if b and b[0] == SOF:
                    break
            else:
                raise TimeoutError("Timed out waiting for SOF")

            def read_until_deadline(n: int) -> bytes:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Deadline expired before reading {n} byte(s)")
                self.ser.timeout = remaining
                data = self.ser.read(n)
                if len(data) != n:
                    raise TimeoutError(f"Expected {n} byte(s), got {len(data)} — frame truncated")
                return data

            # Frame arrived but subsequent bytes didn't — likely a crash mid-response
            length_b = read_until_deadline(1)
            length = length_b[0]

            payload = read_until_deadline(length)
            crc_bytes = read_until_deadline(2)
            eof_byte = read_until_deadline(1)

            # EOF marker wrong — framing error, possible serial corruption or baud mismatch
            if eof_byte[0] != EOF:
                raise RuntimeError(f"Bad EOF marker: got {eof_byte[0]:#04x}, expected {EOF:#04x}")

            rx_crc = crc_bytes[0] | (crc_bytes[1] << 8)
            calc_crc = self.crc16(bytes([length]) + payload)

            # CRC wrong — data arrived but was corrupted in transit
            if rx_crc != calc_crc:
                raise RuntimeError(f"CRC mismatch: got {rx_crc:#06x}, expected {calc_crc:#06x}")

            opcode = payload[0]
            data = payload[1:]
            return opcode, data

        finally:
            self.ser.timeout = original_timeout  # always restored, whether success or exception
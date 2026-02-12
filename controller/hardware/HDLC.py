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

        # wait for SOF
        # TODO this is blocking, maybe will cause problems on real hardware
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] == SOF:
                break
        else:
            return None

        length_b = self.ser.read(1)
        if len(length_b) != 1:
            return None
        length = length_b[0]

        payload = self.ser.read(length)
        if len(payload) != length:
            return None

        crc_bytes = self.ser.read(2)
        if len(crc_bytes) != 2:
            return None

        eof = self.ser.read(1)
        if eof != bytes([EOF]):
            return None

        rx_crc = crc_bytes[0] | (crc_bytes[1] << 8)
        calc_crc = self.crc16(bytes([length]) + payload)

        if rx_crc != calc_crc:
            return None

        opcode = payload[0]
        data = payload[1:]
        return opcode, data
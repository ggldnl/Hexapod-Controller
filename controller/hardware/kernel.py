import struct

from .HDLC import HDLC


class Kernel:
    
    def __init__(self, port: str, baud: int):
        self.transport = HDLC(port, baud)

    def close(self):
        self.transport.close()

    def _request(self, opcode: int, payload: bytes = b'', timeout: float = 0.1) -> bytes:
        self.transport.send_frame(opcode, payload)

        frame = self.transport.read_frame(timeout)
        if frame is None:
            raise TimeoutError("No response")

        rx_opcode, data = frame
        if rx_opcode != opcode:
            raise RuntimeError("Opcode mismatch")

        return data

    # Commands

    def connect_power(self) -> bool:
        data = self._request(0x12)
        return data[0] == 0x01

    def disconnect_power(self) -> bool:
        data = self._request(0x13)
        return data[0] == 0x01

    def get_voltage(self) -> float:
        data = self._request(0x01)
        return struct.unpack('<f', data)[0]

    def get_current(self) -> float:
        data = self._request(0x02)
        return struct.unpack('<f', data)[0]

    def read_sensor(self, pin: int) -> float:
        data = self._request(0x03, struct.pack('B', pin))
        return struct.unpack('<f', data)[0]

    def set_led(self, pin: int, r: int, g: int, b: int) -> bool:
        payload = struct.pack('BBBB', pin, r, g, b)
        data = self._request(0x04, payload)
        return data[0] == 0x01

    def set_leds(self, leds: list[tuple[int, int, int, int]]) -> bool:
        payload = bytearray()
        payload.append(len(leds))
        for pin, r, g, b in leds:
            payload.extend([pin, r, g, b])
        data = self._request(0x05, bytes(payload))
        return data[0] == 0x01

    def get_led(self, pin: int) -> tuple[int, int, int]:

        # TODO properly handle the return once you test this on the Raspberry
        """
        # Maybe this is what you need
        struct.unpack('<I', data[i:i+4])[0]
            for i in range(0, len(data), 4)
        """

        data = self._request(0x06, struct.pack('B', pin))
        return tuple(data)

    def get_leds(self, pins: list[int]) -> list[tuple[int, int, int]]:
        payload = bytearray()
        payload.append(len(pins))
        payload.extend(pins)
        data = self._request(0x07, bytes(payload))

        result = []
        for i in range(0, len(data), 3):
            result.append(tuple(data[i:i+3]))
        return result

    def attach_servos(self) -> bool:
        data = self._request(0x08)
        return data[0] == 0x01

    def detach_servos(self) -> bool:
        data = self._request(0x09)
        return data[0] == 0x01

    def set_servo_pulse_width(self, pin: int, pulse: int) -> bool:
        payload = struct.pack('<BI', pin, pulse)
        data = self._request(0x0A, payload)
        return data[0] == 0x01

    def set_servo_pulse_widths(self, values: list[tuple[int, int]]) -> bool:
        payload = bytearray()
        payload.append(len(values))
        for pin, pulse in values:
            payload.extend(struct.pack('<BI', pin, pulse))
        data = self._request(0x0B, bytes(payload))
        return data[0] == 0x01

    def set_servo_angle(self, pin: int, angle: float) -> bool:
        payload = struct.pack('<Bf', pin, angle)
        data = self._request(0x0C, payload)
        return data[0] == 0x01

    def set_servo_angles(self, values: list[tuple[int, float]]) -> bool:
        payload = bytearray()
        payload.append(len(values))
        for pin, angle in values:
            payload.extend(struct.pack('<Bf', pin, angle))
        data = self._request(0x0D, bytes(payload))
        return data[0] == 0x01

    def get_servo_pulse_width(self, pin: int) -> int:
        data = self._request(0x0E, struct.pack('B', pin))
        return struct.unpack('<I', data)[0]

    def get_servo_pulse_widths(self, pins: list[int]) -> list[int]:
        payload = bytearray()
        payload.append(len(pins))
        payload.extend(pins)
        data = self._request(0x0F, bytes(payload))

        return [
            struct.unpack('<I', data[i:i+4])[0]
            for i in range(0, len(data), 4)
        ]

    def get_servo_angle(self, pin: int) -> float:
        data = self._request(0x10, struct.pack('B', pin))
        return struct.unpack('<f', data)[0]

    def get_servo_angles(self, pins: list[int]) -> list[float]:
        payload = bytearray()
        payload.append(len(pins))
        payload.extend(pins)
        data = self._request(0x11, bytes(payload))

        return [
            struct.unpack('<f', data[i:i+4])[0]
            for i in range(0, len(data), 4)
        ]


if __name__ == "__main__":

    iface = Kernel("/dev/ttyAMA0", 115200)

    try:
        v = iface.get_voltage()
        i = iface.get_current()
        print("Voltage:", v)
        print("Current:", i)
    finally:
        iface.close()

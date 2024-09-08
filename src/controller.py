import serial
import struct


class Controller:

    def __init__(self, dev='/dev/ttyACM0', baud=115200):
        self.dev = dev
        self.baud = baud
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.dev, self.baud)

    def close(self):
        self.ser.close()

    def __enter__(self):
        self.open()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def send_command(self, opcode, *args):
        self.ser.write(bytes([opcode]))
        for arg in args:
            self.ser.write(arg)

    def get_voltage(self):
        self.send_command(0x01)
        response = self.ser.read(4)  # Float is 4 bytes
        return struct.unpack('f', response)[0]

    def get_current(self):
        self.send_command(0x02)
        response = self.ser.read(4)  # Float is 4 bytes
        return struct.unpack('f', response)[0]

    def read_sensor(self, pin):
        self.send_command(0x03, bytes([pin]))
        response = self.ser.read(4)  # Float is 4 bytes
        return struct.unpack('f', response)[0]

    def set_led(self, pin, r=0, g=0, b=255):
        self.send_command(0x04, bytes([pin, r, g, b]))
        return self.ser.read(1)

    def set_leds(self, pins, arr_rgb_tuples):
        comm = [len(pins)] + [elem for pin, rgb in zip(pins, arr_rgb_tuples) for elem in [pin] + list(rgb)]
        args = bytes(comm)
        self.send_command(0x05, args)
        return self.ser.read(1)

    def attach_servos(self):
        self.send_command(0x06)
        return self.ser.read(1)

    def detach_servos(self):
        self.send_command(0x07)
        return self.ser.read(1)

    def set_pulse(self, pin, pulse):
        pulse_bytes = struct.pack('f', pulse)
        self.send_command(0x08, bytes([pin]), pulse_bytes)
        return self.ser.read(1)

    def set_pulses(self, pins, pulses):
        comm = [len(pins)] + [byte for pin, value in zip(pins, pulses) for byte in [pin] + list(struct.pack('f', value))]
        args = bytes(comm)
        self.send_command(0x09, args)
        return self.ser.read(1)

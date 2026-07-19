"""
Byte transports for the client. The client only needs write() and a best-effort
read(); this abstraction lets the same client talk to a real serial port or, in
simulation, to an in-process firmware bridge.
"""
from __future__ import annotations

from abc import ABC, abstractmethod


class Transport(ABC):

    @abstractmethod
    def write(self, data: bytes) -> None:
        ...

    @abstractmethod
    def read(self, n: int) -> bytes:
        """Return up to n bytes; may return fewer (or b'') on timeout."""
        ...

    def close(self) -> None:
        pass


class SerialTransport(Transport):
    """Real UART link to the Servo2040 over pyserial."""

    def __init__(self, port: str, baud: int, timeout: float = 0.02) -> None:
        import serial  # imported lazily so the package works without pyserial
        self._ser = serial.Serial(port, baud, timeout=timeout)

    def write(self, data: bytes) -> None:
        self._ser.write(data)

    def read(self, n: int) -> bytes:
        # pyserial's read(n) returns early only once it has exactly n bytes, so
        # asking for more than the board sent burns the whole timeout waiting for
        # bytes that will never come. Take what is already buffered instead, and
        # block for a single byte only when the buffer is empty
        waiting = self._ser.in_waiting
        if waiting:
            return self._ser.read(min(n, waiting))
        return self._ser.read(1)

    def close(self) -> None:
        self._ser.close()

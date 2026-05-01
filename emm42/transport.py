"""Serial transport layer — thin pyserial wrapper."""
from __future__ import annotations
import serial
from typing import Optional


class Transport:
    """Open/close/read/write over a serial port."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.5) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None

    def open(self) -> None:
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    def __enter__(self) -> "Transport":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise RuntimeError("Transport is not open")
        return self._ser.write(data)  # type: ignore[union-attr]

    def read(self, n: int) -> bytes:
        if not self.is_open:
            raise RuntimeError("Transport is not open")
        return self._ser.read(n)  # type: ignore[union-attr]

    def read_until(self, size: int = 128, idle_ms: float = 100) -> bytes:
        """Read bytes until the port is idle for *idle_ms* or *size* bytes received."""
        import time
        buf = bytearray()
        last_rx = time.monotonic()
        while True:
            chunk = self._ser.read(min(size - len(buf), 64))  # type: ignore[union-attr]
            if chunk:
                buf.extend(chunk)
                last_rx = time.monotonic()
                if len(buf) >= size:
                    break
            else:
                if (time.monotonic() - last_rx) * 1000 >= idle_ms:
                    break
        return bytes(buf)

    def flush(self) -> None:
        if self._ser:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

    @staticmethod
    def list_available_ports() -> list:
        """Return a list of available serial port names."""
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()]

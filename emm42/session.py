"""Session: bundles connection configuration."""
from __future__ import annotations
from dataclasses import dataclass, field
from .protocol.codec import ChecksumMode
from .transport import Transport


@dataclass
class Session:
    """All per-connection parameters.

    Args:
        port: Serial port name ("COM15", "/dev/ttyUSB0", …).
        baudrate: Baud rate matching device setting (default 115200).
        address: Motor address 1–255 (default 1).
        checksum_mode: Must match device checksum configuration (default FIXED_6B).
        timeout: Serial read timeout in seconds (default 0.5).
    """
    port: str
    baudrate: int = 115200
    address: int = 1
    checksum_mode: ChecksumMode = field(default_factory=lambda: ChecksumMode.FIXED_6B)
    timeout: float = 0.5

    def make_transport(self) -> Transport:
        """Create (but do not open) a Transport for this session."""
        return Transport(self.port, self.baudrate, self.timeout)

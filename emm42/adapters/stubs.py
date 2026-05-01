"""Adapter stubs — superseded commands kept for backward compatibility.

All three write commands (write_pid, write_id_address, write_drive_config)
are now fully implemented in MotorDevice (service.py) and Cmd (commands.py).
These stubs exist only so legacy code that imported them still gets a clear
NotImplementedError rather than an ImportError.
"""
from __future__ import annotations
from ..session import Session
from ..protocol.codec import build_frame
from ..protocol.commands import Cmd


class DriveConfigStub:
    """Raw read helper for 0x42 drive-config.

    NOTE: Superseded by MotorDevice.read_drive_config() which parses the response
    into a named dict. Use that instead.
    """

    def __init__(self, session: Session) -> None:
        from ..transport import Transport
        self._session = session
        self._transport: Transport | None = None

    def open(self) -> None:
        self._transport = self._session.make_transport()
        self._transport.open()

    def close(self) -> None:
        if self._transport:
            self._transport.close()

    def read_raw(self) -> bytes:
        """Send 0x42/0x6C query; return raw response bytes."""
        assert self._transport is not None, "Call open() first"
        frame = build_frame(
            Cmd.read_drive_config(self._session.address),
            self._session.checksum_mode,
        )
        self._transport.write(frame)
        return self._transport.read_until(size=64)


class WriteDriveConfigStub:
    """Deprecated stub — use MotorDevice.write_drive_config() instead."""

    def write(self, **_kwargs) -> None:
        raise NotImplementedError(
            "WriteDriveConfigStub is deprecated. "
            "Use MotorDevice.write_drive_config(**fields) instead."
        )


class WritePIDStub:
    """Deprecated stub — use MotorDevice.write_pid_params() instead."""

    def write(self, kp: int, ki: int, kd: int) -> None:
        raise NotImplementedError(
            "WritePIDStub is deprecated. "
            "Use MotorDevice.write_pid_params(kp, ki, kd) instead."
        )


class WriteIDAddressStub:
    """Deprecated stub — use MotorDevice.write_id_address() instead."""

    def write(self, new_addr: int) -> None:
        raise NotImplementedError(
            "WriteIDAddressStub is deprecated. "
            "Use MotorDevice.write_id_address(new_addr) instead."
        )

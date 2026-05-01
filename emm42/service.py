"""MotorDevice — high-level API for one EMM42 motor."""
from __future__ import annotations
import struct
from typing import Optional

from .session import Session
from .transport import Transport
from .protocol.codec import (
    ChecksumMode,
    build_frame,
    decode_position_response,
    decode_angle_response,
    decode_encoder_response,
    decode_uint16_response,
    decode_velocity_response,
    decode_firmware_response,
    decode_phase_rl_response,
    decode_pid_response,
    decode_status_response,
    decode_origin_status_response,
    decode_drive_config_response,
    decode_system_state_response,
)
from .protocol.commands import Cmd


class MotorDevice:
    """High-level interface to one EMM42 V5 stepper motor.

    Usage::

        sess = Session(port="COM15", address=1)
        with MotorDevice(sess) as motor:
            motor.enable()
            motor.velocity(direction=0, rpm=500, acceleration=30)
    """

    def __init__(self, session: Session) -> None:
        self._session = session
        self._transport: Optional[Transport] = None

    # ── lifecycle ────────────────────────────────────────────────────────────

    def open(self) -> None:
        self._transport = self._session.make_transport()
        self._transport.open()

    def close(self) -> None:
        if self._transport:
            self._transport.close()

    def __enter__(self) -> "MotorDevice":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ── internal helpers ─────────────────────────────────────────────────────

    @property
    def _addr(self) -> int:
        return self._session.address

    @property
    def _mode(self) -> ChecksumMode:
        return self._session.checksum_mode

    def _send(self, payload_ints) -> None:
        frame = build_frame(payload_ints, self._mode)
        assert self._transport is not None, "Call open() first"
        self._transport.write(frame)

    def _send_recv(self, payload_ints, recv_bytes: int = 16) -> bytes:
        self._send(payload_ints)
        assert self._transport is not None
        return self._transport.read_until(size=recv_bytes)

    # ── enable / disable ─────────────────────────────────────────────────────

    def enable(self, sync: bool = False) -> None:
        """Enable the motor (使能驱动板)."""
        self._send(Cmd.enable(self._addr, state=True, sync=sync))

    def disable(self, sync: bool = False) -> None:
        """Disable (de-energise) the motor (关闭驱动板)."""
        self._send(Cmd.enable(self._addr, state=False, sync=sync))

    # ── motion commands ───────────────────────────────────────────────────────

    def velocity(
        self,
        direction: int = 0,
        rpm: int = 300,
        acceleration: int = 0,
        sync: bool = False,
    ) -> None:
        """Start velocity (continuous) motion (速度模式).

        Args:
            direction: 0 = CW, 1 = CCW.
            rpm: 0–5000 RPM.
            acceleration: 0 = instant start; 1–255.
            sync: Hold for synchronous trigger.
        """
        self._send(Cmd.velocity(self._addr, direction, rpm, acceleration, sync))

    def position(
        self,
        direction: int = 0,
        rpm: int = 300,
        acceleration: int = 0,
        pulses: int = 1600,
        relative: bool = True,
        sync: bool = False,
    ) -> None:
        """Execute a position move (位置模式).

        Args:
            direction: 0 = CW, 1 = CCW.
            rpm: Move speed in RPM.
            acceleration: 0 = instant; 1–255.
            pulses: Step pulse count (default 1600 = one rev at 16-microstep).
            relative: True = relative move, False = absolute.
            sync: Hold for synchronous trigger.
        """
        self._send(Cmd.position(self._addr, direction, rpm, acceleration, pulses, relative, sync))

    def stop(self, sync: bool = False) -> None:
        """Immediately stop the motor (立即停止)."""
        self._send(Cmd.stop(self._addr, sync))

    def sync_trigger(self) -> None:
        """Fire multi-axis synchronous motion trigger (触发多机同步运动)."""
        self._send(Cmd.sync_trigger(self._addr))

    # ── configuration commands ────────────────────────────────────────────────

    def modify_control_mode(self, mode: int, store: bool = False) -> None:
        """Change control mode between open-loop and closed-loop (修改控制模式).

        Args:
            mode: 0 = open-loop, 1 = closed-loop (confirm with device docs).
            store: True = save to EEPROM.
        """
        self._send(Cmd.modify_control_mode(self._addr, mode, store))

    def modify_origin_params(
        self,
        mode: int,
        direction: int,
        speed_rpm: int,
        timeout_ms: int,
        stall_speed_rpm: int,
        stall_current_ma: int,
        stall_time_ms: int,
        power_on_trigger: bool,
        store: bool = False,
    ) -> None:
        """Modify all homing (回零) parameters (修改回零参数).

        Args:
            mode: Homing mode — 0=Nearest, 1=single-turn CW, 2=multi-turn, 3=limit.
            direction: 0=CW, 1=CCW.
            speed_rpm: Homing speed (0–5000 RPM).
            timeout_ms: Homing timeout in milliseconds (0–999999 ms).
            stall_speed_rpm: No-limit collision detection speed (0–5000 RPM).
            stall_current_ma: No-limit collision detection current (0–5000 mA).
            stall_time_ms: No-limit collision detection time (0–5000 ms).
            power_on_trigger: Auto-trigger homing on power-on.
            store: Save to EEPROM.
        """
        self._send(Cmd.origin_modify_params(
            self._addr, store, mode, direction, speed_rpm,
            timeout_ms, stall_speed_rpm, stall_current_ma,
            stall_time_ms, power_on_trigger,
        ))

    # ── homing commands ───────────────────────────────────────────────────────

    def homing_set_origin(self, store: bool = False) -> None:
        """Set current position as homing origin (设置单圈回零零点)."""
        self._send(Cmd.origin_set_zero(self._addr, store))

    def homing_trigger(self, mode: int = 0, sync: bool = False) -> None:
        """Trigger return-to-origin homing (触发回零)."""
        self._send(Cmd.origin_trigger_return(self._addr, mode, sync))

    def homing_interrupt(self) -> None:
        """Force abort of homing operation (强制中断并退出回零)."""
        self._send(Cmd.origin_interrupt(self._addr))

    # ── utility commands ──────────────────────────────────────────────────────

    def reset_position(self) -> None:
        """Reset current position counter to zero (将当前位置清零, no EEPROM)."""
        self._send(Cmd.reset_current_pos_to_zero(self._addr))

    def clear_stall(self) -> None:
        """Clear stall/clog protection latch (解除堵转保护)."""
        self._send(Cmd.reset_clog_protection(self._addr))

    # ── read: motion state ────────────────────────────────────────────────────

    def read_position(self) -> float:
        """Read real-time position in degrees (实时位置角度).

        Response: [addr, 0x36, sign, b3, b2, b1, b0, checksum]
        """
        raw = self._send_recv(Cmd.read_position(self._addr), recv_bytes=8)
        return decode_position_response(raw)

    def read_target_position(self) -> float:
        """Read target position in degrees (目标位置角度).

        Response: [addr, 0x33, sign, b3, b2, b1, b0, checksum]
        """
        raw = self._send_recv(Cmd.read_param(self._addr, 0x33), recv_bytes=8)
        return decode_angle_response(raw)

    def read_position_error(self) -> float:
        """Read position error in degrees (位置角度误差).

        Response: [addr, 0x37, sign, b3, b2, b1, b0, checksum]
        """
        raw = self._send_recv(Cmd.read_param(self._addr, 0x37), recv_bytes=8)
        return decode_angle_response(raw)

    def read_velocity(self) -> int:
        """Read real-time velocity in RPM (实时转速); negative = CCW.

        Response: [addr, 0x35, sign, velH, velL, checksum]  — 6 bytes.
        Source: Arduino 读取电机实时转速 example (rxCount==6 check confirmed).
        """
        raw = self._send_recv(Cmd.read_velocity(self._addr), recv_bytes=6)
        return decode_velocity_response(raw)

    def read_encoder(self) -> int:
        """Read linearised encoder value (编码器线性值).

        Response: [addr, 0x31, sign, b3, b2, b1, b0, checksum]
        Returns signed raw encoder count.
        """
        raw = self._send_recv(Cmd.read_param(self._addr, 0x31), recv_bytes=8)
        return decode_encoder_response(raw)

    # ── read: electrical ──────────────────────────────────────────────────────

    def read_bus_voltage(self) -> int:
        """Read bus voltage in mV (总线电压).

        Response: [addr, 0x24, voltH, voltL, checksum]
        """
        raw = self._send_recv(Cmd.read_bus_voltage(self._addr), recv_bytes=5)
        return decode_uint16_response(raw)

    def read_phase_current(self) -> int:
        """Read phase current in mA (相电流).

        Response: [addr, 0x27, curH, curL, checksum]
        """
        raw = self._send_recv(Cmd.read_phase_current(self._addr), recv_bytes=5)
        return decode_uint16_response(raw)

    def read_phase_rl(self) -> dict:
        """Read phase resistance and inductance (相电阻/相电感).

        Response: [addr, 0x20, R_H, R_L, L_H, L_L, checksum]
        Returns: {"resistance_mohm": int, "inductance_uh": int}
        """
        raw = self._send_recv(Cmd.read_param(self._addr, 0x20), recv_bytes=7)
        return decode_phase_rl_response(raw)

    # ── read: status flags ────────────────────────────────────────────────────

    def read_status(self) -> dict:
        """Read motor status flags (状态标志位 0x3A).

        Response: [addr, 0x3A, flags, checksum]
        Returns: {"enabled", "in_position", "stalled", "stall_protection"}

        Source: GUI labels — 使能状态标志/电机到位标志/电机堵转标志/堵转保护标志
        """
        raw = self._send_recv(Cmd.read_status(self._addr), recv_bytes=4)
        return decode_status_response(raw)

    def read_origin_status(self) -> dict:
        """Read homing status flags (回零状态标志位 0x3B).

        Response: [addr, 0x3B, flags, checksum]
        Returns: {"is_homing", "homing_failed"}

        Source: GUI labels — 正在回零标志/回零失败标志
        """
        raw = self._send_recv(Cmd.read_param(self._addr, 0x3B), recv_bytes=4)
        return decode_origin_status_response(raw)

    # ── read: configuration ───────────────────────────────────────────────────

    def read_firmware_version(self) -> dict:
        """Read hardware and firmware version (固件版本 0x1F).

        Response: [addr, 0x1F, hw_major, hw_minor, fw_major, fw_minor, checksum]
        Returns: {"hw_version": "X.Y", "fw_version": "X.Y"}
        """
        raw = self._send_recv(Cmd.read_param(self._addr, 0x1F), recv_bytes=7)
        return decode_firmware_response(raw)

    def read_pid_params(self) -> dict:
        """Read PID parameters (PID参数 0x21).

        Response: [addr, 0x21, Kp_H, Kp_L, Ki_H, Ki_L, Kd_H, Kd_L, checksum]
        Returns: {"kp": int, "ki": int, "kd": int}
        Defaults per V1.2 update: Kp=32000, Ki=100, Kd=32000
        """
        raw = self._send_recv(Cmd.read_param(self._addr, 0x21), recv_bytes=9)
        return decode_pid_response(raw)

    def read_drive_config(self) -> dict:
        """Read and parse drive configuration (读取驱动参数, 0x42 / aux 0x6C).

        Returns a dict of all 20 GUI 驱动参数 fields:
            motor_type, pulse_ctrl_mode, comm_port_multiplex, en_pin_level,
            dir_pin_direction, microstep, microstep_interpolation, auto_screen_off,
            open_loop_current_ma, stall_protection_enable, stall_detect_rpm,
            stall_detect_current_ma, stall_detect_time_ms, stall_max_current_ma,
            position_window_steps, max_output_voltage_mv, uart_baudrate_index,
            can_rate_index, comm_checksum_mode, cmd_response_mode

        If the device returns an extended response (≥44 bytes), an additional
        "origin_params" key is included with the current homing configuration.

        NOTE: Field layout inferred from write-command format. Verify on hardware.
        """
        raw = self._send_recv(Cmd.read_drive_config(self._addr), recv_bytes=64)
        return decode_drive_config_response(raw)

    def read_origin_params(self) -> dict:
        """Read homing parameters (读取回零参数).

        Sends the drive-config read command (0x42 / 0x6C) and extracts the
        "origin_params" section from the extended response. Returns an empty dict
        if the device only returns the standard 30-byte drive-config block (the
        homing params may then be embedded inside the 0x43 system-state response
        or require a separate command — verify on hardware).

        Returns dict with keys: mode, direction, speed_rpm, timeout_ms,
            stall_speed_rpm, stall_current_ma, stall_time_ms, power_on_trigger.
        """
        cfg = self.read_drive_config()
        return cfg.get("origin_params", {})

    def read_system_state(self) -> dict:
        """Read and parse all system-state monitoring values (读取系统状态参数, 0x43 / 0x7A).

        Returns a dict covering all 13 values shown in the GUI 读取系统状态 panel:
            bus_voltage_mv       (总线电压)
            phase_current_ma     (相电流)
            encoder              (编码器线性值, signed int)
            target_position_deg  (目标位置角度)
            velocity_rpm         (实时转速, negative = CCW)
            position_deg         (实时位置角度)
            position_error_deg   (位置角度误差)
            is_homing            (正在回零标志)
            homing_failed        (回零失败标志)
            enabled              (使能状态标志)
            in_position          (电机到位标志)
            stalled              (电机堵转标志)
            stall_protection     (堵转保护标志)

        NOTE: Frame layout inferred from GUI field order. Verify on hardware.
        """
        raw = self._send_recv(Cmd.read_system_state(self._addr), recv_bytes=64)
        return decode_system_state_response(raw)

    # ── write: PID / drive config / ID address ────────────────────────────────

    def write_pid_params(
        self,
        kp: int,
        ki: int,
        kd: int,
        store: bool = False,
    ) -> None:
        """Write PID parameters (修改PID参数).

        Args:
            kp: Proportional gain (uint16, default 32000).
            ki: Integral gain (uint16, default 100).
            kd: Derivative gain (uint16, default 32000).
            store: True = persist to EEPROM.

        Command: 0xC3 / aux 0x1A
        Source: ZDT EMM42 V5 community protocol (github.com/lyehe/ZDT_stepper).
        Verify with hardware before use in production.
        """
        self._send(Cmd.write_pid(self._addr, kp, ki, kd, store))

    def write_id_address(self, new_addr: int) -> None:
        """Change device ID address (修改驱动ID地址).

        The new address is saved to EEPROM immediately. Reconnect with
        Session(address=new_addr) after calling this method.

        Args:
            new_addr: New address 1–255.

        Command: 0xAE / aux 0x4B
        Source: ZDT EMM42 V5 community protocol (github.com/lyehe/ZDT_stepper).
        Verify with hardware before use in production.
        """
        if not 1 <= new_addr <= 255:
            raise ValueError(f"Address must be 1–255, got {new_addr}")
        self._send(Cmd.write_id_address(self._addr, new_addr))

    def write_drive_config(
        self,
        motor_type: int = 0,
        pulse_ctrl_mode: int = 0,
        comm_port_multiplex: int = 0,
        en_pin_level: int = 0,
        dir_pin_direction: int = 0,
        microstep: int = 16,
        microstep_interpolation: int = 1,
        auto_screen_off: int = 0,
        open_loop_current_ma: int = 1000,
        stall_protection_enable: int = 1,
        stall_detect_rpm: int = 200,
        stall_detect_current_ma: int = 2000,
        stall_detect_time_ms: int = 200,
        stall_max_current_ma: int = 3000,
        position_window_steps: int = 10,
        max_output_voltage_mv: int = 5000,
        uart_baudrate_index: int = 3,
        can_rate_index: int = 0,
        comm_checksum_mode: int = 0,
        cmd_response_mode: int = 1,
        store: bool = True,
    ) -> None:
        """Write full drive configuration (修改驱动参数).

        CAUTION: Always call read_drive_config() first and verify the field values
        match your hardware before issuing a write.
        Incorrect values can render the device unresponsive.

        Command: 0x48 / aux 0x46
        Source: ZDT EMM42 V5 community protocol (github.com/lyehe/ZDT_stepper).
        Field layout matched against GUI 驱动参数 page (20 fields).
        """
        self._send(Cmd.write_drive_config(
            self._addr,
            motor_type=motor_type,
            pulse_ctrl_mode=pulse_ctrl_mode,
            comm_port_multiplex=comm_port_multiplex,
            en_pin_level=en_pin_level,
            dir_pin_direction=dir_pin_direction,
            microstep=microstep,
            microstep_interpolation=microstep_interpolation,
            auto_screen_off=auto_screen_off,
            open_loop_current_ma=open_loop_current_ma,
            stall_protection_enable=stall_protection_enable,
            stall_detect_rpm=stall_detect_rpm,
            stall_detect_current_ma=stall_detect_current_ma,
            stall_detect_time_ms=stall_detect_time_ms,
            stall_max_current_ma=stall_max_current_ma,
            position_window_steps=position_window_steps,
            max_output_voltage_mv=max_output_voltage_mv,
            uart_baudrate_index=uart_baudrate_index,
            can_rate_index=can_rate_index,
            comm_checksum_mode=comm_checksum_mode,
            cmd_response_mode=cmd_response_mode,
            store=store,
        ))

    # ── utility helpers ───────────────────────────────────────────────────────

    def wait_in_position(self, timeout_s: float = 10.0, poll_interval_s: float = 0.1) -> bool:
        """Poll until the motor reports in-position or timeout (电机到位标志).

        The GUI displays a real-time 电机到位标志 indicator; this helper
        provides the equivalent programmatic wait.

        Args:
            timeout_s: Maximum seconds to wait (default 10.0).
            poll_interval_s: Time between status polls in seconds (default 0.1).

        Returns:
            True if in-position flag was set before timeout; False on timeout.
        """
        import time
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                status = self.read_status()
                if status.get("in_position"):
                    return True
            except Exception:
                pass
            time.sleep(poll_interval_s)
        return False

    @staticmethod
    def list_ports() -> list:
        """Return available serial port names (equivalent to GUI 串口 dropdown).

        Returns:
            List of port name strings, e.g. ["COM3", "COM15"] on Windows
            or ["/dev/ttyUSB0"] on Linux.
        """
        return Transport.list_available_ports()

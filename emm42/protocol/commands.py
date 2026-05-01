"""Command frame constructors for all known EMM42 V5 function codes.

Each method returns a list[int] payload (WITHOUT checksum).
Pass the result to build_frame() to get the wire bytes.

Sources:
  - 说明书/Emm42_V5.0新增功能说明.txt
  - 例程_树莓派Python/树莓派Python_串口通讯控制.py
"""
from __future__ import annotations
from typing import List


class Cmd:
    """Static frame-payload builders — one method per command."""

    # ── motion control ───────────────────────────────────────────────────────

    @staticmethod
    def enable(addr: int, state: bool, sync: bool = False) -> List[int]:
        """Enable (state=True) or disable (state=False) the motor.

        Frame: addr 0xF3 0xAB state sync
        """
        return [addr, 0xF3, 0xAB, 0x01 if state else 0x00, 0x01 if sync else 0x00]

    @staticmethod
    def velocity(
        addr: int,
        direction: int,
        rpm: int,
        acceleration: int,
        sync: bool = False,
    ) -> List[int]:
        """Velocity (speed) control.

        Args:
            addr: Device address (1–255).
            direction: 0 = CW, 1 = CCW.
            rpm: Target speed in RPM (0–5000).
            acceleration: 0 = instant start; 1–255.
            sync: Multi-axis synchronous motion flag.

        Frame: addr 0xF6 dir velH velL acc sync
        """
        rpm = max(0, min(5000, rpm))
        return [
            addr,
            0xF6,
            direction & 0xFF,
            (rpm >> 8) & 0xFF,
            rpm & 0xFF,
            acceleration & 0xFF,
            0x01 if sync else 0x00,
        ]

    @staticmethod
    def position(
        addr: int,
        direction: int,
        rpm: int,
        acceleration: int,
        pulses: int,
        relative: bool = True,
        sync: bool = False,
    ) -> List[int]:
        """Position control.

        Args:
            addr: Device address.
            direction: 0 = CW, 1 = CCW.
            rpm: Speed in RPM (0–5000).
            acceleration: 0 = instant; 1–255.
            pulses: Number of pulses (0–4 294 967 295).
            relative: True = relative move, False = absolute.
            sync: Multi-axis synchronous motion flag.

        Frame: addr 0xFD dir velH velL acc p3 p2 p1 p0 rel sync
        """
        pulses = max(0, min(0xFFFFFFFF, pulses))
        return [
            addr,
            0xFD,
            direction & 0xFF,
            (rpm >> 8) & 0xFF,
            rpm & 0xFF,
            acceleration & 0xFF,
            (pulses >> 24) & 0xFF,
            (pulses >> 16) & 0xFF,
            (pulses >> 8) & 0xFF,
            pulses & 0xFF,
            0x01 if relative else 0x00,
            0x01 if sync else 0x00,
        ]

    @staticmethod
    def stop(addr: int, sync: bool = False) -> List[int]:
        """Immediate stop.

        Frame: addr 0xFE 0x98 sync
        """
        return [addr, 0xFE, 0x98, 0x01 if sync else 0x00]

    @staticmethod
    def sync_trigger(addr: int) -> List[int]:
        """Trigger multi-axis synchronous motion.

        Frame: addr 0xFF 0x66
        """
        return [addr, 0xFF, 0x66]

    # ── read parameter commands ──────────────────────────────────────────────

    @staticmethod
    def read_param(addr: int, code: int, aux: int | None = None) -> List[int]:
        """Generic read-parameter frame.

        Frame: addr code [aux]   (checksum appended by build_frame)

        Known codes and their aux bytes:
            0x1F           — firmware version
            0x20           — phase R/L
            0x21           — PID params
            0x24           — bus voltage (mV)
            0x27           — phase current (mA)
            0x31           — encoder (linearised)
            0x33           — target position (degrees)
            0x35           — real-time velocity (RPM)
            0x36           — real-time position (raw→degrees)
            0x37           — position error (degrees)
            0x3A           — status flags (enabled/homed/stalled)
            0x3B           — origin status (homing/failed)
            0x42  0x6C     — drive config
            0x43  0x7A     — system state
        """
        frame: List[int] = [addr, code]
        if aux is not None:
            frame.append(aux)
        return frame

    @staticmethod
    def read_position(addr: int) -> List[int]:
        """Shortcut: read real-time position (0x36)."""
        return Cmd.read_param(addr, 0x36)

    @staticmethod
    def read_velocity(addr: int) -> List[int]:
        """Shortcut: read real-time velocity (0x35)."""
        return Cmd.read_param(addr, 0x35)

    @staticmethod
    def read_status(addr: int) -> List[int]:
        """Shortcut: read status flags (0x3A)."""
        return Cmd.read_param(addr, 0x3A)

    @staticmethod
    def read_bus_voltage(addr: int) -> List[int]:
        """Shortcut: read bus voltage in mV (0x24)."""
        return Cmd.read_param(addr, 0x24)

    @staticmethod
    def read_phase_current(addr: int) -> List[int]:
        """Shortcut: read phase current in mA (0x27)."""
        return Cmd.read_param(addr, 0x27)

    @staticmethod
    def read_drive_config(addr: int) -> List[int]:
        """Shortcut: read drive config (0x42 + aux 0x6C)."""
        return Cmd.read_param(addr, 0x42, aux=0x6C)

    @staticmethod
    def read_system_state(addr: int) -> List[int]:
        """Shortcut: read system state (0x43 + aux 0x7A)."""
        return Cmd.read_param(addr, 0x43, aux=0x7A)

    # ── origin / homing ──────────────────────────────────────────────────────

    @staticmethod
    def origin_set_zero(addr: int, store: bool = False) -> List[int]:
        """Set current position as single-turn origin zero.

        Frame: addr 0x93 0x88 store
        """
        return [addr, 0x93, 0x88, 0x01 if store else 0x00]

    @staticmethod
    def origin_trigger_return(addr: int, mode: int = 0, sync: bool = False) -> List[int]:
        """Trigger homing return.

        Args:
            mode: 0=nearest, 1=single-turn direction, 2=multi-turn, 3=limit.
            sync: Multi-axis sync flag.

        Frame: addr 0x9A mode sync
        """
        return [addr, 0x9A, mode & 0xFF, 0x01 if sync else 0x00]

    @staticmethod
    def origin_interrupt(addr: int) -> List[int]:
        """Force-interrupt and exit homing.

        Frame: addr 0x9C 0x48
        """
        return [addr, 0x9C, 0x48]

    @staticmethod
    def origin_modify_params(
        addr: int,
        store: bool,
        mode: int,
        direction: int,
        speed_rpm: int,
        timeout_ms: int,
        stall_speed_rpm: int,
        stall_current_ma: int,
        stall_time_ms: int,
        power_on_trigger: bool,
    ) -> List[int]:
        """Modify origin (homing) parameters.

        Frame: addr 0x4C 0xAE store mode dir velH velL tmH tmH2 tmL2 tmL
               slVH slVL slMH slML slTH slTL potF
        """
        t = max(0, min(0xFFFFFFFF, timeout_ms))
        sv = max(0, min(0xFFFF, stall_speed_rpm))
        sm = max(0, min(0xFFFF, stall_current_ma))
        st = max(0, min(0xFFFF, stall_time_ms))
        vel = max(0, min(5000, speed_rpm))
        return [
            addr, 0x4C, 0xAE,
            0x01 if store else 0x00,
            mode & 0xFF,
            direction & 0xFF,
            (vel >> 8) & 0xFF, vel & 0xFF,
            (t >> 24) & 0xFF, (t >> 16) & 0xFF, (t >> 8) & 0xFF, t & 0xFF,
            (sv >> 8) & 0xFF, sv & 0xFF,
            (sm >> 8) & 0xFF, sm & 0xFF,
            (st >> 8) & 0xFF, st & 0xFF,
            0x01 if power_on_trigger else 0x00,
        ]

    # ── utility commands ─────────────────────────────────────────────────────

    @staticmethod
    def reset_current_pos_to_zero(addr: int) -> List[int]:
        """Reset current position angle to zero (no store).

        Frame: addr 0x0A 0x6D
        """
        return [addr, 0x0A, 0x6D]

    @staticmethod
    def reset_clog_protection(addr: int) -> List[int]:
        """Clear stall/clog protection.

        Frame: addr 0x0E 0x52
        """
        return [addr, 0x0E, 0x52]

    @staticmethod
    def modify_control_mode(addr: int, mode: int, store: bool = False) -> List[int]:
        """Change control mode (e.g. open/closed loop).

        Frame: addr 0x46 0x69 store mode
        """
        return [addr, 0x46, 0x69, 0x01 if store else 0x00, mode & 0xFF]

    # ── write / modify parameter commands ────────────────────────────────────

    @staticmethod
    def write_pid(
        addr: int,
        kp: int,
        ki: int,
        kd: int,
        store: bool = False,
    ) -> List[int]:
        """Write PID parameters (修改PID参数).

        Frame: addr 0xC3 0x1A store kpH kpL kiH kiL kdH kdL

        Args:
            kp: Proportional gain (uint16, default 32000).
            ki: Integral gain (uint16, default 100).
            kd: Derivative gain (uint16, default 32000).
            store: True = save to EEPROM.

        Source: ZDT EMM42 V5 protocol (community-confirmed: github.com/lyehe/ZDT_stepper).
        """
        return [
            addr, 0xC3, 0x1A,
            0x01 if store else 0x00,
            (kp >> 8) & 0xFF, kp & 0xFF,
            (ki >> 8) & 0xFF, ki & 0xFF,
            (kd >> 8) & 0xFF, kd & 0xFF,
        ]

    @staticmethod
    def write_id_address(addr: int, new_addr: int) -> List[int]:
        """Write new device ID address (修改驱动ID地址).

        Frame: addr 0xAE 0x4B new_addr

        The new address takes effect immediately and is saved to EEPROM.
        Valid range: 1–255 (0 = broadcast, not recommended as ID).

        Source: ZDT EMM42 V5 protocol (community-confirmed: github.com/lyehe/ZDT_stepper).
        """
        return [addr, 0xAE, 0x4B, new_addr & 0xFF]

    @staticmethod
    def write_drive_config(
        addr: int,
        motor_type: int,
        pulse_ctrl_mode: int,
        comm_port_multiplex: int,
        en_pin_level: int,
        dir_pin_direction: int,
        microstep: int,
        microstep_interpolation: int,
        auto_screen_off: int,
        open_loop_current_ma: int,
        stall_protection_enable: int,
        stall_detect_rpm: int,
        stall_detect_current_ma: int,
        stall_detect_time_ms: int,
        stall_max_current_ma: int,
        position_window_steps: int,
        max_output_voltage_mv: int,
        uart_baudrate_index: int,
        can_rate_index: int,
        comm_checksum_mode: int,
        cmd_response_mode: int,
        store: bool = True,
    ) -> List[int]:
        """Write full drive configuration block (修改驱动参数).

        Frame: addr 0x48 0x46 store [20 config fields]

        Field widths confirmed from GUI 驱动参数 read-back layout:
            motor_type            (1B)  — 电机类型
            pulse_ctrl_mode       (1B)  — 脉冲控制模式
            comm_port_multiplex   (1B)  — 通讯端口复用
            en_pin_level          (1B)  — En脚有效电平
            dir_pin_direction     (1B)  — Dir脚正方向
            microstep             (1B)  — 细分 (1/2/4/8/16/32/64/128/256)
            microstep_interpolation (1B) — 细分插补 0=off 1=on
            auto_screen_off       (1B)  — 自动熄屏
            open_loop_current_ma  (2B)  — 开环工作电流 mA
            stall_protection_enable (1B) — 堵转保护功能
            stall_detect_rpm      (2B)  — 堵转检测转速 RPM
            stall_detect_current_ma (2B) — 堵转检测电流 mA
            stall_detect_time_ms  (2B)  — 堵转检测时间 ms
            stall_max_current_ma  (2B)  — 堵转最大电流 mA
            position_window_steps (2B)  — 位置到达窗口 steps
            max_output_voltage_mv (2B)  — 最大输出电压 mV
            uart_baudrate_index   (1B)  — 串口波特率档位
            can_rate_index        (1B)  — CAN速率档位
            comm_checksum_mode    (1B)  — 通讯校验方式
            cmd_response_mode     (1B)  — 控制命令应答

        Source: ZDT EMM42 V5 protocol (community-confirmed: github.com/lyehe/ZDT_stepper).
        IMPORTANT: Verify by reading read_drive_config_raw() first and comparing byte layout
        before issuing a write, as field order must match device firmware exactly.
        """
        return [
            addr, 0x48, 0x46,
            0x01 if store else 0x00,
            motor_type & 0xFF,
            pulse_ctrl_mode & 0xFF,
            comm_port_multiplex & 0xFF,
            en_pin_level & 0xFF,
            dir_pin_direction & 0xFF,
            microstep & 0xFF,
            microstep_interpolation & 0xFF,
            auto_screen_off & 0xFF,
            (open_loop_current_ma >> 8) & 0xFF, open_loop_current_ma & 0xFF,
            stall_protection_enable & 0xFF,
            (stall_detect_rpm >> 8) & 0xFF, stall_detect_rpm & 0xFF,
            (stall_detect_current_ma >> 8) & 0xFF, stall_detect_current_ma & 0xFF,
            (stall_detect_time_ms >> 8) & 0xFF, stall_detect_time_ms & 0xFF,
            (stall_max_current_ma >> 8) & 0xFF, stall_max_current_ma & 0xFF,
            (position_window_steps >> 8) & 0xFF, position_window_steps & 0xFF,
            (max_output_voltage_mv >> 8) & 0xFF, max_output_voltage_mv & 0xFF,
            uart_baudrate_index & 0xFF,
            can_rate_index & 0xFF,
            comm_checksum_mode & 0xFF,
            cmd_response_mode & 0xFF,
        ]

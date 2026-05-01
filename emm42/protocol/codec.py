"""Frame codec: checksum computation and frame building.

Supports three checksum modes (device-configurable):
  - FIXED_6B : every frame ends with 0x6B  (factory default)
  - XOR      : XOR of all payload bytes
  - CRC8     : CRC-8 via lookup table from manual
  - MODBUS   : CRC-16/Modbus (added in V1.2.2)

Source: 说明书/Emm_V5.0通讯校验算法.txt
"""
from __future__ import annotations
import enum
import struct
from typing import Sequence

# ── CRC-8 table extracted from manual ───────────────────────────────────────
_CRC8_TABLE: tuple[int, ...] = (
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
)


class ChecksumMode(enum.Enum):
    """Checksum mode — must match device configuration (default: FIXED_6B).

    The GUI 串口设置 → 校验字节 dropdown exposes three options:
        FIXED_6B  (0x6B, factory default)
        XOR
        CRC8

    MODBUS is NOT a GUI option. V1.2.2 added Modbus-RTU support to the
    device firmware for PLC / industrial-master integration. If you are
    communicating with the device from a Modbus master (not using this
    GUI-parity library), pass ChecksumMode.MODBUS to use CRC-16/Modbus
    framing instead of the proprietary protocol.
    """
    FIXED_6B = "fixed"
    XOR = "xor"
    CRC8 = "crc8"
    MODBUS = "modbus"  # device firmware only — not used by 上位机 GUI


def compute_checksum(payload: bytes, mode: ChecksumMode) -> bytes:
    """Return the checksum byte(s) for *payload* (excludes checksum itself).

    Args:
        payload: All frame bytes before the checksum field.
        mode: Checksum algorithm matching device setting.

    Returns:
        1-byte ``bytes`` for FIXED_6B / XOR / CRC8; 2-byte big-endian for MODBUS.
    """
    if mode is ChecksumMode.FIXED_6B:
        return b"\x6B"

    if mode is ChecksumMode.XOR:
        result = payload[0]
        for b in payload[1:]:
            result ^= b
        return bytes([result])

    if mode is ChecksumMode.CRC8:
        crc = payload[0]
        for b in payload[1:]:
            crc = _CRC8_TABLE[crc ^ b]
        return bytes([crc])

    if mode is ChecksumMode.MODBUS:
        crc = 0xFFFF
        for b in payload:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        # Modbus: low byte first
        return struct.pack("<H", crc)

    raise ValueError(f"Unknown checksum mode: {mode}")


def build_frame(payload: Sequence[int], mode: ChecksumMode = ChecksumMode.FIXED_6B) -> bytes:
    """Append checksum to *payload* bytes and return the complete frame.

    Args:
        payload: Frame bytes WITHOUT trailing checksum (addr + func + data…).
        mode: Checksum algorithm.

    Returns:
        Complete frame as ``bytes``.
    """
    raw = bytes(payload)
    return raw + compute_checksum(raw, mode)


def decode_position_response(frame: bytes) -> float:
    """Decode a 0x36 real-time position response frame to degrees.

    Frame layout (8 bytes):
        [addr, 0x36, sign, b3, b2, b1, b0, checksum]

    Returns:
        Position in degrees; negative if sign byte != 0.

    Source: 例程_树莓派Python/树莓派Python_串口通讯控制.py  Real_time_location()
    """
    if len(frame) < 7:
        raise ValueError(f"Position frame too short: {len(frame)} bytes")
    sign_byte = frame[2]
    raw = struct.unpack(">I", frame[3:7])[0]
    degrees = raw * 360.0 / 65536.0
    return -degrees if sign_byte != 0 else degrees


def decode_angle_response(frame: bytes) -> float:
    """Generic signed angle decoder for 0x33 (target pos) and 0x37 (position error).

    Frame layout (8 bytes):
        [addr, code, sign, b3, b2, b1, b0, checksum]

    Same encoding as 0x36 position response.
    """
    if len(frame) < 7:
        raise ValueError(f"Angle frame too short: {len(frame)} bytes")
    sign_byte = frame[2]
    raw = struct.unpack(">I", frame[3:7])[0]
    degrees = raw * 360.0 / 65536.0
    return -degrees if sign_byte != 0 else degrees


def decode_encoder_response(frame: bytes) -> int:
    """Decode 0x31 encoder (linearised) response.

    Frame layout (8 bytes):
        [addr, 0x31, sign, b3, b2, b1, b0, checksum]

    Returns raw signed encoder count.
    """
    if len(frame) < 7:
        raise ValueError(f"Encoder frame too short: {len(frame)} bytes")
    sign_byte = frame[2]
    raw = struct.unpack(">I", frame[3:7])[0]
    return -raw if sign_byte != 0 else raw


def decode_uint16_response(frame: bytes) -> int:
    """Decode a 2-byte big-endian unsigned response (bus voltage, phase current).

    Frame layout (5 bytes):
        [addr, code, highByte, lowByte, checksum]
    """
    if len(frame) < 4:
        raise ValueError(f"uint16 frame too short: {len(frame)} bytes")
    return (frame[2] << 8) | frame[3]


def decode_velocity_response(frame: bytes) -> int:
    """Decode a 0x35 real-time velocity response to signed RPM.

    Frame layout (6 bytes):
        [addr, 0x35, sign, velH, velL, checksum]

    Returns:
        Velocity in RPM; negative if sign byte != 0 (CCW).

    Source: 例程_Arduino/串口_RS232_RS485通讯控制/Arduino_串口通讯__读取电机实时转速/
            rxCmd[2]=sign, rxCmd[3]=velH, rxCmd[4]=velL, rxCount==6 check
    """
    if len(frame) < 5:
        raise ValueError(f"Velocity frame too short: {len(frame)} bytes")
    sign_byte = frame[2]
    rpm = (frame[3] << 8) | frame[4]
    return -rpm if sign_byte != 0 else rpm


def decode_firmware_response(frame: bytes) -> dict:
    """Decode 0x1F firmware version response.

    Frame layout (7 bytes):
        [addr, 0x1F, hw_major, hw_minor, fw_major, fw_minor, checksum]

    Returns:
        {"hw_version": "X.Y", "fw_version": "X.Y"}
    """
    if len(frame) < 6:
        raise ValueError(f"Firmware frame too short: {len(frame)} bytes")
    return {
        "hw_version": f"{frame[2]}.{frame[3]}",
        "fw_version": f"{frame[4]}.{frame[5]}",
    }


def decode_phase_rl_response(frame: bytes) -> dict:
    """Decode 0x20 phase resistance and inductance response.

    Frame layout (7 bytes):
        [addr, 0x20, R_high, R_low, L_high, L_low, checksum]

    Returns:
        {"resistance_mohm": int, "inductance_uh": int}
    """
    if len(frame) < 6:
        raise ValueError(f"Phase R/L frame too short: {len(frame)} bytes")
    return {
        "resistance_mohm": (frame[2] << 8) | frame[3],
        "inductance_uh": (frame[4] << 8) | frame[5],
    }


def decode_pid_response(frame: bytes) -> dict:
    """Decode 0x21 PID parameter response.

    Frame layout (9 bytes):
        [addr, 0x21, Kp_H, Kp_L, Ki_H, Ki_L, Kd_H, Kd_L, checksum]

    Each value is uint16 big-endian. Default values per V1.2 update: Kp=Kd=32000, Ki=100.
    """
    if len(frame) < 8:
        raise ValueError(f"PID frame too short: {len(frame)} bytes")
    return {
        "kp": (frame[2] << 8) | frame[3],
        "ki": (frame[4] << 8) | frame[5],
        "kd": (frame[6] << 8) | frame[7],
    }


def decode_status_response(frame: bytes) -> dict:
    """Decode 0x3A status flags response.

    Frame layout (4 bytes):
        [addr, 0x3A, flags, checksum]

    Bit map (from GUI labels — 使能状态/电机到位/电机堵转/堵转保护):
        bit 0: enabled       (使能状态标志)
        bit 1: in_position   (电机到位标志)
        bit 2: stalled       (电机堵转标志)
        bit 3: stall_protection  (堵转保护标志)
    """
    if len(frame) < 3:
        raise ValueError(f"Status frame too short: {len(frame)} bytes")
    f = frame[2]
    return {
        "enabled":          bool(f & 0x01),
        "in_position":      bool(f & 0x02),
        "stalled":          bool(f & 0x04),
        "stall_protection": bool(f & 0x08),
    }


def decode_origin_status_response(frame: bytes) -> dict:
    """Decode 0x3B origin/homing status response.

    Frame layout (4 bytes):
        [addr, 0x3B, flags, checksum]

    Bit map (from GUI labels — 正在回零/回零失败):
        bit 0: is_homing      (正在回零标志)
        bit 1: homing_failed  (回零失败标志)
    """
    if len(frame) < 3:
        raise ValueError(f"Origin status frame too short: {len(frame)} bytes")
    f = frame[2]
    return {
        "is_homing":     bool(f & 0x01),
        "homing_failed": bool(f & 0x02),
    }


def decode_drive_config_response(frame: bytes) -> dict:
    """Decode 0x42 drive configuration read response into named fields.

    Frame layout (30 bytes):
        [addr, 0x42,
         motor_type(1), pulse_ctrl_mode(1), comm_port_multiplex(1),
         en_pin_level(1), dir_pin_direction(1), microstep(1),
         microstep_interpolation(1), auto_screen_off(1),
         open_loop_current_H(1), open_loop_current_L(1),
         stall_protection_enable(1),
         stall_detect_rpm_H(1), stall_detect_rpm_L(1),
         stall_detect_current_H(1), stall_detect_current_L(1),
         stall_detect_time_H(1), stall_detect_time_L(1),
         stall_max_current_H(1), stall_max_current_L(1),
         position_window_H(1), position_window_L(1),
         max_voltage_H(1), max_voltage_L(1),
         uart_baudrate_index(1), can_rate_index(1),
         comm_checksum_mode(1), cmd_response_mode(1),
         checksum]

    Field order mirrors Cmd.write_drive_config() — covers all 20 GUI 驱动参数 fields.

    If the response is ≥44 bytes, homing parameters are also decoded from bytes 29–42
    (layout mirrors Cmd.origin_modify_params() write format).

    NOTE: Layout inferred from write-command field order. Verify on hardware.
    """
    if len(frame) < 29:
        raise ValueError(f"Drive config response too short: {len(frame)} bytes (expected ≥29)")
    d = frame[2:]
    result = {
        "motor_type":               d[0],
        "pulse_ctrl_mode":          d[1],
        "comm_port_multiplex":      d[2],
        "en_pin_level":             d[3],
        "dir_pin_direction":        d[4],
        "microstep":                d[5],
        "microstep_interpolation":  d[6],
        "auto_screen_off":          d[7],
        "open_loop_current_ma":     (d[8] << 8) | d[9],
        "stall_protection_enable":  d[10],
        "stall_detect_rpm":         (d[11] << 8) | d[12],
        "stall_detect_current_ma":  (d[13] << 8) | d[14],
        "stall_detect_time_ms":     (d[15] << 8) | d[16],
        "stall_max_current_ma":     (d[17] << 8) | d[18],
        "position_window_steps":    (d[19] << 8) | d[20],
        "max_output_voltage_mv":    (d[21] << 8) | d[22],
        "uart_baudrate_index":      d[23],
        "can_rate_index":           d[24],
        "comm_checksum_mode":       d[25],
        "cmd_response_mode":        d[26],
    }
    # If response is extended, also decode homing parameters (bytes 27–41)
    if len(d) >= 42:
        h = d[27:]
        result["origin_params"] = {
            "mode":             h[0],
            "direction":        h[1],
            "speed_rpm":        (h[2] << 8) | h[3],
            "timeout_ms":       (h[4] << 24) | (h[5] << 16) | (h[6] << 8) | h[7],
            "stall_speed_rpm":  (h[8] << 8) | h[9],
            "stall_current_ma": (h[10] << 8) | h[11],
            "stall_time_ms":    (h[12] << 8) | h[13],
            "power_on_trigger": bool(h[14]),
        }
    return result


def decode_system_state_response(frame: bytes) -> dict:
    """Decode 0x43 system state read response — all 13 GUI monitoring fields.

    Frame layout (32 bytes):
        [addr, 0x43,
         bus_voltage_H(1), bus_voltage_L(1),           # mV  uint16
         phase_current_H(1), phase_current_L(1),       # mA  uint16
         enc_sign(1), enc_b3(1), enc_b2(1), enc_b1(1), enc_b0(1),  # signed int32
         tpos_sign(1), tp_b3(1)..tp_b0(1),             # signed → degrees (×360/65536)
         vel_sign(1), vel_H(1), vel_L(1),              # signed RPM
         pos_sign(1), pos_b3(1)..pos_b0(1),            # signed → degrees
         perr_sign(1), pe_b3(1)..pe_b0(1),             # signed → degrees
         origin_flags(1),   # bit0=is_homing, bit1=homing_failed
         motor_flags(1),    # bit0=enabled, bit1=in_pos, bit2=stalled, bit3=stall_prot
         checksum]

    Covers all fields shown in GUI 读取系统状态 panel (13 labelled values).

    NOTE: Layout inferred from GUI field order and individual read command formats.
    Verify on hardware.
    """
    if len(frame) < 31:
        raise ValueError(f"System state response too short: {len(frame)} bytes (expected ≥31)")
    d = frame[2:]

    def _angle(sign: int, b3: int, b2: int, b1: int, b0: int) -> float:
        raw = struct.unpack(">I", bytes([b3, b2, b1, b0]))[0]
        deg = raw * 360.0 / 65536.0
        return -deg if sign != 0 else deg

    bus_voltage    = (d[0] << 8) | d[1]
    phase_current  = (d[2] << 8) | d[3]
    enc_raw        = struct.unpack(">I", bytes([d[5], d[6], d[7], d[8]]))[0]
    encoder        = -enc_raw if d[4] != 0 else enc_raw
    target_pos_deg = _angle(d[9],  d[10], d[11], d[12], d[13])
    vel_rpm        = (d[15] << 8) | d[16]
    vel_rpm        = -vel_rpm if d[14] != 0 else vel_rpm
    pos_deg        = _angle(d[17], d[18], d[19], d[20], d[21])
    perr_deg       = _angle(d[22], d[23], d[24], d[25], d[26])
    oflags         = d[27]
    mflags         = d[28]

    return {
        # 电气量
        "bus_voltage_mv":      bus_voltage,
        "phase_current_ma":    phase_current,
        "encoder":             encoder,
        # 位置 / 速度
        "target_position_deg": target_pos_deg,
        "velocity_rpm":        vel_rpm,
        "position_deg":        pos_deg,
        "position_error_deg":  perr_deg,
        # 回零标志 (正在回零/回零失败)
        "is_homing":           bool(oflags & 0x01),
        "homing_failed":       bool(oflags & 0x02),
        # 电机状态标志 (使能/到位/堵转/堵转保护)
        "enabled":             bool(mflags & 0x01),
        "in_position":         bool(mflags & 0x02),
        "stalled":             bool(mflags & 0x04),
        "stall_protection":    bool(mflags & 0x08),
    }

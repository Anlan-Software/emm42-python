"""Tests for emm42.protocol.codec and commands — run with: python -m pytest tests/"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import struct
import pytest
from emm42.protocol.codec import (
    ChecksumMode,
    compute_checksum,
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
from emm42.protocol.commands import Cmd


# ── Checksum modes ────────────────────────────────────────────────────────────

class TestFixed6B:
    def test_returns_6b(self):
        assert compute_checksum(b"\x01\x36", ChecksumMode.FIXED_6B) == b"\x6B"

    def test_frame_ends_with_6b(self):
        frame = build_frame([0x01, 0x36], ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0x36, 0x6B])


class TestXOR:
    def test_single_byte_payload(self):
        assert compute_checksum(b"\xAB", ChecksumMode.XOR) == b"\xAB"

    def test_two_bytes(self):
        assert compute_checksum(b"\x01\x36", ChecksumMode.XOR) == bytes([0x01 ^ 0x36])

    def test_known_frame(self):
        frame = build_frame([0x01, 0x36], ChecksumMode.XOR)
        assert frame == bytes([0x01, 0x36, 0x37])


class TestCRC8:
    def test_known_value(self):
        result = compute_checksum(b"\x01", ChecksumMode.CRC8)
        assert result == b"\x5E"

    def test_empty_raises(self):
        with pytest.raises((IndexError, KeyError)):
            compute_checksum(b"", ChecksumMode.CRC8)

    def test_frame_length(self):
        frame = build_frame([0x01, 0x36], ChecksumMode.CRC8)
        assert len(frame) == 3
        assert frame[:2] == bytes([0x01, 0x36])


class TestModbus:
    def test_returns_two_bytes(self):
        cs = compute_checksum(b"\x01\x03\x00\x6B", ChecksumMode.MODBUS)
        assert len(cs) == 2

    def test_deterministic(self):
        a = compute_checksum(b"\x01\x03\x00\x6B", ChecksumMode.MODBUS)
        b = compute_checksum(b"\x01\x03\x00\x6B", ChecksumMode.MODBUS)
        assert a == b


# ── Response decoders ─────────────────────────────────────────────────────────

class TestDecodePosition:
    def test_positive_180deg(self):
        raw_bytes = struct.pack(">I", 32768)
        frame = bytes([0x01, 0x36, 0x00]) + raw_bytes + bytes([0x6B])
        assert decode_position_response(frame) == pytest.approx(180.0)

    def test_negative(self):
        raw_bytes = struct.pack(">I", 32768)
        frame = bytes([0x01, 0x36, 0x01]) + raw_bytes + bytes([0x6B])
        assert decode_position_response(frame) == pytest.approx(-180.0)

    def test_zero(self):
        raw_bytes = struct.pack(">I", 0)
        frame = bytes([0x01, 0x36, 0x00]) + raw_bytes + bytes([0x6B])
        assert decode_position_response(frame) == pytest.approx(0.0)

    def test_full_rotation(self):
        raw_bytes = struct.pack(">I", 65536)
        frame = bytes([0x01, 0x36, 0x00]) + raw_bytes + bytes([0x6B])
        assert decode_position_response(frame) == pytest.approx(360.0)

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_position_response(b"\x01\x36")


class TestDecodeAngle:
    def test_positive_90deg(self):
        raw_bytes = struct.pack(">I", 16384)
        frame = bytes([0x01, 0x33, 0x00]) + raw_bytes + bytes([0x6B])
        assert decode_angle_response(frame) == pytest.approx(90.0)

    def test_negative(self):
        raw_bytes = struct.pack(">I", 16384)
        frame = bytes([0x01, 0x37, 0x01]) + raw_bytes + bytes([0x6B])
        assert decode_angle_response(frame) == pytest.approx(-90.0)

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_angle_response(b"\x01\x33")


class TestDecodeEncoder:
    def test_positive(self):
        raw_bytes = struct.pack(">I", 1000)
        frame = bytes([0x01, 0x31, 0x00]) + raw_bytes + bytes([0x6B])
        assert decode_encoder_response(frame) == 1000

    def test_negative(self):
        raw_bytes = struct.pack(">I", 1000)
        frame = bytes([0x01, 0x31, 0x01]) + raw_bytes + bytes([0x6B])
        assert decode_encoder_response(frame) == -1000

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_encoder_response(b"\x01\x31")


class TestDecodeUint16:
    def test_bus_voltage(self):
        frame = bytes([0x01, 0x24, 0x0F, 0xA0, 0x6B])  # 4000 mV
        assert decode_uint16_response(frame) == 4000

    def test_zero(self):
        frame = bytes([0x01, 0x24, 0x00, 0x00, 0x6B])
        assert decode_uint16_response(frame) == 0

    def test_max(self):
        frame = bytes([0x01, 0x27, 0xFF, 0xFF, 0x6B])
        assert decode_uint16_response(frame) == 65535

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_uint16_response(b"\x01\x24")


class TestDecodeVelocity:
    """Confirmed from Arduino 读取电机实时转速 example:
    rxCount==6, rxCmd[2]=sign, rxCmd[3]=velH, rxCmd[4]=velL; negate if sign!=0.
    """

    def test_positive_500rpm(self):
        frame = bytes([0x01, 0x35, 0x00, 0x01, 0xF4, 0x6B])
        assert decode_velocity_response(frame) == 500

    def test_negative_ccw(self):
        frame = bytes([0x01, 0x35, 0x01, 0x01, 0xF4, 0x6B])
        assert decode_velocity_response(frame) == -500

    def test_zero(self):
        frame = bytes([0x01, 0x35, 0x00, 0x00, 0x00, 0x6B])
        assert decode_velocity_response(frame) == 0

    def test_5000rpm(self):
        frame = bytes([0x01, 0x35, 0x00, 0x13, 0x88, 0x6B])
        assert decode_velocity_response(frame) == 5000

    def test_ccw_nonzero_sign(self):
        frame = bytes([0x01, 0x35, 0xFF, 0x13, 0x88, 0x6B])
        assert decode_velocity_response(frame) == -5000

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_velocity_response(b"\x01\x35\x00")


class TestDecodeFirmware:
    def test_basic(self):
        frame = bytes([0x01, 0x1F, 0x01, 0x02, 0x05, 0x00, 0x6B])
        result = decode_firmware_response(frame)
        assert result == {"hw_version": "1.2", "fw_version": "5.0"}

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_firmware_response(b"\x01\x1F\x01")


class TestDecodePhaseRL:
    def test_basic(self):
        frame = bytes([0x01, 0x20, 0x01, 0xF4, 0x04, 0xB0, 0x6B])  # 500 mohm, 1200 uH
        result = decode_phase_rl_response(frame)
        assert result == {"resistance_mohm": 500, "inductance_uh": 1200}

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_phase_rl_response(b"\x01\x20\x01")


class TestDecodePID:
    def test_defaults(self):
        # Default values per V1.2 update: Kp=32000, Ki=100, Kd=32000
        frame = bytes([0x01, 0x21,
                       0x7D, 0x00,   # kp=32000
                       0x00, 0x64,   # ki=100
                       0x7D, 0x00,   # kd=32000
                       0x6B])
        result = decode_pid_response(frame)
        assert result == {"kp": 32000, "ki": 100, "kd": 32000}

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_pid_response(b"\x01\x21\x00")


class TestDecodeStatus:
    def test_all_off(self):
        frame = bytes([0x01, 0x3A, 0x00, 0x6B])
        result = decode_status_response(frame)
        assert result == {"enabled": False, "in_position": False,
                          "stalled": False, "stall_protection": False}

    def test_enabled_bit(self):
        frame = bytes([0x01, 0x3A, 0x01, 0x6B])
        result = decode_status_response(frame)
        assert result["enabled"] is True
        assert result["in_position"] is False

    def test_all_on(self):
        frame = bytes([0x01, 0x3A, 0x0F, 0x6B])
        result = decode_status_response(frame)
        assert all(result.values())

    def test_in_position_bit(self):
        frame = bytes([0x01, 0x3A, 0x02, 0x6B])
        result = decode_status_response(frame)
        assert result["in_position"] is True
        assert result["enabled"] is False

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_status_response(b"\x01\x3A")


class TestDecodeOriginStatus:
    def test_idle(self):
        frame = bytes([0x01, 0x3B, 0x00, 0x6B])
        result = decode_origin_status_response(frame)
        assert result == {"is_homing": False, "homing_failed": False}

    def test_homing(self):
        frame = bytes([0x01, 0x3B, 0x01, 0x6B])
        result = decode_origin_status_response(frame)
        assert result["is_homing"] is True
        assert result["homing_failed"] is False

    def test_failed(self):
        frame = bytes([0x01, 0x3B, 0x02, 0x6B])
        result = decode_origin_status_response(frame)
        assert result["is_homing"] is False
        assert result["homing_failed"] is True

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_origin_status_response(b"\x01\x3B")


# ── Drive config decoder ──────────────────────────────────────────────────────

class TestDecodeDriveConfig:
    def _make_frame(self, **overrides):
        """Build a minimal 30-byte drive-config response frame."""
        defaults = dict(
            motor_type=0, pulse_ctrl_mode=1, comm_port_multiplex=0,
            en_pin_level=0, dir_pin_direction=0, microstep=16,
            microstep_interpolation=1, auto_screen_off=0,
            open_loop_current_ma=1000, stall_protection_enable=1,
            stall_detect_rpm=200, stall_detect_current_ma=2000,
            stall_detect_time_ms=200, stall_max_current_ma=3000,
            position_window_steps=10, max_output_voltage_mv=5000,
            uart_baudrate_index=3, can_rate_index=0,
            comm_checksum_mode=0, cmd_response_mode=1,
        )
        defaults.update(overrides)
        d = defaults
        payload = bytes([
            d["motor_type"], d["pulse_ctrl_mode"], d["comm_port_multiplex"],
            d["en_pin_level"], d["dir_pin_direction"], d["microstep"],
            d["microstep_interpolation"], d["auto_screen_off"],
            (d["open_loop_current_ma"] >> 8) & 0xFF, d["open_loop_current_ma"] & 0xFF,
            d["stall_protection_enable"],
            (d["stall_detect_rpm"] >> 8) & 0xFF, d["stall_detect_rpm"] & 0xFF,
            (d["stall_detect_current_ma"] >> 8) & 0xFF, d["stall_detect_current_ma"] & 0xFF,
            (d["stall_detect_time_ms"] >> 8) & 0xFF, d["stall_detect_time_ms"] & 0xFF,
            (d["stall_max_current_ma"] >> 8) & 0xFF, d["stall_max_current_ma"] & 0xFF,
            (d["position_window_steps"] >> 8) & 0xFF, d["position_window_steps"] & 0xFF,
            (d["max_output_voltage_mv"] >> 8) & 0xFF, d["max_output_voltage_mv"] & 0xFF,
            d["uart_baudrate_index"], d["can_rate_index"],
            d["comm_checksum_mode"], d["cmd_response_mode"],
        ])
        return bytes([0x01, 0x42]) + payload + bytes([0x6B])

    def test_basic_fields(self):
        frame = self._make_frame()
        result = decode_drive_config_response(frame)
        assert result["microstep"] == 16
        assert result["open_loop_current_ma"] == 1000
        assert result["stall_detect_rpm"] == 200
        assert result["max_output_voltage_mv"] == 5000
        assert result["cmd_response_mode"] == 1

    def test_all_20_fields_present(self):
        frame = self._make_frame()
        result = decode_drive_config_response(frame)
        expected_keys = {
            "motor_type", "pulse_ctrl_mode", "comm_port_multiplex",
            "en_pin_level", "dir_pin_direction", "microstep",
            "microstep_interpolation", "auto_screen_off", "open_loop_current_ma",
            "stall_protection_enable", "stall_detect_rpm", "stall_detect_current_ma",
            "stall_detect_time_ms", "stall_max_current_ma", "position_window_steps",
            "max_output_voltage_mv", "uart_baudrate_index", "can_rate_index",
            "comm_checksum_mode", "cmd_response_mode",
        }
        assert expected_keys.issubset(result.keys())

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_drive_config_response(b"\x01\x42\x00")

    def test_no_origin_params_for_short_response(self):
        frame = self._make_frame()
        result = decode_drive_config_response(frame)
        assert "origin_params" not in result


# ── System state decoder ──────────────────────────────────────────────────────

class TestDecodeSystemState:
    def _make_frame(self, bus_mv=4800, cur_ma=500, enc=1000,
                    tpos_deg=90.0, vel_rpm=300, pos_deg=45.0, perr_deg=0.0,
                    oflags=0x00, mflags=0x01):
        """Build a minimal 32-byte system-state response frame."""
        def enc_angle(deg):
            raw = int(abs(deg) * 65536 / 360) & 0xFFFFFFFF
            sign = 0x01 if deg < 0 else 0x00
            return bytes([sign]) + struct.pack(">I", raw)

        def enc_vel(rpm):
            sign = 0x01 if rpm < 0 else 0x00
            v = abs(rpm) & 0xFFFF
            return bytes([sign, (v >> 8) & 0xFF, v & 0xFF])

        payload = (
            struct.pack(">H", bus_mv) +
            struct.pack(">H", cur_ma) +
            enc_angle(enc if enc >= 0 else -enc) +   # encoder: sign then uint32
            enc_angle(tpos_deg) +
            enc_vel(vel_rpm) +
            enc_angle(pos_deg) +
            enc_angle(perr_deg) +
            bytes([oflags, mflags])
        )
        # Patch encoder sign manually
        enc_sign = 0x01 if enc < 0 else 0x00
        enc_bytes = bytes([enc_sign]) + struct.pack(">I", abs(enc))
        payload = struct.pack(">H", bus_mv) + struct.pack(">H", cur_ma) + enc_bytes + payload[9:]
        return bytes([0x01, 0x43]) + payload + bytes([0x6B])

    def test_bus_voltage(self):
        frame = self._make_frame(bus_mv=4800)
        result = decode_system_state_response(frame)
        assert result["bus_voltage_mv"] == 4800

    def test_enabled_flag(self):
        frame = self._make_frame(mflags=0x01)
        result = decode_system_state_response(frame)
        assert result["enabled"] is True
        assert result["stalled"] is False

    def test_all_13_keys_present(self):
        frame = self._make_frame()
        result = decode_system_state_response(frame)
        expected_keys = {
            "bus_voltage_mv", "phase_current_ma", "encoder",
            "target_position_deg", "velocity_rpm", "position_deg",
            "position_error_deg", "is_homing", "homing_failed",
            "enabled", "in_position", "stalled", "stall_protection",
        }
        assert expected_keys == set(result.keys())

    def test_short_raises(self):
        with pytest.raises(ValueError):
            decode_system_state_response(b"\x01\x43\x00")


# ── Write command frame builders ──────────────────────────────────────────────

class TestWritePIDCmd:
    """Cmd.write_pid — 0xC3/0x1A, store + kp(2B) + ki(2B) + kd(2B)."""

    def test_frame_structure(self):
        payload = Cmd.write_pid(addr=1, kp=32000, ki=100, kd=32000, store=False)
        assert payload[0] == 0x01   # addr
        assert payload[1] == 0xC3   # func code
        assert payload[2] == 0x1A   # aux
        assert payload[3] == 0x00   # store=False
        assert (payload[4] << 8 | payload[5]) == 32000   # kp
        assert (payload[6] << 8 | payload[7]) == 100     # ki
        assert (payload[8] << 8 | payload[9]) == 32000   # kd
        assert len(payload) == 10

    def test_store_flag(self):
        payload = Cmd.write_pid(addr=1, kp=100, ki=10, kd=100, store=True)
        assert payload[3] == 0x01

    def test_round_trip_with_frame(self):
        payload = Cmd.write_pid(addr=1, kp=1000, ki=50, kd=2000, store=False)
        frame = build_frame(payload, ChecksumMode.FIXED_6B)
        assert frame[-1] == 0x6B
        assert len(frame) == 11


class TestWriteIDAddressCmd:
    """Cmd.write_id_address — 0xAE/0x4B + new_addr."""

    def test_frame_structure(self):
        payload = Cmd.write_id_address(addr=1, new_addr=5)
        assert payload == [0x01, 0xAE, 0x4B, 0x05]

    def test_broadcast_addr(self):
        payload = Cmd.write_id_address(addr=0, new_addr=2)
        assert payload[0] == 0x00
        assert payload[3] == 0x02

    def test_max_addr(self):
        payload = Cmd.write_id_address(addr=1, new_addr=255)
        assert payload[3] == 0xFF

    def test_frame_length(self):
        frame = build_frame(Cmd.write_id_address(1, 3), ChecksumMode.FIXED_6B)
        assert len(frame) == 5


class TestWriteDriveConfigCmd:
    """Cmd.write_drive_config — 0x48/0x46 + store + 20 fields."""

    def _default_payload(self):
        return Cmd.write_drive_config(
            addr=1,
            motor_type=0, pulse_ctrl_mode=0, comm_port_multiplex=0,
            en_pin_level=0, dir_pin_direction=0, microstep=16,
            microstep_interpolation=1, auto_screen_off=0,
            open_loop_current_ma=1000, stall_protection_enable=1,
            stall_detect_rpm=200, stall_detect_current_ma=2000,
            stall_detect_time_ms=200, stall_max_current_ma=3000,
            position_window_steps=10, max_output_voltage_mv=5000,
            uart_baudrate_index=3, can_rate_index=0,
            comm_checksum_mode=0, cmd_response_mode=1,
            store=True,
        )

    def test_func_code(self):
        payload = self._default_payload()
        assert payload[0] == 0x01   # addr
        assert payload[1] == 0x48   # func code
        assert payload[2] == 0x46   # aux
        assert payload[3] == 0x01   # store=True

    def test_microstep_field(self):
        payload = self._default_payload()
        assert payload[9] == 16     # microstep at index 9

    def test_open_loop_current_2b(self):
        payload = self._default_payload()
        current = (payload[12] << 8) | payload[13]
        assert current == 1000

    def test_total_length(self):
        payload = self._default_payload()
        assert len(payload) == 31   # 4 header + 13×1B + 7×2B = 31

    def test_store_false(self):
        payload = Cmd.write_drive_config(
            addr=1, motor_type=0, pulse_ctrl_mode=0, comm_port_multiplex=0,
            en_pin_level=0, dir_pin_direction=0, microstep=8,
            microstep_interpolation=0, auto_screen_off=0,
            open_loop_current_ma=500, stall_protection_enable=0,
            stall_detect_rpm=100, stall_detect_current_ma=1000,
            stall_detect_time_ms=100, stall_max_current_ma=2000,
            position_window_steps=5, max_output_voltage_mv=4000,
            uart_baudrate_index=2, can_rate_index=0,
            comm_checksum_mode=0, cmd_response_mode=0,
            store=False,
        )
        assert payload[3] == 0x00

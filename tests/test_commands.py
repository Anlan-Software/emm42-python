"""Tests for emm42.protocol.commands frame builders."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from emm42.protocol.commands import Cmd
from emm42.protocol.codec import build_frame, ChecksumMode


ADDR = 0x01


class TestEnable:
    def test_enable_frame(self):
        frame = build_frame(Cmd.enable(ADDR, state=True), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0xF3, 0xAB, 0x01, 0x00, 0x6B])

    def test_disable_frame(self):
        frame = build_frame(Cmd.enable(ADDR, state=False), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0xF3, 0xAB, 0x00, 0x00, 0x6B])

    def test_enable_sync(self):
        frame = build_frame(Cmd.enable(ADDR, state=True, sync=True), ChecksumMode.FIXED_6B)
        assert frame[4] == 0x01


class TestVelocity:
    def test_basic_frame_length(self):
        payload = Cmd.velocity(ADDR, direction=0, rpm=300, acceleration=50)
        assert len(payload) == 7

    def test_rpm_encoding(self):
        payload = Cmd.velocity(ADDR, direction=0, rpm=0x0190, acceleration=0)
        # 0x0190 = 400 RPM
        assert payload[3] == 0x01
        assert payload[4] == 0x90

    def test_rpm_clamp_max(self):
        payload = Cmd.velocity(ADDR, direction=0, rpm=9999, acceleration=0)
        vel = (payload[3] << 8) | payload[4]
        assert vel == 5000

    def test_wire_frame(self):
        frame = build_frame(Cmd.velocity(ADDR, 0, 1000, 50, False), ChecksumMode.FIXED_6B)
        assert frame[0] == ADDR
        assert frame[1] == 0xF6
        assert frame[-1] == 0x6B


class TestPosition:
    def test_frame_length(self):
        payload = Cmd.position(ADDR, 0, 300, 50, 3200)
        assert len(payload) == 12

    def test_pulse_encoding(self):
        pulses = 0x00012345
        payload = Cmd.position(ADDR, 0, 0, 0, pulses)
        enc = (payload[6] << 24) | (payload[7] << 16) | (payload[8] << 8) | payload[9]
        assert enc == pulses

    def test_relative_flag(self):
        p_rel = Cmd.position(ADDR, 0, 0, 0, 100, relative=True)
        p_abs = Cmd.position(ADDR, 0, 0, 0, 100, relative=False)
        assert p_rel[10] == 0x01
        assert p_abs[10] == 0x00


class TestStop:
    def test_stop_frame(self):
        frame = build_frame(Cmd.stop(ADDR), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0xFE, 0x98, 0x00, 0x6B])


class TestReadParam:
    def test_read_position(self):
        frame = build_frame(Cmd.read_position(ADDR), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0x36, 0x6B])

    def test_read_drive_config_has_aux(self):
        payload = Cmd.read_drive_config(ADDR)
        assert payload == [ADDR, 0x42, 0x6C]

    def test_read_system_state_has_aux(self):
        payload = Cmd.read_system_state(ADDR)
        assert payload == [ADDR, 0x43, 0x7A]


class TestOrigin:
    def test_set_origin_no_store(self):
        frame = build_frame(Cmd.origin_set_zero(ADDR, store=False), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0x93, 0x88, 0x00, 0x6B])

    def test_set_origin_with_store(self):
        frame = build_frame(Cmd.origin_set_zero(ADDR, store=True), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0x93, 0x88, 0x01, 0x6B])

    def test_interrupt(self):
        frame = build_frame(Cmd.origin_interrupt(ADDR), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0x9C, 0x48, 0x6B])

    def test_trigger_return(self):
        frame = build_frame(Cmd.origin_trigger_return(ADDR, mode=0, sync=False), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0x9A, 0x00, 0x00, 0x6B])

    def test_modify_params_length(self):
        payload = Cmd.origin_modify_params(
            ADDR, store=False, mode=0, direction=0,
            speed_rpm=30, timeout_ms=10000,
            stall_speed_rpm=300, stall_current_ma=800,
            stall_time_ms=60, power_on_trigger=False,
        )
        assert len(payload) == 19  # without checksum

    def test_modify_params_speed_encoding(self):
        payload = Cmd.origin_modify_params(
            ADDR, store=False, mode=0, direction=0,
            speed_rpm=300, timeout_ms=0,
            stall_speed_rpm=0, stall_current_ma=0,
            stall_time_ms=0, power_on_trigger=False,
        )
        speed = (payload[6] << 8) | payload[7]
        assert speed == 300

    def test_modify_params_timeout_encoding(self):
        payload = Cmd.origin_modify_params(
            ADDR, store=False, mode=0, direction=0,
            speed_rpm=0, timeout_ms=0x0001ABCD,
            stall_speed_rpm=0, stall_current_ma=0,
            stall_time_ms=0, power_on_trigger=False,
        )
        tm = (payload[8] << 24) | (payload[9] << 16) | (payload[10] << 8) | payload[11]
        assert tm == 0x0001ABCD


class TestModifyControlMode:
    def test_no_store(self):
        frame = build_frame(Cmd.modify_control_mode(ADDR, mode=1, store=False), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0x46, 0x69, 0x00, 0x01, 0x6B])

    def test_with_store(self):
        frame = build_frame(Cmd.modify_control_mode(ADDR, mode=0, store=True), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0x46, 0x69, 0x01, 0x00, 0x6B])


class TestResetCommands:
    def test_reset_pos_to_zero(self):
        frame = build_frame(Cmd.reset_current_pos_to_zero(ADDR), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0x0A, 0x6D, 0x6B])

    def test_reset_clog_protection(self):
        frame = build_frame(Cmd.reset_clog_protection(ADDR), ChecksumMode.FIXED_6B)
        assert frame == bytes([0x01, 0x0E, 0x52, 0x6B])


class TestAdapterStubs:
    def test_write_drive_config_raises(self):
        from emm42.adapters.stubs import WriteDriveConfigStub
        stub = WriteDriveConfigStub()
        with pytest.raises(NotImplementedError):
            stub.write()

    def test_write_pid_raises(self):
        from emm42.adapters.stubs import WritePIDStub
        stub = WritePIDStub()
        with pytest.raises(NotImplementedError):
            stub.write(32000, 100, 32000)

    def test_write_id_raises(self):
        from emm42.adapters.stubs import WriteIDAddressStub
        stub = WriteIDAddressStub()
        with pytest.raises(NotImplementedError):
            stub.write(2)


# Copyright (c) 2024 Your Name
# SPDX-License-Identifier: MIT
"""
Unit tests for pca9685_hardware.PCA9685.

All tests mock smbus2.SMBus so that no real I2C hardware is required.
These tests exercise the driver's logic, register calculations, and error
handling without depending on any physical device.
"""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock, call, patch

import pytest

# ---------------------------------------------------------------------------
# Provide a minimal smbus2 stub so this module can be imported without the
# real smbus2 package installed (e.g. in a CI environment without I2C support).
# ---------------------------------------------------------------------------
if "smbus2" not in sys.modules:
    stub = types.ModuleType("smbus2")
    stub.SMBus = MagicMock  # type: ignore[attr-defined]
    sys.modules["smbus2"] = stub

from pca9685_driver.pca9685_hardware import PCA9685, PCA9685Error  # noqa: E402


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def mock_bus(monkeypatch):
    """Return a MagicMock that replaces smbus2.SMBus inside the module."""
    bus_instance = MagicMock()
    bus_instance.read_byte_data.return_value = 0x00
    with patch("pca9685_driver.pca9685_hardware.smbus2.SMBus",
               return_value=bus_instance):
        yield bus_instance


@pytest.fixture
def driver(mock_bus):
    """Return an initialised PCA9685 driver backed by the mock bus."""
    with patch("pca9685_driver.pca9685_hardware.time.sleep"):
        drv = PCA9685(i2c_bus=1, i2c_address=0x40)
    mock_bus.reset_mock()  # Discard __init__ calls so tests start clean.
    return drv


# ---------------------------------------------------------------------------
# Construction / teardown
# ---------------------------------------------------------------------------

class TestInit:
    def test_opens_bus(self, mock_bus):
        with patch("pca9685_driver.pca9685_hardware.time.sleep"):
            PCA9685(i2c_bus=1, i2c_address=0x40)
        # SMBus was called with the requested bus number.
        import smbus2
        smbus2.SMBus.assert_called_once_with(1)

    def test_raises_on_bus_error(self, mock_bus, monkeypatch):
        import smbus2
        smbus2.SMBus.side_effect = OSError("no such device")
        with pytest.raises(PCA9685Error, match="Cannot open I2C bus"):
            PCA9685(i2c_bus=99)
        smbus2.SMBus.side_effect = None  # Restore for other tests.

    def test_raises_on_comms_error(self, monkeypatch):
        bad_bus = MagicMock()
        bad_bus.write_byte_data.side_effect = OSError("NACK")
        with patch("pca9685_driver.pca9685_hardware.smbus2.SMBus",
                   return_value=bad_bus):
            with patch("pca9685_driver.pca9685_hardware.time.sleep"):
                with pytest.raises(PCA9685Error, match="Cannot communicate"):
                    PCA9685()

    def test_close_calls_bus_close(self, driver, mock_bus):
        driver.close()
        mock_bus.close.assert_called_once()


# ---------------------------------------------------------------------------
# set_pwm_frequency
# ---------------------------------------------------------------------------

class TestSetPWMFrequency:
    def test_prescale_calculation_50hz(self, driver, mock_bus):
        """50 Hz should produce prescale value 121."""
        mock_bus.read_byte_data.return_value = 0x00
        with patch("pca9685_driver.pca9685_hardware.time.sleep"):
            driver.set_pwm_frequency(50.0)

        # Extract every write_byte_data call and find the prescale write
        # (register 0xFE).
        prescale_calls = [
            c for c in mock_bus.write_byte_data.call_args_list
            if c.args[1] == PCA9685._REG_PRESCALE
        ]
        assert len(prescale_calls) == 1
        assert prescale_calls[0].args[2] == 121

    def test_prescale_calculation_333hz(self, driver, mock_bus):
        """333 Hz should produce prescale value 17."""
        mock_bus.read_byte_data.return_value = 0x00
        with patch("pca9685_driver.pca9685_hardware.time.sleep"):
            driver.set_pwm_frequency(333.0)

        prescale_calls = [
            c for c in mock_bus.write_byte_data.call_args_list
            if c.args[1] == PCA9685._REG_PRESCALE
        ]
        assert len(prescale_calls) == 1
        assert prescale_calls[0].args[2] == 17

    def test_frequency_clamped_to_minimum(self, driver, mock_bus):
        mock_bus.read_byte_data.return_value = 0x00
        with patch("pca9685_driver.pca9685_hardware.time.sleep"):
            driver.set_pwm_frequency(1.0)  # Below minimum
        assert driver.frequency_hz >= PCA9685._FREQ_MIN_HZ

    def test_frequency_clamped_to_maximum(self, driver, mock_bus):
        mock_bus.read_byte_data.return_value = 0x00
        with patch("pca9685_driver.pca9685_hardware.time.sleep"):
            driver.set_pwm_frequency(9999.0)  # Above maximum
        assert driver.frequency_hz <= PCA9685._FREQ_MAX_HZ

    def test_raises_on_i2c_error(self, driver, mock_bus):
        mock_bus.write_byte_data.side_effect = OSError("I2C error")
        with patch("pca9685_driver.pca9685_hardware.time.sleep"):
            with pytest.raises(PCA9685Error):
                driver.set_pwm_frequency(50.0)
        mock_bus.write_byte_data.side_effect = None


# ---------------------------------------------------------------------------
# set_pwm
# ---------------------------------------------------------------------------

class TestSetPWM:
    def test_writes_four_bytes_to_correct_register(self, driver, mock_bus):
        """Channel 0 writes to register 0x06 (LED0_ON_L)."""
        driver.set_pwm(0, 0, 307)
        mock_bus.write_i2c_block_data.assert_called_once_with(
            0x40, 0x06, [0x00, 0x00, 0x33, 0x01]
        )

    def test_channel_offset(self, driver, mock_bus):
        """Channel N base register = 0x06 + 4*N."""
        driver.set_pwm(5, 0, 0)
        expected_reg = PCA9685._REG_LED0_ON_L + 4 * 5
        args = mock_bus.write_i2c_block_data.call_args.args
        assert args[1] == expected_reg

    def test_invalid_channel_raises(self, driver):
        with pytest.raises(ValueError, match="channel"):
            driver.set_pwm(16, 0, 0)
        with pytest.raises(ValueError, match="channel"):
            driver.set_pwm(-1, 0, 0)

    def test_invalid_on_tick_raises(self, driver):
        with pytest.raises(ValueError, match="on_tick"):
            driver.set_pwm(0, 4096, 0)

    def test_invalid_off_tick_raises(self, driver):
        with pytest.raises(ValueError, match="off_tick"):
            driver.set_pwm(0, 0, 4096)

    def test_raises_on_i2c_error(self, driver, mock_bus):
        mock_bus.write_i2c_block_data.side_effect = OSError("bus error")
        with pytest.raises(PCA9685Error):
            driver.set_pwm(0, 0, 100)
        mock_bus.write_i2c_block_data.side_effect = None


# ---------------------------------------------------------------------------
# set_duty_cycle
# ---------------------------------------------------------------------------

class TestSetDutyCycle:
    def test_zero_duty_cycle(self, driver, mock_bus):
        driver.set_duty_cycle(0, 0.0)
        data = mock_bus.write_i2c_block_data.call_args.args[2]
        # off_tick should be 0
        off_tick = data[2] | (data[3] << 8)
        assert off_tick == 0

    def test_full_duty_cycle(self, driver, mock_bus):
        driver.set_duty_cycle(0, 1.0)
        data = mock_bus.write_i2c_block_data.call_args.args[2]
        off_tick = data[2] | (data[3] << 8)
        assert off_tick == 4095

    def test_clamped_above_one(self, driver, mock_bus):
        driver.set_duty_cycle(0, 1.5)
        data = mock_bus.write_i2c_block_data.call_args.args[2]
        off_tick = data[2] | (data[3] << 8)
        assert off_tick == 4095

    def test_clamped_below_zero(self, driver, mock_bus):
        driver.set_duty_cycle(0, -0.5)
        data = mock_bus.write_i2c_block_data.call_args.args[2]
        off_tick = data[2] | (data[3] << 8)
        assert off_tick == 0

    def test_half_duty_cycle(self, driver, mock_bus):
        driver.set_duty_cycle(0, 0.5)
        data = mock_bus.write_i2c_block_data.call_args.args[2]
        off_tick = data[2] | (data[3] << 8)
        assert off_tick == round(0.5 * 4096)


# ---------------------------------------------------------------------------
# set_pulse_width_us
# ---------------------------------------------------------------------------

class TestSetPulseWidthUs:
    def test_1500us_at_50hz_maps_to_correct_duty(self, driver, mock_bus):
        """1500 µs at 50 Hz = 7.5 % duty cycle."""
        with patch("pca9685_driver.pca9685_hardware.time.sleep"):
            driver.set_pwm_frequency(50.0)
            mock_bus.reset_mock()

        driver.set_pulse_width_us(0, 1500.0)
        data = mock_bus.write_i2c_block_data.call_args.args[2]
        off_tick = data[2] | (data[3] << 8)
        # 1500 µs / 20000 µs * 4096 ≈ 307
        assert abs(off_tick - 307) <= 2

    def test_raises_on_invalid_channel(self, driver):
        with pytest.raises(ValueError):
            driver.set_pulse_width_us(16, 1500.0)

    def test_raises_on_zero_frequency(self, driver):
        with pytest.raises(ValueError, match="frequency_hz"):
            driver.set_pulse_width_us(0, 1500.0, frequency_hz=0.0)


# ---------------------------------------------------------------------------
# set_all_off
# ---------------------------------------------------------------------------

class TestSetAllOff:
    def test_writes_full_off_bit(self, driver, mock_bus):
        driver.set_all_off()
        calls = mock_bus.write_i2c_block_data.call_args_list
        # The second call should write 0x10 to ALL_LED_OFF_H to assert FULL_OFF.
        assert len(calls) == 2
        reg_off_l, data_off = calls[1].args[1], calls[1].args[2]
        assert reg_off_l == PCA9685._REG_ALL_LED_OFF_L
        assert data_off[1] & 0x10  # Bit 4 of the high byte

    def test_raises_on_i2c_error(self, driver, mock_bus):
        mock_bus.write_i2c_block_data.side_effect = OSError("bus error")
        with pytest.raises(PCA9685Error):
            driver.set_all_off()
        mock_bus.write_i2c_block_data.side_effect = None


# ---------------------------------------------------------------------------
# Context manager
# ---------------------------------------------------------------------------

class TestContextManager:
    def test_close_called_on_exit(self, mock_bus):
        with patch("pca9685_driver.pca9685_hardware.time.sleep"):
            with PCA9685() as drv:
                pass
        mock_bus.close.assert_called()

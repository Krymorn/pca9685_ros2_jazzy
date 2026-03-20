# Copyright (c) 2024 Your Name
# SPDX-License-Identifier: MIT
"""
Low-level driver for the PCA9685 16-channel, 12-bit PWM controller.

Communicates directly over I2C using smbus2, imposing no dependency on
Adafruit Blinka or any other hardware-abstraction layer.  This module is
intentionally ROS2-agnostic so that it can be tested without a ROS2
installation and reused in other contexts.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Optional

import smbus2

logger = logging.getLogger(__name__)


class PCA9685Error(Exception):
    """Raised when communication with the PCA9685 fails."""


class PCA9685:
    """
    Driver for the NXP PCA9685 16-channel, 12-bit PWM controller.

    Each of the 16 channels has a 12-bit ON counter and a 12-bit OFF counter.
    The PWM output transitions HIGH at the ON tick and LOW at the OFF tick
    within each 4096-tick period.  For simple duty-cycle control, set
    on_tick=0 and vary off_tick between 0 (0 %) and 4095 (≈100 %).

    Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

    Args:
        i2c_bus: I2C bus number (e.g. 1 → /dev/i2c-1).
        i2c_address: 7-bit I2C address of the device (default 0x40).

    Raises:
        PCA9685Error: If the device cannot be reached or initialised.
    """

    # ── Register map ─────────────────────────────────────────────────────────
    _REG_MODE1         = 0x00
    _REG_MODE2         = 0x01
    _REG_LED0_ON_L     = 0x06   # Base address for channel 0; +4*N for channel N
    _REG_ALL_LED_ON_L  = 0xFA
    _REG_ALL_LED_ON_H  = 0xFB
    _REG_ALL_LED_OFF_L = 0xFC
    _REG_ALL_LED_OFF_H = 0xFD
    _REG_PRESCALE      = 0xFE

    # ── MODE1 bit masks ───────────────────────────────────────────────────────
    _MODE1_RESTART = 0x80
    _MODE1_AI      = 0x20   # Auto-increment register pointer
    _MODE1_SLEEP   = 0x10
    _MODE1_ALLCALL = 0x01

    # ── Hardware constants ────────────────────────────────────────────────────
    _INTERNAL_OSC_HZ: int = 25_000_000   # 25 MHz internal oscillator
    _RESOLUTION:      int = 4096         # 12-bit counter (ticks 0–4095)
    _PRESCALE_MIN:    int = 3
    _PRESCALE_MAX:    int = 255
    _FREQ_MIN_HZ:   float = 24.0
    _FREQ_MAX_HZ:   float = 1526.0

    NUM_CHANNELS: int = 16

    # ── Construction / teardown ───────────────────────────────────────────────

    def __init__(
        self,
        i2c_bus: int = 1,
        i2c_address: int = 0x40,
    ) -> None:
        self._address = i2c_address
        self._frequency_hz: float = 50.0  # Updated by set_pwm_frequency()

        try:
            self._bus = smbus2.SMBus(i2c_bus)
        except OSError as exc:
            raise PCA9685Error(
                f"Cannot open I2C bus {i2c_bus}: {exc}"
            ) from exc

        try:
            self._reset()
        except OSError as exc:
            self._bus.close()
            raise PCA9685Error(
                f"Cannot communicate with PCA9685 at I2C address "
                f"0x{i2c_address:02X} on bus {i2c_bus}: {exc}"
            ) from exc

        logger.debug(
            "PCA9685 initialised at I2C address 0x%02X on bus %d",
            i2c_address,
            i2c_bus,
        )

    def close(self) -> None:
        """
        Turn off all channels and release the I2C bus.

        Safe to call multiple times.
        """
        try:
            self.set_all_off()
        except PCA9685Error:
            pass  # Best-effort; we are shutting down anyway.
        try:
            self._bus.close()
        except Exception:  # noqa: BLE001
            pass
        logger.debug("PCA9685 I2C bus closed.")

    def __enter__(self) -> "PCA9685":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    # ── Public API ────────────────────────────────────────────────────────────

    def set_pwm_frequency(self, frequency_hz: float) -> None:
        """
        Set the PWM frequency for all 16 channels simultaneously.

        The achievable frequency range is approximately 24 Hz–1526 Hz,
        determined by the 8-bit prescaler register.  The requested value is
        clamped to this range.

        The device must enter SLEEP mode while the prescaler is being updated;
        this function handles that transition automatically.

        Args:
            frequency_hz: Desired PWM frequency in Hz.

        Raises:
            PCA9685Error: On I2C communication failure.
        """
        frequency_hz = max(self._FREQ_MIN_HZ, min(self._FREQ_MAX_HZ, frequency_hz))
        prescale = round(
            self._INTERNAL_OSC_HZ / (self._RESOLUTION * frequency_hz)
        ) - 1
        prescale = max(self._PRESCALE_MIN, min(self._PRESCALE_MAX, prescale))

        # The prescaler can only be set while the oscillator is sleeping.
        try:
            old_mode = self._read_byte(self._REG_MODE1)
            sleep_mode = (old_mode & ~self._MODE1_RESTART) | self._MODE1_SLEEP
            self._write_byte(self._REG_MODE1, sleep_mode)
            self._write_byte(self._REG_PRESCALE, prescale)
            self._write_byte(self._REG_MODE1, old_mode)
            # Datasheet §7.3.5: wait ≥500 µs for the oscillator to stabilise.
            time.sleep(0.0005)
            self._write_byte(self._REG_MODE1, old_mode | self._MODE1_RESTART)
        except OSError as exc:
            raise PCA9685Error(f"set_pwm_frequency failed: {exc}") from exc

        # Recalculate actual frequency from the integer prescale value.
        self._frequency_hz = self._INTERNAL_OSC_HZ / (
            self._RESOLUTION * (prescale + 1)
        )
        logger.debug(
            "PWM frequency set to %.2f Hz (prescale=%d, actual=%.2f Hz)",
            frequency_hz,
            prescale,
            self._frequency_hz,
        )

    @property
    def frequency_hz(self) -> float:
        """Actual PWM frequency in Hz (updated by set_pwm_frequency)."""
        return self._frequency_hz

    def set_pwm(
        self,
        channel: int,
        on_tick: int,
        off_tick: int,
    ) -> None:
        """
        Write raw ON/OFF ticks for a single channel.

        Both ticks are 12-bit values (0–4095) that reference positions within
        the PWM period.  The output goes HIGH at *on_tick* and LOW at
        *off_tick*.

        To use the special "full ON" or "full OFF" modes supported by the
        hardware, set the corresponding bit in the high byte directly via
        set_pwm_raw_bytes() instead.

        Args:
            channel:  Channel index (0–15).
            on_tick:  Tick at which the output goes HIGH (0–4095).
            off_tick: Tick at which the output goes LOW (0–4095).

        Raises:
            ValueError: If *channel*, *on_tick*, or *off_tick* is out of range.
            PCA9685Error: On I2C communication failure.
        """
        if not 0 <= channel < self.NUM_CHANNELS:
            raise ValueError(
                f"channel must be 0–{self.NUM_CHANNELS - 1}, got {channel}"
            )
        if not 0 <= on_tick <= 4095:
            raise ValueError(f"on_tick must be 0–4095, got {on_tick}")
        if not 0 <= off_tick <= 4095:
            raise ValueError(f"off_tick must be 0–4095, got {off_tick}")

        base = self._REG_LED0_ON_L + 4 * channel
        try:
            self._bus.write_i2c_block_data(
                self._address,
                base,
                [
                    on_tick  & 0xFF,
                    (on_tick  >> 8) & 0x0F,
                    off_tick & 0xFF,
                    (off_tick >> 8) & 0x0F,
                ],
            )
        except OSError as exc:
            raise PCA9685Error(
                f"set_pwm(channel={channel}) failed: {exc}"
            ) from exc

    def set_duty_cycle(self, channel: int, duty_cycle: float) -> None:
        """
        Set a channel's duty cycle as a normalised fraction.

        Args:
            channel:    Channel index (0–15).
            duty_cycle: Desired duty cycle, clamped to [0.0, 1.0].
                        0.0 → always LOW, 1.0 → always HIGH.

        Raises:
            ValueError:  If *channel* is out of range.
            PCA9685Error: On I2C communication failure.
        """
        duty_cycle = max(0.0, min(1.0, duty_cycle))
        off_tick = min(4095, round(duty_cycle * self._RESOLUTION))
        self.set_pwm(channel, 0, off_tick)

    def set_pulse_width_us(
        self,
        channel: int,
        pulse_width_us: float,
        frequency_hz: Optional[float] = None,
    ) -> None:
        """
        Set a channel's output pulse width in microseconds.

        Useful for servo and ESC control, where the specification is given in
        pulse-width terms rather than duty-cycle percentage.

        Args:
            channel:        Channel index (0–15).
            pulse_width_us: Desired pulse width in microseconds.
            frequency_hz:   PWM frequency to use for the calculation.
                            Defaults to the last value set by
                            set_pwm_frequency().

        Raises:
            ValueError:   If *channel* is out of range.
            PCA9685Error: On I2C communication failure.
        """
        freq = frequency_hz if frequency_hz is not None else self._frequency_hz
        if freq <= 0:
            raise ValueError("frequency_hz must be positive")
        period_us = 1_000_000.0 / freq
        duty_cycle = pulse_width_us / period_us
        self.set_duty_cycle(channel, duty_cycle)

    def set_all_off(self) -> None:
        """
        Assert the hardware FULL-OFF bit on all channels simultaneously.

        This uses the ALL_LED_OFF register, which overrides per-channel
        settings and forces every output LOW until the next per-channel write.

        Raises:
            PCA9685Error: On I2C communication failure.
        """
        try:
            # Clear the ALL_LED_ON registers first.
            self._bus.write_i2c_block_data(
                self._address, self._REG_ALL_LED_ON_L, [0x00, 0x00]
            )
            # Set bit 4 (FULL_OFF) of ALL_LED_OFF_H.
            self._bus.write_i2c_block_data(
                self._address, self._REG_ALL_LED_OFF_L, [0x00, 0x10]
            )
        except OSError as exc:
            raise PCA9685Error(f"set_all_off failed: {exc}") from exc

    # ── Private helpers ───────────────────────────────────────────────────────

    def _reset(self) -> None:
        """
        Perform a software reset and enable auto-increment addressing.

        Mirrors the power-on reset sequence described in PCA9685 §7.6.
        """
        # Enable auto-increment and ALLCALL; clear SLEEP.
        self._write_byte(
            self._REG_MODE1,
            self._MODE1_AI | self._MODE1_ALLCALL,
        )
        time.sleep(0.005)
        mode1 = self._read_byte(self._REG_MODE1)
        self._write_byte(self._REG_MODE1, mode1 & ~self._MODE1_SLEEP)
        time.sleep(0.005)

    def _write_byte(self, register: int, value: int) -> None:
        self._bus.write_byte_data(self._address, register, value)

    def _read_byte(self, register: int) -> int:
        return self._bus.read_byte_data(self._address, register)

# Copyright (c) 2024 Your Name
# SPDX-License-Identifier: MIT
"""
ROS2 node for the PCA9685 16-channel I2C PWM controller.

Topics (subscribed)
-------------------
~/command            (pca9685_interfaces/msg/ChannelCommand)
    Normalized command in [0.0, 1.0].  Linearly mapped to
    [min_pulse_us, max_pulse_us] from per-channel parameters.

~/command_pulse_us   (pca9685_interfaces/msg/ChannelCommand)
    Direct pulse-width command in microseconds.

~/commands           (pca9685_interfaces/msg/ChannelCommandArray)
    Batch normalized commands for multiple channels in one message.

Services
--------
~/set_pwm_raw        (pca9685_interfaces/srv/SetPWMRaw)
    Write raw ON/OFF ticks directly to the PCA9685 register (0–4095).

~/enable_output      (std_srvs/srv/SetBool)
    True → enable PWM output; False → assert full-OFF on all channels.

Parameters
----------
i2c_bus              int    = 1        I2C bus number.
i2c_address          int    = 64       Device address in decimal (0x40).
pwm_frequency        float  = 50.0     PWM frequency in Hz (24–1526).
default_min_pulse_us float  = 1000.0   Global default minimum pulse (µs).
default_max_pulse_us float  = 2000.0   Global default maximum pulse (µs).
channel_N_min_pulse_us float           Per-channel minimum pulse override.
channel_N_max_pulse_us float           Per-channel maximum pulse override.
"""

from __future__ import annotations

import logging

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from pca9685_interfaces.msg import ChannelCommand, ChannelCommandArray
from pca9685_interfaces.srv import SetPWMRaw

from pca9685_driver.pca9685_hardware import PCA9685, PCA9685Error

# Forward smbus2 / hardware log messages through the rclpy logging system.
logging.basicConfig(level=logging.DEBUG)


class PCA9685Node(Node):
    """
    ROS2 driver node for the PCA9685 16-channel PWM controller.

    The node exposes every hardware capability through typed ROS2 interfaces
    while remaining agnostic to the type of actuator (servo, ESC, LED, etc.)
    connected to each channel.  All actuator-specific configuration is handled
    through parameters and the parameter callback mechanism.
    """

    def __init__(self) -> None:
        super().__init__("pca9685_node")

        self._driver: PCA9685 | None = None
        self._output_enabled: bool = True

        self._declare_parameters()
        self._channel_config: list[dict[str, float]] = self._build_channel_config()

        self._initialize_hardware()

        self._create_subscriptions()
        self._create_services()

        # Allow frequency and per-channel limits to be updated at runtime.
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        self.get_logger().info(
            f"PCA9685 node ready — "
            f"I2C bus {self.get_parameter("i2c_bus").value}, "
            f"address 0x{self.get_parameter("i2c_address").value:02X}, "
            f"{self._driver.frequency_hz:.1f} Hz"  #
        )

    # ── ROS2 lifecycle helpers ────────────────────────────────────────────────

    def _declare_parameters(self) -> None:
        """Declare all ROS2 node parameters with descriptions and constraints."""

        self.declare_parameter(
            "i2c_bus",
            1,
            ParameterDescriptor(
                description="I2C bus number (e.g. 1 → /dev/i2c-1).",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "i2c_address",
            0x40,
            ParameterDescriptor(
                description=(
                    "7-bit I2C address of the PCA9685 in decimal "
                    "(default 64 = 0x40)."
                ),
                integer_range=[
                    IntegerRange(from_value=0x40, to_value=0x7F, step=1)
                ],
                read_only=True,
            ),
        )
        self.declare_parameter(
            "pwm_frequency",
            50.0,
            ParameterDescriptor(
                description="PWM carrier frequency in Hz (24.0–1526.0).",
                floating_point_range=[
                    FloatingPointRange(
                        from_value=PCA9685._FREQ_MIN_HZ,
                        to_value=PCA9685._FREQ_MAX_HZ,
                        step=0.0,
                    )
                ],
            ),
        )
        self.declare_parameter(
            "default_min_pulse_us",
            1000.0,
            ParameterDescriptor(
                description=(
                    "Default minimum pulse width in microseconds, "
                    "applied to all channels unless overridden."
                ),
            ),
        )
        self.declare_parameter(
            "default_max_pulse_us",
            2000.0,
            ParameterDescriptor(
                description=(
                    "Default maximum pulse width in microseconds, "
                    "applied to all channels unless overridden."
                ),
            ),
        )

        # Per-channel pulse-range overrides — declared after the defaults so
        # that the default values can be read from the already-declared params.
        default_min = self.get_parameter("default_min_pulse_us").value
        default_max = self.get_parameter("default_max_pulse_us").value

        for ch in range(PCA9685.NUM_CHANNELS):
            self.declare_parameter(
                f"channel_{ch}_min_pulse_us",
                default_min,
                ParameterDescriptor(
                    description=(
                        f"Channel {ch} minimum pulse width in microseconds.  "
                        f"Overrides default_min_pulse_us for this channel."
                    ),
                ),
            )
            self.declare_parameter(
                f"channel_{ch}_max_pulse_us",
                default_max,
                ParameterDescriptor(
                    description=(
                        f"Channel {ch} maximum pulse width in microseconds.  "
                        f"Overrides default_max_pulse_us for this channel."
                    ),
                ),
            )

    def _build_channel_config(self) -> list[dict[str, float]]:
        """
        Build per-channel configuration from current parameter values.

        Returns:
            A 16-element list; element N contains ``min_pulse_us`` and
            ``max_pulse_us`` for channel N.
        """
        return [
            {
                "min_pulse_us": self.get_parameter(
                    f"channel_{ch}_min_pulse_us"
                ).value,
                "max_pulse_us": self.get_parameter(
                    f"channel_{ch}_max_pulse_us"
                ).value,
            }
            for ch in range(PCA9685.NUM_CHANNELS)
        ]

    def _initialize_hardware(self) -> None:
        """Open the I2C device and configure the initial PWM frequency."""
        i2c_bus     = self.get_parameter("i2c_bus").value
        i2c_address = self.get_parameter("i2c_address").value
        frequency   = self.get_parameter("pwm_frequency").value

        try:
            self._driver = PCA9685(i2c_bus=i2c_bus, i2c_address=i2c_address)
            self._driver.set_pwm_frequency(frequency)
        except PCA9685Error as exc:
            self.get_logger().fatal(f"Hardware initialisation failed: {exc}")
            raise

    def _create_subscriptions(self) -> None:
        """Register all topic subscriptions."""
        self.create_subscription(
            ChannelCommand,
            "~/command",
            self._on_command,
            10,
        )
        self.create_subscription(
            ChannelCommand,
            "~/command_pulse_us",
            self._on_command_pulse_us,
            10,
        )
        self.create_subscription(
            ChannelCommandArray,
            "~/commands",
            self._on_commands,
            10,
        )

    def _create_services(self) -> None:
        """Register all ROS2 services."""
        self.create_service(
            SetPWMRaw,
            "~/set_pwm_raw",
            self._handle_set_pwm_raw,
        )
        self.create_service(
            SetBool,
            "~/enable_output",
            self._handle_enable_output,
        )

    # ── Parameter callback ────────────────────────────────────────────────────

    def _on_parameters_changed(
        self, params: list[Parameter]
    ) -> SetParametersResult:
        """
        React to runtime parameter updates.

        - ``pwm_frequency`` → reconfigures the hardware prescaler immediately.
        - ``channel_N_min/max_pulse_us`` → updates the in-memory channel map.
        """
        for param in params:
            if param.name == "pwm_frequency" and self._driver is not None:
                try:
                    self._driver.set_pwm_frequency(param.value)
                    self.get_logger().info(
                        "PWM frequency updated to %.2f Hz (actual %.2f Hz)",
                        param.value,
                        self._driver.frequency_hz,
                    )
                except PCA9685Error as exc:
                    return SetParametersResult(
                        successful=False, reason=str(exc)
                    )

            elif param.name.startswith("channel_") and (
                param.name.endswith("_min_pulse_us")
                or param.name.endswith("_max_pulse_us")
            ):
                # Rebuild the entire channel map so dependent fields stay
                # consistent even if multiple params arrive in one batch.
                self._channel_config = self._build_channel_config()

        return SetParametersResult(successful=True)

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _on_command(self, msg: ChannelCommand) -> None:
        """
        Handle a normalized command on ``~/command``.

        The value is linearly mapped from [0.0, 1.0] to
        [min_pulse_us, max_pulse_us] for the addressed channel.
        """
        if not self._output_enabled or not self._validate_channel(msg.channel):
            return

        cfg = self._channel_config[msg.channel]
        value = max(0.0, min(1.0, msg.value))
        pulse_us = cfg["min_pulse_us"] + value * (
            cfg["max_pulse_us"] - cfg["min_pulse_us"]
        )
        self._set_pulse_us(msg.channel, pulse_us)

    def _on_command_pulse_us(self, msg: ChannelCommand) -> None:
        """
        Handle a direct pulse-width command on ``~/command_pulse_us``.

        The value is interpreted as microseconds without any remapping.
        """
        if not self._output_enabled or not self._validate_channel(msg.channel):
            return

        self._set_pulse_us(msg.channel, msg.value)

    def _on_commands(self, msg: ChannelCommandArray) -> None:
        """
        Handle a batch of normalized commands on ``~/commands``.

        Each command is processed in the order listed in the message.
        Channels not mentioned are left unchanged.
        """
        if not self._output_enabled:
            return

        for cmd in msg.commands:
            if not self._validate_channel(cmd.channel):
                continue
            cfg = self._channel_config[cmd.channel]
            value = max(0.0, min(1.0, cmd.value))
            pulse_us = cfg["min_pulse_us"] + value * (
                cfg["max_pulse_us"] - cfg["min_pulse_us"]
            )
            self._set_pulse_us(cmd.channel, pulse_us)

    # ── Service handlers ──────────────────────────────────────────────────────

    def _handle_set_pwm_raw(
        self,
        request: SetPWMRaw.Request,
        response: SetPWMRaw.Response,
    ) -> SetPWMRaw.Response:
        """Write raw ON/OFF ticks directly to a PCA9685 channel register."""
        if not self._validate_channel(request.channel):
            response.success = False
            response.message = (
                f"Invalid channel {request.channel}: "
                f"must be 0–{PCA9685.NUM_CHANNELS - 1}."
            )
            return response

        try:
            self._driver.set_pwm(  # type: ignore[union-attr]
                request.channel, request.on_tick, request.off_tick
            )
            response.success = True
            response.message = (
                f"Channel {request.channel}: "
                f"on_tick={request.on_tick}, off_tick={request.off_tick}"
            )
        except (PCA9685Error, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"set_pwm_raw service error: {exc}")

        return response

    def _handle_enable_output(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """Enable or disable all 16 PWM outputs."""
        self._output_enabled = request.data

        if not self._output_enabled:
            try:
                self._driver.set_all_off()  # type: ignore[union-attr]
            except PCA9685Error as exc:
                response.success = False
                response.message = str(exc)
                self.get_logger().error(f"enable_output(False) error: {exc}")
                return response

        state = "enabled" if self._output_enabled else "disabled"
        response.success = True
        response.message = f"PWM output {state}."
        self.get_logger().info(f"PWM output {state}.")
        return response

    # ── Private helpers ───────────────────────────────────────────────────────

    def _set_pulse_us(self, channel: int, pulse_us: float) -> None:
        """Apply a pulse-width value to the hardware, logging any error."""
        assert self._driver is not None  # Guaranteed by _initialize_hardware
        try:
            self._driver.set_pulse_width_us(channel, pulse_us)
        except (PCA9685Error, ValueError) as exc:
            self.get_logger().error(
                f"Channel %d — failed to set %.1f µs: {exc}",
                channel,
                pulse_us,
            )

    def _validate_channel(self, channel: int) -> bool:
        """Return True if *channel* is in [0, NUM_CHANNELS); log on failure."""
        if not 0 <= channel < PCA9685.NUM_CHANNELS:
            self.get_logger().error(
                "Invalid channel %d — must be 0–%d.",
                channel,
                PCA9685.NUM_CHANNELS - 1,
            )
            return False
        return True

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        """Turn off all outputs and release the I2C bus on shutdown."""
        if self._driver is not None:
            try:
                self._driver.close()
                self.get_logger().info("PCA9685 I2C bus released.")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Error during shutdown: {exc}")
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args: list[str] | None = None) -> None:
    """Launch the PCA9685 driver node."""
    rclpy.init(args=args)
    node: PCA9685Node | None = None

    try:
        node = PCA9685Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Normal termination via Ctrl-C; suppress the traceback.
    except PCA9685Error as exc:
        if node:
            node.get_logger().fatal(f"Fatal hardware error: {exc}")
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

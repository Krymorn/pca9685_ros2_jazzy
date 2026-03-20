# Copyright (c) 2024 Your Name
# SPDX-License-Identifier: MIT
"""
Launch file for the PCA9685 driver node.

Usage examples
--------------
# Default settings (50 Hz, I2C bus 1, address 0x40):
ros2 launch pca9685_driver pca9685.launch.py

# Override I2C bus and frequency:
ros2 launch pca9685_driver pca9685.launch.py i2c_bus:=0 pwm_frequency:=333.0

# Load a custom parameter file:
ros2 launch pca9685_driver pca9685.launch.py \
    params_file:=/path/to/my_params.yaml
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("pca9685_driver")
    default_params_file = os.path.join(
        pkg_share, "config", "pca9685_params.yaml"
    )

    declared_args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description=(
                "Full path to a ROS2 YAML parameter file. "
                "Command-line overrides below take precedence."
            ),
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Optional ROS2 namespace for the node.",
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="pca9685_node",
            description="Name of the driver node.",
        ),
        DeclareLaunchArgument(
            "i2c_bus",
            default_value="1",
            description="I2C bus number (e.g. 1 for /dev/i2c-1).",
        ),
        DeclareLaunchArgument(
            "i2c_address",
            default_value="64",
            description="I2C address in decimal (default 64 = 0x40).",
        ),
        DeclareLaunchArgument(
            "pwm_frequency",
            default_value="50.0",
            description="PWM frequency in Hz (24.0-1526.0).",
        ),
        DeclareLaunchArgument(
            "default_min_pulse_us",
            default_value="1000.0",
            description="Default minimum pulse width in microseconds.",
        ),
        DeclareLaunchArgument(
            "default_max_pulse_us",
            default_value="2000.0",
            description="Default maximum pulse width in microseconds.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level: debug/info/warn/error/fatal.",
        ),
    ]

    pca9685_node = Node(
        package="pca9685_driver",
        executable="pca9685_node",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            # Explicit launch args override YAML file values.
            {
                "i2c_bus": LaunchConfiguration("i2c_bus"),
                "i2c_address": LaunchConfiguration("i2c_address"),
                "pwm_frequency": LaunchConfiguration("pwm_frequency"),
                "default_min_pulse_us": LaunchConfiguration(
                    "default_min_pulse_us"
                ),
                "default_max_pulse_us": LaunchConfiguration(
                    "default_max_pulse_us"
                ),
            },
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    return LaunchDescription([*declared_args, pca9685_node])

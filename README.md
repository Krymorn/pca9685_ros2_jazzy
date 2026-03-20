# pca9685_driver — ROS2 Jazzy

A clean, production-grade ROS2 **Jazzy** driver for the **NXP PCA9685** 16-channel,
12-bit I²C PWM controller.  The driver is intentionally actuator-agnostic: it
exposes every PWM channel through generic ROS2 topics and services and leaves
actuator-specific configuration entirely to parameters and YAML files.

## Repository layout

```
ros2_pca9685/
├── pca9685_interfaces/       # Custom msgs + srv (CMake package)
│   ├── msg/
│   │   ├── ChannelCommand.msg
│   │   └── ChannelCommandArray.msg
│   └── srv/
│       └── SetPWMRaw.srv
└── pca9685_driver/           # Driver node (Python package)
    ├── pca9685_driver/
    │   ├── pca9685_hardware.py   # Pure-Python, ROS2-agnostic I²C driver
    │   └── pca9685_node.py       # ROS2 node wrapping the hardware driver
    ├── launch/
    │   └── pca9685.launch.py
    ├── config/
    │   └── pca9685_params.yaml
    └── test/
        └── test_pca9685_hardware.py
```

## Prerequisites

| Requirement | Version |
|---|---|
| ROS2 | **Jazzy Jalisco** |
| Python | 3.10 + |
| [smbus2](https://pypi.org/project/smbus2/) | ≥ 0.4 |

Install the Python dependency:

```bash
sudo apt install python3-smbus2
# or
pip3 install smbus2
```

Enable I²C on your SBC if it is not already active (consult your board's
documentation — e.g. `raspi-config` on Raspberry Pi).

## Installation

```bash
# Clone into your workspace source directory
cd ~/ros2_ws/src
git clone https://github.com/Krymorn/PCA9685_ROS2_Jazzy.git

# Install ROS2 dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select pca9685_interfaces pca9685_driver

# Source the workspace
source install/setup.bash
```

> **Build order matters.** `pca9685_interfaces` must be built before
> `pca9685_driver`.  `colcon` resolves this automatically from the package
> dependencies; no manual ordering is required.

## Quick start

```bash
# Launch with all defaults (50 Hz, I²C bus 1, address 0x40):
ros2 launch pca9685_driver pca9685.launch.py

# Send a normalized command to channel 0 (centre position for a servo):
ros2 topic pub --once /pca9685_node/command \
    pca9685_interfaces/msg/ChannelCommand "{channel: 0, value: 0.5}"

# Send a direct pulse-width command (1500 µs) to channel 1:
ros2 topic pub --once /pca9685_node/command_pulse_us \
    pca9685_interfaces/msg/ChannelCommand "{channel: 1, value: 1500.0}"

# Disable all outputs:
ros2 service call /pca9685_node/enable_output std_srvs/srv/SetBool "{data: false}"

# Re-enable outputs:
ros2 service call /pca9685_node/enable_output std_srvs/srv/SetBool "{data: true}"

# Write raw PWM ticks to channel 3:
ros2 service call /pca9685_node/set_pwm_raw \
    pca9685_interfaces/srv/SetPWMRaw "{channel: 3, on_tick: 0, off_tick: 307}"
```

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `~/command` | `ChannelCommand` | sub | Normalised value [0.0, 1.0] → pulse width |
| `~/command_pulse_us` | `ChannelCommand` | sub | Direct pulse width in µs |
| `~/commands` | `ChannelCommandArray` | sub | Batch normalised commands |

The `~/command` topic maps the normalised value linearly:

```
pulse_us = min_pulse_us + value * (max_pulse_us - min_pulse_us)
```

where `min_pulse_us` and `max_pulse_us` come from the per-channel parameters.

## Services

| Service | Type | Description |
|---|---|---|
| `~/set_pwm_raw` | `SetPWMRaw` | Write raw 12-bit ON/OFF ticks directly |
| `~/enable_output` | `std_srvs/SetBool` | Enable (`true`) or disable (`false`) all outputs |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `i2c_bus` | `int` | `1` | I²C bus number (**read-only** at runtime) |
| `i2c_address` | `int` | `64` | Device address in decimal (**read-only**) |
| `pwm_frequency` | `float` | `50.0` | PWM frequency Hz — reconfigures hardware immediately |
| `default_min_pulse_us` | `float` | `1000.0` | Default minimum pulse width (µs) |
| `default_max_pulse_us` | `float` | `2000.0` | Default maximum pulse width (µs) |
| `channel_N_min_pulse_us` | `float` | *(default_min)* | Per-channel minimum pulse override |
| `channel_N_max_pulse_us` | `float` | *(default_max)* | Per-channel maximum pulse override |

N is 0–15.  All non-read-only parameters can be changed at runtime:

```bash
ros2 param set /pca9685_node pwm_frequency 333.0
ros2 param set /pca9685_node channel_0_min_pulse_us 500.0
```

## Launch arguments

```bash
ros2 launch pca9685_driver pca9685.launch.py \
    i2c_bus:=0 \
    i2c_address:=65 \
    pwm_frequency:=333.0 \
    default_min_pulse_us:=500.0 \
    default_max_pulse_us:=2500.0 \
    params_file:=/path/to/my_params.yaml \
    log_level:=debug
```

## Custom parameter file

Copy `config/pca9685_params.yaml`, uncomment the desired per-channel entries,
and pass it to the launch file:

```yaml
pca9685_node:
  ros__parameters:
    i2c_bus: 1
    i2c_address: 64
    pwm_frequency: 50.0
    default_min_pulse_us: 1000.0
    default_max_pulse_us: 2000.0
    # Example: widen the range for channels 0 and 1
    channel_0_min_pulse_us: 500.0
    channel_0_max_pulse_us: 2500.0
    channel_1_min_pulse_us: 500.0
    channel_1_max_pulse_us: 2500.0
```

## Running tests

```bash
cd ~/ros2_ws
colcon test --packages-select pca9685_driver
colcon test-result --verbose
```

The hardware unit tests (`test_pca9685_hardware.py`) fully mock `smbus2` and
require no physical I²C device.

## Architecture notes

The package is split into two layers:

- **`pca9685_hardware.py`** — a pure-Python, ROS2-agnostic class that talks
  directly to the PCA9685 over `smbus2`.  It can be imported and tested
  independently of any ROS2 installation.
- **`pca9685_node.py`** — the ROS2 node that wraps the hardware class,
  declares parameters, registers topics and services, and handles lifecycle
  events.

This separation keeps the hardware logic easy to unit-test and makes it
straightforward to adapt the driver to other middleware if needed.

## Troubleshooting

**`Cannot open I2C bus N`** — The I²C interface is not enabled or the bus
number is wrong.  On Raspberry Pi run `sudo raspi-config → Interface Options →
I2C → Enable`.  Verify with `ls /dev/i2c-*`.

**`Cannot communicate with PCA9685 at I2C address 0x40`** — Check wiring and
confirm the device appears on the bus with `i2cdetect -y 1`.

**`smbus2` not found** — Install it: `sudo apt install python3-smbus2`.

## License

MIT — see [LICENSE](LICENSE).

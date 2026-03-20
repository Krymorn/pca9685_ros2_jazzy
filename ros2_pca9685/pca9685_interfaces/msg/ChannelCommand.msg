# Command for a single PCA9685 channel.
#
# The interpretation of 'value' depends on which topic this message is
# published to:
#
#   ~/command          — normalized value in [0.0, 1.0]; linearly mapped to
#                        [min_pulse_us, max_pulse_us] per channel parameters.
#   ~/command_pulse_us — pulse width directly in microseconds (e.g. 1500.0).

# Channel index on the PCA9685 (0–15).
uint8 channel

# Command value. See topic description above for interpretation.
float64 value

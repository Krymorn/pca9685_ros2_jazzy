# Batch command for multiple PCA9685 channels in a single message.
# All commands are applied atomically in the order listed.
# Values are normalized [0.0, 1.0] — the same semantics as ~/command.

std_msgs/Header header

# One entry per channel to update. Channels not listed are left unchanged.
ChannelCommand[] commands

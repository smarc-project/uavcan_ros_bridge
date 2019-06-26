#
# Generic ESC status.
# Unknown fields should be set to NAN.
#

uint32 error_count          # Resets when the motor restarts

float32 voltage             # Volt
float32 current             # Ampere. Can be negative in case of a regenerative braking.
float32 temperature         # Kelvin

int32 rpm                   # Negative value indicates reverse rotation

uint8 power_rating_pct      # Instant demand factor in percent (percent of maximum power); range 0% to 127%.

uint8 esc_index

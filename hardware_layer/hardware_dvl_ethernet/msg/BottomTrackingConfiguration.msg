Header header

# Number of pings to average in one ensemble */
uint16 ping_per_ensemble

# Delay in ensemble to wait before trying to reacquire */
uint16 delay_before_reacquiring

# Minimum acceptable correlation magnitude (between 0 and 1) */
float32 correlation_threshold

# Minimum acceptable evaluation magnitude (between 0 and 1) */
float32 evaluation_threshold

# Minimum ratio of good pings needed to output a velocity (between 0 and 1)
float32 good_ping_threshold

# Bottom tracking mode (see BM command) */
uint8 mode

# Maximum acceptable velocity error (in m/s) */
float32 max_velocity_error

# Maximum configured tracking depth */
float32 max_tracking_depth

# Gain level for shallow water. See the WJ command
uint8 gain

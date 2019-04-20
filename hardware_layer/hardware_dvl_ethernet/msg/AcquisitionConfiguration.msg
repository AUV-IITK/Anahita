Header header

Sensors used_sensors

uint8 lag_duration

# How many depth cells are sampled
uint8 cell_count

uint8 profiling_mode

uint8 low_correlation_threshold

uint8 code_repetition_count

# Total count of pings per ensemble
uint16 pings_per_ensemble

# Length of a depth cell in meters
float32 cell_length

# ??? Is it the minimum ground distance allowed ?
float32 blank_after_transmit_distance

# Minimum percentage of good pings in the water profiling data
# required to output a velocity
float32 water_layer_min_ping_threshold

# Reject water velocity estimate if the error is above this
# threshold (in m/s)
float32 water_layer_velocity_threshold

# Time between two consecutive ping groups
uint64 time_between_ping_groups

# Angular correction between beam 3 and the yaw sensor, in radians
float32 yaw_alignment

# Magnetic correction: angular correction applied to the yaw
# sensor reading, in radians
float32 yaw_bias

# Distance to the first depth cell, in meters
float32 first_cell_distance

# Length, in meters, of a pulse
float32 transmit_pulse_length

# Start of the water layer in depth cells
uint8 water_layer_start

# End of the water layer in depth cells
uint8 water_layer_end

# Intensity threshold above which a cell or a beam are discarded
# during processing
uint8 false_target_threshold

# If true, the low latency hardware trigger is enabled
bool low_latency_trigger

# Distance between pulse repetitions, in meters
float32 transmit_lag_distance

# If true, the device is in narrow bandwidth mode (higher range
# but lower accuracy)
bool narrow_bandwidth_mode

uint8 base_frequency_index

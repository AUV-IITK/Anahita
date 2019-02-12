Header header

# Acquisition timestamp
float64 time

# Ranges to the bottom, in meters
float64[4] range

# Velocities
float64[4] velocity

# Correlation at the bottom cell (between 0 and 1)
float64[4] correlation

# Magnitude in the evaluation filter
float64[4] evaluation

# Ratio of good bottom tracking pings
float64[4] good_ping_ratio

# RSSI at the center of the bottom ping (in dB)
float64[4] rssi

# Single pressure reading.  This message is appropriate for measuring the
# pressure inside of a fluid (air, water, etc).  This also includes
# atmospheric or barometric pressure.

Header header             # timestamp of the measurement
                          # frame_id is the location of the pressure sensor

float32 fluid_pressure    # Absolute pressure reading in Pascals

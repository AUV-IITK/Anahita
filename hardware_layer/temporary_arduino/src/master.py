#!/usr/bin/python
"""
Handles Arduino messages and publishes them to ROS topics
"""
import serial
import rospy
from std_msgs.msg import String
from roslib.message import get_message_class

def ros_next(rate_hz):
    prev_time = rospy.get_time()
    timeout = 1 / rate_hz
    def closure():
        curr_time = rospy.get_time()
        if curr_time - prev_time > timeout:
            prev_time = curr_time
            return true
        else:
            return false

    return closure()

# Read the serial message string, and publish to the correct topics
def process_message(line):
    try:
        values = line.decode().split(',')
        pairs = {k: v for k, v in zip(sensor_csv_headers, values)}
       
        # Zip values with the corresponding environmental variable
        variable_values = values[1:]
        pairs = zip(sensor_csv_headers, variable_values)

        for header, value in pairs:
            if VALID_SENSOR_VARIABLES[header].

        return {"ok": pairs}

    except IndexError:
        rospy.logwarn("Short read, received part of a message: {}".format(buf.decode()))
        serial_connection.close()
        serial_connection.open()
        continue

    # Occasionally, we get rotten bytes which couldn't decode
    except UnicodeDecodeError:
        rospy.logwarn("Received weird bits, ignoring: {}".format(buf))
        serial_connection.close()
        serial_connection.open()
        continue

if __name__ == '__main__':
    rospy.init_node('handle_arduino')

    # Read configurable params
    sensor_csv_headers = rospy.get_param("~sensor_csv_headers", (
        "status",
        "pressure",
    ))

    actuator_csv_headers = rospy.get_param("~actuator_csv_headers", (
        "status",
        "thruster_e",
        "thruster_w",
        "thruster_n",
        "thruster_s",
        "thruster_ne",
        "thruster_nw",
        "thruster_se",
        "thruster_sw",
        "marker_dropper",
        "torpedo"
    ))

    VALID_SENSOR_VARIABLES = ["pressure_sensor"]

    PUBLISHERS = {
        variable.name: rospy.Publisher(
            "{}/raw".format(variable.name),
            get_message_class(variable.type),
            queue_size=10)
        for variable in VALID_SENSOR_VARIABLES
    }

    serial_port_id = rospy.get_param("~serial_port_id", "/dev/ttyACM0")
    publisher_rate_hz = rospy.get_param("~publisher_rate_hz", 1)
    baud_rate = rospy.get_param("~baud_rate", 115200)

    timeout_s = 1 / publisher_rate_hz
    # Initialize the serial connection
    serial_connection = serial.Serial(serial_port_id, baud_rate, timeout=timeout_s)

    publish_time = ros_next(publisher_rate_hz)
    while not rospy.is_shutdown():
        # Read before writing
        inputBuffer = serial_connection.readline()

        # Generate the message for the current state
        # status, thrusters one after another
        message = "0,{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
            state["thruster_e"],
            state["thruster_w"],
            state["thruster_n"],
            state["thruster_s"],
            state["thruster_ne"],
            state["thruster_nw"],
            state["thruster_se"],
            state["thruster_sw"],
            state["marker_dropper"],
            state["torpedo"]
        ).encode('utf-8')

        serial_connection.write(message)
        serial_connection.flush()

        pairs = process_message(inputBuffer)
        
        if publish_time() and pairs is not None:
        
            for header, value in pairs:
                PUBLISHERS[header].publish(value)

    serial_connection.close()

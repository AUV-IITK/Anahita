#!/usr/bin/python
"""
Handles Arduino messages and publishes them to ROS topics
"""
import serial
import rospy
from std_msgs.msg import Float32
from anahita_msgs.msg import Thrust
from roslib.message import get_message_class

current_values = []

def ros_next(rate_hz):
    prev_time = rospy.get_time()
    timeout = 1 / rate_hz
    curr_time = rospy.get_time()
    if curr_time - prev_time > timeout:
        prev_time = curr_time
        return True
    else:
        return False

def callback(data):
    current_values = [data.sideward_front, data.sideward_back, data.forward_right, data.forward_left, data.upward_north_east, data.upward_north_west, data.upward_south_east, data.upward_south_west, data.marker_dropper, data.torpedo]
    print(current_values[2])
    rospy.loginfo("Listening to a new message")


# Read the serial message string, and publish to the correct topics
def process_message(line):
    try:
        values = line.decode().split(',')
        pairs = {k: v for k, v in zip(sensor_csv_headers, values)}
       
        # Zip values with the corresponding environmental variable
        variable_values = values[1:]
        pairs = zip(sensor_csv_headers, variable_values)

        return pairs

    except IndexError:
        rospy.logwarn("Short read, received part of a message: {}".format(inputBuffer.decode()))
        serial_connection.close()
        serial_connection.open()
        return

    # Occasionally, we get rotten bytes which couldn't decode
    except UnicodeDecodeError:
        rospy.logwarn("Received weird bits, ignoring: {}".format(inputBuffer))
        serial_connection.close()
        serial_connection.open()
        return

if __name__ == '__main__':
    
    rospy.init_node('handle_arduino')
    
    rospy.Subscriber("/pwm", Thrust, callback)
    depth_publisher = rospy.Publisher("/depth", Float32, queue_size=10)
    sensor_csv_headers = ["status","pressure"]

    actuator_csv_headers = ["status","thruster_e","thruster_w","thruster_n","thruster_s","thruster_ne","thruster_nw","thruster_se","thruster_sw","marker_dropper","torpedo"]  

    VALID_SENSOR_VARIABLES = ["pressure_sensor"]

    serial_port_id = rospy.get_param("~serial_port_id", "/dev/arduino")
    publisher_rate_hz = rospy.get_param("~publisher_rate_hz", 1)
    baud_rate = rospy.get_param("~baud_rate", 115200)

    timeout_s = 1 / publisher_rate_hz

    # Initialize the serial connection
    try:
        serial_connection = serial.Serial(serial_port_id, baud_rate, timeout=timeout_s)
        
    except:
        rospy.logerr("Could not start serial_connection")
        exit()

    publish_time = ros_next(publisher_rate_hz)

    while not rospy.is_shutdown():

        # Read before writing
        rospy.loginfo("Inside the spin loop")
        try:
            print("Going to listen")
            inputBuffer = serial_connection.readline()
            print("input coming: " + str(inputBuffer) + "lol")
        except: 
            rospy.logerr("Failed, could not read the serial connection")

        # Generate the message for the current state
        # status, thrusters one after another
        try:
            message = "0,{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                current_values[0],
                current_values[1],
                current_values[2],
                current_values[3],
                current_values[4],
                current_values[5],
                current_values[6],
                current_values[7],
                current_values[8],
                current_values[9],    
            ).encode('utf-8')
            print(message)
            serial_connection.write(message)
            serial_connection.flush()

        except:
            rospy.logerr("Start the pwm publisher message")

        
        pairs = process_message(inputBuffer)
        
        if publish_time and pairs is not None:
        
            for header, value in pairs:
                depth_publisher.publish(value)

    rospy.spin()

    serial_connection.close()

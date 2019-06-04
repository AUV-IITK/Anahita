#!/usr/bin/python
"""
Handles Arduino messages and publishes them to ROS topics
"""
import serial
import rospy
from std_msgs.msg import Float32
from anahita_msgs.msg import Thrust
from roslib.message import get_message_class

pwm_str = ""

def callback(msg):
    global pwm_str
    pwm_str = str(msg.sideward_front) + ',' + str(msg.sideward_back) + ',' + \
                str(msg.forward_left) + ',' + str(msg.forward_right) + ',' + \
                str(msg.upward_north_east) + ',' + str(msg.upward_north_west) + ',' + \
                str(msg.upward_south_east) + ',' + str(msg.upward_south_west) + '\n'

if __name__ == '__main__':
    
    rospy.init_node('handle_arduino')
    
    rospy.Subscriber("/pwm", Thrust, callback)
    depth_publisher = rospy.Publisher("/depth", Float32, queue_size=10)

    serial_port_id = rospy.get_param("~serial_port_id", "/dev/arduino")
    baud_rate = rospy.get_param("~baud_rate", 115200)

    sensor_msg = Float32()

    # Initialize the serial connection
    try:
        serial_connection = serial.Serial("/dev/ttyACM0", 115200, timeout=5)
        rospy.sleep(2)
        
    except:
        rospy.logerr("Could not start serial_connection")
        exit()

    while not rospy.is_shutdown():
        try:
            rospy.loginfo('Inside the spin loop')
            serial_connection.flush()
            pwm_msg = pwm_str.encode('utf-8')
            serial_connection.write(pwm_msg)
            rospy.sleep(0.05)

            msg = 0
            # Serial read section
            msg = serial_connection.read(serial_connection.inWaiting()) # read all characters in buffer
            print(msg)
            #sensor_msg.data = float(msg)
            #depth_publisher.publish(sensor_msg)
            print msg
        except:
            rospy.logerr('Some error has occured')


    rospy.spin()

    serial_connection.close()

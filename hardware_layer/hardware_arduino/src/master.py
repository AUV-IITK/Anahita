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
    #rospy.loginfo("Here")
    pwm_str = str(msg.sideward_front) + ',' + str(msg.sideward_back) + ',' + \
                str(msg.forward_left) + ',' + str(msg.forward_right) + ',' + \
                str(msg.upward_north_east) + ',' + str(msg.upward_north_west) + ',' + \
                str(msg.upward_south_east) + ',' + str(msg.upward_south_west) + '\n'

if __name__ == '__main__':
    
    rospy.init_node('handle_arduino')
    
    rospy.Subscriber("/pwm", Thrust, callback)
    depth_publisher = rospy.Publisher("/anahita/depth", Float32, queue_size=10)
    bat_voltage_pub = rospy.Publisher("/anahita/battery_voltage", Float32, queue_size=10)

    serial_port_id = rospy.get_param("~serial_port_id", "/dev/ttyACM0")
    baud_rate = rospy.get_param("~baud_rate", 115200)

    depth_msg = Float32()
    bat_voltage_msg = Float32()   

    # Initialize the serial connection
    try:
        serial_connection = serial.Serial("/dev/ttyACM0", 115200, timeout=5)
        rospy.sleep(2)
        
    except:
        rospy.logerr("Could not start serial_connection")
        exit()

    while not rospy.is_shutdown():

            rospy.loginfo('Inside the spin loop')
            serial_connection.flush()
            pwm_msg = pwm_str.encode('utf-8')
            serial_connection.write(pwm_msg)
            rospy.loginfo(pwm_str)
            rospy.sleep(0.05)

            msg = 0
            # Serial read section
            msg = serial_connection.read(serial_connection.inWaiting()) # read all characters in buffer

            print(msg)
        except:
            rospy.logerr('Some error has occured')
            rospy.loginfo("Recieved: " + str(msg))  
            try:
                data = list(map(float,[x.strip() for x in msg.split(',')]))
                print("Data:" + str(data))
                bat_voltage_msg.data = data[0]                
                rospy.loginfo("Battery Voltage: " + str(bat_voltage_msg))
                depth_msg.data = data[1]
                rospy.loginfo("Depth: " + str(depth_msg))
                depth_publisher.publish(depth_msg)
                bat_voltage_pub.publish(bat_voltage_msg)
            except:
                rospy.loginfo("Error in incoming values")
        

    rospy.spin()
    serial_connection.close()

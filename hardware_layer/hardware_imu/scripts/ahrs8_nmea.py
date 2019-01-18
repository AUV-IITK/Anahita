#!/usr/bin/env python

import rospy
import serial, math
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Verify the checksum obtained in the NMEA message.
def verify_checksum(response):
    # strip leading $
    message_and_checksum = response.strip("$")
    # split into message text and trailing checksum as a 2-tuple
    message_and_checksum = message_and_checksum.split("*")
    # message text is first component, checksum second.  Make sure we strip whitespace.
    message = message_and_checksum[0]
    checksum = message_and_checksum[1].strip()

    calculated_checksum = 0

    # NMEA checksums are obtained by xor'ing the ascii value of every character between $ and *
    for character in message:
        calculated_checksum = calculated_checksum ^ ord(character)

    # Get the 2 digit hex representation of the result.
    hexstring = format(calculated_checksum, "02X")
    # Return if the checksums match or not.
    return hexstring == checksum

# Populate message angular velocity values.
def populate_G(string, message):
    split_string = string.split(",")
    # Split off each value expression "<a>=<value>" as a string.
    Gx_string = split_string[1]
    Gy_string = split_string[2]
    # Split checksum off of last output string.
    Gz_string = split_string[3].split("*")[0]

    # Parse the angular velocity values.
    Gx_float = millidegrees_to_radians(float(Gx_string.split("=")[1]))
    Gy_float = millidegrees_to_radians(float(Gy_string.split("=")[1]))
    Gz_float = millidegrees_to_radians(float(Gz_string.split("=")[1]))

    # Populate the message using X FORWARD Y LEFT Z UP
    message.angular_velocity.x = Gx_float
    message.angular_velocity.y = -Gy_float
    message.angular_velocity.z = -Gz_float

# Populate message quaternion data
def populate_QUAT(string, message):
    split_string = string.split(",")
    # Split off each value expression "<a>=<value>" as a string.
    w_string = split_string[1]
    x_string = split_string[2]
    y_string = split_string[3]
    # Split checksum off of last output string.
    z_string = split_string[4].split("*")[0]

    # Parse the quaternion values.
    w_float = float(w_string.split("=")[1])
    x_float = float(x_string.split("=")[1])
    y_float = float(y_string.split("=")[1])
    z_float = float(z_string.split("=")[1])

    # Build euler angle array (r,p,y) from quaternion using ENU convention.
    euler = euler_from_quaternion([y_float, x_float, -z_float, w_float])

    # Reference is still 0 yaw facing north, -pi/2 East.  Add pi/2 to zero yaw when facing East.
    # Get new quaternion from this modification. Returned as (x, y, z, w)
    quat = quaternion_from_euler(euler[0], euler[1], euler[2] + math.pi/2)

    # Populate IMU message using ENU coordinate convention.
    message.orientation.w = quat[3]
    message.orientation.x = quat[0]
    message.orientation.y = quat[1]
    message.orientation.z = quat[2]

# Populate linear acceleration data.
def populate_A(string, message):
    split_string = string.split(",")
    # Split off each value expression "<a>=<value>" as a string.
    Ax_string = split_string[1]
    Ay_string = split_string[2]
    # Split checksum off of last output string.
    Az_string = split_string[3].split("*")[0]

    # Parse the linear acceleration values.
    Ax_float = millig_to_meter(float(Ax_string.split("=")[1]))
    Ay_float = millig_to_meter(float(Ay_string.split("=")[1]))
    Az_float = millig_to_meter(float(Az_string.split("=")[1]))

    # Populate the message using X FORWARD Y LEFT Z UP
    message.linear_acceleration.x = -Ax_float
    message.linear_acceleration.y = Ay_float
    message.linear_acceleration.z = Az_float

def millidegrees_to_radians(value):
    return (value/1000) * (math.pi/180.0)

def millig_to_meter(value):
    return (value/1000) * 9.81

# Set all covariance matrices to one matrix.
# Only needed until we understand better how to deal with the covariance of each data set.
def set_all_covariance(imu_msg, covariance_matrix):
    # Copy covariance_matrix to each imu_msg covariance
    for i in range(0, 9):
        imu_msg.orientation_covariance[i] = covariance_matrix[i]
        imu_msg.angular_velocity_covariance[i] = covariance_matrix[i]
        imu_msg.linear_acceleration_covariance[i] = covariance_matrix[i]

if __name__ == '__main__':
    # Initialize node
    rospy.init_node("ahrs8_node")


    default_port = "/dev/ttyUSB0"
    default_baud = 115200
    default_frameid = "ahrs8_imu"
    compass_port = rospy.get_param("~port", default_port)
    compass_baud = rospy.get_param("~baud", default_baud)
    compass_frame = rospy.get_param("~frame_id", default_frameid)

    # Initialize publisher
    imu_pub = rospy.Publisher("/sparton/imu/data", Imu, queue_size=10)
    imu_msg = Imu()

    # Set IMU device transform frame.
    imu_msg.header.frame_id = compass_frame

    # Default matrix to use for covariance of each measurement set.
    default_covariance_matrix = [1e-6, 0, 0,
                                 0, 1e-6, 0,
                                 0, 0, 1e-6]

    # Populate the imu message with the covariance matrix.
    set_all_covariance(imu_msg, default_covariance_matrix)

    compass_serial = serial.Serial(compass_port, compass_baud, timeout=1)

    try:
        # Clear all repeating I/O that could be leftover from operation.
        compass_serial.write("\x13")
        compass_serial.write("$xxHDM\r\n")
        compass_serial.write("printmask 0 set drop\r\n")
        compass_serial.write("printmodulus 0 set drop\r\n")
        compass_serial.write("printtrigger 0 set drop\r\n")

        rospy.sleep(0.1)
        # Resume output allowed. Ctrl-Q
        compass_serial.write("\x11")
        rospy.sleep(0.1)
        # Flush all buffers.
        compass_serial.flushInput()
        compass_serial.flushOutput()
        rospy.sleep(0.1)
    except serial.SerialException:
        rospy.logerr("AHRS-8: Serial communications not opened properly!")

    rospy.loginfo("AHRS-8: Output reset, beginning to retrieve data.")

    while not rospy.is_shutdown():
	then = rospy.get_time()
        # Get angular velocity
        compass_serial.write("$PSPA,G\r\n")
        response = compass_serial.readline()
        if (verify_checksum(response)):
            populate_G(response, imu_msg)
        else:
            rospy.logerr("AHRS-8: Bad checksum, skipping dataset.")
            continue

        # Get orientation in ENU convention with East as 0 yaw reference.
        compass_serial.write("$PSPA,QUAT\r\n")
        response = compass_serial.readline()
        if (verify_checksum(response)):
            populate_QUAT(response, imu_msg)
        else:
            rospy.logerr("AHRS-8: Bad checksum, skipping dataset.")
            continue

        # Get linear acceleration.
        compass_serial.write("$PSPA,A\r\n")
        response = compass_serial.readline()
        if (verify_checksum(response)):
            populate_A(response, imu_msg)
        else:
            rospy.logerr("AHRS-8: Bad checksum, skipping dataset.")
            continue
        imu_msg.header.stamp = rospy.Time.now()
        # Publish the current message.
        imu_pub.publish(imu_msg)
	now = rospy.get_time()
	rospy.loginfo("diff: %f", now - then)


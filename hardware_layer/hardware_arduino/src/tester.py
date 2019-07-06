#!/usr/bin/env python

import rospy
from anahita_msgs.msg import Thrust

looping = 1
pwm_msg = Thrust()

if __name__ == '__main__':

    rospy.init_node('tester_arduino')

    pose_cmd_pub = rospy.Publisher('/pwm', Thrust, queue_size=10, latch=True)

    try:
	while not rospy.is_shutdown():

		pwm_msg.sideward_front = 0
		pwm_msg.sideward_back = 0
		pwm_msg.forward_left = 0
		pwm_msg.forward_right = 0
		pwm_msg.upward_north_east = 0
		pwm_msg.upward_north_west = 0
		pwm_msg.upward_south_east = 0
		pwm_msg.upward_south_west = 0

		for i in range (-400,400,10):
			pwm_msg.forward_left = i
			pwm_msg.upward_north_east = i
			i=i+10
			for j in range(-400,400,10):
				pwm_msg.sideward_back = j
				pwm_msg.upward_north_west = j
				j= j+10
				for k in range(-400,400,10):
				    pwm_msg.sideward_front = k
				    pwm_msg.upward_south_east = k
				    k= k+10
				    for l in range(-400,400,10):
					    pwm_msg.forward_right = l
					    pwm_msg.upward_south_west = l
					    l = l+10

					    pose_cmd_pub.publish(pwm_msg)
	                                    rospy.loginfo("Published the message: " + str(pwm_msg))
					    rospy.sleep(0.0001)
						
	
    except:
	print("Error in file")
	exit()


 
    	

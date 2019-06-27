#!/usr/bin/env python

import rospy

from master_layer.srv import GoToIncremental
from master_layer.srv import GoTo
from master_layer.msg import Waypoint
from master_layer.srv import ChangeOdom
from master_layer.srv import CurrentTask
from master_layer.srv import GoToPose
from master_layer.srv import InitCircularTrajectory
from master_layer.srv import Hold
from master_layer.srv import TrajectoryComplete
from master_layer.srv import PoseReach
from master_layer.srv import ChangeTorpedoHole
from xsens_driver.srv import ResetIMUOrient

import time
from std_msgs.msg import Time, String

from anahita_utils import *

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
import tf.transformations as trans

current_p = Pose()

def odometry_callback(msg):
    global current_p
    current_p = msg.pose.pose

if __name__ == '__main__':

    rospy.init_node('torpedo')

    sub_odometry = rospy.Subscriber('/anahita/pose_gt', numpy_msg(Odometry), odometry_callback)
    pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10, latch=True)

    go_to_incremental = rospy.ServiceProxy('anahita/go_to_incremental', GoToIncremental)
    go_to = rospy.ServiceProxy('anahita/go_to', GoTo)
    current_task = rospy.ServiceProxy('anahita/current_task', CurrentTask)
    change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)
    go_to_pose = rospy.ServiceProxy('anahita/go_to_pose', GoToPose)
    reset_imu = rospy.ServiceProxy('/nav/set_imu_quat', ResetIMUOrient)
    init_circular_trajectory = rospy.ServiceProxy('anahita/start_circular_trajectory', InitCircularTrajectory)
    hold_vehicle = rospy.ServiceProxy('anahita/hold_vehicle', Hold)
    trajectory_complete = rospy.ServiceProxy('anahita/trajectory_complete', TrajectoryComplete)
    pose_reach = rospy.ServiceProxy('anahita/pose_reach', PoseReach)
    change_torpedo_hole = rospy.ServiceProxy('anahita/change_torpedo_hole', ChangeTorpedoHole)

    pose = Pose()
    step_point = Point()

#    current_task_resp = current_task(current_task="torpedo")
  #  change_odom_response = change_odom(odom="vision")
    rospy.sleep(0.05)
    intial_orientation = quaternion_to_eulerRPY(current_p.orientation)
    reset_imu_resp = reset_imu()
	
    # 1st torpedo shoot
   # change_torpedo_hole(hole="BOT")
    pose = fill_pose(0, 0, 0, 0, 0, 0, 1)
    go_to_pose(target_pose=pose)
    rospy.loginfo('Publishing cmd to align')

    pose_reach(time_out=2000)
    rospy.sleep(5)
    rospy.loginfo('Aligned center')

    rospy.sleep(100000)

    # change_odom(odom="stereo_vision")

    # pose = fill_pose(-2.0, 0, 0, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('cmd to move near the torpedo hole')

    # pose_reach(time_out=20)
    # rospy.sleep(1)
    # rospy.loginfo('Infront of the torpedo hole')

    # # fire in the hole

    # change_odom(odom="dvl")
    # rospy.sleep(0.1)

    # pose = fill_pose(current_p.x - 1, current_p.y, current_p.z, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('cmd to go back')

    # pose_reach(time_out=20)
    # rospy.loginfo('Bot is back')

    # # 2nd torpedo shoot

    # change_torpedo_hole(hole="TR")
    # pose = fill_pose(0, 0, 0, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('Publishing cmd to align')

    # pose_reach(time_out=20)
    # rospy.sleep(5)
    # rospy.loginfo('Aligned center')

    # change_odom(odom="stereo_vision")

    # pose = fill_pose(-2.0, 0, 0, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('cmd to move near the torpedo hole')

    # pose_reach(time_out=20)
    # rospy.sleep(1)
    # rospy.loginfo('Infront of the torpedo hole')

    # # fire in the hole

    # change_odom(odom="dvl")
    # rospy.sleep(0.1)

    # pose = fill_pose(current_p.x - 1, current_p.y, current_p.z, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('cmd to go back')

    # pose_reach(time_out=20)
    # rospy.loginfo('Bot is back')

    # # 3rd torpedo shoot

    # change_torpedo_hole(hole="BOT")
    # pose = fill_pose(0, 0, 0, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('Publishing cmd to align')

    # pose_reach(time_out=20)
    # rospy.sleep(5)
    # rospy.loginfo('Aligned center')

    # change_odom(odom="stereo_vision")

    # pose = fill_pose(-2.0, 0, 0, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('cmd to move near the torpedo hole')

    # pose_reach(time_out=20)
    # rospy.sleep(1)
    # rospy.loginfo('Infront of the torpedo hole')

    # # fire in the hole

    # change_odom(odom="dvl")
    # rospy.sleep(0.1)

    # pose = fill_pose(current_p.x - 1, current_p.y, current_p.z, 0, 0, 0, 1)
    # go_to_pose(target_pose=pose)
    # rospy.loginfo('cmd to go back')

    # pose_reach(time_out=20)
    # rospy.loginfo('Bot is back')

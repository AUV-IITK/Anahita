#!/usr/bin/env python

import rospy
import smach
from uuv_control_msgs.srv import *
from uuv_control_msgs.msg import Waypoint as WaypointMsg
from std_msgs.msg import Time, String
import time
from nav_msgs.msg import Odometry
import numpy
from rospy.numpy_msg import numpy_msg
from master_layer.srv import VerifyObject

class Transition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','timeout', 'failed'],
                            input_keys=['target_waypoint'])
        self._odom_topic_sub = rospy.Subscriber(
            '/anahita/pose_gt', numpy_msg(Odometry), self.odometry_callback)

        self._pose = numpy.zeros(3)
        self._threshold = 0.5
        self._timeout = 60 # in seconds
        self._cutoff = 1 # must be comparable to intial distance from the starting point

    def calc_dist(self, current_pose, target_waypoint_msg):
        dist = (current_pose.x - target_waypoint_msg.point.x)*(current_pose.x - target_waypoint_msg.point.x) + \
                (current_pose.y - target_waypoint_msg.point.y)*(current_pose.y - target_waypoint_msg.point.y) + \
                (current_pose.z - target_waypoint_msg.point.z)*(current_pose.z - target_waypoint_msg.point.z)
        return dist

    def odometry_callback(self, msg):
        self._pose = numpy.array([msg.pose.pose.position.x,
                                msg.pose.pose.position.y,
                                msg.pose.pose.position.z])
    
    def execute(self, userdata):
        target_waypoint_msg = WaypointMsg()
        target_waypoint_msg = userdata.target_waypoint.to_message()
        
        print 'waiting for go to waypoint server'
        rospy.wait_for_service('anahita/go_to')

        go_to = rospy.ServiceProxy('anahita/go_to', GoTo)

        start_time = Time()
        start_time.data.secs = rospy.get_rostime().to_sec()
        start_time.data.nsecs = rospy.get_rostime().to_nsec()
        
        interpolator = String()
        interpolator.data = 'cubic_interpolator'
        print 'adding waypoints....'
        resp = init_waypoint_set(max_forward_speed = 0.5, 
                                interpolator = interpolator,
                                waypoint = target_waypoint_msg)

        then = rospy.get_time()

        while not rospy.is_shutdown():
            now = rospy.get_time()
            dt = now - then
            dist = calc_dist(self._pose, target_waypoint_msg)
            
            if dist < self._threshold:
                return 'success'

            if dt > self._timeout:
                return 'timeout'

            if dist > self._cutoff:
                return 'failed'

class TaskBaseClass(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['random_outcome1', 'random_outcome2'],
                            input_keys=['field1', 'field2', 'field3'],
                            output_keys=['field1', 'field2', 'field2'])

    def stabilise(self):
        # after the start of stereo vision get to the supposed 
        # start point avoiding any targets
        pass

    def load_params(self):
        # start point for the task
        pass

    def start_stereo(self):
        # after the verification of the object start visual odometry
        pass

    def execute(self):
        pass

    def start_detecting(self):  
        # to start object recognition as soon the bot enters
        # that task and confirm it is the same object
        # a service to ask vision layer if it is the same object
        # vision layer will recognize for a particular period of time
        # and verify the object

        print 'waiting for object recognition server'
        rospy.wait_for_service('anahita/object_recognition')

        verify_object = rospy.ServiceProxy('anahita/object_recognition', VerifyObject)
        resp = verify_object(object='')

        if resp:
            return True
        else:
            return False

class BouyTask(TaskBaseClass):
    def __init__(self):
        TaskBaseClass.__init__(self)

    def hit_buoy(self):
        # a service call to goto for hitting the buoy
        pass

    def get_back(self):
        # a goto request to get back to its initial point
        pass

    def execute(self):
        # can make a sub state machine to define the before hitting and after 
        # hitting in separate states
        pass

# if every task state execute will contains different set of member functions to 
# complete the task like in buoy it will be hit_buoy() and in gate it will be just
# move forward
class GateTask(TaskBaseClass):
    def __init__(self):
        TaskBaseClass.__init__(self)

    def wait_for_crossing(self):
        # to wait for the lower part of a pipe to be identified by the vision layer
        # to send a request to the vision layer to do it with a timeout
        pass

    def move_forward(self):
        # cross the gate
        pass

    def execute(self):
        pass

class TorpedoTask(TaskBaseClass):
    def __init__(self):
        TaskBaseClass.__init__(self)

    def take_position(self):
        # take position to fire the torpedo
        # by making a req to the goto server
        pass

    def fire_torpedo(self):
        pass

    def execute(self):
        pass

class MarkerDropperTask(TaskBaseClass):
    def __init__(self):
        TaskBaseClass.__init__(self)

    def align(self):
        pass

    def execute(self):
        pass

class OctagonTask(TaskBaseClass):
    def __init__(self):
        TaskBaseClass.__init__(self)

    def localise(self):
        pass

    def execute(self):
        pass

class LineTask(TaskBaseClass):
    def __init__(self):
        TaskBaseClass.__init__(self)

    def execute(self):
        pass

class FindTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        pass

class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['some_outcome'])

    def execute():
        pass

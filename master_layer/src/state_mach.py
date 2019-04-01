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

    def align(self):
        # to align with the line
        pass
    
    def move_forward(self):
        # after aligning to the line move forward

    def execute(self):
        pass

# there are two kinds of findings that need to be dealt with
# 1. one to find the object with the bottom camera
# 2. and one with the front camera

class FindBottomTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        # request to move in a circle or helix to the local planner
        pass

    def explore(self):
        # move around to find a target

class FindFrontTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        # move the bot to the pose according to the odom frame of ref 
        # and then rotate the bot to search object
        pass

    def explore(self):
        # move around to find a target

# there are two kinds of scenarios for finding
# 1. when the bot is working properly but is not able to find the
#    object at the current position so it goes into `find target state`
# 2. something very bad happens to the like the thruster behaving weirdly 
#    and dragging to the direction where it should not go, in that situation 
#    it becomes necessary for the reset all the things and plan for itself 
#    according to the task it has done and last pose that it remebers and 
#    use path planning algorithms to generate waypoints for the trajectory generation

class RescueMode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['some_outcome'])

    def execute(self):
        # monitor the path tracing
        pass

    def localise(self):
        # locate where is it now
        # can make a special service for this
        pass

    def next_task(self):
        # gives the next task to approach

    def path_to_new_target(self):
        # use a service to get the path from a path algorithm

    def trace_path(self):
        # make a request to the init_waypoint_server for trajectory generation


class TooFar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[''],
                            input_keys=['target'])

    def execute(self):
        pass
    
    def stop(self):
        # if the bot is going out of the way of any target
        # then stop it
        pass

    def restore(self):
        # restore to a position from where it can still do the task
        pass

class StationKeeping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self):
        # just maintain the current pose

if __init__ == '__main__':
    # do something

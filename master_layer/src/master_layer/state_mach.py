#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import numpy

from uuv_control_msgs.srv import *
from uuv_control_msgs.msg import Waypoint as WaypointMsg

from rospy.numpy_msg import numpy_msg

from std_msgs.msg import Time, String, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion

from anahita_utils import *

from master_layer.srv import GoToIncremental
from master_layer.srv import GoTo
from master_layer.srv import ChangeOdom
from master_layer.srv import CurrentTask
from master_layer.srv import GoToPose
from master_layer.srv import TrajectoryComplete
from master_layer.srv import PoseReach
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
        smach.State.__init__(self, outcomes=['near', 'never_moved', 'thruster_inactive', 'unstable', 'far', 'success', 'fail'], 
                             input_keys=['input_stage'], output_keys=['output_stage'])

        self.odom_topic_sub = rospy.Subscriber('/anahita/pose_gt', Odometry, self.odometry_callback)
        self.conventional_detection_pub = rospy.Subscriber('/anahita/conventional_detection', Int32, self.conventional_detector_cb)
        self.pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10)
        self.conventional_prob = 0
        self.pose = None
        self.odom_init = False
        self.service_timeout = 5
        self.stages = None
        self.current_stage = ''
        self.task_timeout = None
        self.current_odom_source = ''
        
        try:
            self.services = dict()

            print 'Task Base Class: waiting for change odom server ....'
            rospy.wait_for_service('odom_source')
            self.services['change_odom'] = rospy.ServiceProxy('odom_source', ChangeOdom)
            print 'Task Base Class: waiting for go_to_incremental server ....'
            rospy.wait_for_service('anahita/go_to_incremental')
            self.services['go_to_incremental'] = rospy.ServiceProxy('anahita/go_to_incremental', GoToIncremental)
            print 'Task Base Class: waiting for go_to server ....'
            rospy.wait_for_service('anahita/go_to')
            self.services['go_to'] = rospy.ServiceProxy('anahita/go_to', GoTo)
            print 'Task Base Class: waiting for current task server ....'
            rospy.wait_for_service('anahita/current_task')
            self.services['current_task'] = rospy.ServiceProxy('anahita/current_task', CurrentTask)
            print 'Task Base Class: waiting for go_to_pose server ....'
            rospy.wait_for_service('anahita/go_to_pose')
            self.services['go_to_pose'] = rospy.ServiceProxy('anahita/go_to_pose', GoToPose)
            print 'Task Base Class: waiting for trajectory_complete server ....'
            rospy.wait_for_service('anahita/trajectory_complete')
            self.services['trajectory_complete'] = rospy.ServiceProxy('anahita/trajectory_complete', TrajectoryComplete)
            print 'Task Base Class: waiting for pose_reach server ....'
            rospy.wait_for_service('anahita/pose_reach')
            self.services['pose_reach'] = rospy.ServiceProxy('anahita/pose_reach', PoseReach)
            print 'Task Base Class: waiting for object recognition server ....'
            rospy.wait_for_service('anahita/object_recognition')
            self.services['verify_object'] = rospy.ServiceProxy('anahita/object_recognition', VerifyObject)
        except:
            print("Service Instantiation failed")
    
    def odometry_callback(self, msg):
        self.pose = msg.pose.pose.position
        self.odom_init = True
        
    def execute(self):
        raise NotImplementedError()
    
    def conventional_detector_cb(self, msg):
        self.conventional_prob = msg.data

    def service_call_handler (self, task, service, req):
        then = rospy.get_time()
        while not rospy.is_shutdown():
            diff = rospy.get_time() - then
            if diff > self.service_timeout:
                print ('{} controls: {} service timeout'.format(task, service))
                # task failure, return a suitable outcome
                return False
            try:
                self.services[service](req)
                rospy.loginfo ('{} controls: sent a request to {} service'.format(task, service))
                return True
            except:
                print ('{} controls: execption occured in {} service'.format(task, service))

class MoveToXYZ(TaskBaseClass):
    
    def __init__(self):
        TaskBaseClass.__init__(self)
        print("Creating an approachXYZ task")
        smach.State.__init__(self, outcomes=['found_visually', 'cannot_see', 'lost'],
                            input_keys=['task_found', 'field2', 'field3'],
                            output_keys=['field1', 'field2', 'field2'])
        change_odom_response = self._change_odom(odom="dvl")
        self.then_ = rospy.get_time()
        self.timeout_ = 50
        self.target_pose_ = Pose()
        self.start_moving()
        self.present_status()
    
    def start_moving(self):
        fill_pose_data(self.target_pose_, 10, 10, 1.5, 0, 0, 0, 1)
        self._pose_cmd_pub.publish(self.target_pose_)
        rospy.loginfo("We are now moving towards the target pose" + str(self.target_pose_))
    
    def present_status(self):
        rospy.loginfo("Now, I'll check whatever is happening")
        
        while True:
            rospy.loginfo("I am seraching, prob: " + str(self._conventional_prob))
            if self._conventional_prob > .80:
                rospy.loginfo("changing to visual odom")
                change_odom_response = self._change_odom(odom="vision")
                return 'found_visually'
        
            if self.has_reached(self.target_pose_.position, 3) is True:
                rospy.loginfo("reached the position")
                return 'cannot_see'
            
            now = rospy.get_time()
            dt = now - self.then_
            if dt > self.timeout_:
                rospy.loginfo("I've lost")
                return 'lost'

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

class Examine (object):
    # to find the problem
    def __init__(self, task, stage, odom_source, target_pose):
        self.task = task
        self.stage = stage
        self.odom_source = odom_source
        self.target_pose = target_pose

    def isThrusterActive (self):
        # probably check the state of arduino and its serial connection
        # and the respective topics are published
        pass

    def isUnstable (self):
        # should not move far away and has a lot of thrust
        pass

    def isNear (self):
        # just a check using the target_pose in the userdata
        pass

    def execute(self):
        if self.isNear():
            return 'near'
        else:
            if self.neverMoved():
                if self.isThrusterActive():
                    return 'never_moved'
                else:
                    return 'thruster_inactive'
            else:
                if self.isUnstable():
                    return 'unstable'
                else:
                    return 'far'

class GateTask(TaskBaseClass):
    
    def __init__(self):
        TaskBaseClass.__init__(self)
        self.stages = ['align', 'front', 'cross']

    def align(self, userdata):
        pose = self.pose
        pose.position.y = 0
        pose.position.z = 0

        rospy.loginfo('gate controls: cmd to align to center')
        self.current_stage = 'align'
        if not self.service_call_handler ("gate", 'go_to_pose', GoToPoseRequest(pose)):
            return 'local_planner_inactive'
        if not self.service_call_handler ("gate", 'pose_reach', PoseReachRequest(20)):
            examine = Examine ("gate", self.current_stage, self.current_odom_source, pose)
            return examine.execute()
        rospy.loginfo('gate controls: aligned to the center')
        return 'success'
    
    def front(self, userdata):
        self.current_stage = 'front'
        if not self.service_call_handler ("gate", 'odom_source', ChangeOdomRequest('dvl'))
            return 'odom_inactive'
        rospy.loginfo('gate: changed the odom source to dvl')
        rospy.sleep(0.1)
        
        dist = 0 # a service to request the depth of the target 
                 # service will be implemented in the vision layer
        step_point = fill_point (dist - 2, 0, 0)
        rospy.loginfo('Gate Controls: cmd to come near the gate')
        if not self.service_call_handler ("gate", 'go_to_incremental', GoToIncrementalRequest(step_point, 0.5, 'cubic'))
            return 'local_planner_inactive'
        if not self.service_call_handler ("gate", 'trajectory_complete', TrajectoryCompleteRequest(30))
            pose = Pose()
            pose.position = step_point
            pose.orientation = Quaternion(0, 0, 0, 1)
            examine = Examine ("gate", self.current_stage, self.current_odom_source, pose)
            return examine.execute()
        rospy.loginfo('Gate Controls: near the gate')
        return 'success'

    def cross(self, userdata):
        self.current_stage = 'cross'
        step_point = fill_point (5, 0, 0)
        rospy.loginfo('Gate Controls: cmd to cross the gate')
        if not self.service_call_handler ("gate", 'go_to_incremental', GoToIncrementalRequest(step_point, 0.5, 'cubic'))
            return 'local_planner_inactive'
        if not self.service_call_handler ("gate", 'trajectory_complete', TrajectoryCompleteRequest(30))
            pose = Pose()
            pose.position = step_point
            pose.orientation = Quaternion(0, 0, 0, 1)
        rospy.loginfo('Gate Controls: gate crossed')
        return 'success'

    def execute(self, userdata):

        if not self.service_call_handler ("gate", 'current_task', CurrentTaskRequest('gate')):
            return 'vision_inactive'
        rospy.loginfo('gate: changed current task to gate')
        if not self.service_call_handler ("gate", 'odom_source', ChangeOdomRequest('vision')):
            return 'odom_inactive'
        rospy.loginfo('gate: odom source set to vision')
        self.current_odom_source = 'vision'

        if userdata.stage == 'align':
            result = self.align()
            if (result != 'sucess'):
                return result
            userdata.stage = 'front'

        if userdata.stage == 'front':
            result = self.front()
            if (result != 'success'):
                return result
            userdata.stage = 'cross'

        if userdata.stage == 'cross':
            result = self.cross()
            if (result != 'success'):
                return result

        return 'success'

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
        pass

    def execute(self):
        pass

class FindBottomTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        # request to move in a circle or helix to the local planner
        pass

    def explore(self):
        # move around to find a target
        pass

class FindFrontTarget(smach.State):

    def __init__(self):
        rospy.loginfo("I am not able to see things, let's turn around and find something")
        smach.State.__init__(self, outcomes=['found_visually', 'lost'])
    
        global status
        status = True
        self._odom_topic_sub = rospy.Subscriber('/anahita/pose_gt', Odometry, self.odometry_callback)
        self._conventional_detection_pub = rospy.Subscriber('/anahita/conventional_detection', Int32, self.conventional_detector_cb)
        self._ml_detection_pub = rospy.Subscriber('/anahita/ml', Int32, self.ml_cb)
        self._pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10)
        
        self._conventional_prob = 0
        self._ml_prob = 0
        self._then = rospy.get_time()
        self._timeout = 120
        self._step_timeout = 5
        self._target_pose = Pose()
        self._pos = self._target_pose.position
        self._orientation = self._target_pose.orientation
        self._current_pose = Pose()
        self.execute()
        

        try:
            self._change_odom = rospy.ServiceProxy('odom_source', ChangeOdom)   
        except rospy.ServiceException, e:
            print "Service call failed: %s"

        change_odom_response = self._change_odom(odom="dvl")
    
    def conventional_detector_cb(self, msg):
        self._conventional_prob = msg.data
    def ml_cb(self, msg):
        self._ml_prob = msg.data

    def odometry_callback(self, msg):
        self._current_pose = msg.pose.pose
        self._pos = msg.pose.pose.position
        self._orientation = msg.pose.pose.orientation
        _curr_roll, _curr_pitch, _curr_yaw = quaternion_to_eulerRPY(self._orientation)
        global status
        status = True 
    
    def has_reached_with_yaw(self, target_pose, curr_pose, pos_threshold, yaw_threshold):
        while (not status or rospy.is_shutdown()):
            return False
        while (calc_dist(target_pose.position, curr_pose.position) > pos_threshold or rospy.is_shutdown()):
            return False
        _curr_roll, _curr_pitch, _curr_yaw = quaternion_to_eulerRPY(curr_pose.orientation)
        _target_roll, _target_pitch, _target_yaw = quaternion_to_eulerRPY(target_pose.orientation)
        while (abs(_curr_yaw - _target_yaw) > yaw_threshold):
            return False
        return True

    def execute(self):
        # move the bot to the pose according to the odom frame of ref 
        # and then rotate the bot to search object

        #we'll use the initial yaw reference over here
        fill_pose_data(self._target_pose, self._pos.x, self._pos.y, self._pos.z, 0, 0, 0, 1)
        self._pose_cmd_pub.publish(self._target_pose)
        while self.has_reached_with_yaw(self._target_pose, self._current_pose, 1, 10) is False:
            rospy.loginfo("Still reaching the initial start position")
               
        _now = rospy.get_time()
        dt = _now - self._then
        
        _target_pos = self._pos
        _start_roll, _start_pitch, _start_yaw = quaternion_to_eulerRPY(self._current_pose.orientation)
        _target_roll = 0
        _target_pitch = 0
        _target_yaw = TO_DEGREE(_start_yaw)
        rospy.loginfo("The yaw before starting out is: " + str(_start_yaw))
        
        while dt < self._timeout:
            rospy.loginfo("Still trying to search my boi")

            _target_yaw += 30
            if(_target_yaw>180):
                _target_yaw -= 360
            rospy.loginfo('The target yaw is: ' + str(_target_yaw))
            target_quaternion = eulerRPY_to_quaternion(_target_roll, _target_pitch, TO_RADIAN(_target_yaw))
            
            fill_pose_data_pq(self._target_pose, self._pos, target_quaternion)
            self._pose_cmd_pub.publish(self._target_pose)
            rospy.sleep(self._step_timeout)

            if(self._conventional_prob > 0.8):
                rospy.loginfo("Conventionally mila hai kuch chod")  
                return 'found_visually'

            if(self._ml_prob > 0.8):
                rospy.loginfo("ML se milgya, nice bro")
                return 'found_visually'
            
        return 'lost'

    def explore(self):
        pass
        # move around to find a target

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
        pass

    def path_to_new_target(self):
        # use a service to get the path from a path algorithm
        pass

    def trace_path(self):
        # make a request to the init_waypoint_server for trajectory generation
        pass

class IdentifyTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[''], input_keys=['target'])
        rospy.loginfo("Identifying the target")
        
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
        pass

if __name__ == '__main__':
    rospy.init_node('state_machine')

    sm = smach.StateMachine(outcomes=[])
    sm.userdata.stage = 'align'

    with sm:
        smach.StateMachine.add('gate', GateTask(), transitions={
           'success' : 'path_marker', # next task
           'near' : 'gate', # to the gate again from the failed stage
           'far' : 'fail', # means some logic error in the low level controller
           'unstable' : 'reinit' # a state to make it stable
           'never_moved' : 'check_infra' # a state to check all the relevant topics to the task, to see if it is working
           'thruster_inactive' : 'fix_thruster' # a state to handle thruster failure
           'fail' : 'next_task' # a state to handle the failure
        }, remapping={
            'input_stage' : 'stage', # the point from which the state should start
            'output_stage' : 'stage' # the state where the state failed
        })
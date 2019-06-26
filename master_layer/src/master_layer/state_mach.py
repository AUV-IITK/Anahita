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
from master_layer.srv import Hold

import threading
import ctypes
import time
import os
import subprocess

# https://www.geeksforgeeks.org/python-different-ways-to-kill-a-thread/
class thread_with_exception(threading.Thread):
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name
        self.response_time = 0

    def run(self):
        now = rospy.get_time()
        try:
            rospy.wait_for_service(name)
            self.response_time = rospy.get_time() - now
        finally:
            self.response_time = rospy.get_time() - now
            print('rospy wait for service ended') 
    
    def get_time(self):
        return self.response_time

    def get_id(self): 
  
        # returns id of the respective thread 
        if hasattr(self, '_thread_id'): 
            return self._thread_id 
        for id, thread in threading._active.items(): 
            if thread is self: 
                return id
   
    def raise_exception(self): 
        thread_id = self.get_id() 
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 
              ctypes.py_object(SystemExit)) 
        if res > 1: 
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0) 
            print('Exception raise failure') 

class FailReport (object):

    def __init__(self, task='', stage='', local_pose='', 
                 global_pose='', problem='', next_task=''):
        self.task = task
        self.stage = stage 
        self.local_pose = local_pose
        self.global_pose = global_pose
        self.problem = problem
        self.next_task = next_task

class CheckTopic (object):

    def __init__(self):
        print 'initiated checktopic object'
        self.sub = None
        self.timeout = 5
        self.isActive = False
        self.msg_cnt = 0

    def check (self, topic, message_type):
        if self.sub is not None:
            self.sub.unregister()
        self.sub = rospy.Subscriber(topic, message_type, self.callback)
        rospy.sleep(2)

        then = rospy.get_time()
        while (not rospy.is_shutdown()):
            diff = rospy.get_time() - then
            if self.isActive:
                self.sub.unregister()
                return True
            if diff > self.timeout:
                self.sub.unregister()
                return False

    def callback (self, msg):
        if (self.msg_cnt > 7):
            self.isActive = True
        self.msg_cnt += 1

class CheckService (object):
    def __init__(self):
        print 'initiated checkservice object'
        self.service = None
        self.timout = 5

    def check (self, service):
        t = thread_with_exception(service)
        t.start()
        rospy.sleep(timeout)
        t.raise_exception()
        t.join()

        if t.get_time() >= self.timeout:
            return False
        return True

class Fix(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_fixable', 'success', 'fail'], 
                             input_keys=['problem'], output_keys=[])
        self.fixable_list = ['local_planner_inactive', 'odom_inactive',
                             'vision_inactive', 'camera_inactive', 'imu_inactive']
        self.num_try = 1 # no. of try to fix the problem
        self.kill_thrust_pub = rospy.Publisher('/anahita/kill_thrust', Bool, latch=True, queue_size=1)
    
    # in all the fixes make sure there is no thrust to any thruster

    def fix_local_planner(self):
        kill_thrust_pub.publish(Bool(True))
        # https://www.journaldev.com/16140/python-system-command-os-subprocess-call
        
        # first kill all the script running already
        # relaunch the local_planner script
        cmd = "cd ~/anahita_ws && launch uuv_trajectory_control cascaded_pid_dp_controller.launch"
    
        cnt = 0
        returned_value = False
        while (cnt < self.try_count):
            returned_value = subprocess.call(cmd, shell=True)
            if returned_value:
                kill_thrust_pub.publish(Bool(False))
                break
        # return true if launched
        return returned_value

    def isFixable (self, problem):
        # if the problem falls within a list of fixable things
        if problem in self.fixable_list:
            return True
        return False

    def fix_odom(self):
        kill_thrust_pub.publish(Bool(True))

        # relaunch the local_vision_node
        cmd = "cd ~/anahita_ws && launch odom_dvl_imu odom.launch"

        cnt = 0
        returned_value = False
        while (cnt < self.try_count):
            returned_value = subprocess.call(cmd, shell=True)
            if returned_value:
                kill_thrust_pub.publish(Bool(False))
                break
        # return true if launched
        return returned_value

    def fix_vision(self):
        kill_thrust_pub.publish(Bool(True))

        # first kill the local_vision_node & and change the odom source if enabled
        # relaunch the vision_node
        cmd = "cd ~/anahita_ws && launch vision_tasks vision_layer.launch env:=real"

        cnt = 0
        returned_value = False
        while (cnt < self.try_count):
            returned_value = subprocess.call(cmd, shell=True)
            if returned_value:
                kill_thrust_pub.publish(Bool(False))
                break
        # return true if launched
        return returned_value

    def fix_imu(self):
        kill_thrust_pub.publish(Bool(True))

        # relaunch the imu driver
        cmd = "cd ~/anahita_ws && launch xsens_driver xsens_driver.launch"

        cnt = 0
        returned_value = False
        while (cnt < self.try_count):
            returned_value = subprocess.call(cmd, shell=True)
            if returned_value:
                kill_thrust_pub.publish(Bool(False))
                break
        # return true if launched
        return returned_value

    def fix_camera(self):
        kill_thrust_pub.publish(Bool(True))

        # relaunch the imu driver
        cmd = "cd ~/anahita_ws && launch hardware_camera camera_logitech.launch"

        cnt = 0
        returned_value = False
        while (cnt < self.try_count):
            returned_value = subprocess.call(cmd, shell=True)
            if returned_value:
                kill_thrust_pub.publish(Bool(False))
                break
        # return true if launched
        return returned_value

    def execute(self, userdata):
        try_count = 0
        if not self.isFixable(userdata.problem):
            return 'not_fixable'
        if userdata.problem is 'local_planner_inactive':
            return self.fix_local_planner():
        elif userdata.problem is 'vision_inactive':
            return self.fix_vision()
        elif userdata.problem is 'odom_inactive':
            return self.fix_odom()
        elif userdata.problem is 'camera_inactive':
            return self.fix_camera()
        elif userdata.problem is 'imu_inactive':
            return self.fix_imu()

class RescueMode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'target_not_found'],
                             input_keys=['target'], output_keys=[''])
        print 'RescueMode Class: waiting for hold server ....'
        rospy.wait_for_service('/anahita/hold')
        self.services = dict()
        self.services['hold_vehicle'] = rospy.ServiceProxy("/anahita/hold", Hold)
        self.target_pose = Pose()

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

    # for bottom targets
    def bottom_manoeuvre (self): 
        # trace a circular trajectory which completely covers a circle
        pass

    # for front targets
    def front_manoeuvre (self):
        # rotate the bot for sometime
        pass

    def search (self, target, timeout):
        # a call to object detection layer to recognise target
        # a service to make a swiping motion for a timeout
        # if found within timeout then return true else false
        pass

    def align (self, target, timeout):
        # change the vision current_task to 'target'
        # change the odom source to 'vision'
        # make the request to local planner and wait for a timeout
        # return true within timout and false when fail
        pass

    def execute(self, userdata):
        # hold
        if not self.service_call_handler ('rescue', 'hold_vehicle', HoldRequest()):
            print ('rescue class: \'hold_vehicle\' service unresponsive')
            return 'local_planner_inactive'
        
        # search
        if not self.search (userdata.target, 120):
            return 'target_not_found'
        
        # align to target
        if not self.align (userdata.target, 30):
            examine = Examine ("rescue", 'vision', self.target_pose)
            return examine.execute()
        
        # get an approximate dist
        # feed the dist to smach and transit to transition state
        return 'success'

    def localise(self):
        # locate where is it now
        # can make a special service for this
        pass

class Transition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','timeout', 'failed', 'local_planner_inactive'],
                             input_keys=['stage', 'prev_task', 'next_task', 'focus_camera', 'time_out', 'map'],
                             output_keys=['stage'])
        self.init_pose = None
        self.final_approx_pose = None
        self.focus_camera = None
        self.time_out = None
        self.stages = ['initiated', 'visible', 'near', 'reached']
        self.odom_init = False
        self.pose = Pose()
        self.current_stage = ''
        self.current_odom_source = ''

        try:
            self.services = dict()

            print 'Intermediate Class: waiting for change odom server ....'
            rospy.wait_for_service('odom_source')
            self.services['change_odom'] = rospy.ServiceProxy('odom_source', ChangeOdom)
            print 'Intermediate Class: waiting for go_to_incremental server ....'
            rospy.wait_for_service('anahita/go_to_incremental')
            self.services['go_to_incremental'] = rospy.ServiceProxy('anahita/go_to_incremental', GoToIncremental)
            print 'Intermediate Class: waiting for go_to server ....'
            rospy.wait_for_service('anahita/go_to')
            self.services['go_to'] = rospy.ServiceProxy('anahita/go_to', GoTo)
            print 'Intermediate Class: waiting for current task server ....'
            rospy.wait_for_service('anahita/current_task')
            self.services['current_task'] = rospy.ServiceProxy('anahita/current_task', CurrentTask)
            print 'Intermediate Class: waiting for go_to_pose server ....'
            rospy.wait_for_service('anahita/go_to_pose')
            self.services['go_to_pose'] = rospy.ServiceProxy('anahita/go_to_pose', GoToPose)
            print 'Intermediate Class: waiting for trajectory_complete server ....'
            rospy.wait_for_service('anahita/trajectory_complete')
            self.services['trajectory_complete'] = rospy.ServiceProxy('anahita/trajectory_complete', TrajectoryComplete)
            print 'Intermediate Class: waiting for pose_reach server ....'
            rospy.wait_for_service('anahita/pose_reach')
            self.services['pose_reach'] = rospy.ServiceProxy('anahita/pose_reach', PoseReach)
            print 'Intermediate Class: waiting for object recognition server ....'
            rospy.wait_for_service('anahita/object_recognition')
            self.services['verify_object'] = rospy.ServiceProxy('anahita/object_recognition', VerifyObject)
        except:
            print("Service Instantiation failed")

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

    def odometry_callback(self, msg):
        self.pose = msg.pose.pose.position
        self.odom_init = True

    def execute(self, userdata):
        self.init_pose = self.pose
        next_task = userdata.next_task
        map_ = userdata.map
        target_pose = map_[next_task] # prefed map
        step_point = Point()
        step_point = target_pose.position - self.init_pose.position
        
        if not self.service_call_handler ("intermediate", 'current_task', ChangeOdomRequest(next_task)):
            return 'vision_inactive'
        rospy.loginfo('intermediate: current task set to %s', next_task)

        if not self.service_call_handler ("intermediate", 'odom_source', ChangeOdomRequest('dvl')):
            return 'odom_inactive'
        rospy.loginfo('intermediate: odom source set to dvl')
        self.current_odom_source = 'dvl'

        self.current_stage = 'initiated'
        if not self.service_call_handler ('intermediate', 'go_to_incremental', GoToIncrementalRequest(step_point, 0.5, 'libp')):
            return 'local_planner_inactive'
        if not self.service_call_handler ("intermediate", 'trajectory_complete', TrajectoryCompleteRequest(60))
            pose = Pose()
            pose.position = step_point
            pose.orientation = Quaternion(0, 0, 0, 1)
            examine = Examine ("intermediate", self.current_stage, self.current_odom_source, pose)
            return examine.execute()
        rospy.loginfo('Gate Controls: gate crossed')
        return 'success'

class TaskBaseClass(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['near', 'never_moved', 'thruster_inactive', 'unstable', 'far', 'success', 'fail'], 
                             input_keys=['input_stage'], output_keys=['output_stage', 'next_task'])

        self.odom_topic_sub = rospy.Subscriber('/anahita/pose_gt', Odometry, self.odometry_callback)
        self.trajectory_sub = rospy.Subscriber('/anahita/dp_controller/waypoints', Point, self.monitor_cb)
        self.conventional_detection_pub = rospy.Subscriber('/anahita/conventional_detection', Int32, self.conventional_detector_cb)
        self.pose_cmd_pub = rospy.Publisher('/anahita/cmd_pose', Pose, queue_size=10)
        self.conventional_prob = 0
        self.pose = Pose()
        self.odom_init = False
        self.service_timeout = 5
        self.stages = None # to set
        self.current_stage = ''
        self.task_timeout = None # to set
        self.current_odom_source = ''
        self.out_of_path_threshold = 0 # to set
        self.in_range = True
        
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
    
    def verifyObject (self, object):
        # request to object detection service
        # return true if found else false

    def odometry_callback(self, msg):
        self.pose = msg.pose.pose.position
        self.odom_init = True
        
    def monitor_cb (self, msg):
        point = msg
        if (dist (self.pose.position, point) < out_of_path_threshold):
            self.in_range = True
        else:
            self.in_range = False

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

class Examine (object):
    # to find the problem
    def __init__(self, task, odom_source, target_pose):
        self.task = task
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
            examine = Examine ("gate", self.current_stage, self.current_odom_source, pose)
            return examine.execute()
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

        userdata.next_task = 'path_marker'
        return 'success'

class PathMarker(TaskBaseClass):
    def __init__(self):
        TaskBaseClass.__init__(self)
        self.outcomes = ['target_not_found', 'success'
                         'align_fail', 'turn_fail']
        self.stages = ['align', 'turn']
        self.current_stage = 'align'

    def align(self, timeout):
        # use go_to_pose
        pass

    def turn(self, timeout):
        # use go_to_pose
        pass
    
    def execute(self, userdata):
        # verify the object in the bottom camera frame is path marker
        if not self.verifyObject('path_marker'):
            return 'target_not_found'
        if userdata.stage == 'align':
            if not self.align():
                return 'align_fail'
            self.current_stage = 'turn'
        if userdata.stage == 'turn':
            if not self.turn():
                return 'turn_fail'
        return 'success'

class BouyTask(TaskBaseClass):
    def __init__(self):
        TaskBaseClass.__init__(self)
        self.stages = ['align', 'front', 'align_again', 'hit']
        self.outcomes = ['align_fail', 'align_again_fail', 
                         'front_fail', 'hit_fail', 'success']

    def hit(self):
        # a service call to goto for hitting the buoy
        pass

    def align(self):
        pass

    def align_again(self):
        pass

    def front(self):
        pass

    def execute(self):
        if not self.verifyObject('buoy'):
            return 'target_not_found'
        if userdata.stage == 'align':
            if not self.align():
                return 'align_fail'
            self.current_stage = 'front'
        if userdata.stage == 'front':
            if not self.front():
                return 'front_fail'
            self.current_stage = 'align_again'
        if userdata.stage == 'align_again':
            if not self.align_again():
                return 'align_again_fail'
            self.current_stage = 'hit'
        if (userdata.stage == 'hit'):
            self.hit()
        return 'success'

class FireTorpedo (TaskBaseClass):
    def __init__(self):
        TaskBaseClass.__init__(self)
    
    def align(self):
        pass

    def front(self):
        pass

    def fire(self):
        pass

    def execute(self, userdata):
        # align
        # front
        # align
        # fire
        pass

class TorpedoTask(TaskBaseClass):
    def __init__(self):
        TaskBaseClass.__init__(self)
        self.stages = ['stabilise', 'fire_torpedo', 'get_back',
                       'fire_torpedo_again']
        self.outcomes =['stabilise_fail', 'torpedo_fire_fail',
                        'target_not_found', 'success']
        self.current_stage = 'stabilise'

    def stabilise(self, timeout):
        pass

    def fire_torpedo(self, timeout):
        torpedo_sm = smach.StateMachine()
        torpedo_sm.userdata.stage = 'align'
        with torpedo_sm:
            smach.StateMachine.add('fire_torpedo', FireTorpedo(), transitions={
                'success': 'success'
                'align_fail': 'fire_torpedo'
                'align_again_fail': 'fire_torpedo'
                'fire_fail': 'fail'
                'front_fail': 'fire_torpedo'
            }, remapping={
                'stage': 'stage'
            })
        outcome = torpedo_sm.execute()
        if (outcome == 'fail'):
            return False
        return True

    def get_back(self, timeout):
        pass

    def get_positions (self):
        pass

    def execute(self, userdata):
        # verify
        if not self.verifyObject('buoy'):
            return 'target_not_found'
        # stabilise
        if (userdata.stage == 'stabilise'):
            if not self.stabilise():
                return 'stabilise_fail'
            # note the positions of the torpedo holes
            self.get_positions()
            userdata.stage = 'fire_torpedo'
        # fire 1st torpedo
        if userdata.stage == 'fire_torpedo':
            if not self.fire_torpedo():
                return 'torpedo_fire_fail'
            userdata.stage = 'get_back'
        # get back a bit
        if userdata.stage == 'get_back':
            if not self.get_back():
                return 'get_back_fail'
            userdata.stage = 'fire_torpedo_again'
        # fire 2nd torpedo
        if userdata.stage == 'fire_torpedo_again':
            if not self.fire_torpedo():
                return 'torpedo_fire_again_fail'
        return 'success'

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

if __name__ == '__main__':
    rospy.init_node('state_machine')

    sm = smach.StateMachine(outcomes=[])
    sm.userdata.stage = 'align'
    sm.userdata.next_task = 'gate'
    sm.userdata.prev_task = ''

    with sm:
        smach.StateMachine.add('gate', GateTask(), transitions={
           'success' : 'intermediate', # next task
           'near' : 'gate', # to the gate again from the failed stage
           'far' : 'fail', # means some logic error in the low level controller
           'unstable' : 'reinit' # a state to make it stable
           'never_moved' : 'check_infra' # a state to check all the relevant topics to the task, to see if it is working
           'thruster_inactive' : 'fix_thruster' # a state to handle thruster failure
           'fail' : 'next_task' # a state to handle the failure
        }, remapping={
            'input_stage' : 'stage', # the point from which the state should start
            'output_stage' : 'stage' # the state where the state failed
            'next_task' : 'next_task'
        })
        smach.StateMachine.add('intermediate', Transition(), transitions={
            'success' : 'intermediate',
            'fail' : 'rescue'
        }, remapping= {})
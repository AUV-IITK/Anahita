#!/usr/bin/env python

import rospy
import smach

class Transition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def get_path():
        pass
    
    def get_trajectory();
        pass

    def get_trajectory():
        pass

    def activate_dp_controller():
        pass

    def monitor():
        pass

    def get_status():
        pass

    def get_result():
        pass
    
    def execute(self, userdata):
        pass

class TaskBaseClass(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['random_outcome1', 'random_outcome2'])

    def get_plan():
        pass
    
    def execute();
        pass
    
    def get_status():
        pass

    def get_result():
        pass
    
    def monitor():
        pass

    def execute():
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

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Transition', Transition(), 
                               transitions={'outcome1':'FindTarget', 
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('FindTarget', FindTarget(), 
                               transitions={'outcome2':'E'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
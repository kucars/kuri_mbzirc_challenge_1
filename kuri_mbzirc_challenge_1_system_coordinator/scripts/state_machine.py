#!/usr/bin/env python

import roslib; 
import rospy
import smach
import smach_ros
import time


# define state : exploration
class exploration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move_to_waypoint' , 'hovering'])
        self.exploration_time=0
       
    def execute(self, userdata):
        rospy.loginfo('Executing state EXPLORATION')
	time.sleep(1)
	if self.exploration_time<3:
        	self.exploration_time+=1
                return 'move_to_waypoint'
        else:
        	return 'hovering'
       
# define state : target detection 
class target_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detectingTarget','targetDetected'])
        self.counter=0

    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH')
        time.sleep(2)
        if self.counter<3:
                self.counter+=1
                return 'detectingTarget'
        else:
        	return 'targetDetected'
            
# define state : marker_tracking
class marker_tracking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['visible','notVisible'])
	self.counter=0
    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH')
        time.sleep(2)
        if self.counter <3 :
                self.counter+=1
                return 'notVisible'
        else: 
        	return 'visible'


# define state : marker_detection
class trajectory_following(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached','following'])
	self.counter=0
    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH')
        time.sleep(2)
        if self.counter <3 :
                self.counter+=1
                return 'following'
        else: 
        	return 'reached'


# define state : marker_detection
class docking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['docking','docked'])
	self.counter=0
    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH')
        time.sleep(2)
        if self.counter <3 :
                self.counter+=1
                return 'docking'
        else: 
        	return 'docked'


# main
class Challenge1():
	
	rospy.init_node('MBZIRC_ch1_state_machine')
        # Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['Done'])
	with sm: 
		# add states to the container 
		smach.StateMachine.add('EXPLORATION', exploration(),
                                transitions={'move_to_waypoint':'EXPLORATION',
                                             'hovering':'TARGET_DETECTION'})

		smach.StateMachine.add('TARGET_DETECTION', target_detection(),
                                transitions={'detectingTarget':'TARGET_DETECTION',
                                             'targetDetected':'MARKER_DETECTION'})

		smach.StateMachine.add('MARKER_TRACKING', marker_tracking(),
                                transitions={'visible':'TRAJECTORY_FOLLOWING',
                                             'notVisible':'EXPLORATION'})

		smach.StateMachine.add('TRAJECTORY_FOLLOWING', trajectory_following(),
                                transitions={'reached':'DOCKING',
                                             'following':'MARKER_DETECTION'})

		smach.StateMachine.add('DOCKING', docking(),
                                transitions={'docking':'DOCKING',
                                             'docked':'Done'})


	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()
        rospy.spin()
        sis.stop()



if __name__ == '__main__':
	Challenge1()

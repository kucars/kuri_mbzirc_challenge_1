#!/usr/bin/env python

import roslib; 
import rospy
import actionlib
import smach
import smach_ros
import time
from std_msgs.msg import String
from kuri_mbzirc_challenge_1_msgs.msg import explorationAction
import kuri_mbzirc_challenge_1_msgs.msg





# define state : exploration
class exploration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move_to_waypoint' , 'hovering'])


    def execute(self, userdata):
    	client = actionlib.SimpleActionClient('exploration', explorationAction)
        client.wait_for_server()
        goal_ex = kuri_mbzirc_challenge_1_msgs.msg.explorationGoal()
        # Fill in the goal here
        goal_ex.startMission_str = 'startMission'
        client.send_goal(goal_ex)
        r = client.wait_for_result()
        if r == True:
            return 'hovering'
       
# define state : target detection 
class target_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detectingTarget','targetDetected'])
	self.endDetection_sub = rospy.Subscriber('/endDetection', String, self.truckDetected)
        self.startDetection_pub = rospy.Publisher('/startDetection' , String, queue_size=1)
	self.moveToTracking = False 
        #self.counter=0

    def truckDetected(self , topic):
	if topic.data == 'TDetected':
		self.moveToTracking = True 


    def execute(self, userdata):
        rospy.loginfo('Executing state TARGET_DETECTION')
	msg1 = 'startDetection' 
	self.startDetection_pub.publish(msg1) 
        time.sleep(2)
	if self.moveToTracking == False: 
		return 'detectingTarget'
	else:
        	return 'targetDetected'

    
# define state : marker_tracking
class marker_tracking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['visible','notVisible'])
	self.endTracking_sub = rospy.Subscriber('/endTracking', String, self.marekerTracking)
        self.startTracking_pub = rospy.Publisher('/startTracking' , String, queue_size=1)
	self.moveToTrajectory = False 
	#self.counter=0

    def marekerTracking(self , topic):
	if topic.data == 'inFov':
		self.moveToTrajectory = True 



    def execute(self, userdata):
        rospy.loginfo('Executing state MARKER_TRACKING')
	msg2 = 'startTracking' 
	self.startTracking_pub.publish(msg2) 
        time.sleep(2)
	if self.moveToTrajectory == False:
		return 'notVisible'
	else: 
		return 'visible'
	



# define state : marker_detection
class trajectory_following(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached','following'])
	self.endTrajectory_sub = rospy.Subscriber('/endTrajectory', String, self.trajectoryFollowing)
        self.startTrajectory_pub = rospy.Publisher('/startTrajectory' , String, queue_size=1)
	self.moveToDocking = False 
	#self.counter=0


    def trajectoryFollowing(self , topic):
	if topic.data == 'onTopOfMarker':
		self.moveToDocking = True 


    def execute(self, userdata):
        rospy.loginfo('Executing state TRAJECTORY_FOLLOWING')
	msg3 = 'startFollowing' 
	self.startTrajectory_pub.publish(msg3) 
        time.sleep(2)
        if self.moveToDocking == False:
        	return 'following'
        else: 
        	return 'reached'

# define state : marker_detection
class docking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['docking','docked'])
	self.endDocking_sub = rospy.Subscriber('/endDocking', String, self.docking)
        self.startDocking_pub = rospy.Publisher('/startDocking' , String, queue_size=1)
	self.moveToEnd = False
	#self.counter=0

    def docking(self , topic):
	if topic.data == 'dockDone':
		self.moveToEnd = True 


    def execute(self, userdata):
        rospy.loginfo('Executing state DOCKING')
	msg4 = 'startDocking' 
	self.startDocking_pub.publish(msg4) 
        time.sleep(2)
        if self.moveToEnd == False:
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
		#smach.StateMachine.add('INIT', Initilization(),
                 #               transitions={'start':'EXPLORATION'})

		smach.StateMachine.add('EXPLORATION', exploration(),
                                transitions={'move_to_waypoint':'EXPLORATION',
                                             'hovering':'TARGET_DETECTION'})

		smach.StateMachine.add('TARGET_DETECTION', target_detection(),
                                transitions={'detectingTarget':'TARGET_DETECTION',
                                             'targetDetected':'MARKER_TRACKING'})

		smach.StateMachine.add('MARKER_TRACKING', marker_tracking(),
                                transitions={'visible':'TRAJECTORY_FOLLOWING',
                                             'notVisible':'EXPLORATION'})

		smach.StateMachine.add('TRAJECTORY_FOLLOWING', trajectory_following(),
                                transitions={'reached':'DOCKING',
                                             'following':'MARKER_TRACKING'})

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

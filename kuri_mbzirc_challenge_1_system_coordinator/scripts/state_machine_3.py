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
        self.endexploration_sub = rospy.Subscriber('/endMission', String, self.missionCallback)
        self.startexploration_pub = rospy.Publisher('/startMission' , String, queue_size=1)
        self.tracked = False 

    def missionCallback(self , topic):
        print "CallBack " 
	if topic.data == "detected":
		self.tracked = True 


    def execute(self, userdata):
        msg1 = 'startMission' 
	self.startexploration_pub.publish(msg1) 
	
        if self.tracked == True:
	    print "Reached, lets move on " 
            return 'hovering'
	    self.tracked = False 
	else: 
 	    print "moveWAYPOINT " 
	    return 'move_to_waypoint' 
       

# define state : marker_detection
class trajectory_following(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached','following'])
	self.endTrajectory_sub = rospy.Subscriber('/endLanding', String, self.trajectoryFollowing)
        self.startTrajectory_pub = rospy.Publisher('/startLanding' , String, queue_size=1)
	self.moveToDocking = False 
	#self.counter=0


    def trajectoryFollowing(self , topic):
	if topic.data == 'onTopOfMarker':
		self.moveToDocking = True 


    def execute(self, userdata):
        rospy.loginfo('Executing state TRAJECTORY_FOLLOWING')
	msg3 = 'startLanding' 
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
                                             'hovering':'TRAJECTORY_FOLLOWING'})


		smach.StateMachine.add('TRAJECTORY_FOLLOWING', trajectory_following(),
                                transitions={'reached':'DOCKING',
                                             'following':'TRAJECTORY_FOLLOWING'})

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

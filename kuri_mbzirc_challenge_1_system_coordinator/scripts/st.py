#!/usr/bin/env python

import roslib; 
import rospy
import actionlib
import smach
import smach_ros
import time
from std_msgs.msg import String
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import PoseStamped
from kuri_mbzirc_challenge_1_msgs.srv import PES 
from kuri_mbzirc_challenge_1_msgs.srv import navigation
from kuri_msgs.msg import Object
from cftld_ros.msg import Track
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

FixPoseX = 465711.21694 
FixPoseY = 5249465.0281 
FixPoseZ = 10  

# define state : marker_detection
class marker_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Tracked','Stay'],output_keys=['X','Y','Z'])
        self.box_sub = rospy.Subscriber('/ch1/marker_bb', RegionOfInterest, self.boxCallback)
	self.h = 0 
	self.resp1 = Object() 
	self.res = PoseStamped () 
        self.a = 0 
        self.b = 0 
	self.count = 0 
    	self.lastx = 0 
	self.lasty = 0 


    def boxCallback(self , topic): 
	self.h = topic.height
	self.a = topic.x_offset + (topic.width/2.0)
	self.b = topic.y_offset + (topic.height/2.0)
	client = rospy.ServiceProxy('position_estimation', PES)
	self.resp1 = client(self.a, self.b)
	self.lastx = self.resp1.X
	self.lasty = self.resp1.Y	
		
    def execute(self, userdata):
        rospy.loginfo('Executing state MARKER_DETECTION')
        time.sleep(2)
	for i in range(1):
		#Standered Daviation 
		if self.h!=0:
			self.count = self.cout + 1 
	        	
	if self.count == 1: 
		userdata.X = self.lastx
		userdata.Y = self.lasty
		userdata.Z = 10 
		return 'Tracked'
	else: 
		return 'Stay'
	      
# main
class Challenge1():
	rospy.init_node('MBZIRC_ch1_state_machine')
	sm = StateMachine(['succeeded','aborted','preempted'])
	with sm:

 		def exploration_request_cb (request):
			land_request = navigation().Request	
			land_request.type=1
			land_request.wayPointX = FixPoseX
			land_request.wayPointY = FixPoseY
			land_request.wayPointZ = FixPoseZ
			land_request.LDFlag = 0

		def landing_request_cb (userdata, request):
			land_request = navigation().Request	
			land_request.type=2
			land_request.wayPointX = userdata.X
			land_request.wayPointY = userdata.Y
			land_request.wayPointZ = 10			
			if False == True:
				land_request.LDFlag = 1
			else :   
				land_request.LDFlag = 0

	

 		
    		smach.StateMachine.add('EXPLORATION',
					ServiceState('controller_service',
					navigation,
					request  = exploration_request_cb),
                           transitions={'succeeded':'MARKER_DETECTION' , 'aborted':'EXPLORATION'})

		smach.StateMachine.add('MARKER_DETECTION', marker_detection(),
                                transitions={'Tracked':'LANDING','Stay':'MARKER_DETECTION'})

		smach.StateMachine.add('LANDING',
					ServiceState('controller_service',
					navigation,
					request  = landing_request_cb,
 					input_keys = ['X','Y','Z']),
                           transitions={'succeeded':'DOCKING','aborted':'MARKER_DETECTION'})



	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()
        rospy.spin()
        sis.stop()


if __name__ == '__main__':
	Challenge1()

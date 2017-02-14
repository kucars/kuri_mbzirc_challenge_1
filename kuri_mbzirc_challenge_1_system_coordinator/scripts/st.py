#!/usr/bin/env python

import roslib; 
import rospy
import actionlib
import smach
import smach_ros
import time
import math 
from std_msgs.msg import String
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import PoseStamped
from kuri_mbzirc_challenge_1_msgs.srv import PES 
from kuri_mbzirc_challenge_1_msgs.srv import navigation , navigationResponse , navigationRequest
from kuri_msgs.msg import Object
from cftld_ros.msg import Track
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from smach_ros import ServiceState

############################################################################3
# This fixed position os for the actual position of the center of the areana OR the start point of the longest straight line 
############################################################################3
# now these points are in local frame 
FixPoseX = 0 # 465711.21694 
FixPoseY = 0 #5249465.0281 
FixPoseZ = 10  

sleep_time = 0.3
 
# define state : initialization
class initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['srvtype','wayPointX','wayPointY','wayPointZ','LDFlag'])
    def execute(self, userdata):
        rospy.loginfo('Executing state INITIALIZATION')
        userdata.srvtype = 1 
        userdata.wayPointX = FixPoseX
        userdata.wayPointY = FixPoseY
        userdata.wayPointZ = FixPoseZ
        userdata.LDFlag = 0 
        time.sleep(sleep_time)
        #I should publish to the offboard control to publish 
        return 'succeeded'


#######################################################################################################################3
# This state has been done like this becuase the tracking and detection are just nodes and not services nor actions libs
#######################################################################################################################3
# define state : marker_detection
class marker_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Tracked','explore'],output_keys=['srvtype','wayPointX','wayPointY','wayPointZ','LDFlag'])
	# name of the topics that I wanted to use 
        #visptracker_data'
	#/ch1/marker_bb
	
	# Detection and tracking params 
	self.h = 0 
        self.a = 0 
        self.b = 0 
	self.resp1 = Object() 
	self.res = PoseStamped () 
	
    	# estimations params 
    	self.lastx = 0 
	self.lasty = 0 
	self.lastz = 1
	
	# robot pose 
	self.px =0 
	self.py = 0 
	self.pz =0

	# Standered Daviation params 
	self.detectionCount = 0 
	self.hArray = [] 
	self.SD = 100000 
	self.hSum = 0 
	self.hMean = 0 
	
	# control flags 
	self.TrackedForFirstTime = False 
	self.count = 0 
	self.Startflag = False

    def localGlobalPoseCallback(self, topic):
      self.px = topic.pose.pose.position.x
      self.py = topic.pose.pose.position.y 
      self.pz = topic.pose.pose.position.z
      

    def boxCallback(self , topic): 
	self.h = topic.height
	self.w = topic.height
	##############################################3
	#Make sure from the following equations 
	##############################################3
	
	# finding the center point in the Image 
	self.a = topic.x_offset + (self.w/2.0)
	self.b = topic.y_offset + (self.h/2.0)
	
	# calling the estimation service 
	client = rospy.ServiceProxy('position_estimation', PES)
	self.resp1 = client(self.a, self.b)
	self.lastx = self.resp1.X
	self.lasty = self.resp1.Y	
	self.lastz = self.resp1.Z
	
	# for each 4 reading Find the standered daviation
	if (self.detectionCount >= 4):
	    #print 'sum' , self.hSum
	    #print 'count' , self.detectionCount
    	    #print 'len' , len(self.hArray)
    	    
	    if ( self.hArray[0] !=0   and self.hArray[1] != 0 and self.hArray[2] !=0 and self.hArray[3] !=0 ): 
		self.hMean = self.hSum / 4 
		self.SD = 1.0* math.sqrt ((1/4) * ( (self.hArray[0]-self.hMean) +(self.hArray[1]-self.hMean)+(self.hArray[2]-self.hMean)+(self.hArray[3]-self.hMean) ) )
		print 'SD' , self.SD 
		if (self.SD  < 0.02):
		  self.detectionCount = 0
		  self.Startflag = True 
		  self.hSum = 0 
		  self.hMean = 0
		  del self.hArray[:]
		  self.SD = 100000 
		else :
		  self.detectionCount = 0 
		  self.Startflag = False 
		  self.hSum = 0 
		  self.hMean = 0
		  del self.hArray[:]
		  self.SD = 10000 
	else: 
	  print 'element' , self.detectionCount , '   value' , self.h
	  self.detectionCount = self.detectionCount + 1
	  self.hSum = self.hSum + self.h
	  self.hArray.append(self.h)
	  
    def execute(self, userdata):
        rospy.loginfo('Executing state MARKER_DETECTION')
        #subscriber 
        box_sub = rospy.Subscriber('/ch1/marker_bb', RegionOfInterest, self.boxCallback)
        pose_global_local_sub = rospy.Subscriber('/mavros/global_position/local', Odometry, self.localGlobalPoseCallback);
        time.sleep(2) 
        
	if self.Startflag == True: 
		self.TrackedForFirstTime= True 
	  	userdata.srvtype = 2
		userdata.wayPointX = self.lastx
		userdata.wayPointY = self.lasty
		userdata.wayPointZ = self.lastz
		userdata.LDFlag = 0
		######3 should the following be changed to False or not 
		self.Startflag = False
		print "2- Tracked" 
	else:
		if self.TrackedForFirstTime == True: 		  
		  userdata.srvtype = 2
		  userdata.wayPointX = self.px
		  userdata.wayPointY = self.py
		  userdata.wayPointZ = self.pz + 0.05 # Should I increase the distance  
		  userdata.LDFlag = 0 
		  self.count = self.count + 1  # how can I make it with time not counter 
		  print "3- Moving up"
		  if (self.count == 5 ) :
		     # start Over I should add a predefined point 
		     userdata.srvtype = 1
		     userdata.wayPointX = FixPoseX
		     userdata.wayPointY = FixPoseY
		     userdata.wayPointZ = FixPoseZ 
		     userdata.LDFlag = 0 
		     self.TrackedForFirstTime = False      
		     print "4- Moving to start point"
		     self.count = 0 
		     return 'explore'
		else: 
		  userdata.srvtype = 2
		  userdata.wayPointX = self.px
		  userdata.wayPointY = self.py
		  userdata.wayPointZ = self.pz 
		  userdata.LDFlag = 0 
		  print "1- Hovering exes"
	
	if (abs(self.px - self.lastx) < 0.2 and abs(self.py - self.lasty) <0.2 and abs(self.pz - self.lastz) < 0.2 ):
		  userdata.LDFlag = 1 
	  	
	##############3
	# do they really work ??? 
	##############3
	box_sub.unregister()
	pose_global_local_sub.unregister()
	return 'Tracked' 	
	      
# main
class Challenge1():
	rospy.init_node('MBZIRC_ch1_state_machine')
	#sm = StateMachine(['succeeded','aborted','preempted'])
	sm = smach.StateMachine(outcomes=['Done'])

	with sm:
	  
		def exploration_response_cb(userdata, response):
			print "navigationResponse" , navigationResponse
			if response.navResponse == 'succeeded' :
			   return 'succeeded' 
			  
			else :
			  return 'aborted'
			
		def landing_response_cb(userdata, response):
			#print "navigationResponse" , navigationResponse
			print 'response.navResponse' , response.navResponse 
			if response.navResponse == 'succeeded' :
			   return 'succeeded' 
			else :
  			  #print 'FOREVER LANDING' 
			  return 'aborted'			
		      
		smach.StateMachine.add('INITIATING', initialization(),
                            transitions={'succeeded':'EXPLORATION'})
	 		
	 	############################################3
	 	# call service with request 1 and userdata #
	 	############################################3
    		smach.StateMachine.add('EXPLORATION',
					smach_ros.ServiceState(
					'controller_service', navigation,
					request_slots  = ['srvtype','wayPointX','wayPointY','wayPointZ','LDFlag'],
					#request_cb  = exploration_request_cb,
					response_cb=exploration_response_cb,
					input_keys = ['srvtype','wayPointX','wayPointY','wayPointZ','LDFlag']),
					transitions={'succeeded':'MARKER_DETECTION' , 'aborted':'EXPLORATION','preempted':'MARKER_DETECTION'})



		############################################3
	 	# MArker Detection and tracking 	    # in this state the drone will not perform RTL becuase it is build it to hover 
	 	############################################3
		smach.StateMachine.add('MARKER_DETECTION', marker_detection(),
                                transitions={'Tracked':'LANDING','explore':'EXPLORATION'})


	 	############################################3
	 	# call service with request 2 and userdata #
	 	############################################3
		smach.StateMachine.add('LANDING',
					ServiceState('controller_service',
					navigation,
					#request_cb  = landing_request_cb,
					request_slots  = ['srvtype','wayPointX','wayPointY','wayPointZ','LDFlag'],
					response_cb=landing_response_cb,
					input_keys = ['srvtype','wayPointX','wayPointY','wayPointZ','LDFlag']),
                           transitions={'succeeded':'Done','aborted':'MARKER_DETECTION','preempted':'MARKER_DETECTION'})
					
		#######################################################3
		#Before the Done There should be a disarming state 
		#######################################################3

		#I should publish to the offboard control to change to manual mode and then to Disarm 


	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()
        rospy.spin()
        sis.stop()
        ############################################3
        # smach view is not working so no visialization tool for the states right now
        ############################################3
        
        


if __name__ == '__main__':
	Challenge1()

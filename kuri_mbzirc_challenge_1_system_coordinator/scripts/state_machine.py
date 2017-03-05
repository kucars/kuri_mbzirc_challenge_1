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
import tf 

############################################################################3
# This fixed position os for the actual position of the center of the areana OR the start point of the longest straight line 
############################################################################3
# now these points are in local frame 
FixPoseX =  rospy.get_param("FixPoseX") # 465711.21694
FixPoseY = rospy.get_param("FixPoseY")#5249465.0281 
FixPoseZ = rospy.get_param("FixPoseZ")
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
	self.px = 0 
	self.py = 0 
	self.pz = 0

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
	self.dataAvailable = False
	self.listener = tf.TransformListener()
	self.preT = 0 
	self.start = True 
	
    def localGlobalPoseCallback(self, topic):
      self.px = topic.pose.pose.position.x
      self.py = topic.pose.pose.position.y 
      self.pz = topic.pose.pose.position.z
      
    def estimatedPoseCallback(self, topic):
        if (self.start):
	      self.preT = topic.pose.position.z
	      self.start = False
	      self.TrackedForFirstTime = True 

	else: 
	      self.listener.waitForTransform("base_link",'downward_cam_optical_frame',rospy.Time() , rospy.Duration(1.0))
	      (staticDistance , Quat) = self.listener.lookupTransform('base_link','downward_cam_optical_frame',rospy.Time(0));
	      eulerAngles = tf.transformations.euler_from_quaternion(Quat)
	      yaw = eulerAngles[2] 
	      theta1 = 0 # this is the head of the marker 
	      self.lastx = staticDistance[0] + (topic.pose.position.x * math.cos( theta1 + yaw) ) 
	      self.lasty = staticDistance[1] + (topic.pose.position.y * math.sin( theta1 + yaw) ) #topic.pose.position.y 
	      self.lastz = staticDistance[2] + (topic.pose.position.z * math.cos( 3.14159265359) ) #topic.pose.position.z 
	      '''
	      print 'yaw'   ,  yaw 
	      
	      print 'topic.pose.position.x' , topic.pose.position.x
	      print 'topic.pose.position.y' , topic.pose.position.y
	      print 'topic.pose.position.z' , topic.pose.position.z
	      print 'staticDistance[0]' , staticDistance[0]
	      print 'staticDistance[1]' , staticDistance[1]
	      print 'staticDistance[2]' , staticDistance[2] 
	      print 'self.lastx' , self.lastx
	      print 'self.lasty' , self.lasty
	      print 'self.lastz' , self.lastz 
	      '''
	      
	      # check for the sirst time that the tracker worked 
	      print 'topic.pose.position.z - self.preT' , topic.pose.position.z - self.preT 
	      if topic.pose.position.z - self.preT != 0: 
		self.dataAvailable = True 
	      else: 
		self.dataAvailable = False 

	      self.preT = topic.pose.position.z

  
    def execute(self, userdata):
        rospy.loginfo('Executing state MARKER_DETECTION')
        #subscriber 
        pose_estimaton_sub = rospy.Subscriber('visptracker_pose', PoseStamped, self.estimatedPoseCallback)
        #box_sub = rospy.Subscriber('/ch1/marker_bb', RegionOfInterest, self.boxCallback)
        pose_global_local_sub = rospy.Subscriber('/mavros/global_position/local', Odometry, self.localGlobalPoseCallback);
        time.sleep(2) 
        
        print "self.dataAvailable" , self.dataAvailable
        
        if self.dataAvailable: 
	  userdata.srvtype = 2
	  userdata.wayPointX = self.lastx
	  userdata.wayPointY = self.lasty
	  userdata.wayPointZ = self.lastz
	  userdata.LDFlag = 0
	  self.TrackedForFirstTime = True
	  print "Keep Landing"

	else:
	      if (self.TrackedForFirstTime == False):
		      userdata.srvtype = 2
		      userdata.wayPointX = 0 
		      userdata.wayPointY = 0 
		      userdata.wayPointZ = 0 
		      userdata.LDFlag = 0 
		      print "Hover till the truck is seen"
	      else: 
		      if (self.count == 5 ) :
			    # start Over I should add a predefined point 
			    userdata.srvtype = 1
			    userdata.wayPointX = FixPoseX
			    userdata.wayPointY = FixPoseY
			    userdata.wayPointZ = FixPoseZ 
			    userdata.LDFlag = 0 
			    self.TrackedForFirstTime = False      
			    print "4- Moving to start point"
			    self.start = True 
			    self.count = 0 
			    return 'explore'
		      else: 
			    userdata.srvtype = 2
			    userdata.wayPointX = 0 #self.px
			    userdata.wayPointY = 0 #self.py
			    userdata.wayPointZ = 0.05 # Should I increase the distance  
			    userdata.LDFlag = 0 
			    self.count = self.count + 1  # how can I make it with time not counter 
			    print "3- Moving up"

	
	if (abs(self.px - self.lastx) < 0.2 and abs(self.py - self.lasty) <0.2 and abs(self.pz - self.lastz) < 0.2 ):
		  userdata.LDFlag = 1 
	  	
	pose_estimaton_sub.unregister()
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
					'controllerService', navigation,
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
					ServiceState('controllerService',
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

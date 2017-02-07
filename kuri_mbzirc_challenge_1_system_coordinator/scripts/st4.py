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
import kuri_mbzirc_challenge_1_msgs.msg
from kuri_mbzirc_challenge_1_msgs.srv import PES 
from kuri_msgs.msg import Object
from cftld_ros.msg import Track
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

xPose = 0 
yPose = 0 
zPose = 0 
arenaPosex = 465711.21694
arenaPosey = 5249465.0281
arenaPosez = 10

# define state : exploration
class exploration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['StayInExploration' , 'Move2Landing'])
        self.goal_pub = rospy.Publisher('/kuri_offboard_attitude_control_test/goal' , PoseStamped, queue_size=10)
        self.pose_global_local_sub = rospy.Subscriber('/mavros/global_position/local', Odometry, self.localGlobalPoseCallback);
        #self.box_sub = rospy.Subscriber('/ch1/marker_bb', RegionOfInterest, self.boxCallback)
        self.box_sub = rospy.Subscriber('/cftld/track', Track, self.boxCallback)
        self.px = 0 
        self.py = 0 
        self.pz = 0 
        self.h = 0 
        self.errorx = 1000000000
	self.errory = 1000000000
	self.errorz = 1000000000
	
    def localGlobalPoseCallback(self, topic):
      self.px = topic.pose.pose.position.x
      self.py = topic.pose.pose.position.y 
      self.pz = topic.pose.pose.position.z
      
    #indication that the marker has been detected   
    def boxCallback(self , topic):
      self.h =  topic.roi.height
      #print "self.h" , self.h 

    def execute(self, userdata):
        self.errorx = abs(self.px - arenaPosex) 
        self.errory = abs(self.py - arenaPosey)
        self.errorz = abs(self.pz - arenaPosez)
        
        if (self.errorx < 0.2) and (self.errory < 0.2) and (self.errorz < 2): #self.h != 0 :	     
	      msg2a = PoseStamped() 
	      #Hovering
	      msg2a.pose.position.x = self.px
	      msg2a.pose.position.y = self.py
	      msg2a.pose.position.z = self.pz
	      self.goal_pub.publish(msg2a)
	      self.h = 0 ; 
	      return 'Move2Landing'
	else: 
 	    msg2 = PoseStamped()  
 	    msg2.pose.position.x = arenaPosex #465711.21694  # 0 #47.3977809
	    msg2.pose.position.y = arenaPosey # 5249465.0281 #0 # 8.5455931
	    msg2.pose.position.z = arenaPosez #10 
	    self.h=0
	    self.goal_pub.publish(msg2)
	    return 'StayInExploration' 
       

# define state : marker_detection
class trajectory_following(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Move2Docking','StayInLanding'])
        self.goal_pub = rospy.Publisher('/kuri_offboard_attitude_control_test/goal' , PoseStamped, queue_size=1)
        #self.box_sub = rospy.Subscriber('/ch1/marker_bb', RegionOfInterest, self.boxCallback)
        self.box_sub = rospy.Subscriber('/cftld/track', Track, self.boxCallback)
        self.pose_global_local_sub = rospy.Subscriber('/mavros/global_position/local', Odometry, self.localGlobalPoseCallback2);
	self.h = 0 
	self.resp1 = Object() 
	self.res = PoseStamped() 
        self.a = 0 
        self.b = 0 
        self.lastx = 0 
        self.lasty = 0 
        self.zzpose = 0 
        self.xxpose = 0 
        self.yypose = 0 
        self.errorxx = 1000000000
	self.erroryy = 1000000000
	self.errorzz = 1000000000
        
        
    def localGlobalPoseCallback2(self, topic):
      self.xxpose = topic.pose.pose.position.x 
      self.yypose = topic.pose.pose.position.y
      self.zzpose = topic.pose.pose.position.z
      
    
    def boxCallback(self , topic): 
	self.h = topic.roi.height
	self.a = topic.roi.x_offset + (topic.roi.width/2.0)
	self.b = topic.roi.y_offset + (topic.roi.height/2.0)
	client = rospy.ServiceProxy('position_estimation', PES)
	self.resp1 = client(self.a, self.b)
	self.lastx = self.resp1.X
	self.lasty = self.resp1.Y
	#print "self.h" , self.h

 
    def execute(self, userdata):
        rospy.loginfo('Executing state TRAJECTORY_FOLLOWING')
        time.sleep(2)
        print "self.h IN execute" , self.h
        if self.h !=0 :
	      print "landing"
	      msg4 = PoseStamped() 
	      print "Marker Position in X" , self.resp1.X  , "Robot Position X" , xPose  , "    " , self.xxpose
	      print "Marker Position in Y" , self.resp1.Y  , "Robot Position Y" , yPose  , "    " , self.yypose 	      
	      msg4.pose.position.x = self.resp1.X
	      msg4.pose.position.y = self.resp1.Y
	      self.lastx = self.resp1.X
	      self.lasty = self.resp1.Y
	      msg4.pose.position.z = 1.0
	      self.h = 0 
	      self.goal_pub.publish(msg4)
	      return 'StayInLanding'
	else: 
	    if abs(self.lastx - self.xxpose) < 1 and abs(self.lasty - self.yypose) < 1 and (self.zzpose < 2): 
	        print "reached the marker" 
		msg4a = PoseStamped() 
		msg4a.pose.position.x = self.xxpose 
		msg4a.pose.position.y = self.yypose
		msg4a.pose.position.z = self.zzpose - 0.5
		self.h = 0 
		self.goal_pub.publish(msg4a)
		return 'Move2Docking'
	      
	    else: 
		print "moveing up" 
		msg4b = PoseStamped( ) 
		msg4b.pose.position.x = self.xxpose  
		msg4b.pose.position.y = self.yypose 
		msg4b.pose.position.z = self.zzpose  + 0.2
		self.h = 0
		self.goal_pub.publish(msg4b)
		return 'StayInLanding'
	      

	
# define state : docking
class docking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['docking','docked'])
        self.goal_pub = rospy.Publisher('/kuri_offboard_attitude_control_test/goal' , PoseStamped, queue_size=1)
        self.pose_global_local_sub = rospy.Subscriber('/mavros/global_position/local', Odometry, self.localGlobalPoseCallback);
        self.moveToEnd = False
        self.posex = 0 
        self.posey = 0 
        self.posez = 0 
 
    def localGlobalPoseCallback(self, topic):
      self.posex = topic.pose.pose.position.x 
      self.posey = topic.pose.pose.position.y
      self.posez = topic.pose.pose.position.z

    def execute(self, userdata):
        rospy.loginfo('Executing state DOCKING')
        time.sleep(2)
        if self.moveToEnd == False:
		msg6 = PoseStamped() 
		msg6.pose.position.x = self.posex 
		msg6.pose.position.y = self.posey
		msg6.pose.position.z = self.posez
		self.goal_pub.publish(msg6)
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
                                transitions={'StayInExploration':'EXPLORATION',
                                             'Move2Landing':'TRAJECTORY_FOLLOWING'})


		smach.StateMachine.add('TRAJECTORY_FOLLOWING', trajectory_following(),
                                transitions={'Move2Docking':'DOCKING',
                                             'StayInLanding':'TRAJECTORY_FOLLOWING'})

		smach.StateMachine.add('DOCKING', docking(),
                                transitions={'docking':'DOCKING',
                                             'docked':'Done'})


	sis = smach_ros.IntrospectionServer('server_name', sm, '/Challenge1_state')
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()
        rospy.spin()
        sis.stop()



if __name__ == '__main__':
	Challenge1()

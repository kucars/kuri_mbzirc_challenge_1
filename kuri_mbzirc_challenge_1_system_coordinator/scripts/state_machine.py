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
#from cftld_ros.msg import Track
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from smach_ros import ServiceState
import tf 

# define state : initialization
class initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['srvtype'])
    def execute(self, userdata):
        rospy.loginfo('Executing state INITIALIZATION')
        userdata.srvtype = 1 
        time.sleep(1.0)
        return 'succeeded'

# define state : checkMarkerDetection
class checkMarkerDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tracked','explore'],output_keys=['srvtype'])
        self.trackingLost        = False
        self.trackingLostCounter = 0
        self.listener            = tf.TransformListener()
      
    def landingZonePose(self, topic):
        if topic.pose.position.x != 0 :
              self.trackingLost  = False
              self.trackingLostCounter = 0
        else:
          self.trackingLostCounter = self.trackingLostCounter + 1
          if self.trackingLostCounter >= 5:
              self.trackingLost = True

    def execute(self, userdata):
        rospy.loginfo('Executing state MARKER_DETECTION')
        landingZonePoseSub = rospy.Subscriber('/visptracker_pose', PoseStamped, self.landingZonePose)
        time.sleep(1.0)
        if not self.trackingLost:
          userdata.srvtype = 2
          print "I will call for Landing"
          landingZonePoseSub.unregister()
          return 'tracked'
        else:
          userdata.srvtype = 1
          print "I will call for Exploration"
          landingZonePoseSub.unregister()
          return 'explore'
	      
# main
class Challenge1():
	rospy.init_node('MBZIRC_ch1_state_machine')
	sm = smach.StateMachine(outcomes=['Done'])
	with sm:	  
		def exploration_response_cb(userdata, response):
                        print "Exploration Response" , response.navResponse
			if response.navResponse == 'succeeded' :
			   return 'succeeded' 			  
			else :
			  return 'aborted'
			
		def landing_response_cb(userdata, response):
                        print 'Landing Response' , response.navResponse
			if response.navResponse == 'succeeded' :
			   return 'succeeded' 
			else :
			  return 'aborted'			
		      
		smach.StateMachine.add('INITIATING', initialization(),
                            transitions={'succeeded':'EXPLORATION'})
	 		
	 	############################################3
	 	# call service with request 1 and userdata #
	 	############################################3
    		smach.StateMachine.add('EXPLORATION',
					smach_ros.ServiceState(
					'controllerService', navigation,
                                        request_slots  = ['srvtype'],
					#request_cb  = exploration_request_cb,
					response_cb=exploration_response_cb,
                                        input_keys = ['srvtype']),
                                        transitions={'succeeded':'MARKER_DETECTION' , 'aborted':'MARKER_DETECTION','preempted':'MARKER_DETECTION'})
		############################################3
	 	# MArker Detection and tracking 	    # in this state the drone will not perform RTL becuase it is build it to hover 
	 	############################################3
                smach.StateMachine.add('MARKER_DETECTION', checkMarkerDetection(),
                                transitions={'tracked':'LANDING','explore':'EXPLORATION'})
	 	############################################3
	 	# call service with request 2 and userdata # 
	 	############################################3
		smach.StateMachine.add('LANDING',
					ServiceState('controllerService',
					navigation,
					#request_cb  = landing_request_cb,
                                        request_slots  = ['srvtype'],
					response_cb=landing_response_cb,
                                        input_keys = ['srvtype']),
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

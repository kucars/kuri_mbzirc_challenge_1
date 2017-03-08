#! /usr/bin/env python
#Copyright (c) 2017, Reem Ashour 
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#* Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#* Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#* Neither the name of kuri_mbzirc_challenge_1 nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib
import rospy
from kuri_msgs.msg import *
from kuri_mbzirc_challenge_1_msgs.msg import *

from kuri_mbzirc_challenge_1_msgs.srv import *
from kuri_mbzirc_challenge_1_msgs.srv import navigationResponse
from kuri_mbzirc_challenge_1_msgs.srv import navigation , navigationRequest , navigationResponse

from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import Odometry 
import math
import tf

class ControllerService:
	def __init__(self):
                print 'Starting Drone Control Service'
                self.controllerService 		= rospy.Service('controllerService', navigation , self.serviceCallback)
                self.localPosePub 		= rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		self.goalPosePub 		= rospy.Publisher('/visptracker_pose_tunnel', PoseStamped, queue_size=10)
                self.localPoseSub 		= rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.poseCallback)
                self.landingZonePose            = rospy.Subscriber('/visptracker_pose', PoseStamped, self.landingZonePose)
     		self.serviceType 	= 3
                self.explorationDone 	= False
		self.explorationMsg	= PoseStamped() 
		self.landingMsg  	= PoseStamped() 
		self.PoseXlocal 	= 0 
		self.PoseYlocal 	= 0 
		self.PoseZlocal 	= 0
                self.landingDone        = False
                self.listener           = tf.TransformListener()
                self.trackingLost       = False
                self.trackingLostCounter= 0
                self.explorationPoseX   = rospy.get_param("FixPoseX")
                self.explorationPoseY   = rospy.get_param("FixPoseY")
                self.explorationPoseZ   = rospy.get_param("FixPoseZ")

        def landingZonePose(self , poseMsg):
          if poseMsg.pose.position.x != 0 :
                self.trackingLost = False
                self.trackingLostCounter = 0
                self.listener.waitForTransform("downward_cam_optical_frame",'base_link',rospy.Time() , rospy.Duration(1.0))
                (staticDistance , Quat) = self.listener.lookupTransform('downward_cam_optical_frame','base_link',rospy.Time(0));
                eulerAngles = tf.transformations.euler_from_quaternion(Quat)
                yaw = eulerAngles[2]
                theta1 = 0 # this is the head of the marker
                #poseMsgX =  poseMsg.pose.position.x
                #poseMsgY = -1 * poseMsg.pose.position.y
                self.landingMsg.header.stamp = rospy.Time.now()
                self.landingMsg.pose.position.x = -1 * poseMsg.pose.position.y; #staticDistance[0] + (poseMsg.pose.position.x * math.cos( theta1 + yaw) )
                self.landingMsg.pose.position.y = -1 * poseMsg.pose.position.x; #staticDistance[1] + (poseMsg.pose.position.y * math.sin( theta1 + yaw) )  #poseMsg.pose.position.y
                self.landingMsg.pose.position.z = -1 * poseMsg.pose.position.z ; #staticDistance[2] + (poseMsg.pose.position.z * math.cos( 3.14159265359) ) #poseMsg.pose.position.z
                #print "Received a new tracked pose x:", self.landingMsg.pose.position.x, " y:",self.landingMsg.pose.position.y," z:",self.landingMsg.pose.position.z
          else:
              self.trackingLostCounter = self.trackingLostCounter + 1
              if self.trackingLostCounter >= 5:
                  self.trackingLost = True

        def serviceCallback(self , req):
   		if req.srvtype == 1 :
                        self.explorationDone = False
			self.serviceType = 1
	                self.explorationMsg.header.stamp = rospy.Time.now()
                        self.explorationMsg.pose.position.x = self.explorationPoseX
                        self.explorationMsg.pose.position.y = self.explorationPoseY
                        self.explorationMsg.pose.position.z = self.explorationPoseZ
                        print "Received Exploration Request", self.explorationMsg.pose.position.z
                        while not self.explorationDone:
                          rospy.sleep(0.1)
                          #print "Going up";
                        if self.explorationDone:
				n = kuri_mbzirc_challenge_1_msgs.srv.navigationResponse()
				n.navResponse = "succeeded"
				return n 
			else: 
				n = kuri_mbzirc_challenge_1_msgs.srv.navigationResponse() 
				n.navResponse = "aborted"
				return n 
		elif req.srvtype == 2:
                        self.serviceType  = 2
                        self.landingDone  = False
                        self.trackingLost = False
                        print 'Service call: Received Landing Request'
                        while not self.landingDone and not self.trackingLost:
                          rospy.sleep(0.1)
                          #print "Landing";
                        if self.landingDone and not self.trackingLost:
			      print "Landed DONE " 
                              n = kuri_mbzirc_challenge_1_msgs.srv.navigationResponse()
                              n.navResponse = "succeeded"
			      return n 
                        else:
			      n = kuri_mbzirc_challenge_1_msgs.srv.navigationResponse()
			      n.navResponse = "aborted"
			      return n 				  
				
	def poseCallback(self , msgPose):
                self.PoseXlocal = msgPose.pose.position.x
                self.PoseYlocal = msgPose.pose.position.y
                self.PoseZlocal = msgPose.pose.position.z
                if self.serviceType == 1:
                        self.localPosePub.publish(self.explorationMsg)
                        distance2Pose = math.sqrt((self.PoseXlocal - self.explorationMsg.pose.position.x)*(self.PoseXlocal - self.explorationMsg.pose.position.x) +
                                                  (self.PoseYlocal - self.explorationMsg.pose.position.y)*(self.PoseYlocal - self.explorationMsg.pose.position.y) +
                                                  (self.PoseZlocal - self.explorationMsg.pose.position.z)*(self.PoseZlocal- self.explorationMsg.pose.position.z))
                        #print "Requested Altitude", self.explorationMsg.pose.position.z," distance:", distance2Pose
                        if  distance2Pose < 0.2 :
                                self.explorationDone = True
				self.serviceType = 0			
		elif self.serviceType == 2:
			self.goalPosePub.publish(self.landingMsg)
                        distance2Pose = math.sqrt((self.PoseXlocal - self.landingMsg.pose.position.x)*(self.PoseXlocal - self.landingMsg.pose.position.x) +
                                                  (self.PoseYlocal - self.landingMsg.pose.position.y)*(self.PoseYlocal - self.landingMsg.pose.position.y) +
                                                  (self.PoseZlocal - self.landingMsg.pose.position.z)*(self.PoseZlocal - self.landingMsg.pose.position.z))
                        if distance2Pose<=0.2:
                            self.landingDone = True;
                            self.serviceType = 0
                        else:
                            self.landingDone = False
		elif self.serviceType == 3:
			# this is the start point as if we performing the take off 
	                self.explorationMsg.header.stamp = rospy.Time.now()
		  	self.explorationMsg.pose.position.x = self.PoseXlocal
			self.explorationMsg.pose.position.y = self.PoseYlocal
			self.explorationMsg.pose.position.z = 2
			self.localPosePub.publish(self.explorationMsg) 

		else: 
			# this is for hovering when we are performing other states 
	                self.landingMsg.header.stamp = rospy.Time.now()
			self.landingMsg.pose.position.x = 0
			self.landingMsg.pose.position.y = 0#msgPose.pose.pose.position.y
			self.landingMsg.pose.position.z = 0#msgPose.pose.pose.position.z
			self.goalPosePub.publish(self.landingMsg) 						   				
					
def main(args):
  rospy.init_node('Controller_Service')
  Controller = ControllerService()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
 

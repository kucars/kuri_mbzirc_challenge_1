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

class ControllerService:
	def __init__(self):
     		print 'Starting Service 1'
     		self.s = rospy.Service('controller_service', navigation , self.pos_vel_controller)
		self.pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
		self.pub2 = rospy.Publisher('/kuri_offboard_attitude_control_test/goal', PoseStamped, queue_size=10)
		self.sub =  rospy.Subscriber("mavros/global_position/local", Odometry, self.poseCallback)
		self.sub2 =  rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.poseCallback2)
     		self.serviceType = 3
		self.exploreFlag = False
		self.landFlag = False 
		self.landFlagDone = 0 
		self.msg  = PoseStamped() 
		self.msg2 = PoseStamped() 
		self.PoseXlocal = 0 
		self.PoseYlocal = 0 
		self.PoseZlocal = 0
		self.strt = "Nothing"# navResponse

	

   	def pos_vel_controller(self , req):
		self.landFlagDone = req.LDFlag 
   		if req.srvtype == 1 :
			self.serviceType = 1
			self.msg.pose.position.x = req.wayPointX
			self.msg.pose.position.y = req.wayPointY
			self.msg.pose.position.z = req.wayPointZ
			if self.exploreFlag:	
				navigationResponse = "succeeded"
				n = kuri_mbzirc_challenge_1_msgs.srv.navigationResponse()
				n.navResponse = "succeeded"
				print "n" , n 
				return n 
			else: 
				n = kuri_mbzirc_challenge_1_msgs.srv.navigationResponse() 
				n.navResponse = "aborted"
				return n 
		else:
			if req.srvtype == 2:
				self.serviceType = 2 	
				print 'landFlag' , self.landFlagDone , '   ', self.landFlag
				self.msg2.pose.position.x = req.wayPointX
				self.msg2.pose.position.y = req.wayPointY
				self.msg2.pose.position.z = req.wayPointZ
				if self.landFlagDone :#or self.landFlag: # this comes form the state machine 
				  print "Landed DONE " 
				  n = kuri_mbzirc_challenge_1_msgs.srv.navigationResponse()
				  n.navResponse = "succeeded2"
				  return n 
				else: 
				  print "Keep landing" 
				  n = kuri_mbzirc_challenge_1_msgs.srv.navigationResponse()
				  n.navResponse = "aborted"
				  return n 
				
				
	def poseCallback(self , msgPose):
		if self.serviceType == 1: 
			#print "FromCallback 1" 
			self.pub.publish(self.msg) 
			###################################################################################3
			# This step uses the local data instead of the global data until we find a solution 
			###################################################################################3
			#if (abs(msgPose.pose.position.x - self.msg.pose.position.x) < 0.2 and
			#    abs(msgPose.pose.position.y - self.msg.pose.position.y) < 0.2 and 
			#    abs(msgPose.pose.position.z - self.msg.pose.position.z) < 0.2 )
			if abs(self.PoseXlocal - self.msg.pose.position.x) < 0.2 and abs(self.PoseYlocal- self.msg.pose.position.y) < 0.2 and abs(self.PoseZlocal- self.msg.pose.position.z) < 0.2 :
				self.exploreFlag = True 
				self.serviceType = 0
			
		else: 
			if self.serviceType == 2:
			      self.pub2.publish(self.msg2)
			      #print 'publishing msg2' 
			      #if (abs(msgPose.pose.pose.position.x - self.msg2.pose.position.x) < 0.2 and abs(msgPose.pose.pose.position.y - self.msg2.pose.position.y) < 0.2 and abs(msgPose.pose.pose.position.z - self.msg2.pose.position.z) < 0.2 ) :
				#self.landFlag = True
				#self.serviceType = 0
			      if (self.landFlagDone): 
				  self.landFlag = True 
				  self.serviceType = 0
			else:
			      if self.serviceType == 3: 
				#if this is being run before the state machine 
				#print "Publish this msg "
				self.msg.pose.position.x = self.PoseXlocal#msgPose.pose.pose.position.x
				print self.msg.pose.position.x
				self.msg.pose.position.y = self.PoseYlocal#msgPose.pose.pose.position.y
				print self.msg.pose.position.y
				self.msg.pose.position.z = 1 #msgPose.pose.pose.position.z + 0.2 
				print self.msg.pose.position.z
				self.pub.publish(self.msg) 
			      else:  
				#if this Hovering when other states are being run  
				#print "Hover" 
				self.msg.pose.position.x = self.PoseXlocal#msgPose.pose.pose.position.x
				self.msg.pose.position.y = self.PoseYlocal#msgPose.pose.pose.position.y
				self.msg.pose.position.z = self.PoseZlocal #msgPose.pose.pose.position.z + 0.2 
				self.msg2.pose.position.x = msgPose.pose.pose.position.x
				self.msg2.pose.position.y = msgPose.pose.pose.position.y
				self.msg2.pose.position.z = msgPose.pose.pose.position.z
				self.pub.publish(self.msg) 
				self.pub2.publish(self.msg2) 

				
	    
	def poseCallback2(self , msgPoseLocal):
		#print "Reading local position data" 
		################################################################################3
		# Only used becasue the setpoint_position only accept local data local reference 
		################################################################################3
		self.PoseXlocal = msgPoseLocal.pose.position.x
		self.PoseYlocal = msgPoseLocal.pose.position.y
		self.PoseZlocal = msgPoseLocal.pose.position.z
		   	
			
					
def main(args):
  rospy.init_node('Controller_Service')
  Controller = ControllerService()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
 

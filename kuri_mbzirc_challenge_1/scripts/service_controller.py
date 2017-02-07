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
#* Neither the name of kuri_mbzirc_challenge_3 nor the names of its
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
from kuri_mbzirc_challenge_1_msgs.msg import navigation
from geometry_msgs.msg import PoseStamed 


msg  = PoseStamped() 
msg2 = poseStamped() 

class Controller:
	def __init__(self):
     		print 'Starting Service 1'
     		self.s = rospy.Service('controller_service', navigation , self.pos_vel_controller)
		self.pub = rospy.Publisher('mavros/setpoint_position/pose', PoseStamped, queue_size=10)
		self.pub2 = rospy.Publisher('kuri_goal', PoseStamped, queue_size=10)
		self.sub =  rospy.Subscriber("mavros/global_position/local", PoseStamed, poseCallback)
     		self.serviceType = 3
     		self.exploreResult = "" 
     		self.landResult = "" 
		self.exploreFlag = False
		self.landFlagDone = 0 
		self.msg  = PoseStamped() 
		self.msg2 = poseStamped() 
   
   	def pos_vel_controller(self , req):
		self.landFlagDone = req.LDFlag 
   		if req.type == "explore" :
			self.serviceType = 1
			self.msg.pose.position.x = req.xg
			self.msg.pose.position.y = req.yg
			self.msg.pose.position.z = req.zg
			if self.exploreFlag:
				return navigationResponse("Explored")
		else if req.type == "land":
			self.serviceType = 2 	
			self.msg2.pose.position.x = req.xg
			self.msg2.pose.position.y = req.yg
			self.msg2.pose.position.z = req.zg
			if self.landFlag:
				return navigationResponse("Landed")
		else :
			self.serviceType = 3
			if self.landFlag:
				return navigationResponse("Nothing")
			print "self.serviceType", self.serviceType
		print "self.serviceType 2" , self.serviceType

   	def poseCallback(self , msgPose):
		if self.serviceType == 1: 
			self.pub.publisher(self.msg) 
			if (abs(msgPose.pose.position.x - self.msg.pose.position.x) < 0.2 and
			    abs(msgPose.pose.position.y - self.msg.pose.position.y) < 0.2 and 
			    abs(msgPose.pose.position.z - self.msg.pose.position.z) < 0.2 )
				self.exploreFlag = True 
				self.serviceType = 0
			
		if self.serviceType == 2:
			self.pub2.publich(self.msg2) 
			if (self.landFlagDone): 
				self.landFlag = 1 
				self.serviceType = 0
		else:
			self.msg.pose.position.x = msgPose.pose.position.x
			self.msg.pose.position.y = msgPose.pose.position.y
			self.msg.pose.position.z = msgPose.pose.position.z
			self.pub.publich(self.msg) 
			
					
def main(args):
  rospy.init_node('Controller_Service')
  Controller = ServiceOne()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
 

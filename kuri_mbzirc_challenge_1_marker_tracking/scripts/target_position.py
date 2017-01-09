#!/usr/bin/env python
#  Authors: Husameldin Mukhtar
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
# Updated: Husameldin Mukhtar : husameldin.mukhtar@kustar.ac.ae

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from geometry_msgs.msg import PoseStamped
from cftld_ros.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo
from scipy.spatial import distance
from collections import OrderedDict
import numpy as np
from numpy.linalg import inv
from matplotlib import pyplot as plt
from pylab import *
from datetime import datetime
import time
import os
import os
import glob
from os.path import expanduser
home = expanduser("~")

#from __future__ import print_function

#Target size (update this for the actual target size later
trgt_W=1.5
trgt_circle_r=1/float(2)  


class target_position:

	def __init__(self):
		rospy.init_node('target_position', anonymous=True)

		# Initialize the Region of Interest and its publisher
		self.target_pos = PoseStamped()
		self.target_pos_pub = rospy.Publisher("/target_position/pose", PoseStamped, queue_size = 1)

		self.image_width = 0
		self.image_height = 0
		self.cameraInfo_sub = rospy.Subscriber("/downward_cam/camera/camera_info",CameraInfo,self.get_camera_info,  queue_size = 1)		

		self.local_pose_x = 0.0
		self.local_pose_y = 0.0
		self.local_pose_z = 0.0
		self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_local_pose,  queue_size = 1)
		
		self.track_bb_x = 0.0
		self.track_bb_y = 0.0
		self.track_bb_h = 0.0
		self.track_bb_w = 0.0
		self.track_status = 0.0
		self.track_conf = 0.0
		self.track_sub = rospy.Subscriber("/cftld/track",Track,self.callback,  queue_size = 1)

		rate = rospy.Rate(10) 
		while not rospy.is_shutdown():
			rate.sleep()

	def get_camera_info(self, msg):
		self.image_width = msg.width
		self.image_height = msg.height
		self.camera_K = msg.K

	def get_local_pose(self, msg):
		self.local_pose_x = msg.pose.position.x
		self.local_pose_y = msg.pose.position.y
		self.local_pose_z = msg.pose.position.z

	def callback(self,msg):
		self.track_bb_x = msg.roi.x_offset
		self.track_bb_y = msg.roi.y_offset
		self.track_bb_h = msg.roi.height
		self.track_bb_w = msg.roi.width
		self.track_status = msg.status
		self.track_conf = msg.confidence
		
		fx=self.camera_K[0]
		fy=self.camera_K[4]
		u0=self.camera_K[5]
		v0=self.camera_K[2]

		K=np.matrix([[fx, 0, u0, 0], [0, fy, v0, 0], [0, 0, 1, 0]])
		K_INV=pinv(K)

		cX = self.track_bb_x + self.track_bb_w/2
		cY = self.track_bb_y + self.track_bb_h/2

		#print(cX,cY,self.local_pose_z)

		p_pxl_hom=np.matrix([[cY],[cX],[1]])
		P_mtr_hom=np.dot(K_INV,p_pxl_hom)
		P_mtr=P_mtr_hom*(self.local_pose_z/P_mtr_hom[2][0])

		#print(K_INV,p_pxl_hom,P_mtr_hom,P_mtr)

		self.target_pos.pose.position.x = -P_mtr[0][0]
		self.target_pos.pose.position.y = -P_mtr[1][0]
		self.target_pos.pose.position.z = P_mtr[2][0]
		
		self.target_pos_pub.publish(self.target_pos)


def main(args):
	tp = target_position()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)




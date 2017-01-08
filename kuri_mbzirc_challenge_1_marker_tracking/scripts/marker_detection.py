#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from imutils import perspective
from imutils import contours
import imutils
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

"""
# iDS Camera
cam_W=2456
cam_H=2054
cam_f=25e-3
pxl_size=3.45e-6
"""

cam_W=640
cam_H=480
cam_f=25e-3
pxl_size=3.45e-6

#Target size (update this for the actual target size later)
#trgt_W=0.15
#trgt_circle_r=0.1/2

trgt_W=1.5
trgt_circle_r=1/2  

terminate = False 

class marker_detection:

	def __init__(self):
		rospy.init_node('marker_detection', anonymous=True)

		self.image_pub = rospy.Publisher("/detection/image_raw",Image, queue_size = 1)

		# Initialize the Region of Interest and its publisher
		self.ROI = RegionOfInterest()
		self.ROI_pub = rospy.Publisher("/ch1/marker_bb", RegionOfInterest, queue_size = 1)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/downward_cam/camera/image",Image,self.callback, queue_size = 1, buff_size=2**24, tcp_nodelay=True)

		self.image_width = 1
		self.image_height = 1
		self.cameraInfo_sub = rospy.Subscriber("/downward_cam/camera/camera_info",CameraInfo,self.get_camera_info,  queue_size = 1)

	def get_camera_info(self, msg):
		self.image_width = msg.width
		self.image_height = msg.height

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		img_org=cv_image.copy()

		WW=self.image_width
		HH=self.image_height

		WW=self.image_width
		HH=self.image_height

		ratio_act=cam_W/WW

		W_proc = WW
		ratio = WW/float(W_proc)
		H_proc = int(math.floor(HH/ratio))

		px=pxl_size*ratio_act
		f=cam_f

		fx=f/float(px);
		fy=fx
		u0=math.floor(HH/(ratio*2))
		v0=math.floor(WW/(ratio*2))

		K=np.matrix([[fx, 0, u0, 0], [0, fy, v0, 0], [0, 0, 1, 0]])
		K_INV=pinv(K)

		#img = imutils.resize(img_org, width=W_proc)
		img = img_org.copy()
		out = img.copy()

		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(gray,(3,3),0)

		thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,3,2)

		edge = cv2.Canny(blur, 50, 150)

		cnts = cv2.findContours(thresh.copy(), cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]

		detect = False
		if cnts is not None:
			for c in cnts:
				
				if detect:
					break

				# approximate the contour
				peri = cv2.arcLength(c, True)
				approx = cv2.approxPolyDP(c, 0.01 * peri, True)
				
				# ensure that the approximated contour is "roughly" rectangular
				if len(approx) >= 4 and len(approx) <= 6:
					# compute the bounding box of the approximated contour and
					# use the bounding box to compute the aspect ratio
					(x, y, w, h) = cv2.boundingRect(approx)
					aspectRatio = w / float(h)
		 
					# compute the solidity of the original contour
					area = cv2.contourArea(c)
					hullArea = cv2.contourArea(cv2.convexHull(c))
					solidity = area / float(hullArea)
		 
					# compute whether or not the width and height, solidity, and
					# aspect ratio of the contour falls within appropriate bounds
					keepDims = w > 25/ratio and h > 25/ratio
					keepSolidity = solidity > 0.9
					keepAspectRatio = aspectRatio >= 0.6 and aspectRatio <= 1.4
					
					oneCircle = False
					if keepAspectRatio:
						subimg=img[y:y+w,x:x+h]
						subthresh=thresh[y:y+w,x:x+h]				
						circles = cv2.HoughCircles(subthresh, cv2.cv.CV_HOUGH_GRADIENT,2,100,param1=100,param2=27,minRadius=10,maxRadius=WW)
											
						if circles is not None:		 			
							oneCircle = len(circles) == 1
										
					# ensure that the contour passes all our tests
					if keepDims and keepSolidity and keepAspectRatio and oneCircle:
						# draw an outline around the target and update the status text
						
						cv2.drawContours(out, [approx], -1, (0, 0, 255), 2)
						status = "Target(s) Acquired"
						detect = True
						terminate = True
						trgt_Z = cam_f*trgt_W/(float(w)*pxl_size*ratio_act)
											
						circle = np.round(circles[0, :]).astype("int")
						circle_x = circle[0][0]
						circle_y = circle[0][1]
						circle_r = circle[0][2]
						trgt_Z2 = cam_f*trgt_circle_r/(float(circle_r)*pxl_size*ratio_act)

						M = cv2.moments(c)
						cX = int((M["m10"] / M["m00"]))
						cY = int((M["m01"] / M["m00"]))

						p_pxl_hom=np.matrix([[cY],[cX],[1]])
						P_mtr_hom=np.dot(K_INV,p_pxl_hom)
						P_mtr=P_mtr_hom*(trgt_Z2/P_mtr_hom[2][0])
						
						self.ROI = RegionOfInterest()
						self.ROI.x_offset = x
						self.ROI.y_offset = y
						self.ROI.width = w
						self.ROI.height = h

						self.ROI_pub.publish(self.ROI)

						cv2.circle(out, (cX, cY), 3, (0, 0, 255), -1)

						cv2.putText(out, "Marker Detected", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
						cv2.putText(out, "X={}".format(P_mtr[1][0]), (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
						cv2.putText(out, "Y={}".format(-P_mtr[0][0]), (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
						cv2.putText(out, "Z={}".format(P_mtr[2][0]), (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

						cv2.line(out, (W_proc/2,0), (W_proc/2,H_proc), (0, 0, 255), 1) 
						cv2.line(out, (0,H_proc/2), (W_proc,H_proc/2), (0, 0, 255), 1) 
						

		#cv2.imshow("out", out)
		#cv2.waitKey(3)

		#if terminate:
			#sys.exit('Node is stopped once marker is detected!')

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))
		except CvBridgeError as e:
			print(e)

def main(args):
	ic = marker_detection()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)




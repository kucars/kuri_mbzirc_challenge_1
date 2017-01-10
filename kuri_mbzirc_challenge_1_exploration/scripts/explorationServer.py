#! /usr/bin/env python

import roslib

import rospy
import actionlib
import thread
import threading
import time
import mavros
from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from kuri_mbzirc_challenge_1_msgs.msg import explorationAction
import kuri_mbzirc_challenge_1_msgs.msg 
class ExplorationAction:
  _result = kuri_mbzirc_challenge_1_msgs.msg.explorationResult() 
	
  def __init__(self):
    self.server = actionlib.SimpleActionServer('exploration',kuri_mbzirc_challenge_1_msgs.msg.explorationAction, self.execute, False)   
    self.server.start()

    self.x = 0.0
    self.y = 24
    self.z = 0.0
    self.currentPoseX = 0
    self.currentPoseY = 24
    self.currentPoseZ = 0
    self.done = False
    self.startY = 24
    self.startZ = 0



  def execute(self, goal):
      rospy.loginfo('goal recieved')
      rospy.loginfo(goal.startMission_str)

      waypoints= [[0, 24 , 5.0 ,0.0],[0, 20 , 5.0 ,0.0],[0, 15 , 10  ,0.0],[0, 10 , 15  ,0.0],[0, 5  , 15  ,0.0],[0, 0  , 20  ,0.0]]

      def set(x, y, z, delay=0, wait=True):

            rospy.loginfo('set gaol function')
            self.x = x
            self.y = y
            self.z = z

      def poseCallback(topic):
            currentPose = [topic.pose.position.x,topic.pose.position.y,topic.pose.position.z]
            self.currentPoseX = topic.pose.position.x
            self.currentPoseY = topic.pose.position.y
            self.currentPoseZ = topic.pose.position.z

            if abs(topic.pose.position.x  - self.x) < 0.3  and abs(topic.pose.position.y - self.y)  < 0.3 and abs(topic.pose.position.z - self.z) < 0.3:
                self.done = True
                rospy.loginfo('DONE &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&')


      #waypoint_pub = SP.get_pub_position_local(queue_size=10)
      waypoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
      pose_sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped, poseCallback)

      for i in range(len(waypoints)):
          rospy.loginfo('for loop **************************************************')
          rospy.loginfo('i')
          rospy.loginfo(i)
          self.done = False
          self.x = waypoints[i][0]
          self.y = waypoints[i][1]
          self.z = waypoints[i][2]
          rospy.loginfo(self.x )
          rospy.loginfo(self.y )
          rospy.loginfo(self.z )

          if (self.currentPoseX <=0.1 and self.currentPoseX >=-0.1 and self.currentPoseY >=-0.1 and self.currentPoseY <=0.1):
                    rospy.loginfo('reached center')
		    self.result.reachedCenter = True
                    self.server.set_succeeded(self.result)
          else:

                   rospy.loginfo('navigation')
                   rate = rospy.Rate(10)   # 10hz
                   rate.sleep()
		   msg = SP.PoseStamped(
		       header=SP.Header(
		           frame_id="base_footprint",  # no matter, plugin don't use TF
		           stamp=rospy.Time.now()),    # stamp should update
		   )

                   msg.pose.position.x = self.x
                   msg.pose.position.y = self.y
                   msg.pose.position.z = self.z

                   # For demo purposes we will lock yaw/heading to north.
                   yaw_degrees = 0  # North
                   yaw = radians(yaw_degrees)
                   quaternion = quaternion_from_euler(0, 0, yaw)
                   msg.pose.orientation = SP.Quaternion(*quaternion)
                   while not self.done and not rospy.is_shutdown():
                              waypoint_pub.publish(msg)
                              rospy.loginfo('publishing')
                              rate.sleep()





def setpoint_demo():
    rospy.init_node('exploration_server')
    mavros.set_namespace('/mavros')    
    rate = rospy.Rate(10)
    setpointServer = ExplorationAction()
    time.sleep(1)
    rospy.spin()
    print setpointServer.x, setpointServer.y
    rospy.loginfo("Bye!")



if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass

#! /usr/bin/env python
import roslib
import rospy
import actionlib
from kuri_mbzirc_challenge_1_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('Client')
    print "Start"
    client = actionlib.SimpleActionClient('poseEstimationServer', poseEstimationAction)
    client.wait_for_server()
    print "Waiting for server"
    goal = poseEstimationGoal()
    g = PoseEsti()
    g.x = 100
    g.y = 100
    goal.GI = g
    client.send_goal(goal)
    print "Waiting for result"
    client.wait_for_result() 
    #client.wait_for_result(rospy.Duration.from_sec(5.0)) 
print "Action Server Finished, with result:",client.get_result()

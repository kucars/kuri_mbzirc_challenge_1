#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/RegionOfInterest.h>
#include "kuri_mbzirc_challenge_1_msgs/PES.h"
#include "kuri_mbzirc_challenge_1_msgs/pidData.h"
#include "kuri_mbzirc_challenge_1_trajectory_following/landingcontroller.h"
#include <std_msgs/Float64.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

TruckFollower::TruckFollower(const ros::NodeHandle &_nh, const ros::NodeHandle &_nhPrivate):
  nh(_nh),
  nhPrivate(_nhPrivate)
{
  velPub           = nh.advertise <geometry_msgs ::TwistStamped >("/mavros/setpoint_velocity/cmd_vel", 1);
  pidPub           = nh.advertise <kuri_mbzirc_challenge_1_msgs::pidData >("/pidData", 1);
  goalSub          = nh.subscribe("/visptracker_pose_tunnel", 1000, &TruckFollower::goalCallback,this);
  globalPoseSub  = nh.subscribe("/mavros/global_position/global", 1000, &TruckFollower::globalPoseCallback,this);
  compassSub      = nh.subscribe ("/mavros/global_position/compass_hdg", 1, &TruckFollower::headingCallback,this);

  nh.param("kp", kp, 0.05);
  nh.param("ki", ki, 0.0);
  nh.param("kd", kd, 0.05);
  nh.param("tolerance_2_goal", tolerance_2_goal, 0.2);

  goalPose.pose.position.x = 0;
  goalPose.pose.position.y = 0;
  goalPose.pose.position.z = 0;
  stopTwist.twist.linear.x  = 0;
  stopTwist.twist.linear.y  = 0;
  stopTwist.twist.linear.z  = 0;
  stopTwist.twist.angular.z = 0;

  ros::Rate loopRate(10);
  while (ros::ok())
  {
    // Failsafe: if we don't get tracking info for more than 500ms, then stop in place
    if(ros::Time::now() - goalLastReceived > ros::Duration(0.5))
    {
      stopTwist.header.stamp = ros::Time::now();
      //velPub.publish(stopTwist);
    }
    else
    {
      velPub.publish(twist);
    }
    pidPub.publish(pidMsg);
    ros:: spinOnce ();
    loopRate.sleep();
  }
}

void TruckFollower::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
  currentState = *msg;
}

void TruckFollower::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goalLastReceived = ros::Time::now();
  ROS_INFO("TEST"); // this should be done before sending it to this component
  goalPose = *msg;

}

void TruckFollower::headingCallback(const std_msgs::Float64::ConstPtr& msg)
{
  yaw = msg->data * 3.14159265359 / 180.0 ;
}

void TruckFollower::globalPoseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  real.x= 0 ; //msg ->latitude;
  real.y= 0 ; //msg ->longitude;
  real.z= 0 ; //msg ->altitude;
  w = yaw ;

  if (firstDataFlag == false )
  {
    errorX =  goalPose.pose.position.x - real.x;
    errorY =  goalPose.pose.position.y - real.y;
    errorZ =  goalPose.pose.position.z - real.z ;
    errorW =  w;
    prevErrorX = errorX;
    prevErrorY = errorY;
    prevErrorZ = errorZ;
    prevErrorW = errorW;
    firstDataFlag = true;
  }
  else
  {

    errorX =  goalPose.pose.position.x - real.x;
    errorY =  goalPose.pose.position.y - real.y;
    errorZ =  goalPose.pose.position.z - real.z ;
    errorW =  w;
    pX = kp * errorX;
    pY = kp * errorY;
    pZ = kp * errorZ;
    pW = kp * errorW;

    iX += ki * errorX;
    iY += ki * errorY;
    iZ += ki * errorZ;
    iW += ki * errorW;

    dX = kd * (errorX - prevErrorX);
    dY = kd * (errorY - prevErrorY);
    dZ = kd * (errorZ - prevErrorZ);
    dW = kd * (errorW - prevErrorW);

    prevErrorX = errorX;
    prevErrorY = errorY;
    prevErrorZ = errorZ;
    prevErrorW = errorW;

    // PID conroller
    aX = pX     + iX + dX  ;
    aY = pY     + iY + dY  ;
    aZ = pZ      +iZ + dZ  ;
    aW = 10 * pW +iW + dW  ;

    // filling velocity commands
    twist.twist.linear.x = aX;
    twist.twist.linear.y = aY;
    twist.twist.linear.z = aZ;
    twist.twist.angular.z = aW;
    twist.header.stamp = ros::Time::now();

    ROS_INFO("Error X: %0.2f \n", errorX);
    ROS_INFO("Error Y: %0.2f \n", errorY);
    ROS_INFO("Error Z: %0.2f \n", errorZ);
    ROS_INFO("derivative X: %0.2f \n", dX);
    ROS_INFO("derivative Y: %0.2f \n", dY);
    ROS_INFO("derivative Z: %0.2f \n", dZ);
    ROS_INFO("derivative W: %0.2f \n", dZ);
    ROS_INFO("W: %0.2f \n", w);
    ROS_INFO("Action X: %0.2f \n", aX);
    ROS_INFO("Action Y: %0.2f \n", aY);
    ROS_INFO("Action Z: %0.2f \n", aZ);
    ROS_INFO("Action W: %0.2f \n", aW);

    // publishing this data to be recorded in a bag file
    pidMsg.dronePoseX = real.x;                       pidMsg.dronePoseY = real.y;                     pidMsg.dronePoseZ = real.z;
    pidMsg.goalPoseX  = goalPose.pose.position.x ;    pidMsg.goalPoseY  = goalPose.pose.position.y;   pidMsg.goalPoseZ  = goalPose.pose.position.z;
    pidMsg.positionErrorX = errorX;                   pidMsg.positionErrorY = errorY;                 pidMsg.positionErrorZ = errorZ;     pidMsg.positionErrorW = errorW;
    pidMsg.PX = pX;                                   pidMsg.PY = pY;                                 pidMsg.PZ = pZ;                     pidMsg.PW = pW;
    pidMsg.IX = iX;                                   pidMsg.IY = iY;                                 pidMsg.IZ = iZ;                     pidMsg.IW = iW;
    pidMsg.DX = dX;                                   pidMsg.DY = dY;                                 pidMsg.DZ = dZ;                     pidMsg.DW = dW;
    pidMsg.PIDX =aX;                                  pidMsg.PIDY = aY;                               pidMsg.PIDZ= aZ;                    pidMsg.PIDW= aW;
    pidMsg.header.stamp = ros::Time::now();
    pidMsg.header.seq = counter++;

    if ((fabs(errorX) < tolerance_2_goal) && (fabs(errorY) < tolerance_2_goal) && (fabs(errorZ) < tolerance_2_goal))
    {
      twist.twist.linear.x = 0;
      twist.twist.linear.y = 0;
      twist.twist.linear.z = 0;
      twist.twist.angular.z = 0;
    }
  }
}

int main(int argc , char **argv)
{
  ros::init(argc , argv , "offboard_attitude_control");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate( "~" );
  TruckFollower truckFollower(nh,nhPrivate);
}

/***************************************************************************
 *   Copyright (C) 2016 - 2017 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#ifndef LANDING_CONTROLLER_H
#define LANDING_CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/RegionOfInterest.h>
#include "kuri_mbzirc_challenge_1_msgs/PES.h"
#include "kuri_mbzirc_challenge_1_msgs/pidData.h"
#include <std_msgs/Float64.h>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
class TruckFollower
{
public:
  TruckFollower(const ros::NodeHandle &_nh, const ros::NodeHandle &_nhPrivate);
  ~TruckFollower(){}
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void headingCallback(const std_msgs::Float64::ConstPtr& msg);
 // void globalPoseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

private:
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate;
  ros::Time goalLastReceived;
  ros::Publisher velPub;
  ros::Publisher pidPub;
  ros::Subscriber goalSub;
  //ros::Subscriber globalPoseSub;
  ros::Subscriber compassSub;
  ros::ServiceClient armingClient;
  geometry_msgs ::Point real;
  geometry_msgs ::TwistStamped twist;
  geometry_msgs ::TwistStamped stopTwist;
  geometry_msgs::PoseStamped goalPose;
  kuri_mbzirc_challenge_1_msgs::pidData pidMsg;
  mavros_msgs::State currentState;
  mavros_msgs::CommandBool armCommand;

  bool firstDataFlag;
  double roll, pitch, yaw;
  int counter;
  float w;

  double tolerance_2_goal;
  double kp;
  double ki;
  double kd;
  double kpx;
  double kix;
  double kdx;
  double kpy;
  double kiy;
  double kdy;
  double kpz;
  double kiz;
  double kdz;

  float errorX;
  float errorY;
  float errorZ;
  float errorW;
  float prevErrorX;
  float prevErrorY;
  float prevErrorZ;
  float prevErrorW;
  float rise;
  float nonstop;
  float pX;
  float pY;
  float pZ;
  float pW;
  float iX;
  float iY;
  float iZ;
  float iW;
  float dX;
  float dY;
  float dZ;
  float dW;
  float aX;
  float aY;
  float aZ;
  float aW;
  bool mustExit;
  int waypointNum ;
};

#endif

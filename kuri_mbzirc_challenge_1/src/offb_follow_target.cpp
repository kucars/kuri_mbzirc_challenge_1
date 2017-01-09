/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Husameldin Mukhtar  <husameldin.mukhtar@kustar.ac.ae>              *
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

/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <stdio.h>
#include <cftld_ros/Track.h>

mavros_msgs::State currentState;
void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
    currentState = *msg;
}

geometry_msgs::PoseStamped target_current_pos;
geometry_msgs::PoseStamped target_pos;
geometry_msgs::PoseStamped uav_current_pos;
geometry_msgs::PoseStamped uav_pos;
geometry_msgs::PoseStamped set_pos;
cftld_ros::Track track_current_data;
cftld_ros::Track track_data;

void get_target_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    target_current_pos = *msg;
    target_pos.pose.position.x = target_current_pos.pose.position.x;
    target_pos.pose.position.y = target_current_pos.pose.position.y;
    //target_pos.pose.position.z = target_current_pos.pose.position.z;
}

void get_uav_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav_current_pos = *msg;
    uav_pos.pose.position.x = uav_current_pos.pose.position.x;
    uav_pos.pose.position.y = uav_current_pos.pose.position.y;
    //uav_pos.pose.position.z = uav_current_pos.pose.position.z;
}

void get_track(const cftld_ros::Track::ConstPtr& msg){
    track_current_data = *msg;
    track_data.status = track_current_data.status;
    track_data.confidence = track_current_data.confidence;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, mavrosStateCallback);
    //ros::Subscriber pose_sub = nh.subscribe<visualization_msgs::Marker>("visualization_marker", 10, pose_cb);
    ros::Subscriber target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/target_position/pose", 10, get_target_pos);
		ros::Subscriber uav_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, get_uav_pos);
		ros::Subscriber track_sub = nh.subscribe<cftld_ros::Track>("/cftld/track", 10, get_track);
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && currentState.connected){
        ros::spinOnce();
	// ros::spin();
        rate.sleep();
    }

    //printf ("%f \n",current_pose.pose.position.z);

    set_pos.pose.position.x = 0;
    set_pos.pose.position.y = 0;
    set_pos.pose.position.z = 10;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(set_pos);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time arming_time = ros::Time::now();
    //std::cout<<"I am hhere\n";
    while(ros::ok()){
        if( currentState.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !currentState.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
										arming_time = ros::Time::now();
                }
                last_request = ros::Time::now();
            }
        }

        if ( ros::Time::now() - arming_time > ros::Duration(10.0)){

            if ( track_data.confidence > 0.5){
                set_pos.pose.position.x = target_pos.pose.position.x + uav_pos.pose.position.x ;
                set_pos.pose.position.y = target_pos.pose.position.y + uav_pos.pose.position.y ;
                //set_pos.pose.position.z = target_pos.pose.position.z + uav_pos.pose.position.z;
            } /*else {
                set_pos.pose.position.x = uav_pos.pose.position.x ;
                set_pos.pose.position.y = uav_pos.pose.position.y ;
                //set_pos.pose.position.z = uav_pos.pose.position.z;
				    }*/
        }

        local_pos_pub.publish(set_pos);

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}

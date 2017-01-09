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
bool flag = true ;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

<<<<<<< HEAD
=======

>>>>>>> 2cf3cf0b93791dc744847808b6895bc19a8fe636
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
<<<<<<< HEAD
        std::cout << "current_state.mode " << current_state.mode << "Time Diff" << ros::Time::now() - last_request << std::endl ;
        if(current_state.mode != "OFFBOARD" &&  (ros::Time::now() - last_request > ros::Duration(5.0))){
                std::cout << "not OFFboard"  << std::endl ;
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success){
=======
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
>>>>>>> 2cf3cf0b93791dc744847808b6895bc19a8fe636
                ROS_INFO("Offboard enabled");
                }
            last_request = ros::Time::now();
<<<<<<< HEAD
        }
        else {
            std::cout << "IN ELSE    " << std::endl ;
            if( !current_state.armed  && (ros::Time::now() - last_request > ros::Duration(5.0))){
                std::cout << "Not Armed" << std::endl ;
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
=======
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
>>>>>>> 2cf3cf0b93791dc744847808b6895bc19a8fe636
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }


        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

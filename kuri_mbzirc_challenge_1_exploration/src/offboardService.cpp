#include "ros/ros.h"
#include "kuri_mbzirc_challenge_1_msgs/Mission.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandTOL.h>


using namespace std;
mavros_msgs::SetMode setMode;
mavros_msgs::WaypointPush pushService;
mavros_msgs::Waypoint waypoint;
mavros_msgs::CommandTOL srv_takeoff;

bool takeOFFflag = true ;

enum SYSTEM_STATES{
    AUTO_MISSION =0 ,
    OFFBOARD =1 ,
    MANUAL =2,
    AUTO_TAKEOFF=3
};


int currentSystemState = 4;

mavros_msgs::State currentState;
void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    currentState = *msg;
    //  ROS_INFO("Sending Arming message FAILED!");
}

bool missionType(kuri_mbzirc_challenge_1_msgs::Mission::Request  &req,
                 kuri_mbzirc_challenge_1_msgs::Mission::Response &res)
{


    if (req.misionType == AUTO_MISSION)
    {
        waypoint.frame        = mavros_msgs::Waypoint::FRAME_GLOBAL;
        waypoint.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint.is_current   = false;
        waypoint.autocontinue = true;
        waypoint.x_lat        = 47.3977426 ;//47.3977998;
        waypoint.y_long       = 8.5459136 ; //8.5456246;
        waypoint.z_alt        = 2 ; // 10;
        pushService.request.waypoints.push_back(waypoint);


        currentSystemState = AUTO_MISSION;
        waypoint.frame        = mavros_msgs::Waypoint::FRAME_GLOBAL;
        waypoint.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint.is_current   = false;
        waypoint.autocontinue = true;
        waypoint.x_lat        = req.wayPointLat ;//47.3977998;
        waypoint.y_long       = req.wayPointLon ; //8.5456246;
        waypoint.z_alt        = req.wayPointAlt ; // 10;
        pushService.request.waypoints.push_back(waypoint);
        res.navResponse = true ;
        return true ;

    }
    else if (req.misionType == OFFBOARD)
    {
        currentSystemState = OFFBOARD ;
        res.navResponse = true ;
        return true ;
    }
    else if (req.misionType == AUTO_TAKEOFF)
    {
        takeOFFflag = false ;
        srv_takeoff.request.altitude = 5;
        srv_takeoff.request.latitude = 47.397742;
        srv_takeoff.request.longitude = 8.5459138;
        srv_takeoff.request.min_pitch = 0;
        srv_takeoff.request.yaw = 0;
        currentSystemState = AUTO_TAKEOFF ;
        res.navResponse = true ;

        return true ;
    }
    else
        return false ;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "missionService");
    ros::NodeHandle nh;
    ros::ServiceServer systemStateServer   = nh.advertiseService("systemState", missionType);
    ros::ServiceClient armingCmdClient     = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient setModeClient       = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient pushMissionClient   = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
    ros::ServiceClient clearMissionClient  = nh.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
    ros::ServiceClient takeoffCmdClient    = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    ros::Subscriber statePub               = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, mavrosStateCallback);

    ros::Rate rate(20.0);

    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::WaypointClear srv_clear_1;
    arm_cmd.request.value = true;
    ros::Time lastRequestT    = ros::Time::now();
    ros::Time statusUpdate    = ros::Time::now();
    bool modeChanged = false;
    int prevSystemState = currentSystemState;

    if(clearMissionClient.call(srv_clear_1)){ROS_INFO("clear send ok %d", srv_clear_1.response.success);}
    else{ROS_ERROR("Failed clear");}



    while(ros::ok())
    {
        if(prevSystemState != currentSystemState)
        {
            std::cout << "Mode Changed" << std:: endl ;
            switch(currentSystemState)
            {
                case AUTO_MISSION:
                    if(pushMissionClient.call(pushService))
                        ROS_INFO("Waypoint Sent Responce %d",pushService.response.success);
                    else
                        ROS_ERROR("FAILED");
                        setMode.request.custom_mode = "AUTO.MISSION";
                    break;
                case OFFBOARD:
                    setMode.request.custom_mode = "OFFBOARD";

                    break;
                case MANUAL:
                    setMode.request.custom_mode = "MANUAL";
                    break;
                case AUTO_TAKEOFF:
                    setMode.request.custom_mode = "AUTO.TAKEOFF";
                    break;
                default:
                    ROS_ERROR("Unknown System State Requested");
            }
            if( setModeClient.call(setMode) && setMode.response.success)
            {
                ROS_INFO(" Mode Changed");

            }
            else
            {
                ROS_ERROR("Failed to change to mode");
            }

            if (currentSystemState == AUTO_MISSION | currentSystemState == OFFBOARD | currentSystemState== AUTO_TAKEOFF )
            {
                if( !currentState.armed && (ros::Time::now() - lastRequestT > ros::Duration(1.0)))
                {

                    if( armingCmdClient.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("ARMING Command send through mavros, check messages related to safety switch");
                    }
                    else
                    {
                        ROS_INFO("Sending Arming message FAILED!");
                    }
                    lastRequestT = ros::Time::now();
                }
            }

            prevSystemState = currentSystemState;
        }

                if(!takeOFFflag )
                {
                    if(takeoffCmdClient.call(srv_takeoff)){ ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);}
                    else{ROS_ERROR("Failed Takeoff");}
                    takeOFFflag=true;
                }





        if(ros::Time::now() - statusUpdate > ros::Duration(0.5))
        {
            std::cout<<"Current Mode is: "<<currentState.mode<<"\n"; fflush(stdout);
            statusUpdate = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    if(clearMissionClient.call(srv_clear_1)){ROS_INFO("clear send ok %d", srv_clear_1.response.success);}
    else{ROS_ERROR("Failed clear");}
    ros::spin();
    return 0;
}

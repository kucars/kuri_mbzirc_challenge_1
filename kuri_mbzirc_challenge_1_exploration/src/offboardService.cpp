#include "ros/ros.h"
#include "kuri_mbzirc_challenge_1_msgs/Mission.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
using namespace std;
mavros_msgs::SetMode setMode;
mavros_msgs::WaypointPush pushService;
mavros_msgs::Waypoint waypoint;

enum SYSTEM_STATES{
    AUTO_MISSION =0 ,
    OFFBOARD =1 ,
    MANUAL =2
};

int currentSystemState = MANUAL;

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
        waypoint.command      = mavros_msgs::CommandCode::NAV_TAKEOFF;
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
    else
        return false ;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "missionService");
    ros::NodeHandle nh;
    ros::ServiceServer service            = nh.advertiseService("systemState", missionType);
    ros::Subscriber statePub              = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, mavrosStateCallback);
    ros::ServiceClient armingClient       = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient setModeClient      = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient setMissionWaypoint = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
    ros::Rate rate(20.0);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time lastRequestT    = ros::Time::now();
    ros::Time statusUpdate    = ros::Time::now();
    bool modeChanged = false;
    int prevSystemState = currentSystemState;

    while(ros::ok())
    {
        if(prevSystemState != currentSystemState)
        {
            switch(currentSystemState)
            {
                case AUTO_MISSION:
                    if(setMissionWaypoint.call(pushService))
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
            if (currentSystemState == AUTO_MISSION | currentSystemState == OFFBOARD)
            {
                if( !currentState.armed && (ros::Time::now() - lastRequestT > ros::Duration(1.0)))
                {

                    if( armingClient.call(arm_cmd) && arm_cmd.response.success)
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
        /*
        if (currentSystemState == 1 )
        {
            setMode.request.custom_mode = "AUTO.MISSION";
            //  std::cout<<"The mode has been sitted to Suto mission"  ;  fflush(stdout);

        }
        else if (currentSystemState == 2)
        {
            setMode.request.custom_mode = "OFFBOARD";
        }
        else
            setMode.request.custom_mode = "MANUAL";


        // This code overrides the RC mode and is dangerous when performing tests: re-use in real flight tests
        if( currentState.mode != "AUTO.MISSION" && (ros::Time::now() - lastRequestT > ros::Duration(5.0)))
        {
            if( setModeClient.call(setMode) && setMode.response.success)
            {
                ROS_INFO("AUTO enabled");

            }
            else
            {
                ROS_ERROR("Failed to change to AUTO mode");
            }
            lastRequestT = ros::Time::now();
        }


        if( currentState.mode != "OFFBOARD" && (ros::Time::now() - lastRequestT > ros::Duration(5.0)))
        {
            if( setModeClient.call(setMode) && setMode.response.success)
            {
                ROS_INFO("Offboard enabled");
            }
            else
            {
                ROS_ERROR("Failed to change to OFFboard mode");
            }
            lastRequestT = ros::Time::now();

        }

        if( !currentState.armed && (ros::Time::now() - lastRequestT > ros::Duration(1.0)))
        {

            if( armingClient.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("ARMING Command send through mavros, check messages related to safety switch");
            }
            else
            {
                ROS_INFO("Sending Arming message FAILED!");
            }
            lastRequestT = ros::Time::now();
        }
        */
        if(ros::Time::now() - statusUpdate > ros::Duration(0.5))
        {
            std::cout<<"Current Mode is: "<<currentState.mode<<"\n"; fflush(stdout);
            statusUpdate = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    return 0;
}

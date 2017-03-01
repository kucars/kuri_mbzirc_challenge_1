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

bool firstDataFlag = false ;
double roll, pitch, yaw;
int counter = 0 ;
float tolerance = 0.2;//0.2; // 0.05; //0.05;
float kp = 0.5;//7;//was 5 , new values //1//0.5//20
float ki = 0;//was 0.002 >> .02//2//0.005
float kd = 0.5;//15;//was 5 >> 15//10//2.5//1

float w;
float gain = 0.01 ; 
float error_x = 0;

float error_y = 0;
float error_z = 0;

float error_w = 0;
float prev_error_x = 0;

float prev_error_y = 0;
float prev_error_z = 0;

float prev_error_w = 0;
float rise = 1;

float nonstop = true;

float proportional_x = 0;
float proportional_y = 0;

float proportional_z = 0;
float proportional_w = 0;

float integral_x = 0;
float integral_y = 0;

float integral_z = 0;
float integral_w = 0;

float derivative_x = 0;
float derivative_y = 0;

float derivative_z = 0;
float derivative_w = 0;

float  action_x = 0;
float  action_y = 0;

float  action_z = 0;
float action_w = 0;

geometry_msgs ::Point real;
geometry_msgs ::TwistStamped twist;
kuri_mbzirc_challenge_1_msgs::pidData pidmsg;

bool mustExit   = false;
int waypointNum = 0;

mavros_msgs::State currentState;
void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    currentState = *msg;
}

geometry_msgs::PoseStamped goalPose;
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Received Goal: (" << msg->pose.position.x <<","<< msg->pose.position.y<<","<< msg->pose.position.z<<")");
    goalPose = *msg ;
}

void headingCallback(const std_msgs::Float64::ConstPtr& msg)
{
    yaw = msg->data * 3.14159265359 / 180.0 ;
}


void globalPoseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    real.x= 0 ; //msg ->latitude;
    real.y= 0 ; //msg ->longitude;
    real.z= 0 ; //msg ->altitude;
    w = yaw ;

    if (firstDataFlag == false )
    {
        error_x =  goalPose.pose.position.x - real.x;
        error_y =  goalPose.pose.position.y - real.y;
        error_z =  goalPose.pose.position.z - real.z ;
        error_w = 0 - w;
        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_error_z = error_z;
        prev_error_w = error_w;
        firstDataFlag = true;

    }
    else {

        error_x =  goalPose.pose.position.x - real.x;
        error_y =  goalPose.pose.position.y - real.y;
        error_z =  goalPose.pose.position.z - real.z ;
        error_w = 0 - w;
        std::cout << "Error X " <<  goalPose.pose.position.x  << "\t\t\t\t\t\t"    <<  real.x  <<  "\t\t\t\t\t\t"  << error_x <<std::endl;
        std::cout << "Error Y " <<  goalPose.pose.position.y  << "\t\t\t\t\t"    <<  real.y  <<  "\t\t\t\t\t"   << error_y <<std::endl;
        std::cout << "Error Z " <<  goalPose.pose.position.z  << "\t\t\t\t\t\t"    <<  real.z  <<  "\t\t\t\t\t\t"   << error_z <<std::endl;

        proportional_x = kp * error_x;
        proportional_y = kp * error_y;
        proportional_z = kp * error_z;
        proportional_w = kp * error_w;

        integral_x += ki * error_x;
        integral_y += ki * error_y;
        integral_z += ki * error_z;
        integral_w += ki * error_w;

        derivative_x = kd * (error_x - prev_error_x);
        derivative_y = kd * (error_y - prev_error_y);
        derivative_z = kd * (error_z - prev_error_z);
        derivative_w = kd * (error_w - prev_error_w);

        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_error_z = error_z;
        prev_error_w = error_w;

        // PID conroller
        action_x = proportional_x     + integral_x + derivative_x  ;
        action_y = proportional_y     + integral_y + derivative_y  ;
        action_z = proportional_z      +integral_z + derivative_z  ;
        action_w = 10 * proportional_w +integral_w + derivative_w  ;

        // filling velocity commands
        twist.twist.linear.x = action_x;
        twist.twist.linear.y = action_y;
        twist.twist.linear.z = action_z;
        twist.twist.angular.z = action_w;


        ROS_INFO("Error X: %0.2f \n", error_x);
        ROS_INFO("Error Y: %0.2f \n", error_y);
        ROS_INFO("Error Z: %0.2f \n", error_z);
        ROS_INFO("derivative X: %0.2f \n", derivative_x);
        ROS_INFO("derivative Y: %0.2f \n", derivative_y);
        ROS_INFO("derivative Z: %0.2f \n", derivative_z);
        ROS_INFO("derivative W: %0.2f \n", derivative_z);
        ROS_INFO("W: %0.2f \n", w);
        ROS_INFO("Action X: %0.2f \n", action_x);
        ROS_INFO("Action Y: %0.2f \n", action_y);
        ROS_INFO("Action Z: %0.2f \n", action_z);
        ROS_INFO("Action W: %0.2f \n", action_w);

        // recording data for analysis
        pidmsg.dronePoseX = real.x;                       pidmsg.dronePoseY = real.y;                     pidmsg.dronePoseZ = real.z;
        pidmsg.goalPoseX  = goalPose.pose.position.x ;    pidmsg.goalPoseY  = goalPose.pose.position.y;   pidmsg.goalPoseZ  = goalPose.pose.position.z;
        pidmsg.positionErrorX = error_x;                  pidmsg.positionErrorY = error_y;                pidmsg.positionErrorZ = error_z;                pidmsg.positionErrorW = error_w;
        pidmsg.PX = proportional_x;                       pidmsg.PY = proportional_y;                     pidmsg.PZ = proportional_z;                     pidmsg.PW = proportional_w;
        pidmsg.IX = integral_x;                           pidmsg.IY = integral_y;                         pidmsg.IZ = integral_z;                         pidmsg.IW = integral_w;
        pidmsg.DX = derivative_x;                         pidmsg.DY = derivative_y;                       pidmsg.DZ = derivative_z;                       pidmsg.DW = derivative_w;
        pidmsg.PIDX =action_x;                            pidmsg.PIDY = action_y;                         pidmsg.PIDZ= action_z;                          pidmsg.PIDW= action_w;
        pidmsg.header.stamp = ros::Time::now();
        pidmsg.header.seq = counter++;

        if ((fabs(error_x) < tolerance) && (fabs(error_y) < tolerance) && (fabs(error_z) < tolerance))
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
    ros::init(argc , argv , "kuri_offboard_attitude_control_test");
    ros::NodeHandle nh;

    ros::Publisher velPub = nh.advertise <geometry_msgs ::TwistStamped >("/mavros/setpoint_velocity/cmd_vel", 1);
    ros::Publisher pidPub = nh.advertise <kuri_mbzirc_challenge_1_msgs::pidData >("/pidData", 1);

    ros::Subscriber goalSub          = nh.subscribe("/kuri_offboard_attitude_control_test/goal", 1000, goalCallback);
    ros::Subscriber pose_global_sub = nh.subscribe("/mavros/global_position/global", 1000, globalPoseCallback);
    ros::Subscriber compass_pub = nh.subscribe ("/mavros/global_position/compass_hdg", 1, headingCallback);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");


    mavros_msgs::CommandBool armCommand;

    armCommand.request.value = true;

    ros::Rate loopRate(10);

    goalPose.pose.position.x = 0;
    goalPose.pose.position.y = 0;
    goalPose.pose.position.z = 0;

    ros::Time last_request    = ros::Time::now();
    ros::Time statusUpdate    = ros::Time::now();
    ros::Time waitingForOFF   = ros::Time::now();

    while (ros::ok())
    {
        velPub.publish(twist);
        pidPub.publish(pidmsg);
        ros:: spinOnce ();
        loopRate.sleep();
    }
}

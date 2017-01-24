    #include <ros/ros.h>
    #include <geometry_msgs/TwistStamped.h>
    #include <geometry_msgs/PoseStamped.h>
    #include <std_msgs/String.h>
    #include <sensor_msgs/RegionOfInterest.h>
    #include "kuri_mbzirc_challenge_1_msgs/PES.h"
#include <sensor_msgs/NavSatFix.h>

    #include <mavros_msgs/CommandBool.h>
    #include <mavros_msgs/SetMode.h>
    #include <mavros_msgs/State.h>
    std_msgs::String endMsg ;  
    int setFlag = 0 ; 
    float tolerance = 1 ; //0.05;
    double kp = 10; //5;
    double ki = 0.002;
    double kd = 5;

    float w;

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

    float action_x = 0;
    float action_y = 0;

    float action_z = 0;
    float action_w = 0;

    geometry_msgs ::Point real;
    geometry_msgs ::TwistStamped twist;
    
    bool mustExit   = false;
    int waypointNum = 0;
    
    int caseSwitch  = 100; 
    int recivedPoint = 0 ; 

    
    
    
    void MissionCallback(const std_msgs::String::ConstPtr& msg)
    {
	if (msg->data == "startMission")
	  caseSwitch = 0 ; 
	else if (msg->data == "startLanding")
	  caseSwitch = 1 ; 
	else if (msg->data == "startDocking" )
	  caseSwitch = 2 ; 
	  //std::cout << "Switch Case 0 call back" <<  caseSwitch << std::endl ; ; 
    }

    mavros_msgs::State currentState;
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
      currentState = *msg;
    }
  
  geometry_msgs::PoseStamped goalPose;
    
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      
      ROS_INFO_STREAM("Received Goal: (" << msg->pose.position.x <<","<< msg->pose.position.y<<","<< msg->pose.position.z<<")");
      goalPose = *msg ;
    }
    
    void globalPoseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
      //std::cout << "switch " <<  caseSwitch << std::endl ; ; 

      //real.x=msg ->pose.position.x;
      //real.y=msg ->pose.position.y;
      //real.z=msg ->pose.position.z;
      //w=msg ->pose.orientation.z;
      real.x=msg ->latitude;
      real.y=msg ->longitude;
      real.z=msg ->altitude;
      //w=msg ->pose.orientation.z;

      prev_error_x = error_x;
      prev_error_y = error_y;
      prev_error_z = error_z;

      prev_error_w = error_w;
      error_x =  goalPose.pose.position.x - real.x;
      error_y =  goalPose.pose.position.y - real.y;
      //error_z = (goalPose.pose.position.z + 0.001 * rise) - real.z;
      error_z = goalPose.pose.position.z  - real.z;

      error_w = 0 - w;

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

      action_x = proportional_x + integral_x + derivative_x;
      action_y = proportional_y + integral_y + derivative_y;
      action_z = proportional_z + integral_z + derivative_z;
      action_w = 10 * proportional_w + integral_w + derivative_w;

      twist.twist.linear.x = 0.001* action_x;
      twist.twist.linear.y = 0.001*action_y;
      twist.twist.linear.z = 0.01*action_z;
      twist.twist.angular.z = action_w;
    /* ROS_INFO("Error X: %0.2f \n", error_x);
      ROS_INFO("Error Y: %0.2f \n", error_y);
      ROS_INFO("Error Z: %0.2f \n", error_y);
      ROS_INFO("W: %0.2f \n", w);
      ROS_INFO("Action X: %0.2f \n", action_x);
      ROS_INFO("Action Y: %0.2f \n", action_y);
      ROS_INFO("Action Z: %0.2f \n", action_y);
      ROS_INFO("Action W: %0.2f \n", action_y);

      ROS_INFO("Voy hacie el objetivo %d \n", waypointNum +1);
  */
      
      if ((fabs(error_x) < tolerance) && (fabs(error_y) < tolerance) && (fabs(error_z) < tolerance))
      {
      // if (mustExit == true)
      // {
	  twist.twist.linear.x = 0;
	  twist.twist.linear.y = 0;
	  twist.twist.linear.z = 0;
	  twist.twist.angular.z = 0;
	  std::cout << "Swich Value " << std::endl; 
      switch (caseSwitch)
      {
	case 0 : 
	  endMsg.data = "reachedCenter" ; 
	  break ; 
	case 1: 
	  endMsg.data = "Landed" ; 
	  break ; 
	case 2 : 
	  endMsg.data = "Docked" ; 
	  break ; 
	default: 
	  endMsg.data = "Nothing" ; 
      }

	  
	  //exit (0);
	}
	else
	{
	  waypointNum += 1;
	  rise += 1;
	}
      
      /*
      if (waypointNum == (sizeof(goal)/sizeof(goal [0])))
      {
	if (nonstop)
	{
	  waypointNum = 1;
	}
	else
	{
	  waypointNum = 0;
	  mustExit = true;
	}
      }
      */
    }

    int main(int argc , char **argv)
    {
      ros::init(argc , argv , "kuri_offboard_attitude_control_test");
      ros::NodeHandle nh;
      ros::Publisher velPub = nh.advertise <geometry_msgs ::TwistStamped >("/mavros/setpoint_velocity/cmd_vel", 1);
      ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
      //ros::Subscriber localPoseSub     = nh.subscribe("/mavros/local_position/pose", 1000, localPoseCallback);
      ros::Subscriber pose_global_sub = nh.subscribe("/mavros/global_position/global", 1000, globalPoseCallback);
      //ros::Subscriber localPoseSub     = nh.subscribe("/mavros/local_position/pose", 1000, localPoseCallback);
      ros::Subscriber goalSub          = nh.subscribe("/kuri_offboard_attitude_control_test/goal", 1000, goalCallback);
      ros::Subscriber missionSub      = nh.subscribe("/startMission", 1000, MissionCallback);
      ros::Publisher endMissionPub = nh.advertise <std_msgs::String>("/endMission", 1);   
      mavros_msgs::CommandBool armCommand;

      armCommand.request.value = true;

      ros::Rate loopRate(10);
      int count = 0;
      
      //goalPose.pose.position.x = 0;
      //goalPose.pose.position.y = 0;
      //goalPose.pose.position.z = 0;        
      ros::Time last_request    = ros::Time::now();
      ros::Time statusUpdate    = ros::Time::now();
      ros::Time waitingForOFF   = ros::Time::now();

      while (ros::ok())
      {
	if( currentState.mode != "OFFBOARD")
	{
	  if(ros::Time::now() - waitingForOFF > ros::Duration(0.5))
	  {
	    //std::cout<<"USE RC to set the system in offboard mode => Current Mode is: "<<currentState.mode<<"\n"; fflush(stdout);
	    waitingForOFF = ros::Time::now();
	  }
	}
	else
	{
	  if( !currentState.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
	  {
	    if( arming_client.call(armCommand) && armCommand.response.success)
	    {
	      ROS_INFO("ARMING Command send through mavros, check messages related to safety switch");
	    }
	    else
	    {
	      ROS_INFO("Sending Arming message FAILED!");
	    }
	    last_request = ros::Time::now();
	  }
	  if(ros::Time::now() - statusUpdate > ros::Duration(1.0))
	  {
	  //  std::cout<<"Current Mode is: "<<currentState.mode<<"\n"; fflush(stdout);
	    statusUpdate = ros::Time::now();
	  }
	}
	velPub.publish(twist);
	endMissionPub.publish(endMsg) ; 
	ros:: spinOnce ();
	loopRate.sleep();
	++count;
      }
    }

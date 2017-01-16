#include "ros/ros.h"
#include <cstdlib>


#include "kuri_mbzirc_challenge_1_msgs/PoseEsti.h"
#include "kuri_mbzirc_challenge_1_msgs/PES.h"
#include <kuri_msgs/Object.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_estimation_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<kuri_mbzirc_challenge_1_msgs::PES>("position_estimation");
  kuri_mbzirc_challenge_1_msgs::PES srv;
  srv.request.A = 320;
  srv.request.B = 240;
  if (client.call(srv))
  {
    ROS_INFO("poseX: %ld", (long int)srv.response.obj.pose.pose.position.x);
    ROS_INFO("poseY: %ld", (long int)srv.response.obj.pose.pose.position.y);
    ROS_INFO("poseZ: %ld", (long int)srv.response.obj.pose.pose.position.z);

  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}

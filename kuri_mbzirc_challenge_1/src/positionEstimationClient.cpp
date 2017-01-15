#include "ros/ros.h"
#include "kuri_mbzirc_challenge_1_msgs/PES.h"
#include "kuri_mbzirc_challenge_1_msgs/PoseEsti.h"

#include "kuri_msgs/Object.h"

#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "poseEstimationClient");
    ros::NodeHandle n;

   // ros::service::waitForService("poseEstimationServer");
    ros::ServiceClient client = n.serviceClient<kuri_mbzirc_challenge_1_msgs::PES>("poseEstimationServer");
    kuri_mbzirc_challenge_1_msgs::PES srv;

    srv.request.uv.x = 100;
    srv.request.uv.y = 100;
    kuri_msgs::Object w ;

    std::cout << "calling the servise " << client.call(srv) << std::endl  << std::flush ;
   // while (ros::ok())
   // {
        if (client.call(srv))
        {
           // w =  srv.response.obj ;
           ROS_INFO("srv.response.obj.pose.pose.position.x");
        }
        else
        {
            ROS_ERROR("Failed to call service position Estimation");
            //return 1;
        }
  //  }
    return 0;
}

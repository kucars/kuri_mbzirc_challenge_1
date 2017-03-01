#include "ros/ros.h"
#include <cstdlib>


#include "kuri_mbzirc_challenge_1_msgs/PES.h"
#include <kuri_msgs/Object.h>
#include <sensor_msgs/RegionOfInterest.h>

int cameraU = 0  , cameraV = 0 ;

void cameraCallback(const sensor_msgs::RegionOfInterestConstPtr& topic)
{
    cameraU =  topic->x_offset + (topic->height /2.0)    ;
    cameraV =  topic->y_offset + (topic->width /2.0)   ;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_estimation_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<kuri_mbzirc_challenge_1_msgs::PES>("position_estimation");
  ros::Subscriber detectionPub  = n.subscribe<sensor_msgs::RegionOfInterest>("/visptracker_data", 1, cameraCallback);
  kuri_mbzirc_challenge_1_msgs::PES srv;
  ros::Rate rate(20.0);

  while(ros::ok())
  {

      if (cameraU != 0 )
      {

      srv.request.A = cameraU ;
      srv.request.B = cameraV;

      if (client.call(srv))
      {
          //std::cout<<"poseX is: "<<currentState.mode<<"\n"; fflush(stdout);

          ROS_INFO("poseX: %ld", (long int)srv.response.X);
          ROS_INFO("poseY: %ld", (long int)srv.response.Y);
          ROS_INFO("poseZ: %ld", (long int)srv.response.Z);
      }
      else
      {
        ROS_ERROR("Failed to call service");
        return 1;
      }

      }
      else
          ROS_ERROR("waiting for data");

      ros::spinOnce();
      rate.sleep();
  }



  ros::spin();




  return 0;
}

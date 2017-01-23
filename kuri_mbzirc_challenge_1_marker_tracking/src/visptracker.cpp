#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"

#include "visp_bridge/image.h"
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMeLine.h>
#include <visp/vpTemplateTracker.h>
#include <visp/vpTemplateTrackerSSDESM.h>
#include <visp/vpTemplateTrackerSSDForwardAdditional.h>
#include <visp/vpTemplateTrackerSSDForwardCompositional.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerZNCCForwardAdditional.h>
#include <visp/vpTemplateTrackerZNCCInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#include <visp/vpTrackingException.h>

#include "detector/landing_mark_detection.h"

#include "mark_tracker.h"

#include <time.h>

ros::Publisher roiPub;
TrackLandingMark * detectorTracker;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  clock_t start, end;
  //ROS_INFO("Image Recevied, %d %d %d", msg->header.seq, msg->width, msg->height);
  start = clock();
  detectorTracker->detectAndTrack(msg);
  end = clock();
  //ROS_INFO("detectorTracker took %f seconds", ((float)(end - start))/CLOCKS_PER_SEC);
  sensor_msgs::RegionOfInterest data = detectorTracker->getRegionOfInterest();
  roiPub.publish(data);
}

// Reads params, or sets default parameters if parameters are not found
void readParams(int& camSizeX, int& camSizeY, int& trackerType, std::string& camTopic, std::string& pubTopic)
{
  if(!(ros::param::get("/visptracker/cam_resolution_x", camSizeX) && 
	 ros::param::get("/visptracker/cam_resolution_y", camSizeY)))
  {
	ROS_WARN("Cannot find camera resolution params cam_resolution_x or cam_resolution_y");
	camSizeX = 1920;
	camSizeY = 1080;
  }
  ROS_INFO("Set camera resolution to %d %d", camSizeX, camSizeY);
  
  if(ros::param::get("/visptracker/tracker_type", trackerType) && (trackerType < 0 || trackerType > 5))
  {
	ROS_WARN("Cannot read valid tracker type parameter");
	trackerType = 1;
  }
  ROS_INFO("Set tracker type to %d", trackerType);

  
  if(!ros::param::get("/visptracker/cam_topic", camTopic))
  {
	ROS_WARN("Cannot get cam_topic string parameter");
	camTopic = "/ardrone/downward_cam/camera/image";
  }
  ROS_INFO("Set camera topic to %s", camTopic.c_str());

  
  if(!ros::param::get("/visptracker/pub_topic", pubTopic))
  {
	ROS_WARN("Cannot get publisher topic string parameter");
	pubTopic = "visptracker_data";
  }
  ROS_INFO("Set publisher topic name as %s", pubTopic.c_str());
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "visptracker");
  ros::NodeHandle n;
  
  // read parameters first
  int camSizeX, camSizeY, trackerType;
  std::string camTopic, pubTopic;
  ROS_INFO("Reading parameters...");
  readParams(camSizeX, camSizeY, trackerType, camTopic, pubTopic);
  ROS_INFO("Parameters loaded successfully!");
  
  detectorTracker = new TrackLandingMark(camSizeX, camSizeY, (TrackerType)trackerType);
  
  ros::Subscriber sub = n.subscribe(camTopic, 1, imageCallback);
  roiPub = n.advertise<sensor_msgs::RegionOfInterest>(pubTopic, 1000);

  detectorTracker->setSampling(4, 4);
  detectorTracker->setLambda(0.001);
  detectorTracker->setIterationMax(50);
  detectorTracker->setPyramidal(4, 2);

  detectorTracker->enableTrackerDisplay(true);
  
  ros::spin();
   
  return 0;
}

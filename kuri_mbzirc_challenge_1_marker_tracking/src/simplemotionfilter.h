#include <ros/ros.h>
#include "sensor_msgs/RegionOfInterest.h"

// This class is a motion filter that operates in a simple manner
// It takes in a TrackerData and can linearly interpolate based on previous motion
class SimpleMotionFilter
{
public:
  SimpleMotionFilter();
  SimpleMotionFilter(SimpleMotionFilter& copy);
  ~SimpleMotionFilter();
  
  void boxUpdate(sensor_msgs::RegionOfInterest& data, float step);
  sensor_msgs::RegionOfInterest interpolate(float step); // input seconds since last update, output interpolated bounding box
  
private:
  sensor_msgs::RegionOfInterest current;
  sensor_msgs::RegionOfInterest prev;
  float timestep; // seconds between the two updates
};

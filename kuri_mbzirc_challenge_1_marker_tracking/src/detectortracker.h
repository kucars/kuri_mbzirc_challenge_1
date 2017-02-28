#ifndef DETECTOR_TRACKER_H
#define DETECTOR_TRACKER_H

#include "sensor_msgs/Image.h"
#include "sensor_msgs/RegionOfInterest.h"

/* This class represents an interface to any Tracker used in this program
 * to allow the main program to easily switch between them
 */

class DetectorTracker
{
public:
  virtual bool detectAndTrack(const sensor_msgs::Image::ConstPtr& msg) = 0;
  virtual void reset() = 0;
  virtual sensor_msgs::RegionOfInterest getRegionOfInterest() = 0;
  virtual bool markDetected() = 0;
  virtual bool isTracking() = 0;

  virtual void enableTrackerDisplay(bool enabled) = 0;

};

#endif

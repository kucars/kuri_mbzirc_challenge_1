#ifndef MARK_TRACKER_H
#define MARK_TRACKER_H

#include <visp/vpDisplayX.h>
#include <visp/vpTemplateTracker.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#include "detectortracker.h"
#include "detector/landing_mark_detection.h"
#include "visp_bridge/image.h"
#include "sensor_msgs/RegionOfInterest.h"


enum TrackerType 
{
  SSDESM,
  SSDForwardAdditional,
  SSDForwardCompositional,
  SSDInverseCompositional,
  ZNCCForwardAdditional,
  ZNCCInverseCompositional,
};

class TrackLandingMark : public DetectorTracker
{
public:
  TrackLandingMark();
  TrackLandingMark(TrackLandingMark& copy);
  ~TrackLandingMark();
  
  TrackLandingMark(int x, int y, TrackerType type);

  // returns true if detected and tracking, false if not detected
  bool detectAndTrack(const sensor_msgs::Image::ConstPtr& msg);
  void reset();
  
  // get methods
  sensor_msgs::RegionOfInterest getRegionOfInterest();
  vpTemplateTrackerWarpHomography getHomography();
  bool markDetected();
  bool isTracking();
  
  // set methods
  void enableTrackerDisplay(bool enabled);
  void setSampling(int i, int j);
  void setLambda(double l);
  void setIterationMax(const unsigned int& n);
  void setPyramidal(unsigned int nlevels, unsigned int level_to_stop);
  
private:
  DetectLandingMark detector;
  vpTemplateTracker * tracker;
  vpTemplateTrackerWarpHomography * warp;
  
  sensor_msgs::RegionOfInterest roi;
  
  bool detectedState; // true if we detected something in the last state;
  bool trackingState; // true if we're tracking
  bool displayEnabled; // true if we want to display the results of the detector/tracker
  vpImage<unsigned char> I;
  vpDisplayX * display;
};

#endif

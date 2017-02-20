#ifndef VISP_MB_TRACKER_H
#define VISP_MB_TRACKER_H

#include <visp/vpDisplayX.h>
//#include <visp/
#include <visp/vpMbEdgeTracker.h>

#include "detector/landing_mark_detection.h"
#include "visp_bridge/image.h"
#include "sensor_msgs/RegionOfInterest.h"


/**
 * This class represents a model-based tracker using the VISP framework	
 */

class TrackMarkerModel
{
public:
  TrackMarkerModel();
  TrackMarkerModel(TrackMarkerModel& copy);
  ~TrackMarkerModel();
  
  // X and Y are the size of the image
  TrackMarkerModel(int x, int y);
  
  // returns true if it's tracking
  bool detectAndTrack(const sensor_msgs::Image::ConstPtr& msg);
  void reset();
  
  // get methods
  bool markerDetected();
  bool isTracking();
  sensor_msgs::RegionOfInterest getRegionOfInterest();
  
  // set methods
  void enableTrackerDisplay(bool enabled);
  void setMaskSize(int size);
  void setMaskNumber(int num);
  void setRange(int range);
  void setThreshold(int threshold);
  void setMu1(double mu1);
  void setMu2(double mu2);
  void setSampleStep(int samplestep);
  void setCameraParameters(double px, double py, double u0, double v0);

  
private:
  DetectLandingMark detector;
  vpHomogeneousMatrix cMo;
  vpMe * me;
  vpMbTracker * tracker;
  
  bool detectedState;
  bool trackingState;
  bool displayEnabled;
  
  vpImage<unsigned char> I;
  vpDisplayX * display;
};

#endif

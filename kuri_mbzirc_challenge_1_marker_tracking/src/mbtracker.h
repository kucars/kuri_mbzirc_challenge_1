#ifndef VISP_MB_TRACKER_H
#define VISP_MB_TRACKER_H

#include <visp/vpDisplayX.h>
#include <visp/vpMbEdgeTracker.h>
#include "detectortracker.h"
#include "detector/landing_mark_detection.h"
#include "simpleposeextrapolator.h"
#include "posemeanfilter.h"
#include "visp_bridge/image.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "geometry_msgs/PoseStamped.h"
#include <time.h>

/**
 * This class represents a model-based tracker using the VISP framework	
 */

class TrackMarkerModel : public DetectorTracker
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
  bool markDetected();
  bool isTracking();
  sensor_msgs::RegionOfInterest getRegionOfInterest();
  geometry_msgs::PoseStamped getPoseStamped();
  geometry_msgs::PoseStamped getFilteredPoseStamped();
  double getExtrapolateMaxTime();
  clock_t getLastTrackTime();
  bool isResultPredicted(); // returns true if the result was predicted

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
  void setExtrapolateMaxTime(double newTime);
  void setMeanFilterSize(unsigned int newSize);

  
private:
  DetectLandingMark detector;
  vpHomogeneousMatrix cMo;
  vpHomogeneousMatrix filteredCMo; // pose matrix taken from filter
  vpMe * me;
  vpMbTracker * tracker;
  PoseMeanFilter meanFilter;
  SimplePoseExtrapolator extrapolator;
  
  bool detectedState;
  bool trackingState;
  bool resultIsPredicted;
  bool displayEnabled;

  sensor_msgs::RegionOfInterest roi;
  sensor_msgs::RegionOfInterest filteredROI;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped filteredPose;

  clock_t lastTrackTime; // stores the last time we successfully tracked
  double extrapolateMaxTime;
  
  vpImage<unsigned char> I;
  vpDisplayX * display;
  std::vector<vpPoint> marker3DPoints; // holds the 3D points of the face of the marker

  // helper functions
  void getROIFromBoxPoints(std::vector<vpImagePoint>& boxPoints, sensor_msgs::RegionOfInterest& roi);
};

#endif

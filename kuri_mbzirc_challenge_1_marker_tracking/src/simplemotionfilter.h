#include "kuri_mbzirc_challenge_1_marker_tracking/TrackerData.h"

// This class is a motion filter that operates in a simple manner
// It takes in a TrackerData and can linearly interpolate based on previous motion
class SimpleMotionFilter
{
public:
  SimpleMotionFilter();
  SimpleMotionFilter(SimpleMotionFilter& copy);
  ~SimpleMotionFilter();
  
  void boxUpdate(kuri_mbzirc_challenge_1_marker_tracking::TrackerData& data, float step);
  kuri_mbzirc_challenge_1_marker_tracking::TrackerData interpolate(float step); // input seconds since last update, output interpolated bounding box
  
private:
  kuri_mbzirc_challenge_1_marker_tracking::TrackerData current;
  kuri_mbzirc_challenge_1_marker_tracking::TrackerData prev;
  float timestep; // seconds between the two updates
};

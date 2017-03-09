#ifndef SIMPLEPOSEEXTRAPOLATOR_H
#define SIMPLEPOSEEXTRAPOLATOR_H

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"

// This class simply keeps checking the latest few poses and then allows you to extrapolate
// based on the last few pose updates. This class is designed to work with VISP's
// Model-based tracker, which can provide a pose matrix
class SimplePoseExtrapolator
{
public:
  SimplePoseExtrapolator();
  SimplePoseExtrapolator(const SimplePoseExtrapolator& copy);
  ~SimplePoseExtrapolator();

  void poseUpdate(geometry_msgs::Pose newPose, double step);
  geometry_msgs::Pose extrapolate(double step); // input seconds since last update
  void reset();

private:
  geometry_msgs::Pose current;
  geometry_msgs::Pose prev;
  double timestep; //seconds between the two updates
  bool ready; // set to true if we have updated the pose at least twice
};

#endif
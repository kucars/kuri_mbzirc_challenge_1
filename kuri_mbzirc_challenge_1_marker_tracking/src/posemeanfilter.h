#ifndef POSEMEANFILTER_H
#define POSEMEANFILTER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <deque>

// Takes in a bunch of poses and is able to give you their average
class PoseMeanFilter
{
public:
  PoseMeanFilter();
  PoseMeanFilter(const PoseMeanFilter& copy);
  ~PoseMeanFilter();

  PoseMeanFilter(unsigned int numPoses);

  void poseUpdate(geometry_msgs::Pose& newPose);
  geometry_msgs::Pose getMeanPose();

  void reset();
  void setMaxSize(unsigned int newSize);
  unsigned int getMaxSize();
  unsigned int getCurrentSize();

private:
  unsigned int maxSize; // maxPoses
  std::deque<geometry_msgs::Pose> poses;
};

#endif
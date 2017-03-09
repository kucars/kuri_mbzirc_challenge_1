#include "posemeanfilter.h"
#include <assert.h>

PoseMeanFilter::PoseMeanFilter()
{
  maxSize = 5; // hardcoded default.. had to put something...
}

PoseMeanFilter::~PoseMeanFilter()
{
}

PoseMeanFilter::PoseMeanFilter(unsigned int numPoses)
{
  if(numPoses != 0)
	maxSize = numPoses;
  else
	maxSize = 5;
}

void PoseMeanFilter::poseUpdate(geometry_msgs::Pose& newPose)
{
  assert(poses.size() <= maxSize);
  if(poses.size() == maxSize)
  {
	poses.pop_front();
  }
  poses.push_back(newPose);
}

geometry_msgs::Pose PoseMeanFilter::getMeanPose()
{
  if(poses.size() == 0)
	return geometry_msgs::Pose();

  if(poses.size() == 1){
	return poses[0];
  }

  // there's probably a faster way of doing this but I'm lazy
  double vector[7];
  memset(vector, sizeof(vector), 0);

  for(std::deque<geometry_msgs::Pose>::iterator it = poses.begin(); it != poses.end(); ++it)
  {
	vector[0] += it->position.x;
	vector[1] += it->position.y;
	vector[2] += it->position.z;
	vector[3] += it->orientation.x;
	vector[4] += it->orientation.y;
	vector[5] += it->orientation.z;
	vector[6] += it->orientation.w;
  }

  for(int i = 0; i < sizeof(vector) / sizeof(double); i++){
	vector[i] /= poses.size();
  }

  geometry_msgs::Pose averagePose;
  averagePose.position.x = vector[0];
  averagePose.position.y = vector[1];
  averagePose.position.z = vector[2];
  averagePose.orientation.x = vector[3];
  averagePose.orientation.y = vector[4];
  averagePose.orientation.z = vector[5];
  averagePose.orientation.w = vector[6];

  return averagePose;
}

void PoseMeanFilter::reset()
{
  poses.clear();
}

void PoseMeanFilter::setMaxSize(unsigned int newSize)
{
  if(newSize == 0)
	return; // invalid, don't do anything

  if(newSize < maxSize)
  {
	// pop out the old ones
	for(int i = 0; i < (maxSize - newSize); i++)
	{
	  poses.pop_front();
	}
  }

  maxSize = newSize;
}

unsigned int PoseMeanFilter::getMaxSize()
{
  return maxSize;
}

unsigned int PoseMeanFilter::getCurrentSize()
{
  return poses.size();
}
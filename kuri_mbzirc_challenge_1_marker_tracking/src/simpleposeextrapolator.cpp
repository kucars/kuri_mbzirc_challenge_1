#include "simpleposeextrapolator.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

SimplePoseExtrapolator::SimplePoseExtrapolator()
{
}

SimplePoseExtrapolator::~SimplePoseExtrapolator()
{
}

static void calcVelocityVector(geometry_msgs::Pose& prev, geometry_msgs::Pose& current,
							   double step, double out[3])
{
  out[0] = (current.position.x - prev.position.x) / step;
  out[1] = (current.position.y - prev.position.y) / step;
  out[2] = (current.position.z - prev.position.z) / step;
}

static void calcNewPoint(geometry_msgs::Pose& lastPose, double velVector[3], double time, double out[3])
{
  out[0] = lastPose.position.x + (velVector[0] * time);
  out[1] = lastPose.position.y + (velVector[1] * time);
  out[2] = lastPose.position.z + (velVector[2] * time);
}

void SimplePoseExtrapolator::poseUpdate(geometry_msgs::Pose newPose, double step)
{
  prev = current;
  current = newPose;
  timestep = step;
}

geometry_msgs::Pose SimplePoseExtrapolator::extrapolate(double step) // input seconds since last update
{
  // calculate new position
  double velVec[3];
  double newPos[3];
  calcVelocityVector(prev, current, timestep, velVec);
  calcNewPoint(current, velVec, step, newPos);

  // convert to TF2 quaternions
  tf2::Quaternion prevQuat, currentQuat;
  tf2::fromMsg(prev.orientation, prevQuat);
  tf2::fromMsg(current.orientation, currentQuat);

  // calculate new quaternion (not sure if this is the correct way)
  tf2::Quaternion newQuat = prevQuat.slerp(currentQuat, 1 + (step/timestep));

  // Return as pose
  geometry_msgs::Pose newPose;
  newPose.position.x = newPos[0];
  newPose.position.y = newPos[1];
  newPose.position.z = newPos[2];
  newPose.orientation = tf2::toMsg(newQuat);
  
  return newPose;
}

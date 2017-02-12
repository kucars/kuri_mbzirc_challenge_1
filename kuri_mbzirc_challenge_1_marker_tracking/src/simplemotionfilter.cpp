#include "simplemotionfilter.h"
#include <cmath>

static float calcSpeed(float prev, float current, float step)
{
  return ((current - prev) / step);
}

static int calcNewPoint(float current, float speed, float step)
{
  return (int)round((float)current + (speed * step));
}

SimpleMotionFilter::SimpleMotionFilter()
{
  timestep = 0.0f;
}

SimpleMotionFilter::~SimpleMotionFilter()
{
}

void SimpleMotionFilter::boxUpdate(sensor_msgs::RegionOfInterest& data, float step)
{
  prev = current;
  current = data;
  timestep = step;
}

sensor_msgs::RegionOfInterest SimpleMotionFilter::interpolate(float step)
{
  float minXSpeed = calcSpeed(prev.x_offset, current.x_offset, timestep);
  float minYSpeed = calcSpeed(prev.y_offset, current.y_offset, timestep);
  float maxXSpeed = calcSpeed(prev.x_offset + prev.width, current.x_offset + current.width, timestep);
  float maxYSpeed = calcSpeed(prev.y_offset + prev.height, current.y_offset + current.height, timestep);
  
  sensor_msgs::RegionOfInterest newData;
  
  // confidence is calculated as 1/(1 + frames), where frames is (step / this->timestep)
  // i.e. for each "extra" frame this interpolates, the confidence drops by half
  // newData.confidence = 1.0f / (1.0f + (step / timestep)); // We switched to RegionOfInterest (which doesn't store confidence)
  newData.x_offset = calcNewPoint(current.x_offset, minXSpeed, step);
  newData.y_offset = calcNewPoint(current.y_offset, minYSpeed, step);
  newData.width = calcNewPoint(current.x_offset + current.width, maxXSpeed, step) - newData.x_offset;
  newData.height = calcNewPoint(current.y_offset + current.height, maxYSpeed, step) - newData.y_offset;
  
  return newData;  
}


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

void SimpleMotionFilter::boxUpdate(kuri_mbzirc_challenge_1_marker_tracking::TrackerData& data, float step)
{
  prev = current;
  current = data;
  timestep = step;
}

kuri_mbzirc_challenge_1_marker_tracking::TrackerData SimpleMotionFilter::interpolate(float step)
{
  float minXSpeed = calcSpeed(prev.minX, current.minX, timestep);
  float minYSpeed = calcSpeed(prev.minX, current.minX, timestep);
  float maxXSpeed = calcSpeed(prev.minX, current.minX, timestep);
  float maxYSpeed = calcSpeed(prev.minX, current.minX, timestep);
  
  kuri_mbzirc_challenge_1_marker_tracking::TrackerData newData;
  
  // confidence is calculated as 1/(1 + frames), where frames is (step / this->timestep)
  // i.e. for each "extra" frame this interpolates, the confidence drops by half
  newData.confidence = 1.0f / (1.0f + (step / timestep));
  newData.minX = calcNewPoint(current.minX, minXSpeed, step);
  newData.minY = calcNewPoint(current.minY, minYSpeed, step);
  newData.maxX = calcNewPoint(current.maxX, maxXSpeed, step);
  newData.maxY = calcNewPoint(current.maxY, maxYSpeed, step);
  
  return newData;  
}


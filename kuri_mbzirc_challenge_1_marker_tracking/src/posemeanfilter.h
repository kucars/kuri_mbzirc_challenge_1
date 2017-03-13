#ifndef POSEMEANFILTER_H
#define POSEMEANFILTER_H

#include <ros/ros.h>
#include <visp/vpHomogeneousMatrix.h>
#include <deque>

// Takes in a bunch of poses (as vpHomogeoneous Matrices) and is able to give you their average
class PoseMeanFilter
{
public:
  PoseMeanFilter();
  PoseMeanFilter(const PoseMeanFilter& copy);
  ~PoseMeanFilter();

  PoseMeanFilter(unsigned int numPoses);

  void cMoUpdate(vpHomogeneousMatrix& newCmo);
  vpHomogeneousMatrix getMeanCMo();
  double getStddev();
  double getVariance();

  void reset();
  void setMaxSize(unsigned int newSize);
  unsigned int getMaxSize();
  unsigned int getCurrentSize();

private:
  unsigned int maxSize; // maxPoses
  std::deque<vpHomogeneousMatrix> poses;

  vpHomogeneousMatrix averageMatrix; // mean only
  vpHomogeneousMatrix filteredMatrix; // outliers removed
  double stddev; // standard deviation
  double variance;
};

#endif
#include "posemeanfilter.h"
#include <assert.h>
#include <visp/vpQuaternionVector.h>
#include <cmath>

PoseMeanFilter::PoseMeanFilter()
{
  maxSize = 5; // hardcoded default.. had to put something...
  stddev = 0.0;
  variance = 0.0;
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
/*Taken from: https://stackoverflow.com/questions/21241965/average-transformation-matrix-for-a-list-of-transformations
 * start with q* as above
	do until convergence
	  for each input quaternion i (index)
		  diff = q[i] * inverse(q*)
		  u[i] = log(diff, base q*)
	  //Now perform the linear blend
	  adapt := zero quaternion
	  weights := 0
	  for each input quaternion i
		  adapt += weight[i] * u[i]
		  weights += weight[i]
	  adapt *= 1/weights
	  adaptInOriginalSpace = q* ^ adapt    (^ is the power operator)
	  q* = adaptInOriginalSpace * q*
	  */
static vpQuaternionVector calcSphericalAverage(std::vector<vpQuaternionVector>& quats)
{
  //TODO

}

/* Linear blend, taken from https://stackoverflow.com/questions/21241965/average-transformation-matrix-for-a-list-of-transformations */
/* q* = w1 * q1 + w2 * q2 + ... + w2 * qn
   normalize q*
 */
static vpQuaternionVector linearBlend(std::vector<vpQuaternionVector>& quats)
{
  if(quats.size() == 0)
	return vpQuaternionVector();

  if(quats.size() == 1)
	return quats[0];

  double w = 1.0 / ((double)quats.size()); // equal weights for all quaternions (for now...)
  vpQuaternionVector blended = quats[0];
  for(int i = 1; i < quats.size(); i++)
  {
	blended = blended + (quats[i] * w);
  }
  blended.normalize();

  return blended;
}

static vpHomogeneousMatrix calcNewMean(std::deque<vpHomogeneousMatrix>& poses)
{
  if(poses.size() == 0){
	return vpHomogeneousMatrix(); // identity matrix
  }else if(poses.size() == 1){
	return poses[0];
  }

  assert(poses.size() > 1);
  vpTranslationVector sumT;

  std::vector<vpQuaternionVector> quats;
  for(std::deque<vpHomogeneousMatrix>::iterator it = poses.begin(); it != poses.end(); ++it)
  {
	vpTranslationVector t;
	vpQuaternionVector r;
	it->extract(t);
	it->extract(r);
	sumT = sumT + t;
	quats.push_back(r);
  }
  vpQuaternionVector averageQuat = linearBlend(quats);

  sumT = sumT / (double)poses.size();

  vpHomogeneousMatrix average;
  average.buildFrom(sumT, averageQuat);
  return average;
}

static void calcNewStddev(std::deque<vpHomogeneousMatrix>& poses, vpHomogeneousMatrix& mean, double& stddev,
						  double& variance)
{
  // Standard deviation of translations is the average of the square of differences (magnitude of difference) from the mean translation
  // For rotations, it's a bit more tricky, so just ignore it since we don't care too much about it
  double sum = 0.0;
  vpTranslationVector meanTrans;
  mean.extract(meanTrans);
  for(std::deque<vpHomogeneousMatrix>::iterator it = poses.begin(); it != poses.end(); ++it)
  {
	vpTranslationVector poseTrans;
	it->extract(poseTrans);
	vpTranslationVector diff = poseTrans - mean;
	double mag = diff.euclideanNorm();
	sum += mag;
  }

  variance = sum / poses.size();
  stddev = sqrt(variance);
}

static vpHomogeneousMatrix calcFilteredMatrix(std::deque<vpHomogeneousMatrix>& poses, vpHomogeneousMatrix& mean,
											  double stddev, double threshold = 1.0)
{
  // only look at variation in translation, ignore rotations
  vpTranslationVector meanTrans;
  mean.extract(meanTrans);
  std::deque<vpHomogeneousMatrix> filteredPoses;
  for(std::deque<vpHomogeneousMatrix>::iterator it = poses.begin(); it != poses.end(); it++)
  {
	vpTranslationVector poseTrans;
	it->extract(poseTrans);
	vpTranslationVector diff = poseTrans - meanTrans;
	if(diff.euclideanNorm() <= stddev * threshold)
	{
	  filteredPoses.push_back(*it);
	}
  }

  if(filteredPoses.size() >= 2){
	return calcNewMean(filteredPoses);
  }

  ROS_WARN("filteredPoses is %d!", (int)filteredPoses.size());
  return mean;
}

void PoseMeanFilter::cMoUpdate(vpHomogeneousMatrix& newCmo)
{
  // push the new pose into the queue, pop the oldest pose if needed
  assert(poses.size() <= maxSize);
  if(poses.size() == maxSize)
  {
	poses.pop_front();
  }
  poses.push_back(newCmo);

  // calculate mean of all poses
  averageMatrix = calcNewMean(poses);

  // calculate new standard deviation
  calcNewStddev(poses, averageMatrix, stddev, variance);

  // calculate mean of poses that are within standard deviation
  filteredMatrix = calcFilteredMatrix(poses, averageMatrix, stddev, 3.0);
}

vpHomogeneousMatrix PoseMeanFilter::getMeanCMo()
{
  /*
  if(poses.size() == 0)
	return vpHomogeneousMatrix(); // identity matrix

  if(poses.size() == 1)
	return poses[0];

  vpTranslationVector sumT;

  std::vector<vpQuaternionVector> quats;
  for(std::deque<vpHomogeneousMatrix>::iterator it = poses.begin(); it != poses.end(); ++it)
  {
	vpTranslationVector t;
	vpQuaternionVector r;
	it->extract(t);
	it->extract(r);
	sumT = sumT + t;
	quats.push_back(r);
  }
  vpQuaternionVector averageQuat = linearBlend(quats);

  sumT = sumT / (double)poses.size();

  vpHomogeneousMatrix averageMatrix;
  averageMatrix.buildFrom(sumT, averageQuat);
*/
  // return averageMatrix;
  return filteredMatrix; // debug
}

double PoseMeanFilter::getStddev(){ return stddev; }

double PoseMeanFilter::getVariance(){ return variance; }

void PoseMeanFilter::reset()
{
  poses.clear();
  averageMatrix = vpHomogeneousMatrix();
  double stddev = 0.0;
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
#include "mbtracker.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <visp/vpRect.h>

#define CLOCK_TO_MSECS(x) ((((double)(x))/CLOCKS_PER_SEC) * 1000)

#define MAXLIMIT(x, y) (x = x > y ? y : x)
#define MINLIMIT(x, y) (x = x < y ? y : x)

TrackMarkerModel::~TrackMarkerModel()
{
  if(display != NULL)
	delete display;
  if(tracker != NULL)
	delete tracker;
  if(me != NULL)
	delete me; // Remove me from existence. Make me a nobody. I do not want to exist anymore.

  display = NULL;
  tracker = NULL;
  me = NULL;
}


TrackMarkerModel::TrackMarkerModel(int x, int y)
{
  I.init(y, x);
  
  // set parameters
  me = new vpMe(); //TODO harcoded parameters, should be ros params probably
  me->setMaskSize(5);
  me->setMaskNumber(180);
  me->setRange(8);
  me->setThreshold(10000);
  me->setMu1(0.5);
  me->setMu2(0.5);
  me->setSampleStep(4);

  setExtrapolateMaxTime(250.0); // 1 quarter of a second maximum
  lastTrackTime = 0;

  // initialize tracker
  tracker = new vpMbEdgeTracker();
  dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(*me);
  setCameraParameters(1241.897, 1242.5302, 607.84338, 510.870); // Values obtained after calibrating ueye camera
  try{
	std::string path = ros::package::getPath("kuri_mbzirc_challenge_1_marker_tracking");
	path.append("/config/marker.cao");
	ROS_INFO("Loading model from %s", path.c_str());
	tracker->loadModel(path.c_str()); //TODO hardcoded filename, should be a ros param, but this is a good compromise for now

	// get the main face from the 3D model and store it somewhere for later retrieval
	vpMbtPolygon * poly = tracker->getFaces().getPolygon()[0];
	for(int i = 0; i < 4; i++){
	  marker3DPoints.push_back(poly->getPoint(i));
	}

  }catch(vpException e)
  {
	ROS_FATAL("Cannot open model file, so the tracker cannot know what to track!");
	ROS_FATAL("Exception: %s", e.what());
	ROS_FATAL("Code: %d, String: %s", e.getCode(), e.getMessage());
	throw;
  }
  tracker->setDisplayFeatures(true);
  
  // set up display
  display = new vpDisplayX(I, 0, 0, "Detector and Model-based tracker output");
  
  // set states
  detectedState = false;
  trackingState = false;
  resultIsPredicted = false;
  displayEnabled = false;
}

static void getBoxPoints(vpHomogeneousMatrix& cmo, std::vector<vpPoint>& points3D,
						 std::vector<vpImagePoint>& outPoints2D, vpCameraParameters cam)
{
  for(int i = 0; i < points3D.size(); i++)
  {
	vpPoint P = points3D[i];
	P.project(cmo);
	vpImagePoint ip;
	vpMeterPixelConversion::convertPoint(cam, P.get_x(), P.get_y(), ip);
	outPoints2D.push_back(ip);
  }
}

void TrackMarkerModel::getROIFromBoxPoints(std::vector<vpImagePoint>& boxPoints,
												  sensor_msgs::RegionOfInterest& outRoi)
{
  vpRect rect = vpRect(boxPoints);

  // make sure the rect is within the image area
  int left = floor(rect.getLeft());
  int top = floor(rect.getTop());
  int width = floor(rect.getWidth());
  int height = floor(rect.getHeight());

  // sanitize numbers
  MINLIMIT(left, 0);
  MAXLIMIT(left, I.getWidth());
  MINLIMIT(top, 0);
  MAXLIMIT(left, I.getHeight());
  MINLIMIT(width, 0);
  MAXLIMIT(width, I.getWidth() - left);
  MINLIMIT(height, 0);
  MAXLIMIT(height, I.getHeight() - top);

  outRoi.x_offset = left;
  outRoi.y_offset = top;
  outRoi.width = width;
  outRoi.height = height;
  outRoi.do_rectify = true;
}

bool TrackMarkerModel::detectAndTrack(const sensor_msgs::Image::ConstPtr& msg)
{
  clock_t now = clock();
  I = visp_bridge::toVispImage(*msg);
  if(displayEnabled)
  {
	vpDisplay::display(I);
  }

  vpCameraParameters cam;
  tracker->getCameraParameters(cam);

  if(!detectedState)
	detectedState = detector.detect(msg);
  
  if(detectedState && !trackingState)
  {
	// get points from detector
	double detectorPoints[8];
	detector.getRectCoords(detectorPoints);
	std::vector<vpImagePoint> points2D;
	for(int i = 0; i < 4; i++){
	  points2D.push_back(vpImagePoint(detectorPoints[(i*2) + 1], detectorPoints[i*2]));
	  ROS_INFO("Detector point: y= %f x= %f", points2D[i].get_i(), points2D[i].get_j());
	}

	// initialize tracker
	tracker->initFromPoints(I, points2D, marker3DPoints);
	trackingState = true;
  }
  
  if(trackingState)
  {
	try{ 
	  // track the object
	  tracker->track(I);
	  
	  // get the pose
	  tracker->getPose(cMo);
	  tracker->display(I, cMo, cam, vpColor::red, 2, true);	
	  
	  vpMbHiddenFaces<vpMbtPolygon> &faces = tracker->getFaces();
//	  std::cout << "Number of faces: " << faces.size() << std::endl;

	  // below code taken from VISP tutorial, used for reference
	  /*
	  for(int i = 0; i < faces.size(); i++){
		std::vector<vpMbtPolygon *> &poly = faces.getPolygon();
		std::cout << "face " << i << " with index: " << poly[i]->getIndex()
		<< " is " << (poly[i]->isVisible() ? "visible" : "not visible")
		<< " and has " << poly[i]->getNbPoint() << " points"
		<< " and LOD is " << (poly[i]->useLod ? "enabled" : "disabled") << std::endl;

		for(int j = 0; j < poly[i]->getNbPoint(); j++) {
		  vpPoint P = poly[i]->getPoint(j);
		  P.project(cMo);
		  std::cout << " P obj " << j << ": " << P.get_oX() << " " << P.get_oY() << " " << P.get_oZ() << std::endl;
		  std::cout << " P cam " << j << ": " << P.get_X() << " " << P.get_Y() << " " << P.get_Z() << std::endl;

		  vpImagePoint iP;
		  vpMeterPixelConversion::convertPoint(cam, P.get_x(), P.get_y(), iP);
		  std::cout << " iP " << j << ": " << iP.get_u() << " " << iP.get_v() << std::endl;
		}
	  }
	  */

	  if(faces.size() >= 1){
		// first face is the square, get the 4 points and get pixel coordinates
		std::vector<vpMbtPolygon *> &poly = faces.getPolygon();
		if(poly[0]->getNbPoint() == 5){
		  // This is the face we want (yes it has 5 points, but the last one is the same as the first)
		  std::vector<vpImagePoint> boxpoints;
		  getBoxPoints(cMo, marker3DPoints, boxpoints, cam);

		  getROIFromBoxPoints(boxpoints, roi);

		  vpTranslationVector trans;
		  vpQuaternionVector q;
		  cMo.extract(trans);
		  cMo.extract(q);

		  //ROS_INFO("Translation %f %f %f", trans[0], trans[1], trans[2]);
		  //ROS_INFO("Quats: %f %f %f %f", q[0], q[1], q[2], q[3]);
		  resultIsPredicted = false;
		  pose.header = msg->header;
		  pose.pose.position.x = trans[0] * 0.01; // convert to meters
		  pose.pose.position.y = trans[1] * 0.01;
		  pose.pose.position.z = trans[2] * 0.01;
		  pose.pose.orientation.x = q.x();
		  pose.pose.orientation.y = q.y();
		  pose.pose.orientation.z = q.z();
		  pose.pose.orientation.w = q.w();

		  // update the extrapolator
		  extrapolator.poseUpdate(pose.pose, CLOCK_TO_MSECS(now-lastTrackTime));
		  lastTrackTime = now;

		}else{
		  ROS_WARN("We're tracking the model, but we can't find a face with 5 points.. this shouldn't happen..");
		}
     }else{
	   ROS_WARN("We're tracking the model, but couldn't find any faces. This shuoldn't happen!");
	 }
	}catch(vpException e){
	  ROS_WARN("An exception occurred while tracking...");
	  ROS_WARN("vpException: %s", e.getMessage());
	  detectedState = false;
	  trackingState = false;
  	  resultIsPredicted = true;
	}
  }else{
	// We can't track, but it's been a short time since the last time to track.. so...
	// Extrapolate the last pose if it's not too far off from the last update
	if(CLOCK_TO_MSECS(now-lastTrackTime) < extrapolateMaxTime)
	{
	  if(!resultIsPredicted) // print some text if first time we extrapolate
		ROS_INFO("Tracking lost, extrapolating from last 2 pose positions for %f seconds", extrapolateMaxTime);

	  pose.header = msg->header;
	  pose.pose = extrapolator.extrapolate(CLOCK_TO_MSECS(now-lastTrackTime));

	  ROS_INFO("Extrapolated Pose XYZ: (%f, %f, %f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

	  // calculate ROI from extrapolated pose
	  // create a cMo matrix from the extrapolated pose
	  vpTranslationVector trans(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
	  vpQuaternionVector quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
	  cMo.buildFrom(trans, quat);

	  // use it to get 4 points in the image and update the ROI
	  std::vector<vpImagePoint> boxpoints;
	  getBoxPoints(cMo, marker3DPoints, boxpoints, cam);
	  getROIFromBoxPoints(boxpoints, roi);

	}else{
	  // we're not tracking and it's been a long time since we tracked.
	  // So stop extrapolating and set everything to 0
	  roi.x_offset = 0;
	  roi.y_offset = 0;
	  roi.width = 0;
	  roi.height = 0;
	  roi.do_rectify = false;

	  pose.pose.position.x = 0 ;
	  pose.pose.position.y = 0;
	  pose.pose.position.z = 0;
	  pose.pose.orientation.x = 0;
	  pose.pose.orientation.y = 0;
	  pose.pose.orientation.z = 0;
	  pose.pose.orientation.w = 0;

	  reset();
	}
  }

  // draw the bounding box (blue if predicted)
  if(trackingState || resultIsPredicted)
  {
	const vpColor& c = resultIsPredicted ? vpColor::blue : vpColor::green;
	vpDisplay::displayRectangle(I, vpImagePoint(roi.y_offset, roi.x_offset), roi.width, roi.height, c,
	  false, 3);
  }

  if(displayEnabled)
	vpDisplay::flush(I);
  
  return trackingState;
}

void TrackMarkerModel::reset()
{
//  tracker->resetTracker(); // (seems to delete the loaded model, so don't do it..)
  detectedState = false;
  trackingState = false;
  resultIsPredicted = false;
}

void TrackMarkerModel::setCameraParameters(double px, double py, double u0, double v0)
{
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(px, py, u0, v0);
  tracker->setCameraParameters(cam);
}


bool TrackMarkerModel::markDetected(){ return detectedState; }

bool TrackMarkerModel::isTracking() { return trackingState; }

sensor_msgs::RegionOfInterest TrackMarkerModel::getRegionOfInterest(){ return roi; }

geometry_msgs::PoseStamped TrackMarkerModel::getPoseStamped(){ return pose; }

double TrackMarkerModel::getExtrapolateMaxTime(){ return extrapolateMaxTime; }

clock_t TrackMarkerModel::getLastTrackTime(){ return lastTrackTime; }

bool TrackMarkerModel::isResultPredicted(){ return resultIsPredicted; }

void TrackMarkerModel::enableTrackerDisplay(bool enabled){ displayEnabled = enabled; }

void TrackMarkerModel::setMaskSize(int size){
  me->setMaskSize(size);
  dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(*me);
}

void TrackMarkerModel::setMaskNumber(int num){
  me->setMaskNumber(num);
  dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(*me);
}

void TrackMarkerModel::setRange(int range){
  me->setRange(range);
  dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(*me);
}

void TrackMarkerModel::setThreshold(int threshold){
  me->setThreshold(threshold);
  dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(*me);
}

void TrackMarkerModel::setMu1(double mu1){
  me->setMu1(mu1);
  dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(*me);
}

void TrackMarkerModel::setMu2(double mu2){
  me->setMu2(mu2);
  dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(*me);
}

void TrackMarkerModel::setSampleStep(int samplestep){
  me->setSampleStep(samplestep);
  dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(*me);
}

void TrackMarkerModel::setExtrapolateMaxTime(double newTime)
{
  if(newTime > 0.0 && newTime == newTime)
	extrapolateMaxTime = newTime;
}


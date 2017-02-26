#include "mbtracker.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <visp/vpRect.h>

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
  
  // initialize tracker
  tracker = new vpMbEdgeTracker();
  dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(*me);
  setCameraParameters(1241.897, 1242.5302, 607.84338, 510.870); // Values obtained after calibrating ueye camera
  try{
	std::string path = ros::package::getPath("kuri_mbzirc_challenge_1_marker_tracking");
	path.append("/config/marker.cao");
	ROS_INFO("Loading model from %s", path.c_str());
	tracker->loadModel(path.c_str()); //TODO hardcoded filename, should be a ros param, but this is a good compromise for now

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
  displayEnabled = false;
}

bool TrackMarkerModel::detectAndTrack(const sensor_msgs::Image::ConstPtr& msg)
{
  I = visp_bridge::toVispImage(*msg);
  if(displayEnabled)
  {
	vpDisplay::display(I);
  }
  
  if(!detectedState)
	detectedState = detector.detect(msg);
  
  if(detectedState && !trackingState)
  {
	// initialize the tracker
	// I have to hardcode the 3D points in. I'm sure there's a better way since I already defined these 
	// in the .CAO file...
	std::vector<vpPoint> points3D(4);
	points3D[0] = vpPoint(-75, 75, 0);
	points3D[1] = vpPoint(-75, -75, 0);
	points3D[2] = vpPoint(75, -75, 0);
	points3D[3] = vpPoint(75, 75, 0);
	
	// get points from detector
	double detectorPoints[8];
	detector.getRectCoords(detectorPoints);
	std::vector<vpImagePoint> points2D(4);
	for(int i = 0; i < 4; i++){
	  points2D[i] = vpImagePoint(detectorPoints[(i*2) + 1], detectorPoints[i*2]);
	  ROS_INFO("Detector point: y= %f x= %f", points2D[i].get_i(), points2D[i].get_j());
	}

	// initialize tracker
	tracker->initFromPoints(I, points2D, points3D);
	trackingState = true;
  }
  
  if(trackingState)
  {
	try{ 
	  vpCameraParameters cam;
	  
	  // track the object
	  tracker->track(I);
	  
	  // get the pose
	  tracker->getPose(cMo);
	  tracker->getCameraParameters(cam);
	  tracker->display(I, cMo, cam, vpColor::red, 2, true);	
	  
	  //TODO do more things.. publish to topic...
	  vpMbHiddenFaces<vpMbtPolygon> &faces = tracker->getFaces();
//	  std::cout << "Number of faces: " << faces.size() << std::endl;

	  // below code taken from VISP tutorial, used for reference
/*	  for(int i = 0; i < faces.size(); i++){
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
		*/

	  if(faces.size() >= 1){
		// first face is the square, get the 4 points and get pixel coordinates
		std::vector<vpMbtPolygon *> &poly = faces.getPolygon();
		if(poly[0]->getNbPoint() == 5){
			// This is the face we want (yes it has 5 points, but the last one is the same as the first)
		  std::vector<vpImagePoint> boxpoints(4);
		  for(int i = 0; i < 4; i++)
		  {
			vpPoint P = poly[0]->getPoint(i);
			P.project(cMo);
			vpImagePoint ip;
			vpMeterPixelConversion::convertPoint(cam, P.get_x(), P.get_y(), ip);
			boxpoints[i] = ip;
		  }
		  vpRect rect = vpRect(boxpoints);
		  roi.x_offset = rect.getLeft();
		  roi.y_offset = rect.getTop();
		  roi.width = rect.getWidth();
		  roi.height = rect.getHeight();
		  roi.do_rectify = true;
		}else{
		  ROS_WARN("We're tracking the model, but we can't find a face with 5 points.. this shouldn't happen..");
		}
	 }else{
	   ROS_WARN("We're tracking the model, but couldn't find any faces. This shuoldn't happen!");
	 }
	}catch(vpException e){
	  ROS_WARN("An exception occurred while tracking...");
	  ROS_WARN("vpException: %s", e.getMessage());
	  reset();
	}
  }

  // draw the bounding box (debug)
  if(trackingState)
  {
	vpDisplay::displayRectangle(I, vpImagePoint(roi.y_offset, roi.x_offset), roi.width, roi.height, vpColor::green,
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
}

void TrackMarkerModel::setCameraParameters(double px, double py, double u0, double v0)
{
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(px, py, u0, v0);
  tracker->setCameraParameters(cam);
}


bool TrackMarkerModel::markDetected(){ return detectedState; }

bool TrackMarkerModel::isTracking() { return trackingState; }

sensor_msgs::RegionOfInterest TrackMarkerModel::getRegionOfInterest()
{
  return roi;
}

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


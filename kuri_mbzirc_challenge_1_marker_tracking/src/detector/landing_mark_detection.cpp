#include "landing_mark_detection.h"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <typeinfo>

using namespace cv;
using namespace std;

void DetectLandingMark::getRectCoords(double out[8])
{
	for(int i = 0; i < 8; i++)
		out[i] = rectCoords[i];
}

bool DetectLandingMark::detect(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return false;
  }
  
  return detect(cv_ptr->image, 0);
}

bool DetectLandingMark::detect(Mat frame, int show_result=0) {
    Mat edged;
    Mat gray;
    Mat roi_square;
    Mat gray_threshold;
    string status;
    int status_flag = 0;
    int minimum_marker_wh = 50;
		
	//convert the frame to grayscale, blur it, and detect edges
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	GaussianBlur(gray, gray, Size(7,7), 0);
	//sharpening the frame
	/*
    Mat sharpening_kernel = (Mat_<char>(5, 5) << -1, -1, -1, -1, -1,
                                -1, 2, 2, 2, -1,
                                -1, 2, 8, 2, -1,
                                -1, 2, 2, 2, -1,
                                -1, -1, -1, -1, -1) / 8.0;
    Mat sharpened_gray;
    filter2D(gray, sharpened_gray, gray.depth(), sharpening_kernel);
	*/
	//adaptive thresholding for canny
	double CannyAdaptThresh = threshold(gray,gray_threshold,0,255,CV_THRESH_BINARY|CV_THRESH_OTSU);
	double CannyMinThresh = 0.1 * CannyAdaptThresh;
	Canny(gray, edged, CannyMinThresh, CannyAdaptThresh);	   
	//find contours in the edge map
	vector<vector<Point> > contours;
	vector<Point> approx;
	vector<Vec4i> hierarchy;
	vector<Point> hull;
	findContours(edged, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	
	//loop over the contours
	for( int i = 0; i< contours.size(); i++ ){
		double peri = arcLength(contours[i], true);
		approxPolyDP(contours[i], approx, 0.01 * peri, true);
		//ensure that the approximated contour is "roughly" rectangular
		if(approx.size()>=4 && approx.size()<=6){	
			
			for(int k=0; k<approx.size(); k++){
				ROS_INFO("k=%d -----> x=%d, y=%d",k,approx[k].x,approx[k].y);
			}
			
			Rect rect = boundingRect(approx);
			float aspectRatio = rect.width / float(rect.height);
			
			//compute the solidity of the original contour
			double area = contourArea(contours[i]);
			convexHull(contours[i], hull);
			double hullArea = contourArea(hull);
			double solidity = area / double(hullArea);
			
			//compute whether or not the width and height, solidity, and aspect ratio of the contour falls within appropriate bounds
			bool keepDims = (rect.width > minimum_marker_wh and rect.height > minimum_marker_wh)?true:false;
			bool keepSolidity = (solidity > 0.9)?true:false;
			bool keepAspectRatio = (aspectRatio >= 0.8 and aspectRatio <= 1.2)?true:false;
			
			//ensure that the contour passes all our tests
			if (keepDims and keepSolidity and keepAspectRatio){
				//get ROI (inside the contour)
				roi_square = gray(rect);
				//detect circles in the image
				vector<Vec3f> circles; 
				HoughCircles(roi_square, circles, CV_HOUGH_GRADIENT, 1.2, 75); //circles = (x,y,radius)
				//if more than one circle is detected, select the one with largest radius
				if(circles.size() >= 1){
					//select circle with largest radius
					int max_r_index = 0;
					for(int m=0; m<circles.size(); m++){
						if(circles[m][2]>circles[max_r_index][2])
							max_r_index = m;
					}				
					//take the average of both shapes' centers and display it as the center of the landing mark
					double average_x = ((int(circles[max_r_index][0])+rect.x)+(rect.x+rect.width/2.0))/2.0;
					double average_y = ((int(circles[max_r_index][1])+rect.y)+(rect.y+rect.height/2.0))/2.0;
					//if the two centers are very close, then accept it
					if(average_x-(int(circles[max_r_index][0])+rect.x)<=20){
						status = "Target Detected";
						status_flag = 1;
						//start drawing if requested
						if(show_result){
							drawContours(frame, contours, i, (0, 0, 255), 4);
							circle(frame, Point(int(circles[max_r_index][0])+rect.x, int(circles[max_r_index][1])+rect.y), circles[max_r_index][2], Scalar(0, 255, 0), 4);
							rectangle(frame, Point(average_x - 5, average_y - 5), Point(average_x + 5, average_y + 5), Scalar(0, 128, 255), -1);
							putText(frame, status, Point(20, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100, 120, 0), 2);
						}
						//assign values to the landing_mark struct
						a.x = rect.x;
						a.y = rect.y;
						a.width = rect.width;
						a.height = rect.height;
						a.center_x = average_x;
						a.center_y = average_y;
						//also add rectangle coordinates to the marker_pixel_coord struct
						for(int k=0; k<approx.size(); k++){
							rectCoords[k*2] = approx[k].x;
							rectCoords[k*2 + 1] = approx[k].y;
						}
					}
				}
			}
		}
	}
	if(show_result){    
		imshow("Landing Mark Detection", frame);
		waitKey(0);
	}
	return((status_flag==1)?true:false);
}

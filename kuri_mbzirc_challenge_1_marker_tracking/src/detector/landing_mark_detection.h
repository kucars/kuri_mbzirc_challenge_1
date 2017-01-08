#ifndef LANDING_MARK_DETECTION_H
#define LANDING_MARK_DETECTION_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <cstring>
#include <stdio.h>

#include <sensor_msgs/Image.h>
#include <visp/vpImageIo.h>

using namespace cv;
using namespace std;

struct landing_mark{
	double x, y;				//top-left (x,y) coordinates of the bounding rectangle
	double width, height, center_x, center_y;			//width, height of the rectangle, and the center of the landing mark
};

class DetectLandingMark {
	public:
		bool detect(const sensor_msgs::Image::ConstPtr& msg); // adapter function
		bool detect(Mat, int);
		landing_mark get_landing_mark() {
			return a;
		}
		DetectLandingMark(void){
			a.x = -1;
			a.y = -1;
			a.width = -1;
			a.height = -1;
			a.center_x = -1;
			a.center_y = -1;
		}
	private:
		landing_mark a;
};

#endif
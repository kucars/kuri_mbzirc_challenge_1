/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *     Reem Ashour  <reem.ashour@kustar.ac.ae>                             *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/

#include <PositionEstimation.h>
#include <string>


bool flag_done = false ;

namespace enc = sensor_msgs::image_encodings;
double roll, pitch, yaw;
float MarkerX, MarkerY;


kuri_mbzirc_challenge_1_msgs::PoseEsti PositionEstimation::waitforResults (kuri_mbzirc_challenge_1_msgs::PoseEsti goal)

{
    std::cout << "goal x " <<  goal.x  << std::endl << std::flush ;

    this->goalCam.x = goal.x ;
    this->goalCam.y = goal.y ;
    img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/downward_cam/camera/image", 10);
    camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "/downward_cam/camera/camera_info", 10);
    uav_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/mavros/local_position/odom", 10);

    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(3), *img_sub_, *camera_info_sub_, *uav_odom_sub_);
    sync->registerCallback(boost::bind(&PositionEstimation::imageCallback, this, _1, _2, _3));

    //initialize base_link to camera optical link
    //0.0 0.0 -0.045
    BaseToCamera.setOrigin(tf::Vector3(0.0, 0.0, -0.045l));
    BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000));
    std::cout << " Start the action server " << std::endl << std::flush ;
    #define Ground_Z 0.0
    //test
    std::cout <<"flag value " <<flag_done  << std::endl << std::flush ;
     ros::Rate loopRate(10);
     while (ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
        if (flag_done)
        {
            std::cout << " return in World frame" << std::endl << std::flush ;

            return this->goalWorld ;
        }
        else
        {
            //std::cout << "Continue" << std::endl << std::flush ;

            continue ;
        }
    }
    // tf::TransformBroadcaster br;
    // ros::Rate loopRate(10);


}



PositionEstimation::PositionEstimation ()
{
    std::cout << "constructor " << std::endl << std::flush ;

}


//call back, for processing
void PositionEstimation::imageCallback(const sensor_msgs::ImageConstPtr& img,
                                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                                       const nav_msgs::OdometryConstPtr& odom
                                       )
{
    std::cout << "Image Call Back" << std::endl << std::flush ;

    // make sure that the action hasn't been canceled
    // if (!actionServer.isActive())
    //    return;

    std::cout << "Server Active" << std::endl << std::flush ;

    tf::Transform extrisic;
    cv::Mat P(3, 4, CV_64FC1);
    cv::Mat P_Mat_G(3, 4, CV_64FC1);
    tf::Pose tfpose;
    tfScalar extrisic_data[4 * 4];
    pcl::PointCloud<pcl::PointXYZRGB> Pointcloud;
    Pointcloud.header.frame_id = "/world";
    Pointcloud.height = img->height;
    Pointcloud.width = img->width;
    Pointcloud.resize(img->height * img->width);
    Pointcloud.is_dense = true;
    cv::Mat cvimg = cv_bridge::toCvShare(img, "bgr8")->image.clone();
    tf::poseMsgToTF(odom->pose.pose, tfpose);
    extrisic = BaseToCamera * tfpose.inverse();
    //to test if the tf is correct, create testframe_to_camera
    //br.sendTransform(tf::StampedTransform(extrisic, ros::Time::now(), "/testframe_to_camera", "/world"));
    //pinv of projection matrix...
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
        {
            P.at<double>(i, j) = cam_info->P.at(i * 4 + j);
            //  std::cout << "PP" << P  << std::endl ;
        }
    //however, this P is in camera coordinate..
    extrisic.getOpenGLMatrix(extrisic_data);
    cv::Mat E_MAT(4, 4, CV_64FC1, extrisic_data);
    P_Mat_G = P * (E_MAT.t());
    // now is the ground, namely, world coordinate
    double a[4], b[4], c[4];
    a[0] = P_Mat_G.at<double>(0, 0);
    a[1] = P_Mat_G.at<double>(0, 1);
    a[2] = P_Mat_G.at<double>(0, 2);
    a[3] = P_Mat_G.at<double>(0, 3);
    b[0] = P_Mat_G.at<double>(1, 0);
    b[1] = P_Mat_G.at<double>(1, 1);
    b[2] = P_Mat_G.at<double>(1, 2);
    b[3] = P_Mat_G.at<double>(1, 3);
    c[0] = P_Mat_G.at<double>(2, 0);
    c[1] = P_Mat_G.at<double>(2, 1);
    c[2] = P_Mat_G.at<double>(2, 2);
    c[3] = P_Mat_G.at<double>(2, 3);
    std::clock_t start;
    double duration;
    start = std::clock();

    // ************************** find 3D point ******************** //

    // just for the detected point
    float B[2][2], bvu[2];
    B[0][0] = this->goalCam.x  * c[0] - a[0];
    B[0][1] = this->goalCam.x  * c[1] - a[1];
    B[1][0] = this->goalCam.y  * c[0] - b[0];
    B[1][1] = this->goalCam.y  * c[1] - b[1];
    bvu[0] = a[2] * Ground_Z + a[3] - this->goalCam.x  * c[2] * Ground_Z - this->goalCam.x  * c[3];
    bvu[1] = b[2] * Ground_Z + b[3] - this->goalCam.y * c[2] * Ground_Z - this->goalCam.y * c[3];
    float DomB = B[1][1] * B[0][0] - B[0][1] * B[1][0];
    MarkerX = (B[1][1] * bvu[0] - B[0][1] * bvu[1]) / DomB ;
    MarkerY = (B[0][0] * bvu[1] - B[1][0] * bvu[0]) / DomB ;
    this->goalWorld.x = MarkerX ;
    this->goalWorld.y = MarkerX ;
    flag_done = true ;


}


PositionEstimation::~PositionEstimation()
{
}

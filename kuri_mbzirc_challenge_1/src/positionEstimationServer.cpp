/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Reem Ashour, KURI  <reem.ashour@kustar.ac.ae>                      *

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

#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen_conversions/eigen_msg.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>


#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/CameraInfo.h>
//msg headers.
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <iostream>

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <ctime>


//msg headers.
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <iostream>


#include "kuri_mbzirc_challenge_1_msgs/PoseEsti.h"
#include "kuri_mbzirc_challenge_1_msgs/PES.h"
#include <kuri_msgs/Object.h>


class PositionEstimationServer
{
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry> MySyncPolicy;

protected:

    kuri_mbzirc_challenge_1_msgs::PoseEsti goalCam;
    kuri_msgs::Object goalW;
    message_filters::Subscriber<sensor_msgs::Image> *img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
    message_filters::Synchronizer<MySyncPolicy> *sync;
    bool flag_done ;
    tf::Transform BaseToCamera;

    #define Ground_Z 0.0
    //test
    tf::TransformBroadcaster br;


public:
    ros::NodeHandle nh_;
    PositionEstimationServer(std::string name)
    {
        std::cout << "Constructor 1 " << std::endl  << std::flush ;

        this->flag_done = false;
        /*
        img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/downward_cam/camera/image", 10);
        camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "/downward_cam/camera/camera_info", 10);
        uav_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/mavros/local_position/odom", 10);
        std::cout << "decleared Subscriber " << std::endl  << std::flush ;

        std::cout << "started the service" << std::endl  << std::flush ;

        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(1000), *img_sub_, *camera_info_sub_, *uav_odom_sub_);*/
        //sync->registerCallback(boost::bind(&PositionEstimationServer::imageCallback, this, _1, _2, _3));
        BaseToCamera.setOrigin(tf::Vector3(0.0, 0.0, -0.045l));
        BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000));


#define Ground_Z 0.0

        // tf::TransformBroadcaster br;
        // ros::Rate loopRate(10);

        ros::ServiceServer service = nh_.advertiseService("poseEstimationServer",&PositionEstimationServer::poseEstimateFunction, this);

    }


    ~PositionEstimationServer(void)
    {
    }




    bool poseEstimateFunction(kuri_mbzirc_challenge_1_msgs::PES::Request  &req,
                                kuri_mbzirc_challenge_1_msgs::PES::Response &res)
    {
        std::cout << "waiting for goal" << std::endl  << std::flush ;

        //test
        std::cout <<"flag value " <<flag_done  << std::endl << std::flush ;
        ros::Rate loopRate(10);
        goalCam.x = req.uv.x ;
        goalCam.y = req.uv.y ;
        while (ros::ok()) {
            ros::spinOnce();
            loopRate.sleep();
            if (this->flag_done)
            {
                std::cout << " return in World frame" << std::endl << std::flush ;
                res.obj = goalW;
                return true ;
            }
            else
            {
                //std::cout << "Continue" << std::endl << std::flush ;
              return false;
                continue ;
            }

        }
    }


    //call back, for processing
    void imageCallback(const sensor_msgs::ImageConstPtr& img,
                                           const sensor_msgs::CameraInfoConstPtr& cam_info,
                                           const nav_msgs::OdometryConstPtr& odom
                                           )
    {

        std::cout << "Image Call Back" << std::endl << std::flush ;

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
        this->goalW.pose.pose.position.x  = (B[1][1] * bvu[0] - B[0][1] * bvu[1]) / DomB ;
        this->goalW.pose.pose.position.y= (B[0][0] * bvu[1] - B[1][0] * bvu[0]) / DomB ;
        flag_done = true ;


    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "poseEstimationServer");
    PositionEstimationServer poseEstimatServerObject(ros::this_node::getName());
    ros::spin();
    return 0;
}

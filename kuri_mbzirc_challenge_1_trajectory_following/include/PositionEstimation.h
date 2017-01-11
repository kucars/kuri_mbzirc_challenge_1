#ifndef POSITIONESTIMATION_H
#define POSITIONESTIMATION_H


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
#include "geometry_msgs/PoseStamped.h"
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <actionlib/server/simple_action_server.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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
#include "kuri_mbzirc_challenge_1_msgs/PoseEsti.h"
#include "kuri_mbzirc_challenge_1_msgs/poseEstimationAction.h"

class PositionEstimation
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry> MySyncPolicy;

public:
    void imageCallback(const sensor_msgs::ImageConstPtr& img,
                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                       const nav_msgs::OdometryConstPtr& odom
                       ) ;


   kuri_mbzirc_challenge_1_msgs::PoseEsti waitforResults(kuri_mbzirc_challenge_1_msgs::PoseEsti t);
    PositionEstimation() ;
    ~PositionEstimation();

 
protected:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> *img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
    message_filters::Synchronizer<MySyncPolicy> *sync;
    kuri_mbzirc_challenge_1_msgs::PoseEsti goalCam;
    kuri_mbzirc_challenge_1_msgs::PoseEsti goalWorld;
    tf::Transform BaseToCamera;

  
};

#endif // POSITIONESTIMATION_H

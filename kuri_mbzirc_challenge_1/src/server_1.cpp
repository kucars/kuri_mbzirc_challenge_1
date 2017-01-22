#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>

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


#include "kuri_mbzirc_challenge_1_msgs/PES.h"
#include <kuri_msgs/Object.h>
tf::Pose tfpose;
sensor_msgs::CameraInfo cam_info  ;
#define Ground_Z 0.0
cv::Mat P(3, 4, CV_64FC1);


/*void callback (const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    tf::poseMsgToTF(odom->pose.pose, tfpose);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
        {
            P.at<double>(i, j) = cam_info->P.at(i * 4 + j);
            //  std::cout << "PP" << P  << std::endl ;
        }

}*/

void odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
    tf::poseMsgToTF(odom->pose.pose, tfpose);
}


void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
        {
            P.at<double>(i, j) = cam_info->P.at(i * 4 + j);
            //  std::cout << "PP" << P  << std::endl ;
        }
}


bool poseEstimationFunction(kuri_mbzirc_challenge_1_msgs::PES::Request  &req,
                            kuri_mbzirc_challenge_1_msgs::PES::Response &res)
{
    tf::Transform BaseToCamera;
    double imageWidth = 640 ;
    double imagehight = 480 ;
    BaseToCamera.setOrigin(tf::Vector3(0.0, 0.0, -0.045));
    BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000));

    tf::Transform extrisic;
    cv::Mat P_Mat_G(3, 4, CV_64FC1);
    //cv::Mat P(3, 4, CV_64FC1);
    tfScalar extrisic_data[4 * 4];
    pcl::PointCloud<pcl::PointXYZRGB> Pointcloud;
    Pointcloud.header.frame_id = "/world";
    Pointcloud.height = imagehight;
    Pointcloud.width = imageWidth;
    Pointcloud.resize(imagehight * imageWidth);
    Pointcloud.is_dense = true;
    // cv::Mat cvimg = cv_bridge::toCvShare(img, "bgr8")->image.clone();
    //tf::poseMsgToTF(odom->pose.pose, tfpose);
    extrisic = BaseToCamera * tfpose.inverse();
    //to test if the tf is correct, create testframe_to_camera
    //br.sendTransform(tf::StampedTransform(extrisic, ros::Time::now(), "/testframe_to_camera", "/world"));
    //pinv of projection matrix...
    /*for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
        {
            P.at<double>(i, j) = cam_info.P.at(i * 4 + j);
            //  std::cout << "PP" << P  << std::endl ;
        }*/
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
    //double duration;
    start = std::clock();

    // ************************** find 3D point ******************** //

    // just for the detected point
    float B[2][2], bvu[2];
    B[0][0] = req.A  * c[0] - a[0];
    B[0][1] = req.A  * c[1] - a[1];
    B[1][0] = req.B  * c[0] - b[0];
    B[1][1] = req.B  * c[1] - b[1];
    bvu[0] = a[2] * Ground_Z + a[3] - req.A  * c[2] * Ground_Z - req.A  * c[3];
    bvu[1] = b[2] * Ground_Z + b[3] - req.B  * c[2] * Ground_Z - req.B  * c[3];
    float DomB = B[1][1] * B[0][0] - B[0][1] * B[1][0];
    res.obj.pose.pose.position.x  = (B[1][1] * bvu[0] - B[0][1] * bvu[1]) / DomB ;
    res.obj.pose.pose.position.y  = (B[0][0] * bvu[1] - B[1][0] * bvu[0]) / DomB ;
    //res.obj.pose.pose.position.z = 0 ;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.A, (long int)req.B);
    ROS_INFO("sending back response: [%ld]", (long int)res.obj.pose.pose.position.x);
    return true;
}
using namespace message_filters;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_estimation_server");
    ros::NodeHandle n;
    //message_filters::Subscriber<nav_msgs::Odometry> pose_sub(n, "/mavros/local_position/odom", 1);
    //message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n, "/downward_cam/camera/camera_info", 1);
    //TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::CameraInfo> sync(pose_sub, info_sub, 10);
    //sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Subscriber odom_sub = n.subscribe("/mavros/local_position/odom", 1000, odomCallback);
    ros::Subscriber cam_info_sub = n.subscribe("/downward_cam/camera/camera_info", 1000, camInfoCallback);
    ros::ServiceServer service = n.advertiseService("position_estimation", poseEstimationFunction);
    ROS_INFO("Ready to convert.");
    ros::spin();

    return 0;
}

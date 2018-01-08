/**
* Author: Tim Resink
* Email: timresink@gmail.com


TODOs:
- Add settings to settings file
    -     cameraTopic
    -     tfTopic



*/

// STD
#include <iostream>
//#include <string>

// ROS
#include<ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

// TF
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// cv bridge
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

//Eigen
#include <Eigen/Geometry>

// ORB-SLAM
#include "System.h"


class SubscribeHandler{
public:
	// constructors
    SubscribeHandler(const string &strVocFile,
                     const string &strSettingsFile, ros::NodeHandle *pNodeHandler,
                     tf::TransformListener *pTFlistener, tf::TransformBroadcaster *pTFbroadcaster);

    // variables
    tf::StampedTransform tfT_w_c;
    cv::Mat cvT_w_c;
    cv_bridge::CvImageConstPtr cv_ptr;
    cv::Mat Tcw;

    // methods
	void Shutdown();

private:
    // methods
    void GrabImage(const sensor_msgs::ImageConstPtr &msg);
    cv::Mat tfToMat(const tf::StampedTransform& tfT);
    Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    std::vector<float> toQuaternion(const Eigen::Matrix<double, 3, 3> &M);
    std::vector<float> Normalize(std::vector<float> vect);
    void Publish_Orientation(cv::Mat Tcw, tf::StampedTransform tfT_w_c);

    // ROS
    ros::Subscriber subImage;
    ros::Publisher maqui_orientation;
    ros::NodeHandle* mpNodeHandler;
    tf::TransformListener* mpTFlistener;
    tf::TransformBroadcaster* mpTFbroadcaster;

    // ORB SLAM pointer
    ORB_SLAM2::System* mpSLAM;

    //topics
    std::string cameraTopic;
    std::string tfTopic;
    std::string cameraFrameTopic;
    std::string worldFrameTopic;
    std::string broadCastTopic;

    // flags
    bool mbReferenceWorldFrame;
};

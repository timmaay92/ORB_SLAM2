/**
* This file is part of the odometry integration of pepper in SLAM
* Author: Tim Resink
* Email: timresink@gmail.com
*/

#include"SubscribeHandler.hpp"
using namespace Eigen;



SubscribeHandler::SubscribeHandler(const string &strVocFile,
                                   const string &strSettingsFile, ros::NodeHandle* pNodeHandler,
                                   tf::TransformListener* pTFlistener, tf::TransformBroadcaster* pTFbroadcaster):
mpNodeHandler(pNodeHandler),
mpTFlistener(pTFlistener),
mpTFbroadcaster(pTFbroadcaster),
mbReferenceWorldFrame(false)
{
      cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
      cameraTopic = (std::string) fsSettings["Topic.Camera"];
      cameraFrameTopic = (std::string) fsSettings["Topic.CameraFrame"];
      worldFrameTopic = (std::string) fsSettings["Topic.WorldFrame"];
      tfTopic =(std::string) fsSettings["Topic.TF"];
      int queueSize = (int) fsSettings["Topic.QueueSize"];
      baseFrameTopic = (std::string) fsSettings["Topic.BaseFrame"];

      mpSLAM = new ORB_SLAM2::System(strVocFile, strSettingsFile, ORB_SLAM2::System::MONOCULAR, true);

      broadCastTopic = cameraFrameTopic + "_ORB";

      subImage = mpNodeHandler->subscribe(cameraTopic, 1, &SubscribeHandler::GrabImage, this);
      maqui_orientation = mpNodeHandler->advertise<geometry_msgs::PoseStamped>("/maqui/odom_ORB", queueSize);

      // Initialize ORB system
   // argument 4 boolean is user viewer
}



void SubscribeHandler::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    try
    {
        mpTFlistener->waitForTransform(worldFrameTopic, cameraFrameTopic, ros::Time(0), ros::Duration(0.0001));
        mpTFlistener->lookupTransform(worldFrameTopic, cameraFrameTopic,ros::Time(0), T_w_c);
    }
    catch(tf::TransformException& e)
    {
        ROS_WARN("TF exception while grabbing camera transform \n %s", e.what());
    }
    // TF look up transform between head and base link
    try
    {
        mpTFlistener->waitForTransform(worldFrameTopic, cameraFrameTopic, ros::Time(0), ros::Duration(0.0001));
        mpTFlistener->lookupTransform(worldFrameTopic, cameraFrameTopic,ros::Time(0), T_b_c);
    }
    catch(tf::TransformException& e)
    {
        ROS_WARN("TF exception while grabbing camera transform \n %s", e.what());
    }
    cvT_w_c = tfToMat(T_w_c);
    mpSLAM->SetOdomPose(cvT_w_c);
    Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    if(!Tcw.empty())
    {
        SubscribeHandler::Publish_Orientation(Tcw.clone(), T_w_c, T_b_c);
    }
}



void SubscribeHandler::Publish_Orientation(cv::Mat Tcw, tf::StampedTransform T_w_c , tf::StampedTransform T_b_c)
{
    cv::Mat Tbw;
    Tbw = CameraToBaseFrame(Tcw, T_b_c);

    Eigen::Matrix<double, 3, 3> Tbw_eig = SubscribeHandler::toMatrix3d(Tbw.clone());
    std::vector<float> q = SubscribeHandler::toQuaternion(Tbw_eig);

    // TF fill broadcast message
    tf::Transform TForientation;

    TForientation.setOrigin(tf::Vector3(Tbw.at<float>(0,3), Tbw.at<float>(1,3),Tbw.at<float>(2,3)));

    tf::Quaternion quatTF;
    quatTF.setW(q[0]);
    quatTF.setX(q[1]);
    quatTF.setY(q[2]);
    quatTF.setZ(q[3]);
    TForientation.setRotation(quatTF);

    // publish geometry message
    geometry_msgs::PoseStamped orientation_msg;
    orientation_msg.header.frame_id = "odom_ORB";
    // TODO implement message count and child frame id name
    //    orientation_msg.header.seq = count;
    //    orientation_msg.child_frame_id = "base_link";
    orientation_msg.header.stamp.sec = T_w_c.stamp_.sec;


    orientation_msg.pose.position.x = Tbw.at<float>(0,3);
    orientation_msg.pose.position.y = Tbw.at<float>(1,3);
    orientation_msg.pose.position.z = Tbw.at<float>(2,3);
    orientation_msg.pose.orientation.w = q[0];
    orientation_msg.pose.orientation.x = q[1];
    orientation_msg.pose.orientation.y = q[2];
    orientation_msg.pose.orientation.z = q[3];



    mpTFbroadcaster->sendTransform(tf::StampedTransform(TForientation, T_w_c.stamp_, worldFrameTopic, broadCastTopic));
    maqui_orientation.publish(orientation_msg);

}

cv::Mat SubscribeHandler::CameraToBaseFrame(cv::Mat Tcw, tf::StampedTransform T_b_c)
{
    cv::Mat temp = SubscribeHandler::tfToMat(T_b_c);
    g2o::SE3Quat Temp_cw = SubscribeHandler::toSE3Quat(Tcw);
    g2o::SE3Quat Temp_bc = SubscribeHandler::toSE3Quat(temp);
    g2o::SE3Quat T_b_w = Temp_bc * Temp_cw;
    return SubscribeHandler::toCvMat(T_b_w);
}



cv::Mat SubscribeHandler::tfToMat(const tf::StampedTransform& tfT)
{
    cv::Mat cvT = cv::Mat::eye(4, 4, CV_32F);

    cvT.at<float>(0,3) = tfT.getOrigin().x();
    cvT.at<float>(1,3) = tfT.getOrigin().y();
    cvT.at<float>(2,3) = tfT.getOrigin().z();
    cvT.at<float>(3,3) = 1.0;

    cvT.at<float>(0,0) = tfT.getBasis().getColumn(0).x();
    cvT.at<float>(1,0) = tfT.getBasis().getColumn(0).y();
    cvT.at<float>(2,0) = tfT.getBasis().getColumn(0).z();
    cvT.at<float>(0,1) = tfT.getBasis().getColumn(1).x();
    cvT.at<float>(1,1) = tfT.getBasis().getColumn(1).y();
    cvT.at<float>(2,1) = tfT.getBasis().getColumn(1).z();
    cvT.at<float>(0,2) = tfT.getBasis().getColumn(2).x();
    cvT.at<float>(1,2) = tfT.getBasis().getColumn(2).y();
    cvT.at<float>(2,2) = tfT.getBasis().getColumn(2).z();

    return cvT;
}

g2o::SE3Quat SubscribeHandler::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat SubscribeHandler::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=eigMat(i,j);

    return cvMat.clone();
}

Eigen::Matrix<double,3,3> SubscribeHandler::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;
    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> SubscribeHandler::toQuaternion(const Eigen::Matrix<double, 3, 3> &M)
{
    Eigen::Quaterniond q(M);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    std::vector<float> quat = SubscribeHandler::Normalize(v);
    return quat;
}


std::vector<float> SubscribeHandler::Normalize(std::vector<float> vect)
{
    float sum = 0;
    // normalize
    for (unsigned int i = 0; i == vect.size(); i++)
    {
        sum+= vect[i]*vect[i];
    }

    float scalar = 1/std::sqrt(sum);

    for (unsigned int j = 0; j == vect.size(); j++)
    {
        vect[j] *= scalar;
    }
    return vect;
}

void SubscribeHandler::Shutdown(){

    std::cout << "Shutdown" << std::endl;
    mpSLAM->Shutdown();
    ros::shutdown();
}




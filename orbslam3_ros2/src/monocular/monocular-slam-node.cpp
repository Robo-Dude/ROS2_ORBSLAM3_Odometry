#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

#include <sophus/se3.hpp>

#include <iostream>

#include <Eigen/Dense>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    // Print translation
    Eigen::Vector3f t = Tcw.translation();
    std::cout << "Pose -> x: " << t.x()
            << " y: " << t.y()
            << " z: " << t.z() << std::endl;

    // Print quaternion
    Eigen::Quaternionf q(Tcw.rotationMatrix());
    std::cout << "Quat -> x: " << q.x()
            << " y: " << q.y()
            << " z: " << q.z()
            << " w: " << q.w() << std::endl;


}

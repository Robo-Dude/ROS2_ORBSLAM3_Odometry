#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

#include <sophus/se3.hpp>

#include <iostream>

#include <Eigen/Dense>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

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

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
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
    // Eigen::Vector3f t = Tcw.translation();
    // std::cout << "Pose -> x: " << t.x()
    //         << " y: " << t.y()
    //         << " z: " << t.z() << std::endl;

    // // Print quaternion
    // Eigen::Quaternionf q(Tcw.rotationMatrix());
    // std::cout << "Quat -> x: " << q.x()
    //         << " y: " << q.y()
    //         << " z: " << q.z()
    //         << " w: " << q.w() << std::endl;

    if (Tcw.matrix().isZero(0)) {
        RCLCPP_WARN(this->get_logger(), "Invalid pose from ORB-SLAM3");
        return;
    }

    // 2. Invert to get camera-in-world
    Sophus::SE3f Twc = Tcw.inverse();

    // --- Compute delta and accumulate ---
    if (!has_prev_pose_) {
        prev_pose_ = Twc;
        has_prev_pose_ = true;
    }

    // Delta between frames
    Sophus::SE3f delta = prev_pose_.inverse() * Twc;
    accumulated_pose_ = accumulated_pose_ * delta;
    prev_pose_ = Twc;

    // 3. Extract translation & rotation
    Eigen::Vector3f t_slam = accumulated_pose_.translation();
    Eigen::Matrix3f R_slam = accumulated_pose_.rotationMatrix();

    // 4. Axis conversion SLAM → ROS
    // ORB-SLAM3: X=right, Y=down, Z=forward
    // ROS REP-103: X=forward, Y=left, Z=up
    Eigen::Matrix3f R_slam_to_ros;
    R_slam_to_ros << 0,  0, 1,
                    -1,  0, 0,
                    0,  -1, 0;

    Eigen::Vector3f t_ros = R_slam_to_ros * t_slam;
    Eigen::Matrix3f R_ros = R_slam_to_ros * R_slam * R_slam_to_ros.transpose();
    

    // 5. Convert to quaternion
    Eigen::Quaternionf q_ros(R_ros);
    q_ros.normalize();

    // 6. Publish Odometry
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = t_ros.x();
    odom_msg.pose.pose.position.y = t_ros.y();
    odom_msg.pose.pose.position.z = t_ros.z();

    odom_msg.pose.pose.orientation.x = q_ros.x();
    odom_msg.pose.pose.orientation.y = q_ros.y();
    odom_msg.pose.pose.orientation.z = q_ros.z();
    odom_msg.pose.pose.orientation.w = q_ros.w();

    odom_pub_->publish(odom_msg);

    // 7. Publish TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = odom_msg.child_frame_id;

    tf_msg.transform.translation.x = t_ros.x();
    tf_msg.transform.translation.y = t_ros.y();
    tf_msg.transform.translation.z = t_ros.z();

    tf_msg.transform.rotation.x = q_ros.x();
    tf_msg.transform.rotation.y = q_ros.y();
    tf_msg.transform.rotation.z = q_ros.z();
    tf_msg.transform.rotation.w = q_ros.w();

    tf_broadcaster_->sendTransform(tf_msg);


}

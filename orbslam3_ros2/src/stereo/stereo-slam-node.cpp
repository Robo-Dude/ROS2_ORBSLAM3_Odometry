#include "stereo-slam-node.hpp"

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
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (doRectify){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/right");
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (doRectify){
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Sophus::SE3f Tcw = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));

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
                        1,  0, 0,
                        0,  1, 0;

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
    else
    {
        Sophus::SE3f Tcw = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));

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

}

    

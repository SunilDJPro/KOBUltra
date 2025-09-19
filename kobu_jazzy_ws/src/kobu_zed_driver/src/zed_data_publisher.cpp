#include "kobu_zed_driver/zed_camera_driver.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace kobu_zed_driver
{

void ZEDCameraDriver::setupPublishers()
{
  RCLCPP_INFO(get_logger(), "Setting up publishers");
  
  // Create image transport
  image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  
  // RGB images
  left_rgb_pub_ = image_transport_->advertise(camera_name_ + "/left/image_rect_color", 1);
  right_rgb_pub_ = image_transport_->advertise(camera_name_ + "/right/image_rect_color", 1);
  
  if (publish_raw_images_) {
    left_rgb_raw_pub_ = image_transport_->advertise(camera_name_ + "/left/image_raw_color", 1);
  }
  
  // Depth image
  if (enable_depth_) {
    depth_pub_ = image_transport_->advertise(camera_name_ + "/depth/depth_registered", 1);
  }
  
  // Point cloud
  if (enable_point_cloud_) {
    point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      camera_name_ + "/point_cloud/cloud_registered", 1);
  }
  
  // IMU data - 200Hz for ZED 2i
  if (enable_imu_) {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      camera_name_ + "/imu/data", 100);
  }
  
  // Camera info
  left_cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_name_ + "/left/camera_info", 1);
  right_cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_name_ + "/right/camera_info", 1);
  
  // Odometry and pose
  if (enable_positional_tracking_) {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      camera_name_ + "/odom", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      camera_name_ + "/pose", 10);
  }
  
  // TF broadcaster
  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }
  
  RCLCPP_INFO(get_logger(), "Publishers setup complete");
}

void ZEDCameraDriver::setupCameraInfo(const sl::CameraInformation& cam_info)
{
  // Left camera info
  left_cam_info_msg_.header.frame_id = left_camera_frame_id_;
  left_cam_info_msg_.height = cam_info.camera_configuration.resolution.height;
  left_cam_info_msg_.width = cam_info.camera_configuration.resolution.width;
  left_cam_info_msg_.distortion_model = "plumb_bob";
  
  // Get calibration parameters
  auto calib_params = cam_info.camera_configuration.calibration_parameters;
  
  // Intrinsics for left camera
  left_cam_info_msg_.k[0] = calib_params.left_cam.fx;
  left_cam_info_msg_.k[2] = calib_params.left_cam.cx;
  left_cam_info_msg_.k[4] = calib_params.left_cam.fy;
  left_cam_info_msg_.k[5] = calib_params.left_cam.cy;
  left_cam_info_msg_.k[8] = 1.0;
  
  // Distortion coefficients
  left_cam_info_msg_.d.resize(5);
  for (int i = 0; i < 5; i++) {
    left_cam_info_msg_.d[i] = calib_params.left_cam.disto[i];
  }
  
  // Rectification matrix (identity for rectified images)
  left_cam_info_msg_.r[0] = 1.0;
  left_cam_info_msg_.r[4] = 1.0;
  left_cam_info_msg_.r[8] = 1.0;
  
  // Projection matrix
  left_cam_info_msg_.p[0] = calib_params.left_cam.fx;
  left_cam_info_msg_.p[2] = calib_params.left_cam.cx;
  left_cam_info_msg_.p[5] = calib_params.left_cam.fy;
  left_cam_info_msg_.p[6] = calib_params.left_cam.cy;
  left_cam_info_msg_.p[10] = 1.0;
  
  // Right camera info
  right_cam_info_msg_ = left_cam_info_msg_;
  right_cam_info_msg_.header.frame_id = right_camera_frame_id_;
  
  // Right camera intrinsics
  right_cam_info_msg_.k[0] = calib_params.right_cam.fx;
  right_cam_info_msg_.k[2] = calib_params.right_cam.cx;
  right_cam_info_msg_.k[4] = calib_params.right_cam.fy;
  right_cam_info_msg_.k[5] = calib_params.right_cam.cy;
  
  // Right camera distortion
  for (int i = 0; i < 5; i++) {
    right_cam_info_msg_.d[i] = calib_params.right_cam.disto[i];
  }
  
  // Right camera projection matrix with baseline
  float baseline = calib_params.getCameraBaseline();
  right_cam_info_msg_.p[3] = -baseline * calib_params.right_cam.fx;
  
  RCLCPP_INFO(get_logger(), "Camera calibration loaded. Baseline: %.3f m", baseline);
}

void ZEDCameraDriver::publishRGBImage(const sl::Mat& zed_image, const rclcpp::Time& timestamp)
{
  if (left_rgb_pub_.getNumSubscribers() == 0) return;
  
  auto msg = convertSLMatToROSImage(zed_image, "bgra8", timestamp);
  if (msg) {
    msg->header.frame_id = left_camera_frame_id_;
    left_rgb_pub_.publish(msg);
  }
}

void ZEDCameraDriver::publishRightRGBImage(const sl::Mat& zed_image, const rclcpp::Time& timestamp)
{
  if (right_rgb_pub_.getNumSubscribers() == 0) return;
  
  auto msg = convertSLMatToROSImage(zed_image, "bgra8", timestamp);
  if (msg) {
    msg->header.frame_id = right_camera_frame_id_;
    right_rgb_pub_.publish(msg);
  }
}

void ZEDCameraDriver::publishDepthImage(const sl::Mat& depth_map, const rclcpp::Time& timestamp)
{
  if (depth_pub_.getNumSubscribers() == 0) return;
  
  auto msg = convertSLMatToROSImage(depth_map, "32FC1", timestamp);
  if (msg) {
    msg->header.frame_id = left_camera_frame_id_;
    depth_pub_.publish(msg);
  }
}

void ZEDCameraDriver::publishPointCloud(const sl::Mat& point_cloud, const rclcpp::Time& timestamp)
{
  if (point_cloud_pub_->get_subscription_count() == 0) return;
  
  auto msg = convertSLMatToPointCloud2(point_cloud, timestamp);
  if (msg) {
    msg->header.frame_id = camera_optical_frame_id_;
    point_cloud_pub_->publish(*msg);
  }
}

void ZEDCameraDriver::publishIMUData(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp)
{
  if (imu_pub_->get_subscription_count() == 0) return;
  
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
  imu_msg->header.stamp = timestamp;
  imu_msg->header.frame_id = imu_frame_id_;
  
  // Get IMU data
  auto imu_data = sensors_data.imu;
  
  // Orientation (quaternion)
  imu_msg->orientation.x = imu_data.pose.getOrientation()[0];
  imu_msg->orientation.y = imu_data.pose.getOrientation()[1];
  imu_msg->orientation.z = imu_data.pose.getOrientation()[2];
  imu_msg->orientation.w = imu_data.pose.getOrientation()[3];
  
  // Angular velocity (rad/s)
  imu_msg->angular_velocity.x = imu_data.angular_velocity[0] * M_PI / 180.0;
  imu_msg->angular_velocity.y = imu_data.angular_velocity[1] * M_PI / 180.0;
  imu_msg->angular_velocity.z = imu_data.angular_velocity[2] * M_PI / 180.0;
  
  // Linear acceleration (m/s^2)
  imu_msg->linear_acceleration.x = imu_data.linear_acceleration[0];
  imu_msg->linear_acceleration.y = imu_data.linear_acceleration[1];
  imu_msg->linear_acceleration.z = imu_data.linear_acceleration[2];
  
  // Covariances (using ZED SDK provided values)
  for (int i = 0; i < 9; i++) {
    imu_msg->orientation_covariance[i] = imu_data.pose_covariance.r[i];
    imu_msg->angular_velocity_covariance[i] = imu_data.angular_velocity_covariance.r[i];
    imu_msg->linear_acceleration_covariance[i] = imu_data.linear_acceleration_covariance.r[i];
  }
  
  imu_pub_->publish(std::move(imu_msg));
}

void ZEDCameraDriver::publishCameraInfo(const rclcpp::Time& timestamp)
{
  // Update timestamps
  left_cam_info_msg_.header.stamp = timestamp;
  right_cam_info_msg_.header.stamp = timestamp;
  
  // Publish camera info
  if (left_cam_info_pub_->get_subscription_count() > 0) {
    left_cam_info_pub_->publish(left_cam_info_msg_);
  }
  
  if (right_cam_info_pub_->get_subscription_count() > 0) {
    right_cam_info_pub_->publish(right_cam_info_msg_);
  }
}

void ZEDCameraDriver::publishOdometry(const sl::Pose& zed_pose, const rclcpp::Time& timestamp)
{
  if (odom_pub_->get_subscription_count() == 0) return;
  
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
  odom_msg->header.stamp = timestamp;
  odom_msg->header.frame_id = odom_frame_id_;
  odom_msg->child_frame_id = camera_frame_id_;
  
  // Position
  auto translation = zed_pose.getTranslation();
  odom_msg->pose.pose.position.x = translation.x;
  odom_msg->pose.pose.position.y = translation.y;
  odom_msg->pose.pose.position.z = translation.z;
  
  // Orientation
  auto orientation = zed_pose.getOrientation();
  odom_msg->pose.pose.orientation.x = orientation.x;
  odom_msg->pose.pose.orientation.y = orientation.y;
  odom_msg->pose.pose.orientation.z = orientation.z;
  odom_msg->pose.pose.orientation.w = orientation.w;
  
  // Velocity is not directly available from Pose
  // It should be computed from positional tracking or obtained from sensors
  // For now, set velocities to zero or compute from position changes
  odom_msg->twist.twist.linear.x = 0.0;
  odom_msg->twist.twist.linear.y = 0.0;
  odom_msg->twist.twist.linear.z = 0.0;
  
  // Angular velocity (would need to be computed from orientation changes)
  odom_msg->twist.twist.angular.x = 0.0;
  odom_msg->twist.twist.angular.y = 0.0;
  odom_msg->twist.twist.angular.z = 0.0;
  
  // Covariances
  for (int i = 0; i < 36; i++) {
    odom_msg->pose.covariance[i] = zed_pose.pose_covariance[i];
  }
  
  odom_pub_->publish(std::move(odom_msg));
}

void ZEDCameraDriver::publishPose(const sl::Pose& zed_pose, const rclcpp::Time& timestamp)
{
  if (pose_pub_->get_subscription_count() == 0) return;
  
  auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
  pose_msg->header.stamp = timestamp;
  pose_msg->header.frame_id = map_frame_id_;
  
  // Position
  auto translation = zed_pose.getTranslation();
  pose_msg->pose.position.x = translation.x;
  pose_msg->pose.position.y = translation.y;
  pose_msg->pose.position.z = translation.z;
  
  // Orientation
  auto orientation = zed_pose.getOrientation();
  pose_msg->pose.orientation.x = orientation.x;
  pose_msg->pose.orientation.y = orientation.y;
  pose_msg->pose.orientation.z = orientation.z;
  pose_msg->pose.orientation.w = orientation.w;
  
  pose_pub_->publish(std::move(pose_msg));
}

void ZEDCameraDriver::publishTransforms(const sl::Pose& pose, const rclcpp::Time& timestamp)
{
  if (!publish_tf_ || !tf_broadcaster_) return;
  
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = timestamp;
  transform.header.frame_id = odom_frame_id_;
  transform.child_frame_id = camera_frame_id_;
  
  // Set translation
  auto translation = pose.getTranslation();
  transform.transform.translation.x = translation.x;
  transform.transform.translation.y = translation.y;
  transform.transform.translation.z = translation.z;
  
  // Set rotation
  auto orientation = pose.getOrientation();
  transform.transform.rotation.x = orientation.x;
  transform.transform.rotation.y = orientation.y;
  transform.transform.rotation.z = orientation.z;
  transform.transform.rotation.w = orientation.w;
  
  tf_broadcaster_->sendTransform(transform);
}

} // namespace kobu_zed_driver
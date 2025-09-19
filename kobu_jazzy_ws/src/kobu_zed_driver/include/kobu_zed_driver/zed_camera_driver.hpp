#ifndef KOBU_ZED_DRIVER__ZED_CAMERA_DRIVER_HPP_
#define KOBU_ZED_DRIVER__ZED_CAMERA_DRIVER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>

// ZED SDK 5.0+ includes
#include <sl/Camera.hpp>

namespace kobu_zed_driver
{

class ZEDCameraDriver : public rclcpp::Node
{
public:
  explicit ZEDCameraDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ZEDCameraDriver();
  
  // Initialize method - must be called after node is created as shared_ptr
  void initialize();

private:
  // Initialization methods
  bool initializeCamera();
  bool loadParameters();
  void setupPublishers();
  void setupCameraInfo(const sl::CameraInformation& cam_info);
  
  // Main processing loop
  void grabLoop();
  
  // Publishing methods
  void publishRGBImage(const sl::Mat& zed_image, const rclcpp::Time& timestamp);
  void publishRightRGBImage(const sl::Mat& zed_image, const rclcpp::Time& timestamp);
  void publishDepthImage(const sl::Mat& depth_map, const rclcpp::Time& timestamp);
  void publishPointCloud(const sl::Mat& point_cloud, const rclcpp::Time& timestamp);
  void publishIMUData(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp);
  void publishCameraInfo(const rclcpp::Time& timestamp);
  void publishOdometry(const sl::Pose& zed_pose, const rclcpp::Time& timestamp);
  void publishPose(const sl::Pose& zed_pose, const rclcpp::Time& timestamp);
  
  // Utility methods
  sensor_msgs::msg::Image::SharedPtr convertSLMatToROSImage(
    const sl::Mat& input, const std::string& encoding, const rclcpp::Time& timestamp);
  sensor_msgs::msg::PointCloud2::SharedPtr convertSLMatToPointCloud2(
    const sl::Mat& point_cloud, const rclcpp::Time& timestamp);
  
  // Transform publishing
  void publishTransforms(const sl::Pose& pose, const rclcpp::Time& timestamp);
  
  // Diagnostics
  void diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper & stat);
  
  // IMU-specific methods (implemented in zed_imu_handler.cpp)
  void processIMUData(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp);
  void publishIMUTemperature(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp);
  void computeIMUVelocity(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp);
  void processMagnetometerData(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp);
  void processBarometerData(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp);
  void processAllIMUSensors(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp);
  void configureIMUSettings();
  
  // ZED SDK objects
  std::unique_ptr<sl::Camera> zed_camera_;
  sl::InitParameters init_params_;
  sl::RuntimeParameters runtime_params_;
  sl::PositionalTrackingParameters tracking_params_;
  
  // Thread management
  std::thread grab_thread_;
  std::atomic<bool> stop_node_;
  std::mutex grab_mutex_;
  
  // Publishers - using image_transport for images
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::Publisher left_rgb_pub_;
  image_transport::Publisher left_rgb_raw_pub_;
  image_transport::Publisher right_rgb_pub_;
  image_transport::Publisher depth_pub_;
  
  // Standard ROS2 publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_cam_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_cam_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  
  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Diagnostic updater
  diagnostic_updater::Updater diagnostic_updater_;
  
  // Camera parameters
  std::string camera_model_;
  std::string camera_name_;
  int camera_id_;
  std::string svo_file_;
  int gpu_id_;
  
  // Frame IDs for TF tree
  std::string base_frame_id_;
  std::string camera_frame_id_;
  std::string left_camera_frame_id_;
  std::string right_camera_frame_id_;
  std::string camera_optical_frame_id_;
  std::string imu_frame_id_;
  std::string odom_frame_id_;
  std::string map_frame_id_;
  
  // Resolution and FPS settings
  int resolution_;
  int camera_fps_;
  
  // Depth settings (ZED SDK 5.0+ specific)
  int depth_mode_;  // Will be set to NEURAL_LIGHT
  float depth_minimum_distance_;
  float depth_maximum_distance_;
  int depth_stabilization_;
  int depth_confidence_threshold_;
  int depth_texture_confidence_threshold_;
  
  // Feature flags
  bool enable_positional_tracking_;
  bool enable_imu_;
  bool enable_point_cloud_;
  bool enable_depth_;
  bool publish_tf_;
  bool publish_raw_images_;
  
  // Camera calibration info
  sensor_msgs::msg::CameraInfo left_cam_info_msg_;
  sensor_msgs::msg::CameraInfo right_cam_info_msg_;
  
  // Performance metrics
  int grab_count_;
  int grab_success_count_;
  rclcpp::Time last_diagnostic_time_;
  std::chrono::steady_clock::time_point last_grab_time_;
  
  // Camera state
  bool camera_opened_;
  sl::POSITIONAL_TRACKING_STATE tracking_state_;
  
  // Coordinate transforms
  bool flip_image_;
  int coordinate_system_;
  int coordinate_units_;
};

} // namespace kobu_zed_driver

#endif // KOBU_ZED_DRIVER__ZED_CAMERA_DRIVER_HPP_
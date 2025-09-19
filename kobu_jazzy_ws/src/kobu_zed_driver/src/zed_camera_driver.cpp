#include "kobu_zed_driver/zed_camera_driver.hpp"

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace kobu_zed_driver
{

ZEDCameraDriver::ZEDCameraDriver(const rclcpp::NodeOptions & options)
: Node("zed_camera_driver", options),
  stop_node_(false),
  grab_count_(0),
  grab_success_count_(0),
  camera_opened_(false),
  diagnostic_updater_(this)
{
  RCLCPP_INFO(get_logger(), "Initializing ZED Camera Driver for SDK 5.0+");
  
  // Load parameters
  if (!loadParameters()) {
    RCLCPP_ERROR(get_logger(), "Failed to load parameters");
    rclcpp::shutdown();
    return;
  }
  
  // Initialize camera
  if (!initializeCamera()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize ZED camera");
    rclcpp::shutdown();
    return;
  }
  
  // Note: setupPublishers() will be called from initialize() method
  // which should be called after the node is created as a shared_ptr
  
  RCLCPP_INFO(get_logger(), "ZED Camera Driver constructor completed");
}

// Separate initialization method to be called after node is created as shared_ptr
void ZEDCameraDriver::initialize()
{
  RCLCPP_INFO(get_logger(), "Starting deferred initialization");
  
  // Setup publishers (requires shared_from_this())
  try {
    RCLCPP_INFO(get_logger(), "Setting up publishers");
    setupPublishers();
    RCLCPP_INFO(get_logger(), "Publishers setup complete");
  } catch (const std::bad_weak_ptr& e) {
    RCLCPP_ERROR(get_logger(), "Failed to setup publishers: %s", e.what());
    RCLCPP_ERROR(get_logger(), "Make sure initialize() is called after creating node as shared_ptr");
    return;
  }
  
  // Setup diagnostics
  diagnostic_updater_.setHardwareID("ZED_" + camera_name_);
  diagnostic_updater_.add("Camera Status", this, &ZEDCameraDriver::diagnosticUpdate);
  
  // Initialize timing
  last_diagnostic_time_ = now();
  last_grab_time_ = std::chrono::steady_clock::now();
  
  // Start grab thread
  RCLCPP_INFO(get_logger(), "Starting grab thread");
  grab_thread_ = std::thread(&ZEDCameraDriver::grabLoop, this);
  
  RCLCPP_INFO(get_logger(), "ZED Camera Driver fully initialized");
}

ZEDCameraDriver::~ZEDCameraDriver()
{
  RCLCPP_INFO(get_logger(), "Shutting down ZED Camera Driver");
  
  stop_node_ = true;
  
  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }
  
  if (zed_camera_ && zed_camera_->isOpened()) {
    if (enable_positional_tracking_) {
      zed_camera_->disablePositionalTracking();
    }
    zed_camera_->close();
  }
  
  RCLCPP_INFO(get_logger(), "ZED Camera Driver shutdown complete");
}

bool ZEDCameraDriver::loadParameters()
{
  // General camera configuration
  declare_parameter("general.camera_model", "zed2i");
  declare_parameter("general.camera_name", "zed");
  declare_parameter("general.camera_id", 0);
  declare_parameter("general.svo_file", "");
  declare_parameter("general.gpu_id", -1);
  
  camera_model_ = get_parameter("general.camera_model").as_string();
  camera_name_ = get_parameter("general.camera_name").as_string();
  camera_id_ = get_parameter("general.camera_id").as_int();
  svo_file_ = get_parameter("general.svo_file").as_string();
  gpu_id_ = get_parameter("general.gpu_id").as_int();
  
  // Video settings
  declare_parameter("video.resolution", 2);  // HD1080 = 2
  declare_parameter("video.fps", 30);
  declare_parameter("video.flip_image", false);
  
  resolution_ = get_parameter("video.resolution").as_int();
  camera_fps_ = get_parameter("video.fps").as_int();
  flip_image_ = get_parameter("video.flip_image").as_bool();
  
  // Depth configuration - CRITICAL for ZED SDK 5.0+
  // NEURAL_LIGHT = 6 in ZED SDK 5.0+
  declare_parameter("depth.depth_mode", 6);  
  declare_parameter("depth.min_depth", 0.3);
  declare_parameter("depth.max_depth", 20.0);
  declare_parameter("depth.depth_stabilization", 30);
  declare_parameter("depth.confidence_threshold", 50);
  declare_parameter("depth.texture_confidence_threshold", 100);
  
  depth_mode_ = get_parameter("depth.depth_mode").as_int();
  depth_minimum_distance_ = get_parameter("depth.min_depth").as_double();
  depth_maximum_distance_ = get_parameter("depth.max_depth").as_double();
  depth_stabilization_ = get_parameter("depth.depth_stabilization").as_int();
  depth_confidence_threshold_ = get_parameter("depth.confidence_threshold").as_int();
  depth_texture_confidence_threshold_ = get_parameter("depth.texture_confidence_threshold").as_int();
  
  // Coordinate system
  declare_parameter("coordinate.system", 3);  // RIGHT_HANDED_Z_UP = 3
  declare_parameter("coordinate.units", 0);   // METER = 0
  
  coordinate_system_ = get_parameter("coordinate.system").as_int();
  coordinate_units_ = get_parameter("coordinate.units").as_int();
  
  // Features
  declare_parameter("tracking.enable_tracking", true);
  declare_parameter("tracking.enable_area_memory", true);
  declare_parameter("tracking.enable_imu_fusion", true);
  declare_parameter("sensors.enable_imu", true);
  declare_parameter("point_cloud.enable", true);
  declare_parameter("depth.enable", true);
  declare_parameter("general.publish_tf", true);
  declare_parameter("general.publish_raw_images", false);
  
  enable_positional_tracking_ = get_parameter("tracking.enable_tracking").as_bool();
  enable_imu_ = get_parameter("sensors.enable_imu").as_bool();
  enable_point_cloud_ = get_parameter("point_cloud.enable").as_bool();
  enable_depth_ = get_parameter("depth.enable").as_bool();
  publish_tf_ = get_parameter("general.publish_tf").as_bool();
  publish_raw_images_ = get_parameter("general.publish_raw_images").as_bool();
  
  // Frame IDs for TF tree
  declare_parameter("frame_ids.base_frame", "base_link");
  declare_parameter("frame_ids.camera_frame", camera_name_ + "_camera_link");
  declare_parameter("frame_ids.left_camera_frame", camera_name_ + "_left_camera_frame");
  declare_parameter("frame_ids.right_camera_frame", camera_name_ + "_right_camera_frame");
  declare_parameter("frame_ids.camera_optical_frame", camera_name_ + "_left_camera_optical_frame");
  declare_parameter("frame_ids.imu_frame", camera_name_ + "_imu_link");
  declare_parameter("frame_ids.odom_frame", "odom");
  declare_parameter("frame_ids.map_frame", "map");
  
  base_frame_id_ = get_parameter("frame_ids.base_frame").as_string();
  camera_frame_id_ = get_parameter("frame_ids.camera_frame").as_string();
  left_camera_frame_id_ = get_parameter("frame_ids.left_camera_frame").as_string();
  right_camera_frame_id_ = get_parameter("frame_ids.right_camera_frame").as_string();
  camera_optical_frame_id_ = get_parameter("frame_ids.camera_optical_frame").as_string();
  imu_frame_id_ = get_parameter("frame_ids.imu_frame").as_string();
  odom_frame_id_ = get_parameter("frame_ids.odom_frame").as_string();
  map_frame_id_ = get_parameter("frame_ids.map_frame").as_string();
  
  RCLCPP_INFO(get_logger(), "Parameters loaded successfully");
  RCLCPP_INFO(get_logger(), "Camera Model: %s", camera_model_.c_str());
  RCLCPP_INFO(get_logger(), "Depth Mode: %d (NEURAL_LIGHT for SDK 5.0+)", depth_mode_);
  RCLCPP_INFO(get_logger(), "Resolution: %d, FPS: %d", resolution_, camera_fps_);
  
  return true;
}

bool ZEDCameraDriver::initializeCamera()
{
  RCLCPP_INFO(get_logger(), "Initializing ZED camera with SDK 5.0+");
  
  zed_camera_ = std::make_unique<sl::Camera>();
  
  // Set input source
  if (!svo_file_.empty()) {
    RCLCPP_INFO(get_logger(), "Using SVO file: %s", svo_file_.c_str());
    init_params_.input.setFromSVOFile(svo_file_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Using camera ID: %d", camera_id_);
    init_params_.input.setFromCameraID(camera_id_);
  }
  
  // Video settings
  init_params_.camera_resolution = static_cast<sl::RESOLUTION>(resolution_);
  init_params_.camera_fps = camera_fps_;
  
  // Image flip
  if (flip_image_) {
    init_params_.camera_image_flip = sl::FLIP_MODE::ON;
  } else {
    init_params_.camera_image_flip = sl::FLIP_MODE::OFF;
  }
  
  // CRITICAL: Set NEURAL_LIGHT depth mode for ZED SDK 5.0+
  // In SDK 5.0+, the syntax is DEPTH_MODE::NEURAL_LIGHT
  init_params_.depth_mode = sl::DEPTH_MODE::NEURAL_LIGHT;
  
  // Depth parameters
  init_params_.coordinate_units = static_cast<sl::UNIT>(coordinate_units_);
  init_params_.coordinate_system = static_cast<sl::COORDINATE_SYSTEM>(coordinate_system_);
  init_params_.depth_minimum_distance = depth_minimum_distance_;
  init_params_.depth_maximum_distance = depth_maximum_distance_;
  init_params_.depth_stabilization = depth_stabilization_;
  
  // SDK parameters
  init_params_.sdk_verbose = true;
  init_params_.sdk_verbose_log_file = "";
  
  if (gpu_id_ >= 0) {
    init_params_.sdk_gpu_id = gpu_id_;
  }
  
  // Sensor activation
  init_params_.sensors_required = enable_imu_;
  
  // Enable image enhancement for better depth quality
  init_params_.enable_image_enhancement = true;
  
  // Open the camera
  RCLCPP_INFO(get_logger(), "Opening ZED camera...");
  auto err = zed_camera_->open(init_params_);
  
  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to open ZED camera: %s", sl::toString(err).c_str());
    return false;
  }
  
  camera_opened_ = true;
  RCLCPP_INFO(get_logger(), "ZED camera opened successfully with NEURAL_LIGHT depth mode (SDK 5.0+)");
  
  // Get camera information
  auto camera_info = zed_camera_->getCameraInformation();
  RCLCPP_INFO(get_logger(), "Camera Model: %s", sl::toString(camera_info.camera_model).c_str());
  RCLCPP_INFO(get_logger(), "Serial Number: %d", camera_info.serial_number);
  RCLCPP_INFO(get_logger(), "Camera Firmware: %d", camera_info.camera_configuration.firmware_version);
  RCLCPP_INFO(get_logger(), "Image Resolution: %dx%d", 
              camera_info.camera_configuration.resolution.width,
              camera_info.camera_configuration.resolution.height);
  
  // Setup runtime parameters
  runtime_params_.confidence_threshold = depth_confidence_threshold_;
  runtime_params_.texture_confidence_threshold = depth_texture_confidence_threshold_;
  runtime_params_.enable_depth = enable_depth_;
  runtime_params_.enable_fill_mode = false;
  runtime_params_.remove_saturated_areas = true;
  
  // Enable positional tracking if requested
  if (enable_positional_tracking_) {
    RCLCPP_INFO(get_logger(), "Enabling positional tracking...");
    
    tracking_params_.enable_area_memory = get_parameter("tracking.enable_area_memory").as_bool();
    tracking_params_.enable_pose_smoothing = true;
    tracking_params_.enable_imu_fusion = get_parameter("tracking.enable_imu_fusion").as_bool();
    tracking_params_.set_gravity_as_origin = true;
    
    // Use GEN2 positional tracking for better performance
    tracking_params_.mode = sl::POSITIONAL_TRACKING_MODE::GEN_2;
    
    err = zed_camera_->enablePositionalTracking(tracking_params_);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN(get_logger(), "Failed to enable positional tracking: %s", 
                  sl::toString(err).c_str());
      enable_positional_tracking_ = false;
    } else {
      RCLCPP_INFO(get_logger(), "Positional tracking enabled (GEN2 mode)");
    }
  }
  
  // Setup camera calibration info
  setupCameraInfo(camera_info);
  
  // Configure IMU settings if available
  configureIMUSettings();
  
  return true;
}

} // namespace kobu_zed_driver
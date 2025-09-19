#include "kobu_zed_driver/zed_camera_driver.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace kobu_zed_driver
{

void ZEDCameraDriver::grabLoop()
{
  RCLCPP_INFO(get_logger(), "Starting grab loop");
  
  // Allocate ZED SDK matrices
  sl::Mat left_image, right_image, depth_map, point_cloud;
  sl::Pose zed_pose;
  sl::SensorsData sensors_data;
  
  // Main processing loop
  while (!stop_node_ && rclcpp::ok()) {
    // Check if camera is still opened
    if (!zed_camera_ || !zed_camera_->isOpened()) {
      RCLCPP_ERROR(get_logger(), "Camera is not opened");
      break;
    }
    
    // Grab new frame
    auto grab_status = zed_camera_->grab(runtime_params_);
    
    if (grab_status == sl::ERROR_CODE::SUCCESS) {
      // Update counters
      grab_count_++;
      grab_success_count_++;
      
      // Get current timestamp
      auto timestamp = now();
      
      // Retrieve and publish left RGB image
      if (left_rgb_pub_.getNumSubscribers() > 0 || 
          (publish_raw_images_ && left_rgb_raw_pub_.getNumSubscribers() > 0)) {
        zed_camera_->retrieveImage(left_image, sl::VIEW::LEFT);
        publishRGBImage(left_image, timestamp);
        
        // Publish raw image if enabled
        if (publish_raw_images_ && left_rgb_raw_pub_.getNumSubscribers() > 0) {
          sl::Mat left_raw;
          zed_camera_->retrieveImage(left_raw, sl::VIEW::LEFT_UNRECTIFIED);
          auto msg = convertSLMatToROSImage(left_raw, "bgra8", timestamp);
          if (msg) {
            msg->header.frame_id = left_camera_frame_id_;
            left_rgb_raw_pub_.publish(msg);
          }
        }
      }
      
      // Retrieve and publish right RGB image
      if (right_rgb_pub_.getNumSubscribers() > 0) {
        zed_camera_->retrieveImage(right_image, sl::VIEW::RIGHT);
        publishRightRGBImage(right_image, timestamp);
      }
      
      // Retrieve and publish depth if enabled
      if (enable_depth_ && depth_pub_.getNumSubscribers() > 0) {
        zed_camera_->retrieveMeasure(depth_map, sl::MEASURE::DEPTH);
        publishDepthImage(depth_map, timestamp);
      }
      
      // Retrieve and publish point cloud if enabled
      if (enable_point_cloud_ && point_cloud_pub_->get_subscription_count() > 0) {
        zed_camera_->retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);
        publishPointCloud(point_cloud, timestamp);
      }
      
      // Get IMU data if available (ZED 2i has IMU at 200Hz)
      if (enable_imu_ && imu_pub_->get_subscription_count() > 0) {
        if (zed_camera_->getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE) == sl::ERROR_CODE::SUCCESS) {
          // Use comprehensive IMU processing
          processAllIMUSensors(sensors_data, timestamp);
        }
      }
      
      // Get pose if tracking is enabled
      if (enable_positional_tracking_) {
        tracking_state_ = zed_camera_->getPosition(zed_pose, sl::REFERENCE_FRAME::WORLD);
        
        if (tracking_state_ == sl::POSITIONAL_TRACKING_STATE::OK ||
            tracking_state_ == sl::POSITIONAL_TRACKING_STATE::SEARCHING) {
          
          // Publish odometry
          if (odom_pub_->get_subscription_count() > 0) {
            publishOdometry(zed_pose, timestamp);
          }
          
          // Publish pose
          if (pose_pub_->get_subscription_count() > 0) {
            publishPose(zed_pose, timestamp);
          }
          
          // Publish TF
          if (publish_tf_) {
            publishTransforms(zed_pose, timestamp);
          }
        } else if (tracking_state_ == sl::POSITIONAL_TRACKING_STATE::OFF) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                              "Positional tracking is OFF");
        }
      }
      
      // Publish camera info
      publishCameraInfo(timestamp);
      
      // Update diagnostics periodically
      if ((timestamp - last_diagnostic_time_).seconds() > 1.0) {
        diagnostic_updater_.force_update();
        last_diagnostic_time_ = timestamp;
      }
      
    } else if (grab_status == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
      RCLCPP_INFO(get_logger(), "End of SVO file reached");
      
      // Loop the SVO if it's a file
      if (!svo_file_.empty()) {
        zed_camera_->setSVOPosition(0);
        RCLCPP_INFO(get_logger(), "Restarting SVO playback from beginning");
      } else {
        stop_node_ = true;
      }
      
    } else {
      grab_count_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                          "Grab failed: %s", sl::toString(grab_status).c_str());
      
      // Handle camera disconnection
      if (grab_status == sl::ERROR_CODE::CAMERA_NOT_DETECTED) {
        RCLCPP_ERROR(get_logger(), "Camera disconnected!");
        stop_node_ = true;
      }
    }
    
    // Small sleep to prevent CPU overload
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  
  RCLCPP_INFO(get_logger(), "Exiting grab loop");
}

sensor_msgs::msg::Image::SharedPtr ZEDCameraDriver::convertSLMatToROSImage(
  const sl::Mat& input, const std::string& encoding, const rclcpp::Time& timestamp)
{
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  
  msg->header.stamp = timestamp;
  msg->height = input.getHeight();
  msg->width = input.getWidth();
  
  int num_channels = input.getChannels();
  msg->is_bigendian = false;
  
  if (encoding == "bgra8") {
    msg->encoding = sensor_msgs::image_encodings::BGRA8;
    msg->step = msg->width * 4;
  } else if (encoding == "32FC1") {
    msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    msg->step = msg->width * sizeof(float);
  } else if (encoding == "rgb8") {
    msg->encoding = sensor_msgs::image_encodings::RGB8;
    msg->step = msg->width * 3;
  } else {
    RCLCPP_ERROR(get_logger(), "Unsupported encoding: %s", encoding.c_str());
    return nullptr;
  }
  
  size_t size = msg->step * msg->height;
  msg->data.resize(size);
  
  // Copy data
  sl::Mat input_cpu;
  input.copyTo(input_cpu, sl::COPY_TYPE::CPU_CPU);
  memcpy(&msg->data[0], input_cpu.getPtr<sl::uchar1>(), size);
  
  return msg;
}

sensor_msgs::msg::PointCloud2::SharedPtr ZEDCameraDriver::convertSLMatToPointCloud2(
  const sl::Mat& point_cloud, const rclcpp::Time& timestamp)
{
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  
  msg->header.stamp = timestamp;
  msg->height = point_cloud.getHeight();
  msg->width = point_cloud.getWidth();
  msg->is_dense = false;
  msg->is_bigendian = false;
  
  // Point cloud fields for XYZRGBA
  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  
  msg->point_step = 4 * sizeof(float);
  msg->row_step = msg->point_step * msg->width;
  
  size_t data_size = msg->row_step * msg->height;
  msg->data.resize(data_size);
  
  // Copy point cloud data to CPU
  sl::Mat point_cloud_cpu;
  point_cloud.copyTo(point_cloud_cpu, sl::COPY_TYPE::CPU_CPU);
  
  float* data_ptr = reinterpret_cast<float*>(&msg->data[0]);
  sl::float4* p_cloud = point_cloud_cpu.getPtr<sl::float4>();
  
  // Copy and filter invalid points
  for (size_t i = 0; i < msg->width * msg->height; i++) {
    if (std::isfinite(p_cloud[i].z)) {
      data_ptr[0] = p_cloud[i].x;
      data_ptr[1] = p_cloud[i].y;
      data_ptr[2] = p_cloud[i].z;
      data_ptr[3] = p_cloud[i].w;  // RGBA color
    } else {
      // Set invalid points to NaN
      data_ptr[0] = std::numeric_limits<float>::quiet_NaN();
      data_ptr[1] = std::numeric_limits<float>::quiet_NaN();
      data_ptr[2] = std::numeric_limits<float>::quiet_NaN();
      data_ptr[3] = 0;
    }
    data_ptr += 4;
  }
  
  return msg;
}

void ZEDCameraDriver::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Camera status
  if (!camera_opened_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Camera not opened");
    return;
  }
  
  if (!zed_camera_->isOpened()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Camera disconnected");
    return;
  }
  
  // Calculate success rate
  float success_rate = (grab_count_ > 0) ? 
    (static_cast<float>(grab_success_count_) / grab_count_ * 100.0f) : 0.0f;
  
  // Update status based on success rate
  if (success_rate > 95.0f) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Camera operating normally");
  } else if (success_rate > 80.0f) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Some grab failures");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "High grab failure rate");
  }
  
  // Add diagnostic values
  stat.add("Grab Count", grab_count_);
  stat.add("Success Count", grab_success_count_);
  stat.add("Success Rate", success_rate);
  stat.add("Camera Model", camera_model_);
  
  // Get current camera FPS
  float fps = zed_camera_->getCurrentFPS();
  stat.add("Current FPS", fps);
  stat.add("Target FPS", camera_fps_);
  
  // Tracking status
  if (enable_positional_tracking_) {
    std::string tracking_status = "UNKNOWN";
    switch(tracking_state_) {
      case sl::POSITIONAL_TRACKING_STATE::OK:
        tracking_status = "OK";
        break;
      case sl::POSITIONAL_TRACKING_STATE::OFF:
        tracking_status = "OFF";
        break;
      case sl::POSITIONAL_TRACKING_STATE::SEARCHING:
        tracking_status = "SEARCHING";
        break;
      case sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW:
        tracking_status = "FPS_TOO_LOW";
        break;
    }
    stat.add("Tracking Status", tracking_status);
  }
  
  // Temperature if available
  if (enable_imu_) {
    sl::SensorsData sensors_data;
    if (zed_camera_->getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) {
      float temp = 0.0f;
      // Try to get IMU temperature using the get() method
      sl::ERROR_CODE temp_status = sensors_data.temperature.get(
        sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, temp);
      if (temp_status == sl::ERROR_CODE::SUCCESS) {
        stat.add("IMU Temperature", temp);
      }
    }
  }
}

} // namespace kobu_zed_driver
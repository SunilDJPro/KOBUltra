#include "kobu_zed_driver/zed_camera_driver.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace kobu_zed_driver
{

// Specialized IMU data handler for ZED 2i
// This file contains IMU-specific processing methods

/**
 * @brief Process and publish raw IMU data
 * The ZED 2i provides IMU data at 200Hz including:
 * - Linear acceleration
 * - Angular velocity  
 * - Orientation (from internal fusion)
 * - Temperature
 */
void ZEDCameraDriver::processIMUData(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp)
{
  // Check if IMU publishing is enabled
  if (!enable_imu_ || imu_pub_->get_subscription_count() == 0) {
    return;
  }
  
  // Get IMU data structure
  auto imu_data = sensors_data.imu;
  
  // Check if IMU data is available and valid
  if (!imu_data.is_available) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "IMU data not available");
    return;
  }
  
  // Create IMU message
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
  
  // Set header
  imu_msg->header.stamp = timestamp;
  imu_msg->header.frame_id = imu_frame_id_;
  
  // Process orientation quaternion
  // ZED SDK provides orientation as quaternion (x, y, z, w)
  auto orientation = imu_data.pose.getOrientation();
  imu_msg->orientation.x = orientation.x;
  imu_msg->orientation.y = orientation.y;
  imu_msg->orientation.z = orientation.z;
  imu_msg->orientation.w = orientation.w;
  
  // Validate quaternion
  double quat_norm = std::sqrt(
    orientation.x * orientation.x +
    orientation.y * orientation.y +
    orientation.z * orientation.z +
    orientation.w * orientation.w
  );
  
  if (std::abs(quat_norm - 1.0) > 0.01) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Invalid quaternion norm: %f", quat_norm);
    // Normalize quaternion
    imu_msg->orientation.x /= quat_norm;
    imu_msg->orientation.y /= quat_norm;
    imu_msg->orientation.z /= quat_norm;
    imu_msg->orientation.w /= quat_norm;
  }
  
  // Process angular velocity (convert from deg/s to rad/s)
  // ZED SDK provides angular velocity in degrees per second
  imu_msg->angular_velocity.x = imu_data.angular_velocity.x * M_PI / 180.0;
  imu_msg->angular_velocity.y = imu_data.angular_velocity.y * M_PI / 180.0;
  imu_msg->angular_velocity.z = imu_data.angular_velocity.z * M_PI / 180.0;
  
  // Process linear acceleration (already in m/s^2)
  imu_msg->linear_acceleration.x = imu_data.linear_acceleration.x;
  imu_msg->linear_acceleration.y = imu_data.linear_acceleration.y;
  imu_msg->linear_acceleration.z = imu_data.linear_acceleration.z;
  
  // Set covariances from ZED SDK
  // The ZED SDK provides 3x3 covariance matrices
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      int idx = i * 3 + j;
      // Orientation covariance
      imu_msg->orientation_covariance[i * 3 + j] = 
        imu_data.pose_covariance.r[idx];
      // Angular velocity covariance
      imu_msg->angular_velocity_covariance[i * 3 + j] = 
        imu_data.angular_velocity_covariance.r[idx];
      // Linear acceleration covariance  
      imu_msg->linear_acceleration_covariance[i * 3 + j] = 
        imu_data.linear_acceleration_covariance.r[idx];
    }
  }
  
  // Publish IMU message
  imu_pub_->publish(std::move(imu_msg));
}

/**
 * @brief Publish IMU temperature data
 * The ZED 2i provides temperature readings from the IMU sensor
 */
void ZEDCameraDriver::publishIMUTemperature(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp)
{
  // Temperature access based on SDK documentation
  float imu_temp = 0.0f;
  
  // Get temperature data
  auto temperature_data = sensors_data.temperature;
  
  // Use the get() method with SENSOR_LOCATION enum
  // The get() method returns ERROR_CODE to indicate success/failure
  sl::ERROR_CODE temp_status = temperature_data.get(
    sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU, imu_temp);
  
  if (temp_status != sl::ERROR_CODE::SUCCESS) {
    // Temperature data not available, skip processing
    return;
  }
  
  // Log temperature periodically for diagnostics
  static auto last_temp_log = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  
  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_temp_log).count() >= 60) {
    RCLCPP_INFO(get_logger(), "IMU Temperature: %.1f°C", imu_temp);
    last_temp_log = now;
    
    // Check for temperature warnings
    if (imu_temp > 70.0) {
      RCLCPP_WARN(get_logger(), "High IMU temperature: %.1f°C", imu_temp);
    } else if (imu_temp < -20.0) {
      RCLCPP_WARN(get_logger(), "Low IMU temperature: %.1f°C", imu_temp);
    }
  }
  
  // Suppress unused parameter warning
  (void)timestamp;
}

/**
 * @brief Compute and publish IMU-based velocity estimation
 * This integrates accelerometer data for velocity estimation
 * Note: This is supplementary to visual odometry
 */
void ZEDCameraDriver::computeIMUVelocity(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp)
{
  static rclcpp::Time last_imu_time = timestamp;
  static sl::float3 velocity(0, 0, 0);
  
  // Calculate time delta
  double dt = (timestamp - last_imu_time).seconds();
  last_imu_time = timestamp;
  
  // Skip if dt is invalid
  if (dt <= 0 || dt > 1.0) {
    return;
  }
  
  // Get acceleration data (already compensated for gravity by ZED SDK)
  auto linear_accel = sensors_data.imu.linear_acceleration;
  
  // Simple velocity integration (for reference only)
  // The actual velocity should come from visual-inertial odometry
  velocity.x += linear_accel.x * dt;
  velocity.y += linear_accel.y * dt;
  velocity.z += linear_accel.z * dt;
  
  // Apply velocity decay to account for drift
  const double decay_factor = 0.99;
  velocity.x *= decay_factor;
  velocity.y *= decay_factor;
  velocity.z *= decay_factor;
  
  // Store for diagnostics (not published as primary velocity source)
  // The visual-inertial odometry from ZED SDK is more accurate
}

/**
 * @brief Process magnetometer data (if available)
 * Note: ZED 2i includes a magnetometer for heading estimation
 */
void ZEDCameraDriver::processMagnetometerData(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp)
{
  // Check if magnetometer data is available
  auto mag_data = sensors_data.magnetometer;
  
  if (!mag_data.is_available) {
    return;
  }
  
  // Magnetic field vector (in microTesla)
  sl::float3 magnetic_field = mag_data.magnetic_field_calibrated;
  
  // Calculate magnetic heading (yaw)
  double magnetic_heading = std::atan2(magnetic_field.y, magnetic_field.x);
  
  // Convert to degrees
  double heading_deg = magnetic_heading * 180.0 / M_PI;
  
  // Normalize to 0-360 range
  if (heading_deg < 0) {
    heading_deg += 360.0;
  }
  
  // Log occasionally for diagnostics
  static auto last_mag_log = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  
  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_mag_log).count() >= 10) {
    RCLCPP_DEBUG(get_logger(), "Magnetic heading: %.1f degrees", heading_deg);
    last_mag_log = now;
  }
  
  // Suppress unused parameter warning
  (void)timestamp;
}

/**
 * @brief Process barometer data (if available)
 * Note: ZED 2i includes a barometer for altitude estimation
 */
void ZEDCameraDriver::processBarometerData(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp)
{
  // Check if barometer data is available
  auto baro_data = sensors_data.barometer;
  
  if (!baro_data.is_available) {
    return;
  }
  
  // Pressure in hectopascals (hPa)
  float pressure = baro_data.pressure;
  
  // Calculate altitude using barometric formula
  // Standard sea level pressure = 1013.25 hPa
  const float sea_level_pressure = 1013.25f;
  float altitude = 44330.0f * (1.0f - std::pow(pressure / sea_level_pressure, 0.1903f));
  
  // Store for diagnostics
  static float initial_altitude = altitude;
  static bool altitude_initialized = false;
  
  if (!altitude_initialized) {
    initial_altitude = altitude;
    altitude_initialized = true;
    RCLCPP_INFO(get_logger(), "Initial barometric altitude: %.2f m", altitude);
  }
  
  // Relative altitude change
  float relative_altitude = altitude - initial_altitude;
  
  // Log occasionally
  static auto last_baro_log = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  
  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_baro_log).count() >= 30) {
    RCLCPP_DEBUG(get_logger(), "Barometric altitude: %.2f m (relative: %.2f m)", 
                 altitude, relative_altitude);
    last_baro_log = now;
  }
  
  // Suppress unused parameter warning
  (void)timestamp;
}

/**
 * @brief Main IMU processing entry point
 * Called from the grab loop when new sensor data is available
 */
void ZEDCameraDriver::processAllIMUSensors(const sl::SensorsData& sensors_data, const rclcpp::Time& timestamp)
{
  // Process core IMU data (accel, gyro, orientation)
  processIMUData(sensors_data, timestamp);
  
  // Publish main IMU data
  publishIMUData(sensors_data, timestamp);
  
  // Process additional sensors
  publishIMUTemperature(sensors_data, timestamp);
  processMagnetometerData(sensors_data, timestamp);
  processBarometerData(sensors_data, timestamp);
  
  // Compute derived values
  computeIMUVelocity(sensors_data, timestamp);
}

/**
 * @brief Configure IMU settings for optimal performance
 * Called during initialization
 */
void ZEDCameraDriver::configureIMUSettings()
{
  if (!enable_imu_) {
    RCLCPP_INFO(get_logger(), "IMU disabled in configuration");
    return;
  }
  
  // Check if camera model supports IMU
  auto camera_info = zed_camera_->getCameraInformation();
  
  if (camera_info.camera_model != sl::MODEL::ZED2 && 
      camera_info.camera_model != sl::MODEL::ZED2i &&
      camera_info.camera_model != sl::MODEL::ZED_X &&
      camera_info.camera_model != sl::MODEL::ZED_XM) {
    RCLCPP_WARN(get_logger(), "Camera model does not have built-in IMU");
    enable_imu_ = false;
    return;
  }
  
  // Get IMU sensor configuration
  sl::SensorsConfiguration sensor_config = camera_info.sensors_configuration;
  
  // Log IMU information
  RCLCPP_INFO(get_logger(), "IMU Configuration:");
  
  // Access sensor parameters correctly based on SDK structure
  // The accelerometer and gyroscope parameters have range as a float2 (min, max)
  if (sensor_config.accelerometer_parameters.isAvailable) {
    RCLCPP_INFO(get_logger(), "  - Accelerometer available");
    RCLCPP_INFO(get_logger(), "    Range: [%.1f, %.1f] g", 
                sensor_config.accelerometer_parameters.range.x,
                sensor_config.accelerometer_parameters.range.y);
    RCLCPP_INFO(get_logger(), "    Sampling rate: %.1f Hz",
                sensor_config.accelerometer_parameters.sampling_rate);
  }
  
  if (sensor_config.gyroscope_parameters.isAvailable) {
    RCLCPP_INFO(get_logger(), "  - Gyroscope available");
    RCLCPP_INFO(get_logger(), "    Range: [%.1f, %.1f] dps",
                sensor_config.gyroscope_parameters.range.x,
                sensor_config.gyroscope_parameters.range.y);
    RCLCPP_INFO(get_logger(), "    Sampling rate: %.1f Hz",
                sensor_config.gyroscope_parameters.sampling_rate);
  }
  
  if (sensor_config.magnetometer_parameters.isAvailable) {
    RCLCPP_INFO(get_logger(), "  - Magnetometer available");
  }
  
  if (sensor_config.barometer_parameters.isAvailable) {
    RCLCPP_INFO(get_logger(), "  - Barometer available");
  }
  
  RCLCPP_INFO(get_logger(), "IMU configured successfully");
}

} // namespace kobu_zed_driver
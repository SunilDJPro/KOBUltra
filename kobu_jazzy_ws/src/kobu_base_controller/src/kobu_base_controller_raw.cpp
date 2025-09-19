#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <chrono>
#include <functional>
#include <iostream>
#include <nlohmann/json.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

class KobuBaseControllerRaw : public rclcpp::Node
{
public:
  KobuBaseControllerRaw()
  : Node("kobu_base_controller_raw")
  {
    // Declare parameters
    declare_parameter("uart_device", "/dev/ttyACM0");
    declare_parameter("baud_rate", 115200);
    declare_parameter("cmd_vel_timeout", 0.5);
    declare_parameter("wheel_separation", 0.16);  // Distance between wheels in meters
    declare_parameter("max_linear_speed", 0.5);   // Maximum linear speed in m/s
    declare_parameter("max_angular_speed", 2.0);  // Maximum angular speed in rad/s
    declare_parameter("command_rate", 10);        // Command send rate in Hz

    // Get parameters
    uart_device_ = get_parameter("uart_device").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    int command_rate = get_parameter("command_rate").as_int();

    RCLCPP_INFO(get_logger(), "Using UART device: %s at %d baud", 
                uart_device_.c_str(), baud_rate_);
    RCLCPP_INFO(get_logger(), "Wheel separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(get_logger(), "Max linear speed: %.2f m/s, Max angular speed: %.2f rad/s",
                max_linear_speed_, max_angular_speed_);

    // Initialize UART
    initialize_uart();

    // Create subscription to cmd_vel
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, 
      std::bind(&KobuBaseControllerRaw::cmd_vel_callback, this, std::placeholders::_1));

    // Start command processing thread
    command_thread_ = std::thread(&KobuBaseControllerRaw::process_commands, this);

    // Start watchdog timer to stop robot if no cmd_vel received
    watchdog_timer_ = create_wall_timer(
      100ms, std::bind(&KobuBaseControllerRaw::watchdog_callback, this));

    // Start command send timer to continuously send commands to maintain motion
    command_send_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / command_rate), 
      std::bind(&KobuBaseControllerRaw::send_current_command, this));

    last_cmd_time_ = now();
    RCLCPP_INFO(get_logger(), "KobuBaseControllerRaw initialized");
  }

  ~KobuBaseControllerRaw()
  {
    // Stop command thread
    running_ = false;
    if (command_thread_.joinable()) {
      command_thread_.join();
    }

    // Send stop command before shutting down
    stop_robot();

    // Close UART
    if (uart_fd_ >= 0) {
      close(uart_fd_);
    }
  }

private:
  // Subscription for cmd_vel
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

  // UART parameters
  std::string uart_device_;
  int baud_rate_;
  int uart_fd_{-1};

  // Robot parameters
  double wheel_separation_;
  double max_linear_speed_;
  double max_angular_speed_;

  // Command processing
  std::thread command_thread_;
  std::queue<json> command_queue_;
  std::mutex queue_mutex_;
  bool running_{true};
  double left_wheel_speed_{0.0};
  double right_wheel_speed_{0.0};
  std::mutex speed_mutex_;

  // Timers
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr command_send_timer_;
  rclcpp::Time last_cmd_time_;
  double cmd_vel_timeout_;
  bool cmd_received_{false};

  void initialize_uart()
  {
    // Open UART device
    uart_fd_ = open(uart_device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open UART device: %s", uart_device_.c_str());
      return;
    }

    // Configure UART
    struct termios options;
    tcgetattr(uart_fd_, &options);

    // Set baud rate
    speed_t baud;
    switch (baud_rate_) {
      case 9600: baud = B9600; break;
      case 19200: baud = B19200; break;
      case 38400: baud = B38400; break;
      case 57600: baud = B57600; break;
      case 115200: baud = B115200; break;
      default:
        RCLCPP_ERROR(get_logger(), "Unsupported baud rate: %d", baud_rate_);
        close(uart_fd_);
        uart_fd_ = -1;
        return;
    }

    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver and set local mode
    options.c_cflag &= ~PARENB;          // No parity
    options.c_cflag &= ~CSTOPB;          // 1 stop bit
    options.c_cflag &= ~CSIZE;           // Mask character size bits
    options.c_cflag |= CS8;              // 8 data bits
    options.c_cflag &= ~CRTSCTS;         // No hardware flow control

    // Raw input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Raw output
    options.c_oflag &= ~OPOST;

    // Set the new options
    tcsetattr(uart_fd_, TCSANOW, &options);
    
    // Flush any pending input
    tcflush(uart_fd_, TCIOFLUSH);

    RCLCPP_INFO(get_logger(), "UART initialized successfully");
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Set the last command time to now
    last_cmd_time_ = now();
    cmd_received_ = true;
    
    // Check if this is a stop command (all velocities are zero)
    bool is_stop_command = (std::abs(msg->linear.x) < 0.001 && 
                           std::abs(msg->linear.y) < 0.001 && 
                           std::abs(msg->linear.z) < 0.001 &&
                           std::abs(msg->angular.x) < 0.001 && 
                           std::abs(msg->angular.y) < 0.001 && 
                           std::abs(msg->angular.z) < 0.001);
    
    if (is_stop_command) {
      // Immediately stop the robot when all velocities are zero
      RCLCPP_DEBUG(get_logger(), "Stop command received. Immediately stopping robot.");
      {
        std::lock_guard<std::mutex> lock(speed_mutex_);
        left_wheel_speed_ = 0.0;
        right_wheel_speed_ = 0.0;
      }
      
      // Send stop command directly for immediate effect
      json stop_cmd;
      stop_cmd["T"] = 1;
      stop_cmd["L"] = 0.0;
      stop_cmd["R"] = 0.0;
      send_command(stop_cmd);
      
      // Send multiple stop commands to ensure the robot stops
      // This helps overcome the base vendor's issue where it continues moving
      for (int i = 0; i < 3; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        send_command(stop_cmd);
      }
      
      return;
    }
    
    // Process normal movement commands as before
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;
    
    // Apply limits
    linear_x = std::clamp(linear_x, -max_linear_speed_, max_linear_speed_);
    angular_z = std::clamp(angular_z, -max_angular_speed_, max_angular_speed_);
    
    // Calculate wheel speeds
    double v_left = linear_x - (angular_z * wheel_separation_ / 2.0);
    double v_right = linear_x + (angular_z * wheel_separation_ / 2.0);
    
    // Normalize wheel speeds if they exceed max speed
    double max_wheel_speed = std::max(std::abs(v_left), std::abs(v_right));
    if (max_wheel_speed > max_linear_speed_) {
        double scale = max_linear_speed_ / max_wheel_speed;
        v_left *= scale;
        v_right *= scale;
    }
    
    // Convert to the range -0.5 to 0.5 for the robot's JSON command
    double left_normalized = v_left / max_linear_speed_ * 0.5;
    double right_normalized = v_right / max_linear_speed_ * 0.5;
    
    // Special handling for pure rotation (when linear velocity is near zero)
    // Use a moderate speed (0.3) for better control during rotation
    if (std::abs(linear_x) < 0.05 && std::abs(angular_z) > 0.1) {
        // Pure rotation case - use fixed speed of 0.3 instead of full 0.5
        double rotation_speed = 0.3;
        if (angular_z > 0) {
            // Turn left (counterclockwise)
            left_normalized = -rotation_speed;
            right_normalized = rotation_speed;
        } else {
            // Turn right (clockwise)
            left_normalized = rotation_speed;
            right_normalized = -rotation_speed;
        }
    }
    
    // Ensure we stay within the allowed range
    left_normalized = std::clamp(left_normalized, -0.5, 0.5);
    right_normalized = std::clamp(right_normalized, -0.5, 0.5);
    
    // Update the wheel speeds
    {
        std::lock_guard<std::mutex> lock(speed_mutex_);
        left_wheel_speed_ = left_normalized;
        right_wheel_speed_ = right_normalized;
        
        // Log the wheel speeds for debugging
        RCLCPP_DEBUG(get_logger(), 
                     "cmd_vel: linear=%.2f, angular=%.2f -> wheels: left=%.2f, right=%.2f",
                     linear_x, angular_z, left_wheel_speed_, right_wheel_speed_);
    }
  }

  void process_commands()
  {
    while (running_) {
      json cmd;
      bool has_cmd = false;
      
      // Get command from queue
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!command_queue_.empty()) {
          cmd = command_queue_.front();
          command_queue_.pop();
          has_cmd = true;
        }
      }
      
      // Process command
      if (has_cmd) {
        send_command(cmd);
      }
      
      // Sleep briefly to prevent busy waiting
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void send_command(const json& cmd)
  {
    if (uart_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "UART not initialized");
      return;
    }

    std::string cmd_str = cmd.dump() + "\n";
    ssize_t bytes_written = write(uart_fd_, cmd_str.c_str(), cmd_str.length());
    
    if (bytes_written < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to send command: %s", cmd_str.c_str());
    } else if (bytes_written != static_cast<ssize_t>(cmd_str.length())) {
      RCLCPP_WARN(get_logger(), "Incomplete command sent: %zd of %zu bytes", 
                 bytes_written, cmd_str.length());
    } else {
      RCLCPP_DEBUG(get_logger(), "Sent command: %s", cmd_str.c_str());
    }
  }

  void send_current_command()
  {
    // Check if we've received any commands yet
    if (!cmd_received_) {
      return;
    }
    
    // Get the current wheel speeds
    double left, right;
    {
      std::lock_guard<std::mutex> lock(speed_mutex_);
      left = left_wheel_speed_;
      right = right_wheel_speed_;
    }
    
    // Ensure continuous motion by sending commands regularly
    // even if the velocity values don't change
    json cmd;
    cmd["T"] = 1;          // Raw speed control command
    cmd["L"] = left;       // Left wheel speed
    cmd["R"] = right;      // Right wheel speed
    
    // Send the command directly to ensure immediate action
    // This is crucial for maintaining continuous rotation
    send_command(cmd);
    
    // Log periodically (not every command to avoid flooding the logs)
    static int log_counter = 0;
    if (++log_counter % 30 == 0) {  // Log approximately every 3 seconds at 10Hz
      RCLCPP_DEBUG(get_logger(), "Continuous command: L=%.2f, R=%.2f", left, right);
    }
  }

  void watchdog_callback()
  {
    // Check if we haven't received cmd_vel messages for a while
    auto elapsed = (now() - last_cmd_time_).seconds();
    if (elapsed > cmd_vel_timeout_ && cmd_received_) {
      stop_robot();
      cmd_received_ = false;  // Reset the flag until we receive a new command
    }
  }

  void stop_robot()
  {
    // Update wheel speeds to zero
    {
      std::lock_guard<std::mutex> lock(speed_mutex_);
      left_wheel_speed_ = 0.0;
      right_wheel_speed_ = 0.0;
    }
    
    // Create stop command
    json stop_cmd;
    stop_cmd["T"] = 1;    // Raw speed control command
    stop_cmd["L"] = 0.0;  // Zero left wheel speed
    stop_cmd["R"] = 0.0;  // Zero right wheel speed
    
    // Send multiple stop commands to ensure the robot stops
    for (int i = 0; i < 3; i++) {
      send_command(stop_cmd);
      if (i < 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    
    RCLCPP_DEBUG(get_logger(), "Sent multiple stop commands");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KobuBaseControllerRaw>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
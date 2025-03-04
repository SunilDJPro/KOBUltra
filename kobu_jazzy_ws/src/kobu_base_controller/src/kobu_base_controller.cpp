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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

class KobuBaseController : public rclcpp::Node
{
public:
  KobuBaseController()
  : Node("kobu_base_controller")
  {
    // Declare parameters
    declare_parameter("uart_device", "/dev/ttyUSB0");
    declare_parameter("baud_rate", 115200);
    declare_parameter("cmd_vel_timeout", 0.5);

    // Get parameters
    uart_device_ = get_parameter("uart_device").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();

    RCLCPP_INFO(get_logger(), "Using UART device: %s at %d baud", 
                uart_device_.c_str(), baud_rate_);

    // Initialize UART
    initialize_uart();

    // Create subscription to cmd_vel
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, 
      std::bind(&KobuBaseController::cmd_vel_callback, this, std::placeholders::_1));

    // Start command processing thread
    command_thread_ = std::thread(&KobuBaseController::process_commands, this);

    // Start watchdog timer to stop robot if no cmd_vel received
    watchdog_timer_ = create_wall_timer(
      100ms, std::bind(&KobuBaseController::watchdog_callback, this));

    last_cmd_time_ = now();
    RCLCPP_INFO(get_logger(), "KobuBaseController initialized");
  }

  ~KobuBaseController()
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

  // Command processing
  std::thread command_thread_;
  std::queue<json> command_queue_;
  std::mutex queue_mutex_;
  bool running_{true};

  // Watchdog
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_time_;
  double cmd_vel_timeout_;

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
    last_cmd_time_ = now();
    
    // Create JSON command for ROS control (T:13)
    json cmd;
    cmd["T"] = 13;          // ROS control command
    cmd["X"] = msg->linear.x;  // Linear velocity
    cmd["Z"] = msg->angular.z; // Angular velocity

    // Queue the command
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      command_queue_.push(cmd);
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

  void watchdog_callback()
  {
    // Check if we haven't received cmd_vel messages for a while
    auto elapsed = (now() - last_cmd_time_).seconds();
    if (elapsed > cmd_vel_timeout_) {
      stop_robot();
    }
  }

  void stop_robot()
  {
    // Create stop command
    json stop_cmd;
    stop_cmd["T"] = 13;  // ROS control command
    stop_cmd["X"] = 0.0; // Zero linear velocity
    stop_cmd["Z"] = 0.0; // Zero angular velocity
    
    // Send directly, bypassing the queue to ensure it's sent immediately
    send_command(stop_cmd);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KobuBaseController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
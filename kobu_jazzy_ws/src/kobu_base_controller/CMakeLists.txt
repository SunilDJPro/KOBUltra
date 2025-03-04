cmake_minimum_required(VERSION 3.8)
project(kobu_base_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find nlohmann_json package
include(FindPkgConfig)
pkg_check_modules(NLOHMANN_JSON REQUIRED nlohmann_json>=3.0.0)

# Add include directories
include_directories(include ${NLOHMANN_JSON_INCLUDE_DIRS})

# Add executables
add_executable(kobu_base_controller src/kobu_base_controller.cpp)
target_link_libraries(kobu_base_controller ${NLOHMANN_JSON_LIBRARIES})

add_executable(kobu_base_controller_raw src/kobu_base_controller_raw.cpp)
target_link_libraries(kobu_base_controller_raw ${NLOHMANN_JSON_LIBRARIES})

# Specify dependencies
ament_target_dependencies(kobu_base_controller
  rclcpp
  geometry_msgs
)

ament_target_dependencies(kobu_base_controller_raw
  rclcpp
  geometry_msgs
)

# Install executables
install(TARGETS
  kobu_base_controller
  kobu_base_controller_raw
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Required for ament
ament_package()
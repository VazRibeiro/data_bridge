cmake_minimum_required(VERSION 3.8)
project(data_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Build C++ Protocol Buffer bridge
add_executable(proto_bridge src/proto_bridge.cpp)
ament_target_dependencies(proto_bridge 
  rclcpp can_msgs diagnostic_msgs std_msgs)

# Link required libraries
target_link_libraries(proto_bridge pthread)

install(TARGETS proto_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install proto and launch files
install(DIRECTORY
  proto
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

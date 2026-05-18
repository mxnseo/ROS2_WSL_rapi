cmake_minimum_required(VERSION 3.8)
project(launch_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#build
add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp std_msgs)
add_executable(sub src/sub.cpp)
ament_target_dependencies(sub rclcpp std_msgs)

# Install
install(TARGETS
pub
sub
DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)
ament_package()

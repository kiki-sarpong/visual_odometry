cmake_minimum_required(VERSION 3.8)
project(vio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(image_transport REQUIRED)

include_directories(
  include
  include/vio
  ${OpenCV_INCLUDE_DIRS}
)
# target_include_directories() 


add_executable(visual_odometry 
src/main.cpp
src/LoadData.cpp
src/VisualOdometry.cpp
src/VisualOdometryPublisher.cpp
src/GroundTruthPublisher.cpp
)

add_executable(test_cpp src/test_cpp.cpp)

# First entry is the executable, the rest are dependencies
ament_target_dependencies(
visual_odometry
rclcpp
std_msgs
geometry_msgs
OpenCV
Eigen3
)

ament_target_dependencies(test_cpp rclcpp std_msgs)
# Include executable targets
install(TARGETS
visual_odometry
test_cpp
DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(
 DIRECTORY include/
 DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

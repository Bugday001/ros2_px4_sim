cmake_minimum_required(VERSION 3.8)
project(px4_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)    
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
include_directories(${EIGEN3_INCLUDE_DIR})
# cpp
add_executable(offboard src/offboard.cpp)
ament_target_dependencies(
  offboard
  std_msgs
  mavros_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  Eigen3
)
target_include_directories(offboard PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/px4_offboard)
# target_include_directories(offboard PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(offboard ${NLopt_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/libnlopt.so.0.9.0)

add_executable(rotationLidar_tool src/rotationLidar_tool.cpp)
ament_target_dependencies(
  rotationLidar_tool
  std_msgs
  mavros_msgs
  sensor_msgs
  tf2
  tf2_ros
  geometry_msgs
  rclcpp
  Eigen3
)
target_include_directories(rotationLidar_tool PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/px4_offboard)

install(TARGETS offboard rotationLidar_tool
  DESTINATION lib/${PROJECT_NAME})

#python

ament_python_install_package(${PROJECT_NAME})
 
# Install Python executables
install(PROGRAMS
  scripts/sendPath.py
  DESTINATION lib/${PROJECT_NAME}
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

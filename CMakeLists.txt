cmake_minimum_required(VERSION 3.8)
project(px4_rc_to_joy2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)




# executable rc_to_joy
add_executable(rc_to_joy src/rc_to_joy.cpp)
target_include_directories(rc_to_joy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rc_to_joy PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  rc_to_joy
  "rclcpp"
  "builtin_interfaces"
  "std_msgs"
  "px4_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "Eigen3"
)
install(TARGETS rc_to_joy
  DESTINATION lib/${PROJECT_NAME})

# end executable rc_to_joy




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

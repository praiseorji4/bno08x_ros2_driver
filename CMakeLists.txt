cmake_minimum_required(VERSION 3.8)
project(bno08x_driver)

# Check for compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall 
    -Wextra 
    -Wpedantic
    -Wno-sign-compare
    -Wno-unused-parameter
    -Wno-unused-variable
    )
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-old-style-declaration")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# The interface target must be named after the package so ROS can load its typesupport at
# runtime, so the executable target below is named bno08x_driver_node instead.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ReportInfo.msg"
  "msg/Report.msg"
  "msg/RawSensor.msg"
  "msg/GyroUncalibrated.msg"
  "msg/MagneticFieldUncalibrated.msg"
  "msg/Orientation.msg"
  "msg/StabilityClassification.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add include directories for main driver
include_directories(
  include
)

# Add the sh2 directory to build separately
add_subdirectory(include/sh2)

set(SOURCES
  src/bno08x.cpp
  src/bno08x_ros.cpp
  src/ros_node.cpp
)

# Create executable for the driver
add_executable(bno08x_driver_node ${SOURCES})
# Keep the installed executable name, so launch files still use executable='bno08x_driver'
set_target_properties(bno08x_driver_node PROPERTIES OUTPUT_NAME bno08x_driver)

# Link include directories to the target
target_include_directories(bno08x_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set the required C and C++ standards
target_compile_features(bno08x_driver_node PUBLIC cxx_std_17)

# Link against sh2 library
target_link_libraries(bno08x_driver_node
  sh2_library
  "${cpp_typesupport_target}"
)

ament_target_dependencies(
  bno08x_driver_node
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "geometry_msgs"
)

# Install the executable and the config files
install(TARGETS bno08x_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gmock(test_i2c_mock test/i2c_interface_mock_test.cpp)
  if(TARGET test_i2c_mock)
    target_include_directories(test_i2c_mock PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()

  ament_add_gmock(test_watchdog test/watchdog_test.cpp)
  if(TARGET test_watchdog)
    target_include_directories(test_watchdog PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()

  ament_add_gtest(test_sample_age test/sample_age_test.cpp)
  if(TARGET test_sample_age)
    target_include_directories(test_sample_age PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_sample_age sh2_library)
  endif()

  # Linter checks
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

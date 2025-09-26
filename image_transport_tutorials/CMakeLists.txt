cmake_minimum_required(VERSION 3.5)
project(image_transport_tutorials)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom message
set(msg_files
  "msg/ResizedImage.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Publisher example
add_executable(my_publisher src/my_publisher.cpp)
ament_target_dependencies(my_publisher rclcpp sensor_msgs std_msgs cv_bridge image_transport)
target_link_libraries(my_publisher ${OpenCV_LIBRARIES})
target_include_directories(my_publisher PRIVATE ${cv_bridge_INCLUDE_DIRS})

# Subscriber example
add_executable(my_subscriber src/my_subscriber.cpp)
ament_target_dependencies(my_subscriber rclcpp sensor_msgs std_msgs cv_bridge image_transport)
target_link_libraries(my_subscriber ${OpenCV_LIBRARIES})
target_include_directories(my_subscriber PRIVATE ${cv_bridge_INCLUDE_DIRS})

# Plugin example
add_library(resized_plugins SHARED
  src/manifest.cpp
  src/resized_publisher.cpp
  src/resized_subscriber.cpp
)
ament_target_dependencies(resized_plugins
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  pluginlib
)
target_link_libraries(resized_plugins
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
  cv_bridge::cv_bridge
  image_transport::image_transport
  pluginlib::pluginlib
  rclcpp::rclcpp
)

# Publisher from video example
add_executable(publisher_from_video src/publisher_from_video.cpp)
ament_target_dependencies(publisher_from_video rclcpp sensor_msgs std_msgs cv_bridge image_transport)
target_link_libraries(publisher_from_video ${OpenCV_LIBRARIES})
target_include_directories(publisher_from_video PRIVATE ${cv_bridge_INCLUDE_DIRS})

# Install plugin description (point to image_transport, not this package)
pluginlib_export_plugin_description_file(image_transport resized_plugins.xml)

# Install targets
install(
  TARGETS my_publisher my_subscriber resized_plugins publisher_from_video
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install includes
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

include_directories(include)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  pluginlib
  rosidl_default_runtime
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


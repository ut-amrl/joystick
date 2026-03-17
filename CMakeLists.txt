cmake_minimum_required(VERSION 3.10)
project(joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(config_reader REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(amrl_shared_lib REQUIRED)

add_executable(joystick_node
  joystick/src/joystick.cc
  joystick/src/joystick_driver.cc
)

target_include_directories(joystick_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(joystick_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  config_reader
  amrl_shared_lib
)

target_link_libraries(joystick_node
  gflags
  glog::glog
)

install(TARGETS joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

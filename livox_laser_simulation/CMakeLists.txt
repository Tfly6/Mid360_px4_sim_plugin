cmake_minimum_required(VERSION 3.16)
project(livox_laser_simulation)

# Gazebo Harmonic on Ubuntu 22.04 still ships a jsoncpp config written for
# CMake 3.0. CMake 4 rejects that config unless this compatibility floor is
# selected before gz-sim8 resolves its transitive dependencies.
if(CMAKE_VERSION VERSION_GREATER_EQUAL 4.0)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

add_library(livox_laser_simulation_gz SHARED
  src/livox_gz_system.cpp
)
target_compile_features(livox_laser_simulation_gz PUBLIC cxx_std_17)
target_include_directories(livox_laser_simulation_gz PRIVATE include)
ament_target_dependencies(livox_laser_simulation_gz
  geometry_msgs rclcpp sensor_msgs std_msgs
)
target_link_libraries(livox_laser_simulation_gz
  gz-common5::gz-common5
  gz-msgs10::gz-msgs10
  gz-plugin2::core
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(livox_laser_simulation_gz ${cpp_typesupport_target})

install(TARGETS livox_laser_simulation_gz
  LIBRARY DESTINATION lib
)
install(DIRECTORY models worlds rviz launch px4
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

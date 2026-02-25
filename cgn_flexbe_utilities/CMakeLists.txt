cmake_minimum_required(VERSION 3.8)
project(cgn_flexbe_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(gpd_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate services (⚠️ gpd_ros REMOVED from DEPENDENCIES)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  DEPENDENCIES
    sensor_msgs
    pcl_msgs
    geometry_msgs
    std_msgs
)

include_directories(
  ${PCL_INCLUDE_DIRS}
)


add_executable(move_to_pose_service src/move_to_pose_service.cpp)
ament_target_dependencies(move_to_pose_service
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)


add_executable(reach_to_grasp_service src/reach_to_grasp_service.cpp)
ament_target_dependencies(reach_to_grasp_service
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_geometry_msgs
  std_srvs
)
target_link_libraries(reach_to_grasp_service ${PCL_LIBRARIES})

install(TARGETS
  move_to_pose_service
  reach_to_grasp_service
  DESTINATION lib/${PROJECT_NAME}
)

# Link generated service headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(move_to_pose_service ${cpp_typesupport_target})
target_link_libraries(reach_to_grasp_service ${cpp_typesupport_target})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

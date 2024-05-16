cmake_minimum_required(VERSION 3.8)
project(image_pros_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# added dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_pros_2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/HoleCenter.srv"
  "msg/HoleCenterMsg2.msg"
  DEPENDENCIES 
)



include_directories(
	      include
              ${catkin_INCLUDE_DIRS}
              ${PCL_INCLUDE_DIRS}
              ${image_pros_2_INCLUDE_DIRS}
        )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable(pointcloud_2Dto3D src/pointcloud_2Dto3D.cpp)
#ament_target_dependencies(pointcloud_2Dto3D rclcpp sensor_msgs pcl_ros image_pros_2)
#target_link_libraries(pointcloud_2Dto3D ${PCL_LIBRARIES} )

#add_executable(pc_2Dto3D src/pc_2Dto3D.cpp)
#ament_target_dependencies(pc_2Dto3D rclcpp sensor_msgs pcl_ros image_pros_2)
#target_link_libraries(pc_2Dto3D ${PCL_LIBRARIES} )

#add_executable(service_caller src/service_caller.cpp)
#ament_target_dependencies(service_caller rclcpp image_pros)
#target_link_libraries(service_caller ${PCL_LIBRARIES} )

#add_executable(pc_thread src/pc_thread.cpp)
#ament_target_dependencies(pc_thread rclcpp sensor_msgs pcl_ros image_pros)
#target_link_libraries(pc_thread ${PCL_LIBRARIES} )

add_executable(pc_topic src/pc_topic.cpp)
ament_target_dependencies(pc_topic rclcpp sensor_msgs pcl_ros image_pros_2)
target_link_libraries(pc_topic ${PCL_LIBRARIES} )

install(TARGETS  
  #pointcloud_2Dto3D
  #pc_2Dto3D
 # service_caller
 # pc_thread
  pc_topic
  DESTINATION lib/${PROJECT_NAME})



install(PROGRAMS
  scripts/call_capture.py
  scripts/circle_2D_detection.py
  scripts/circle_2D_topic.py
  scripts/simple_traj_planner.py
  scripts/new_hole_detect.py
  scripts/ros1_capture.py
  scripts/ros1_capture_simple.py
  scripts/call_capture_simple.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

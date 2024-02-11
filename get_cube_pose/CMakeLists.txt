cmake_minimum_required(VERSION 3.8)
project(get_cube_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

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

#include
include_directories(include)

############ get cube pose client ########
add_executable(get_pose_client
    src/get_pose_client.cpp)
target_include_directories(get_pose_client
    PUBLIC include)
ament_target_dependencies(get_pose_client
    ament_cmake
	rclcpp
	rclcpp_action
	grasping_msgs)

# Install the executable
install(TARGETS 
	get_pose_client
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
	launch
	rviz_config
    DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

######### Make Library #########
add_library(get_cube_pose SHARED
  src/get_pose_client.cpp
)
ament_target_dependencies(get_cube_pose
    ament_cmake
	rclcpp
	rclcpp_action
	grasping_msgs)


install(TARGETS get_cube_pose
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(get_cube_pose)

######### End of Make Library #########

ament_package()

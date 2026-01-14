cmake_minimum_required(VERSION 3.8)
project(piper_moves)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Generate the executable
add_executable(test_script
    src/test_script.cpp)
target_include_directories(test_script
    PUBLIC include)
ament_target_dependencies(test_script
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)


add_executable(X_draw_trajectory
    src/X_draw_trajectory.cpp)
target_include_directories(X_draw_trajectory
    PUBLIC include)
ament_target_dependencies(X_draw_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_ros_planning_interface
    interactive_markers
    control_msgs
	moveit_visual_tools)


add_executable(buffer
    src/buffer.cpp)
target_include_directories(buffer
    PUBLIC include)
ament_target_dependencies(buffer
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_ros_planning_interface
    interactive_markers
    control_msgs)

# Install the executable
install(TARGETS 
    test_script
	X_draw_trajectory
	buffer
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()

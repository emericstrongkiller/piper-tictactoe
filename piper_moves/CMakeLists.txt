cmake_minimum_required(VERSION 3.8)
project(piper_moves)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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
add_executable(X_draw_trajectory
    src/X_draw_trajectory.cpp)
target_include_directories(X_draw_trajectory
    PUBLIC include)
ament_target_dependencies(X_draw_trajectory
    ament_cmake
    rclcpp
    moveit_ros_planning_interface
	moveit_visual_tools)


add_executable(piper_move_orders
    src/piper_move_orders.cpp)
target_include_directories(piper_move_orders
    PUBLIC include)
ament_target_dependencies(piper_move_orders
    ament_cmake
    rclcpp
	std_msgs)

add_executable(take_pic
    src/take_pic.cpp)
target_include_directories(take_pic
    PUBLIC include)
ament_target_dependencies(take_pic
    ament_cmake
    rclcpp
	sensor_msgs
	cv_bridge
	OpenCV
	std_msgs)

add_executable(min_max_TicTacToe
    src/min_max_TicTacToe.cpp)
target_include_directories(min_max_TicTacToe
    PUBLIC include)
ament_target_dependencies(min_max_TicTacToe
    ament_cmake
    rclcpp
	std_msgs)

# Install the executable
install(TARGETS 
	X_draw_trajectory
	piper_move_orders
	take_pic
	min_max_TicTacToe
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	templates
	DESTINATION share/${PROJECT_NAME}
)

ament_package()

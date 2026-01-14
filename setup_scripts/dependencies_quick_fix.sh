sudo rm -rf /opt/ros/humble/include/geometry_msgs/
sudo rm -rf /opt/ros/humble/include/tf2/
sudo rm -rf /opt/ros/humble/include/tf2_ros

sudo cp -r ~/ros2_ws/src/piper_tictactoe_project/setup_scripts/geometry_msgs /opt/ros/humble/include/
sudo cp -r ~/ros2_ws/src/piper_tictactoe_project/setup_scripts/tf2 /opt/ros/humble/include/
sudo cp -r ~/ros2_ws/src/piper_tictactoe_project/setup_scripts/tf2_ros /opt/ros/humble/include/
. /home/user/ros2_ws/src/piper_tictactoe_project/setup_scripts/dependencies_quick_fix.sh
colcon build --packages-select piper_gazebo piper_with_gripper_moveit

ros2 launch piper_gazebo piper_gazebo.launch.py & 
sleep 20
ros2 launch piper_with_gripper_moveit move_group_sim.launch.py &
sleep 15
ros2 launch piper_with_gripper_moveit moveit_rviz.launch.py &
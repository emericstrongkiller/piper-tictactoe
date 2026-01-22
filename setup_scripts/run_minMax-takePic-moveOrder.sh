cd ~/ros2_ws
colcon build --packages-select piper_moves
ros2 run piper_moves min_max_TicTacToe &
ros2 run piper_moves piper_move_orders &
ros2 run piper_moves take_pic &
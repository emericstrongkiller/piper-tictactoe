# Piper TicTacToe

A ROS2 project that lets the Piper robotic arm play Tic Tac Toe against humans using computer vision and AI.

## Overview

The robot uses a minimax algorithm to make optimal moves, OpenCV for board detection, and MoveIt for trajectory planning. It can draw X's and O's on a physical or simulated board.

![project_main_picture](images/project_main_picture.png)

## Components

- **X_draw_trajectory**: Motion planning with MoveIt to draw shapes on the board
- **min_max_TicTacToe**: AI decision-making (minimax algorithm)
- **take_pic**: Camera perception with OpenCV for board state recognition
- **piper_move_orders**: Game orchestration node (connects everything)

## Setup

### Requirements
- ROS2 (Humble or later)
- MoveIt
- OpenCV
- Piper robot (real or Gazebo sim)

### Quick Start

**Simulation:**
```bash
./setup_scripts/simulation/all_sim_launch.sh
```

**Real Robot:**
```bash
./setup_scripts/real_robot/real-robot_bringup.sh
```

Then run the game:
```bash
ros2 launch piper_moves joint_space_trajectory.launch.py
ros2 run piper_moves min_max_TicTacToe &
ros2 run piper_moves piper_move_orders &
ros2 run piper_moves take_pic &
```

## Web Interface

There's a simple web dashboard in `webpage_ws/` for controlling the game.

![web_interface](images/web_interface.png)

## System Architecture

![system_diagram](images/system_diagram.png)

## How It Works

1. Camera takes a picture of the board
2. Minimax AI calculates the best move
3. Robot draws the selected shape (X or O)
4. Repeat until someone wins

## Notes

- Camera calibration needed for real robot (see `real_robot_calibration/`)
- Adjust grid positions in `piper_move_orders.cpp` for different board sizes
- Some parts still need cleanup (sorry about that)

## License

MIT
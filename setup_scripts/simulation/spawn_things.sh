cd ~/ros2_ws/src/piper_ros/models
ros2 run gazebo_ros spawn_entity.py -entity grid -file grid/model.sdf -x 0.2 -y 0.02 -z 0.02
ros2 run gazebo_ros spawn_entity.py -entity cross0 -file cross/model.sdf -x 0.245 -y 0.065 -z 0.02
ros2 run gazebo_ros spawn_entity.py -entity circle1 -file circle/model.sdf -x 0.200 -y 0.065 -z 0.02
ros2 run gazebo_ros spawn_entity.py -entity cross2 -file cross/model.sdf -x 0.155 -y 0.065 -z 0.02
ros2 run gazebo_ros spawn_entity.py -entity circle3 -file circle/model.sdf -x 0.245 -y 0.020 -z 0.02
ros2 run gazebo_ros spawn_entity.py -entity circle4 -file circle/model.sdf -x 0.200 -y 0.020 -z 0.02
ros2 run gazebo_ros spawn_entity.py -entity cross5 -file cross/model.sdf -x 0.155 -y 0.020 -z 0.02
ros2 run gazebo_ros spawn_entity.py -entity circle6 -file circle/model.sdf -x 0.245 -y -0.025 -z 0.02
ros2 run gazebo_ros spawn_entity.py -entity cross7 -file cross/model.sdf -x 0.200 -y -0.025 -z 0.02
ros2 run gazebo_ros spawn_entity.py -entity circle8 -file circle/model.sdf -x 0.155 -y -0.025 -z 0.02
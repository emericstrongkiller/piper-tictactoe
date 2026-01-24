cd ~/ros2_ws
ros2 launch piper_with_gripper_moveit move_group.launch.py &
sleep 5
ros2 launch piper_with_gripper_moveit moveit_rviz.launch.py &

# Camera setup
# TF because its not connected to the TF tree by default
# Also Camera recos:
# - In the RViz Image/Camera display, increase Queue Size (e.g. 10 → 50).
# - If this is over Zenoh/WiFi, frames can arrive bursty. Prefer /compressed image in RViz:
ros2 run tf2_ros static_transform_publisher \
  0 0 0.5 0 0 0 base_link D435_link &
# then Zenoh (Although it seems to work without it in rviz ?..)
cd ~/ros2_ws/src/zenoh-pointcloud/
./install_zenoh.sh
cd ~/ros2_ws/src/zenoh-pointcloud/init
./zenoh_pointcloud_rosject.sh &
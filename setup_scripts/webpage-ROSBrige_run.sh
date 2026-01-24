cd ~/ros2_ws/src/piper_tictactoe_project/webpage_ws
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run web_video_server web_video_server --ros-args -p port:=11315 &
python3 -m http.server 7000 &

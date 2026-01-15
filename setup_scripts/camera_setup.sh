ros2 run tf2_ros static_transform_publisher \
  0 0 0.5 0 0 0 base_link D435_color_optical_frame

- In the RViz Image/Camera display, increase Queue Size (e.g. 10 → 50).
- If this is over Zenoh/WiFi, frames can arrive bursty. Prefer /compressed image in RViz:
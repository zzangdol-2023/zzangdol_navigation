# zzangdol_navigation
## zzangdol-ai-car | ROS navigation execution package
Last update : 230701
## 1. Quick start

Below codes will execute cmd_vel_converter node, rosserial node,
```bash
roslaunch zzangdol_navigation move_base_test.launch   
```

## 2. Package Explain

#### src/current_pose_publisher.py
It publishes current pose of mobile robots.   
Current pose of mobile robots, which is local coordinate, will be mapped to global coordinate.
current_pose_publisher node will be executed by move_base_test.launch.




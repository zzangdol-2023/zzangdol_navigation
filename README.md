# zzangdol_navigation
## zzangdol-ai-car | ROS navigation execution package

## 1. Quick start

Below codes will execute cmd_vel_converter node, rosserial node,
```bash
roslaunch zzangdol_navigation move_base_test.launch   
```

## 2. Package Explain

#### src/current_pose_publisher.py
it publishes current pose of robots. use to get goal trajectory. move_base_test.launch executes it.




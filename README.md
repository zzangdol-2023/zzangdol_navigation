# zzangdol_navigation
## zzangdol-ai-car | ROS navigation execution package

## 1. Quick start

Below codes will execute cmd_vel_converter node, rosserial node,
```bash
roslaunch 
```

## 2. Package Explain

Other launch files can execute with below commands

#### src / cmd_vel_converter.cpp

- converter node which converts cmd_vel to cmd_vel_converted topic
- cmd_vel converted topic is specific control value which is compatible with zzangdol-ai-car motor driver.


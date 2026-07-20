# static_avoid 与 hybrid_avoid 实车对比流程

两次测试必须使用同一地图、同一CSV、同一起点、相同障碍位置、相同 `target_speed=0.05` 和相同合法边界宽度。任何时刻只能运行一个避障节点。

## 公共基础节点

保持以下节点运行：Bunker底盘、MID360、S-FAST-LIO重定位。停止键盘遥控和其他速度发布者。

```bash
rostopic hz /Odometry
rostopic hz /cloud_registered_body
rostopic hz /livox/lidar
rostopic info /smoother_cmd_vel
```

## 方案A：原参考线/横向栅格方案

```bash
cd ~/fastlio_ws
source devel/setup.bash
source ~/livox_ws/devel/setup.bash --extend
roslaunch static_avoid static_obstacle_dynamic_test.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/final_route.csv \
  world_frame:=camera_init \
  target_speed:=0.05 \
  planning_horizon:=5.0 \
  lane_half_width:=0.0 \
  enabled:=false
```

```bash
rostopic pub -1 /static_avoid/enable std_msgs/Bool "data: true"
```

结束后停车并完全退出launch：

```bash
rostopic pub -1 /static_avoid/enable std_msgs/Bool "data: false"
```

## 方案B：滚动代价地图/A*混合方案

```bash
cd ~/fastlio_ws
source devel/setup.bash
source ~/livox_ws/devel/setup.bash --extend
roslaunch hybrid_avoid hybrid_hardware_test.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/final_route.csv \
  world_frame:=camera_init \
  target_speed:=0.05 \
  local_goal_distance:=4.5 \
  lane_half_width:=0.0 \
  enabled:=false
```

```bash
rostopic pub -1 /hybrid_avoid/enable std_msgs/Bool "data: true"
```

结束后：

```bash
rostopic pub -1 /hybrid_avoid/enable std_msgs/Bool "data: false"
```

## 建议记录项目

| 项目 | static_avoid | hybrid_avoid |
|---|---:|---:|
| 是否一次通过 |  |  |
| 避障区耗时 |  |  |
| 最大横向偏移 |  |  |
| 距离障碍最小间距 |  |  |
| 是否发生停车 |  |  |
| 是否左右摆动 |  |  |
| 回原轨迹是否平滑 |  |  |
| CPU占用 |  |  |

测试顺序：无障碍、单泡沫箱、两个交错泡沫箱、三个以上随机障碍、左右完全封死、远程横移软障碍。不要使用人员作为第一轮移动障碍。

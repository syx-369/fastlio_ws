# Bunker Mini 从建图、录点到分区避障的完整流程

本文适用于：Bunker Mini、MID360、S-FAST-LIO 重定位、`static_avoid` 分区避障。

## 0. 编译避障包

```bash
cd ~/fastlio_ws
env PYTHONNOUSERSITE=1 catkin_make \
  -DCATKIN_WHITELIST_PACKAGES='static_avoid;delivery_final'
source devel/setup.bash
```

## 1. 建图前备份旧地图

S-FAST-LIO 重定位代码固定读取：

```text
~/fastlio_ws/src/S-FAST_LIO/PCD/GlobalMap_ikdtree.pcd
```

建新图前先备份：

```bash
MAP_TAG=$(date +%Y%m%d_%H%M%S)
mkdir -p ~/fastlio_ws/map_backup/${MAP_TAG}_before
cp -a ~/fastlio_ws/src/S-FAST_LIO/PCD/GlobalMap*.pcd \
  ~/fastlio_ws/map_backup/${MAP_TAG}_before/
```

## 2. 建图终端

### 终端 1：ROS master

```bash
roscore
```

### 终端 2：Bunker CAN

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
timeout 3 candump can0
```

### 终端 3：Bunker 驱动

```bash
cd ~/bunker_ws
source devel/setup.bash
roslaunch bunker_bringup bunker_robot_base.launch
```

### 终端 4：键盘遥控

```bash
cd ~/bunker_ws
source devel/setup.bash
roslaunch bunker_bringup bunker_teleop_keyboard.launch
```

### 终端 5：MID360

```bash
cd ~/livox_ws
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

### 终端 6：S-FAST-LIO 建图

```bash
cd ~/fastlio_ws
source devel/setup.bash
source ~/livox_ws/devel/setup.bash --extend
roslaunch sfast_lio mapping_mid360.launch rviz:=true
```

缓慢、闭环地开完整个比赛区域；起点附近多观察一会，回到起点后再停止。建图终端按 `Ctrl+C` 时会生成地图。

检查并备份新地图：

```bash
ls -lh ~/fastlio_ws/src/S-FAST_LIO/PCD/GlobalMap.pcd \
       ~/fastlio_ws/src/S-FAST_LIO/PCD/GlobalMap_ikdtree.pcd

MAP_TAG=$(date +%Y%m%d_%H%M%S)
mkdir -p ~/fastlio_ws/map_backup/${MAP_TAG}_final
cp -a ~/fastlio_ws/src/S-FAST_LIO/PCD/GlobalMap*.pcd \
  ~/fastlio_ws/map_backup/${MAP_TAG}_final/
```

## 3. 基于新地图录制比赛轨迹

保持 Bunker、遥控和 MID360 终端运行，停止建图节点，然后启动重定位。

### 重定位终端

```bash
cd ~/fastlio_ws
source devel/setup.bash
source ~/livox_ws/devel/setup.bash --extend
roslaunch sfast_lio mapping_mid360_relocalization.launch rviz:=true
```

确认地图与实时点云重合，并检查：

```bash
rostopic hz /Odometry
rostopic echo -n 1 /Odometry/header
```

### 轨迹录制终端

修改 `final_route.csv` 为本次轨迹名：

```bash
cd ~/fastlio_ws
source devel/setup.bash
rosrun waypoint_tools record_waypoints.py \
  _file_name:=final_route.csv \
  _output_dir:=/home/user/fastlio_ws/src/waypoint_tools/data \
  _min_distance:=0.15 \
  _min_yaw_change:=0.17 \
  _default_tol:=0.25 \
  _frame_id:=camera_init
```

使用终端 4 低速遥控，普通航点会自动写入。

### 标记避障区入口

车辆停在避障区入口之前。建议入口点位于第一个可能障碍位置前至少 2m：

```bash
rostopic pub -1 /waypoint_task std_msgs/String "data: 'avoid_start'"
```

### 标记避障区出口

车辆通过整个随机障碍区域，并在最后一个可能障碍位置后至少 2m 停车：

```bash
rostopic pub -1 /waypoint_task std_msgs/String "data: 'avoid_end'"
```

继续录制剩余路线，结束后在轨迹录制终端按 `Ctrl+C`。

检查任务点：

```bash
grep -n 'avoid_start\|avoid_end' \
  ~/fastlio_ws/src/waypoint_tools/data/final_route.csv

source ~/fastlio_ws/devel/setup.bash
rosrun static_avoid validate_avoid_route.py \
  ~/fastlio_ws/src/waypoint_tools/data/final_route.csv
```

验证器必须输出至少一个完整区间，并且 frame 为 `camera_init`。

### CSV 轨迹与未来 5m 占用检查

CSV 中的 `x/y` 是参考轨迹锚点，不是只在这些离散点上做碰撞判断。节点先对相邻录制点做短窗口平滑，再把它们连接成按累计弧长 `s` 参数化的连续折线。每次规划会：

1. 把当前车体中心投影到连续参考轨迹，得到当前进度 `s`；
2. 检查从当前 `s` 到 `s + planning_horizon`，默认未来 5m；
3. 按 `path_sample_step=0.10m` 在线性插值后的轨迹上重新采样位置和切线朝向；
4. 在每个采样位姿放置带安全余量的 `0.8 x 0.7m` Bunker 矩形车身；
5. 只要障碍点落入任一车身矩形，便把相应弧长区间判定为原轨迹被占用。

`planning_horizon=5m` 只限制向前搜索阻塞的距离。如果障碍位于窗口末端，局部绕行轨迹允许超过 5m，以保留绕过障碍后平滑回到原轨迹所需的距离。车辆继续前进后会滚动执行同样检查，因此无需提前知道整个区域的障碍数量。

## 4. 软件仿真

每次仿真完成后按 `Ctrl+C`，再启动下一项。

单障碍：

```bash
source ~/fastlio_ws/devel/setup.bash
roslaunch static_avoid static_obstacle_sim.launch
```

两个交错静态障碍：

```bash
source ~/fastlio_ws/devel/setup.bash
roslaunch static_avoid multi_obstacle_sim.launch
```

单次横穿移动障碍：

```bash
source ~/fastlio_ws/devel/setup.bash
roslaunch static_avoid moving_obstacle_sim.launch
```

状态监控：

```bash
rostopic echo /static_avoid/state
```

## 5. 实物只感知、不动车测试

完成 Bunker、MID360 和重定位启动后：

```bash
cd ~/fastlio_ws
source devel/setup.bash
roslaunch static_avoid static_obstacle_component.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/final_route.csv \
  world_frame:=camera_init \
  zone_mode:=true \
  enabled:=true \
  target_speed:=0.05
```

该模式只发布 `/static_avoid/cmd_vel`，未接速度仲裁器时不会控制 Bunker。

RViz 添加：

```text
/static_avoid/path_markers       MarkerArray
/static_avoid/obstacles          PointCloud2
/static_avoid/vehicle_footprint  Marker
/static_avoid/vehicle_pose       Pose
/static_avoid/target_pose        Pose
```

## 6. 实物低速动态/多障碍测试

停止键盘遥控、普通循迹、TEB、DWA 和比赛总控，确保没有其他速度发布者。

先确认环境和话题：

```bash
source ~/fastlio_ws/devel/setup.bash
source ~/livox_ws/devel/setup.bash --extend

rostopic type /livox/lidar
rostopic hz /livox/lidar
rostopic hz /cloud_registered_body
rostopic hz /Odometry
rostopic info /smoother_cmd_vel
```

`/livox/lidar` 类型应为 `livox_ros_driver2/CustomMsg`。

启动低速测试。`lane_half_width` 应填写参考路线到左右合法边界距离中的较小值；尚未测量时可暂用 0，但正式测试必须设置。

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

确认 RViz、急停和状态正常后启用：

```bash
rostopic pub -1 /static_avoid/enable std_msgs/Bool "data: true"
```

软件停车：

```bash
rostopic pub -1 /static_avoid/enable std_msgs/Bool "data: false"
```

另开终端监控：

```bash
rostopic echo /static_avoid/state
rostopic echo /smoother_cmd_vel
```

实物测试顺序：无障碍、单泡沫障碍、两个错位泡沫障碍、两侧封死、远程横移泡沫障碍。不要让人直接站在运动中的车辆前测试。

预期状态：

```text
FOLLOW_REFERENCE + zone_active=false  区外普通跟踪
FOLLOW_DETOUR                         区内绕静态障碍
DYNAMIC_WAIT                          横穿目标持续占用路径，停车让行
BLOCKED                               没有安全轨迹，停车重试
SAFETY_STOP                           近距离急停
avoidance_zone_exit                   离开避障区
FINISHED                              完成轨迹
```

## 7. 总流程接入

最终比赛总控使用组件输出，不应让多个节点直接发布 `/smoother_cmd_vel`：

```bash
roslaunch delivery_final static_obstacle_component.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/final_route.csv \
  world_frame:=camera_init \
  zone_mode:=true \
  use_raw_livox_safety:=true \
  enabled:=false
```

组件输出为 `/static_avoid/cmd_vel`，后续由总控速度仲裁器转发。

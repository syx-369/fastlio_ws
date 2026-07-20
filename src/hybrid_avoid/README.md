# hybrid_avoid：独立混合避障方案

`hybrid_avoid` 是与 `static_avoid` 完全独立的第二套 Bunker Mini 避障实现。它不修改、不导入也不覆盖旧方案代码，目的是让两套方案使用同一张地图、同一条CSV轨迹和同一组障碍进行实车对比。

## 实现结构

```text
避障区外：CSV连续参考轨迹 + Pure Pursuit

到达 avoid_start
        ↓
10m × 8m、0.10m分辨率滚动局部代价地图
        ↓
障碍膨胀 + 赛道边界约束
        ↓
8邻域A*（禁止穿过栅格对角墙角）
        ↓
安全捷径化 + Chaikin平滑 + 连续碰撞复查
        ↓
Pure Pursuit跟踪，每秒滚动重规划3次
        ↓
到达 avoid_end，恢复CSV参考轨迹
```

规划器直接处理所有障碍点，不检测障碍数量。车辆的半对角线约 `0.532m`，加 `0.12m` 碰撞余量后形成约 `0.652m` 的致命膨胀半径；外侧再增加 `0.25m` 软代价区。这样A*可以把车辆中心当作搜索点，同时保证完整 `0.8m × 0.7m` 车身安全。

## 传感器

- `/cloud_registered_body`：S-FAST-LIO运动补偿后的主体规划点云；
- `/livox/lidar`：实车低速模式下补充3米内局部代价地图，并承担独立近距离安全检查；
- `/Odometry`：S-FAST-LIO定位；
- `/smoother_cmd_vel`：实车测试时直接控制Bunker；
- `/hybrid_avoid/cmd_vel`：组件模式输出，留给后续比赛总控仲裁。

原始Livox近场点使用当前里程计近似变换到世界坐标，保留时间只有 `0.70s`。它没有逐点运动畸变补偿，因此实车首测必须保持 `0.05m/s`；远处规划仍以 `/cloud_registered_body` 为主。

## 编译

```bash
cd ~/fastlio_ws
env PYTHONNOUSERSITE=1 catkin_make \
  -DCATKIN_WHITELIST_PACKAGES='static_avoid;hybrid_avoid;delivery_final'
source devel/setup.bash
```

## 验证CSV

两套方案使用同一个 `final_route.csv` 和同一组 `avoid_start/avoid_end`：

```bash
source ~/fastlio_ws/devel/setup.bash
rosrun hybrid_avoid validate_hybrid_route.py \
  ~/fastlio_ws/src/waypoint_tools/data/final_route.csv
```

## 软件仿真

单障碍：

```bash
source ~/fastlio_ws/devel/setup.bash
roslaunch hybrid_avoid hybrid_single_sim.launch
```

三个交错静态障碍：

```bash
source ~/fastlio_ws/devel/setup.bash
roslaunch hybrid_avoid hybrid_multi_sim.launch
```

横穿移动障碍：

```bash
source ~/fastlio_ws/devel/setup.bash
roslaunch hybrid_avoid hybrid_moving_sim.launch
```

完全封死，预期 `BLOCKED` 且速度为零：

```bash
source ~/fastlio_ws/devel/setup.bash
roslaunch hybrid_avoid hybrid_blocked_sim.launch
```

状态：

```bash
rostopic echo /hybrid_avoid/state
```

## 实车低速测试

先启动Bunker、MID360和S-FAST-LIO重定位，停止键盘遥控、旧 `static_avoid`、TEB、DWA和其他速度发布者。

```bash
source ~/fastlio_ws/devel/setup.bash
source ~/livox_ws/devel/setup.bash --extend
rostopic hz /Odometry
rostopic hz /cloud_registered_body
rostopic hz /livox/lidar
rostopic info /smoother_cmd_vel
```

启动时保持禁用：

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

`lane_half_width=0` 只允许用于空旷首测。正式场地必须填写参考轨迹到左右合法边界距离中的较小值。

启用：

```bash
rostopic pub -1 /hybrid_avoid/enable std_msgs/Bool "data: true"
```

停车：

```bash
rostopic pub -1 /hybrid_avoid/enable std_msgs/Bool "data: false"
```

## RViz

Fixed Frame 设置为 `camera_init`，添加：

```text
/hybrid_avoid/reference_path       nav_msgs/Path
/hybrid_avoid/local_path           nav_msgs/Path
/hybrid_avoid/local_costmap        nav_msgs/OccupancyGrid
/hybrid_avoid/obstacles            sensor_msgs/PointCloud2
/hybrid_avoid/markers              visualization_msgs/MarkerArray
/hybrid_avoid/vehicle_footprint    visualization_msgs/Marker
```

代价地图中：

- `0`：自由区域；
- `1～80`：障碍物软膨胀代价；
- `100`：Bunker车体中心禁止进入的致命区域。

## 状态

```text
FOLLOW_REFERENCE      区外跟踪CSV轨迹
FOLLOW_HYBRID_PATH    区内滚动代价地图+A*路径
BLOCKED               A*没有安全路径
SAFETY_STOP           障碍进入保险杠急停距离
SENSOR_STOP           定位或点云超时
DISABLED              导航未启用
FINISHED              完成CSV路线
```

状态切换只管理执行和安全，不按障碍数量切换算法。

## 已完成的软件验证

- 9项纯算法单元测试；
- 单静态障碍ROS闭环：绕行、回归、退出区域、到达终点；
- 三个交错静态障碍ROS闭环：连续滚动规划并完成；
- 横穿移动障碍ROS闭环：路径随障碍移动实时改变并恢复；
- 完全封死ROS闭环：`BLOCKED`，线速度和角速度均为0；
- `100 × 80 @ 0.10m` OccupancyGrid、Path和Marker话题验证。

软件仿真不能代替实车验收。首轮使用泡沫箱，不要让人直接站在运动中的车前；必须保留实体急停。

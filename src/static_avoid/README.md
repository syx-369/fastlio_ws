# static_avoid：Bunker Mini 随机障碍区导航

`static_avoid` 是面向 Bunker Mini、MID360 和 S-FAST-LIO 重定位的 ROS1 Noetic 局部避障包。它读取提前录制的 CSV 参考轨迹，只在由 `avoid_start` 和 `avoid_end` 标记的区间内启用主动绕障，区间外继续跟踪原轨迹。

完整扩展说明见 [README_WORKFLOW_CN.md](README_WORKFLOW_CN.md)。

## 当前配置

- 车辆尺寸：长 `0.80m`，宽 `0.70m`；
- MID360：车体正前方中心，高度 `0.50m`，相对车体中心 `x=+0.40m`；
- 世界坐标系：`camera_init`；
- 定位：`/Odometry`；
- 主规划点云：S-FAST-LIO 处理后的 `/cloud_registered_body`；
- 近距离安全点云：MID360 原始 `/livox/lidar`；
- 实车底盘速度入口：`/smoother_cmd_vel`；
- 默认滚动规划窗口：沿参考轨迹未来 `5.0m`；
- 避障区标记：`avoid_start`、`avoid_end`。

规划器不需要提前知道障碍物数量。固定左侧或右侧绕行都不安全时，会启用横向栅格生成左右组合路线；没有安全路线、传感器失联或障碍进入紧急距离时停车。

## 数据与控制链路

```text
final_route.csv + /Odometry
             + /cloud_registered_body
             + /livox/lidar（近距离安全）
                         |
                         v
                    static_avoid
                         |
             实车测试：/smoother_cmd_vel
             组件模式：/static_avoid/cmd_vel
```

CSV 的 `x/y` 是参考轨迹锚点。相邻点先连接为按累计弧长参数化的连续轨迹，再每隔 `0.10m` 放置一次带朝向的 Bunker 矩形车身，检查未来 `5m` 的车身扫掠区域是否与障碍点碰撞。5米只限制阻塞搜索范围；如果障碍位于窗口末端，绕行路线可以继续延长，直到平滑回到原轨迹。

## 1. 编译

```bash
cd ~/fastlio_ws
env PYTHONNOUSERSITE=1 catkin_make \
  -DCATKIN_WHITELIST_PACKAGES='static_avoid;delivery_final'
source devel/setup.bash
```

恢复编译工作空间全部包：

```bash
cd ~/fastlio_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

## 2. 建图

### 终端1：ROS Master

```bash
roscore
```

### 终端2：CAN

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
timeout 3 candump can0
```

### 终端3：Bunker Mini

```bash
cd ~/bunker_ws
source devel/setup.bash
roslaunch bunker_bringup bunker_robot_base.launch
```

### 终端4：键盘遥控

```bash
cd ~/bunker_ws
source devel/setup.bash
roslaunch bunker_bringup bunker_teleop_keyboard.launch
```

### 终端5：MID360

```bash
cd ~/livox_ws
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

### 终端6：S-FAST-LIO 建图

```bash
cd ~/fastlio_ws
source devel/setup.bash
source ~/livox_ws/devel/setup.bash --extend
roslaunch sfast_lio mapping_mid360.launch rviz:=true
```

空场、低速、闭环驾驶完整比赛区域。回到起点后在建图终端按 `Ctrl+C`，检查地图：

```bash
ls -lh \
  ~/fastlio_ws/src/S-FAST_LIO/PCD/GlobalMap.pcd \
  ~/fastlio_ws/src/S-FAST_LIO/PCD/GlobalMap_ikdtree.pcd
```

备份地图：

```bash
MAP_TAG=$(date +%Y%m%d_%H%M%S)
mkdir -p ~/fastlio_ws/map_backup/${MAP_TAG}_final
cp -a ~/fastlio_ws/src/S-FAST_LIO/PCD/GlobalMap*.pcd \
  ~/fastlio_ws/map_backup/${MAP_TAG}_final/
```

## 3. 重定位并录制轨迹

停止建图节点，保留 ROS、Bunker、键盘遥控和 MID360，启动重定位：

```bash
cd ~/fastlio_ws
source devel/setup.bash
source ~/livox_ws/devel/setup.bash --extend
roslaunch sfast_lio mapping_mid360_relocalization.launch rviz:=true
```

检查定位：

```bash
rostopic hz /Odometry
rostopic echo -n 1 /Odometry/header
```

新终端启动轨迹录制：

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

## 4. 标记随机避障区

车辆停在第一个可能障碍位置前至少 `2m`，发布入口标记：

```bash
source ~/fastlio_ws/devel/setup.bash
rostopic pub -1 /waypoint_task std_msgs/String "data: 'avoid_start'"
```

穿过整个障碍区，在最后一个可能障碍位置后至少 `2m`，发布出口标记：

```bash
source ~/fastlio_ws/devel/setup.bash
rostopic pub -1 /waypoint_task std_msgs/String "data: 'avoid_end'"
```

录完剩余路线，在录制终端按 `Ctrl+C`。检查并验证：

```bash
grep -n 'avoid_start\|avoid_end' \
  ~/fastlio_ws/src/waypoint_tools/data/final_route.csv

source ~/fastlio_ws/devel/setup.bash
rosrun static_avoid validate_avoid_route.py \
  ~/fastlio_ws/src/waypoint_tools/data/final_route.csv
```

## 5. 软件仿真

每项完成后按 `Ctrl+C` 再启动下一项。

```bash
source ~/fastlio_ws/devel/setup.bash
roslaunch static_avoid static_obstacle_sim.launch
```

```bash
source ~/fastlio_ws/devel/setup.bash
roslaunch static_avoid multi_obstacle_sim.launch
```

```bash
source ~/fastlio_ws/devel/setup.bash
roslaunch static_avoid moving_obstacle_sim.launch
```

监控状态：

```bash
rostopic echo /static_avoid/state
```

## 6. 实车导航

运行实车导航前应保持以下节点正常：

- Bunker 底盘；
- MID360；
- S-FAST-LIO 重定位；
- `/Odometry`、`/cloud_registered_body`、`/livox/lidar` 均持续更新。

停止键盘遥控、普通循迹、TEB、DWA和其他直接速度发布者。确认话题：

```bash
source ~/fastlio_ws/devel/setup.bash
source ~/livox_ws/devel/setup.bash --extend
rostopic type /livox/lidar
rostopic hz /livox/lidar
rostopic hz /cloud_registered_body
rostopic hz /Odometry
rostopic info /smoother_cmd_vel
```

首轮以 `0.05m/s` 启动，并保持禁用：

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

`lane_half_width=0` 只适合空旷首测。正式场地必须改成参考轨迹中心线到左右合法边界距离中的较小值。

确认定位、RViz、障碍点和实体急停正常后启用：

```bash
rostopic pub -1 /static_avoid/enable std_msgs/Bool "data: true"
```

软件停车：

```bash
rostopic pub -1 /static_avoid/enable std_msgs/Bool "data: false"
```

监控：

```bash
rostopic echo /static_avoid/state
```

```bash
rostopic echo /smoother_cmd_vel
```

## 7. RViz

Fixed Frame 设置为 `camera_init`，添加：

```text
/static_avoid/reference_path       nav_msgs/Path
/static_avoid/local_path           nav_msgs/Path
/static_avoid/path_markers         visualization_msgs/MarkerArray
/static_avoid/obstacles            sensor_msgs/PointCloud2
/static_avoid/vehicle_footprint    visualization_msgs/Marker
/static_avoid/vehicle_pose         geometry_msgs/PoseStamped
/static_avoid/target_pose          geometry_msgs/PoseStamped
```

常见状态：

```text
FOLLOW_REFERENCE  跟踪原始CSV轨迹
FOLLOW_DETOUR     执行局部绕行
DYNAMIC_WAIT      移动目标持续占用路径，停车让行
BLOCKED           当前没有安全路线
SAFETY_STOP       障碍进入紧急距离
SENSOR_STOP       定位或点云超时
FINISHED          完成轨迹
```

## 8. 接入比赛总控

组件模式不直接控制底盘，而是输出 `/static_avoid/cmd_vel`，后续由 `delivery_final` 统一仲裁：

```bash
roslaunch delivery_final static_obstacle_component.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/final_route.csv \
  world_frame:=camera_init \
  planning_horizon:=5.0 \
  zone_mode:=true \
  use_raw_livox_safety:=true \
  enabled:=false
```

总控只允许一个最终速度出口：

```text
/static_avoid/cmd_vel -> delivery_final -> /smoother_cmd_vel -> bunker_base_node
```

## 安全测试顺序

1. 车轮悬空或空旷地面，只观察速度和RViz；
2. 无障碍低速完整跟踪；
3. 单个泡沫障碍；
4. 两个交错泡沫障碍；
5. 多个随机静态障碍；
6. 左右封死，确认车辆停车；
7. 用遥控小车或长杆远程移动软障碍验证动态等待。

首轮不要让人直接站在运动中的车辆前测试。任何时候都应保留实体急停，并确保没有多个节点同时发布 `/smoother_cmd_vel`。

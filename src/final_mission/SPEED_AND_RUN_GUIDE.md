# 决赛巡航速度与完整运行命令

本文档说明两件事：

1. 每次运行前如何设置巡航速度，以及车辆在完整比赛流程中的速度变化。
2. 从 CAN、底盘、MID360、S-FAST-LIO、Piper 到 `final_mission` 的完整终端命令。

> 正式比赛前，请把文中的 `RACE_CSV` 替换成新录制并校验通过的正式航迹。
> 当前旧的 `final_route.csv` 不是正式比赛航迹。

## 一、每次运行前设置巡航速度

不需要修改 YAML。启动 `final_race.launch` 时传入 `target_speed` 即可：

```bash
RACE_CSV=/home/user/fastlio_ws/src/waypoint_tools/data/final_competition_route.csv
CRUISE_SPEED=0.30

roslaunch final_mission final_race.launch \
  csv_path:="${RACE_CSV}" \
  target_speed:="${CRUISE_SPEED}"
```

常用速度参考：

| `target_speed` | 换算速度 | 建议用途 |
|---:|---:|---|
| `0.15 m/s` | `0.54 km/h` | 第一次机械臂联合调试 |
| `0.20 m/s` | `0.72 km/h` | 第一次完整路线低速测试 |
| `0.25 m/s` | `0.90 km/h` | 稳定性测试 |
| `0.30 m/s` | `1.08 km/h` | 推荐的比赛前综合测试速度 |
| `0.35 m/s` | `1.26 km/h` | 当前正式启动默认速度 |

`config/tracker.yaml` 中的 `max_linear` 当前为 `0.50 m/s`。即使
`target_speed` 设置得更高，最终也会被限制到 `0.50 m/s`，约
`1.8 km/h`，低于规则规定的 `15 km/h`。

`target_speed` 只在跟踪器启动时读取。运行过程中执行 `rosparam set` 不会让
当前跟踪器立即改变速度；需要停止并重新启动 `final_race.launch`。

建议不要把巡航速度设到 `0.10 m/s` 以下，因为任务点逼近速度上限本身就是
`0.10 m/s`。如果巡航速度低于它，任务点附近的速度配置将不再符合“先巡航、
再减速”的正常关系。

## 二、完整流程中的速度变化

`target_speed` 是普通路段的目标巡航速度，不代表车辆全程恒速。

### 1. 等待红旗

跟踪器未使能并持续发布零速：

```text
线速度 = 0
角速度 = 0
```

### 2. 红旗放行后加速

车辆从零速逐渐加速，线加速度上限为 `0.25 m/s²`。

以 `target_speed=0.35 m/s` 为例，理论上约需：

```text
0.35 ÷ 0.25 ≈ 1.4 秒
```

才能从静止加速到巡航速度。

### 3. 普通直线路段

定位和路径正常、前方没有任务点时，车辆保持在 `target_speed` 附近。

### 4. 转弯路段

跟踪器根据前视点与车头方向的偏差自动调整速度：

- 航向偏差较小时：保持巡航速度。
- 航向偏差约 `29°～40°` 时：线速度降低到约 `60%`。
- 航向偏差超过约 `40°` 时：线速度立即归零，先原地转正，再继续前进。

例如 `target_speed=0.35 m/s` 时，中等转弯速度约为：

```text
0.35 × 0.60 = 0.21 m/s
```

### 5. 避障区

进入 CSV 中的 `avoid_start`～`avoid_end` 区间后，A* 和障碍安全层启用：

- 没有近距离障碍：按照巡航速度跟踪规划路径。
- 前方净距离进入 `0.60～0.28 m`：逐步降低线速度。
- 前方净距离小于 `0.28 m`：线速度归零并尝试转向。
- 侧面距离过近：线速度乘以约 `0.45`。

> 当前 S-FAST-LIO 的 `preprocess/blind` 为 `2 m`，与上述近距离安全阈值存在
> 冲突。在解决该问题并完成实车验证前，不能只依赖这组近距离停车阈值。

### 6. 接近任务点

机械臂点、红绿灯点和终点都使用同一套精确停靠过程：

```text
距离任务点约 0.80 m：巡航速度开始向 0.10 m/s 降低
距离任务点小于 0.45 m：切换点到点 P 控制
P 控制速度范围：0.04～0.10 m/s
距离任务点小于 0.05 m：线速度归零
```

停车后可能进行小角度原地朝向修正。

### 7. 执行任务和等待红绿灯

以下状态都会持续发布零速：

- 等待红绿灯变绿；
- 卡片识别；
- 抓取；
- 卸货；
- 终点完成状态。

任务完成并收到 `done:<task>` 后，车辆重新按照 `0.25 m/s²` 的加速度限制，
逐渐恢复到本次启动设置的巡航速度。

## 三、赛前一次性准备

### 1. 配置两路 CAN

`can0` 用于 Bunker，波特率 `500000`；`can1` 用于 Piper，波特率
`1000000`。

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 restart-ms 100
sudo ip link set can1 up

ip -details link show can0 | grep bitrate
ip -details link show can1 | grep bitrate
```

可选的短时数据检查：

```bash
timeout 3 candump can0
timeout 3 candump can1
```

### 2. 校验正式航迹

```bash
source /opt/ros/noetic/setup.bash
source /home/user/fastlio_ws/devel/setup.bash

RACE_CSV=/home/user/fastlio_ws/src/waypoint_tools/data/final_competition_route.csv

rosrun final_mission validate_route.py "${RACE_CSV}" --rounds 2
```

必须先处理全部 `ERROR`。还要人工确认：

- 红旗和红绿灯均在机械臂任务之前；
- 一次红旗、一次红绿灯；
- 两轮各有完整的 `piper_stop_1`～`piper_stop_7`；
- 每次实际避障路段都有对应的 `avoid_start/avoid_end`；
- 最后一项任务为 `ext:finish`；
- 终点坐标位于最佳停车区中部。

## 四、正式比赛的全部终端

下面除“CAN 配置”外均为持续运行的终端。建议按照编号依次启动，并确认当前终端
没有报错后再启动下一个。

### 终端 1：ROS Master

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 终端 2：Bunker 底盘

```bash
source /opt/ros/noetic/setup.bash
source /home/user/bunker_ws/devel/setup.bash

roslaunch bunker_bringup bunker_robot_base.launch pub_tf:=false
```

这里使用 `pub_tf:=false`，避免 Bunker 轮速里程计与 S-FAST-LIO 同时发布
`odom→base_link`。

可在另一个诊断终端确认：

```bash
rostopic hz /bunker_status
```

### 终端 3：Livox MID360

```bash
source /opt/ros/noetic/setup.bash
source /home/user/livox_ws/devel/setup.bash

roslaunch livox_ros_driver2 msg_MID360.launch
```

诊断：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

### 终端 4：S-FAST-LIO 重定位

车辆必须按建图时的原点位置和朝向摆放好，再启动本终端。

```bash
source /opt/ros/noetic/setup.bash
source /home/user/fastlio_ws/devel/setup.bash
source /home/user/livox_ws/devel/setup.bash

roslaunch sfast_lio mapping_mid360_relocalization.launch rviz:=true
```

诊断：

```bash
rostopic hz /Odometry
rostopic echo -n 1 /Odometry/pose/pose
rostopic hz /cloud_registered
```

### 终端 5：Piper 底层驱动与自动使能

```bash
source /opt/ros/noetic/setup.bash
source /home/user/miniconda3/etc/profile.d/conda.sh
conda activate piper
source /home/user/fastlio_ws/devel/setup.bash
source /home/user/piper_ws/devel/setup.bash

roslaunch piper start_single_piper.launch \
  can_port:=can1 \
  auto_enable:=true
```

诊断：

```bash
rostopic hz /joint_states_single
rostopic hz /end_pose
```

如果赛前需要让机械臂回零，应先确认机械臂周围无人、无障碍，再执行：

```bash
rosservice call /go_zero_srv "is_mit_mode: false"
```

### 终端 6：Piper Pinocchio 逆解

```bash
source /opt/ros/noetic/setup.bash
source /home/user/miniconda3/etc/profile.d/conda.sh
conda activate piper
source /home/user/fastlio_ws/devel/setup.bash
source /home/user/piper_ws/devel/setup.bash

python3 /home/user/piper_ws/src/piper_ros/src/piper/scripts/piper_pinocchio/piper_pinocchio.py
```

### 终端 7：Piper 比赛任务与相机中继

必须使用 `piper_task.launch`，不要使用 `arm_function_test.launch`。

```bash
source /opt/ros/noetic/setup.bash
source /home/user/miniconda3/etc/profile.d/conda.sh
conda activate piper
source /home/user/fastlio_ws/devel/setup.bash
source /home/user/piper_ws/devel/setup.bash

roslaunch piper_task piper_task.launch enable_camera_relay:=true
```

应在日志中看到类似信息：

```text
相机帧转发已就绪（按需）
piper_task ready
```

诊断：

```bash
rostopic echo -n 1 /piper_task/state
rostopic echo -n 1 /piper_task/result
```

### 终端 8：正式比赛总控

只需要修改下面的正式航迹路径和本次巡航速度：

```bash
source /opt/ros/noetic/setup.bash
source /home/user/fastlio_ws/devel/setup.bash

RACE_CSV=/home/user/fastlio_ws/src/waypoint_tools/data/final_competition_route.csv
CRUISE_SPEED=0.30

roslaunch final_mission final_race.launch \
  csv_path:="${RACE_CSV}" \
  target_speed:="${CRUISE_SPEED}" \
  wait_for_start:=true \
  auto_start:=false \
  enable_vision:=true \
  show_image:=true
```

正常现象：

```text
总控状态：BOOT -> WAIT_FLAG
```

此时车辆应保持不动，相机中继按需发图，视觉等待裁判挥红旗。

## 五、正式启动后的诊断命令

另开一个只做诊断的终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/user/fastlio_ws/devel/setup.bash
```

查看总控状态：

```bash
rostopic echo /final_mission/state
```

等待红旗时检查相机图像频率：

```bash
rostopic hz /final_mission/camera/color
```

等待红旗时应该有图像；识别完成进入 `idle` 后停止发图是正常行为。

查看图像通路状态：

```bash
rostopic echo -n 1 /final_mission/vision_status
```

正常状态类似：

```text
ok:frames=123:age=0.1
```

检查速度话题：

```bash
rostopic echo /smoother_cmd_vel
rostopic info /smoother_cmd_vel
```

查看跟踪器阶段：

```bash
rostopic echo /final_tracker/phase
```

查看机械臂任务：

```bash
rostopic echo /piper_task/state
rostopic echo /piper_task/result
rostopic echo /arm_bridge/status
```

## 六、备用和紧急命令

### 紧急软件停车

先按实体急停；在 ROS 仍正常时可同时执行：

```bash
rostopic pub -1 /final_mission/command std_msgs/String "data: 'stop'"
```

### 手动起步

只应在裁判允许，或规则规定的自动起步失败处置阶段使用：

```bash
rostopic pub -1 /final_mission/command std_msgs/String "data: 'start'"
```

比赛中擅自人工介入可能导致本轮成绩为零。

### 测试环境恢复

仅限调试，不应在正式比赛过程中擅自使用：

```bash
rostopic pub -1 /final_mission/command std_msgs/String "data: 'resume'"
```

## 七、其他测试启动方式

### 不启动机械臂和视觉的路线空跑

```bash
source /opt/ros/noetic/setup.bash
source /home/user/fastlio_ws/devel/setup.bash

RACE_CSV=/home/user/fastlio_ws/src/waypoint_tools/data/final_competition_route.csv

roslaunch final_mission dry_run.launch \
  csv_path:="${RACE_CSV}" \
  target_speed:=0.20 \
  task_pause:=3.0
```

### 只联调车辆停靠与机械臂任务

仍需提前运行 Bunker、MID360、S-FAST-LIO、Piper 底层、Pinocchio 和
`piper_task.launch`。

```bash
source /opt/ros/noetic/setup.bash
source /home/user/fastlio_ws/devel/setup.bash

RACE_CSV=/home/user/fastlio_ws/src/waypoint_tools/data/final_competition_route.csv

roslaunch final_mission arm_only.launch \
  csv_path:="${RACE_CSV}" \
  target_speed:=0.15
```

## 八、推荐的调速顺序

正式航迹录制完成后，建议按以下顺序测试：

1. `dry_run.launch`，`target_speed:=0.20`。
2. `arm_only.launch`，`target_speed:=0.15`。
3. `final_race.launch`，`target_speed:=0.20`，完整跑两轮。
4. 提升到 `0.25 m/s`，再次完整跑两轮。
5. 提升到 `0.30 m/s`，检查转弯、避障区入口和任务点停靠。
6. 只有前三档全部稳定后，再评估是否使用默认的 `0.35 m/s`。

每次提高速度后都要重新确认：

- 不压实线；
- 避障区入口不会冲入障碍；
- 任务点停车误差不增大；
- 机械臂侧伸时车辆只以任务点逼近速度移动；
- 终点能够停入最佳停车区。

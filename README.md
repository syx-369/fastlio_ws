# 无人配送车系统说明（Bunker Mini + Piper 机械臂）

本文档整理当前 `fastlio_ws` / `piper_ws` / `livox_ws` / `bunker_ws` 工作空间中各功能包的用途、话题接口与启动方式，对应赛事《2026 第六届智能无人系统应用挑战赛 —— 自主赛道·快递速达4.0》竞赛规则。**本文档为纯说明性文档，未对任何代码或配置做修改。**

## 0. 硬件与工作空间总览

| 硬件 | 说明 |
|---|---|
| 底盘 | 松灵（AgileX）Bunker Mini，差速/履带底盘，速度接口 `/smoother_cmd_vel` |
| 雷达 | Livox MID360，车体正前方中心安装 |
| 机械臂 | AgileX Piper 六轴机械臂 + 末端夹爪 |
| 相机 | Intel RealSense D455（挂载在机械臂/车体上，用于卡片、货物、红绿灯识别） |

| 工作空间 | 内容 |
|---|---|
| `~/livox_ws` | Livox MID360 驱动（`livox_ros_driver2`） |
| `~/fastlio_ws` | S-FAST_LIO 建图/重定位、航迹点录制与跟踪、避障、任务总控 |
| `~/bunker_ws` / `~/hunter_ws` | 底盘驱动（`bunker_bringup` 等，ugv_sdk） |
| `~/piper_ws` | Piper 机械臂驱动、逆运动学、抓取/放置任务节点 |

整体数据链路：

```text
MID360 → livox_ros_driver2 → S-FAST_LIO（建图/重定位）→ /Odometry, /cloud_registered
                                                              │
                        CSV 航迹点（record_waypoints.py 录制）│
                                                              ▼
                              航迹跟踪 + 局部避障（waypoint_tools / static_avoid / hybrid_avoid）
                                                              │
                                                     /smoother_cmd_vel
                                                              ▼
                                                   bunker_bringup（底盘驱动）
                                                              │
                                   任务点触发（/waypoint_task, ext:xxx）
                                                              ▼
                              视觉识别（红旗/红绿灯/卡片，RealSense + YOLO）
                                                              │
                              机械臂任务（piper_task 或 piper_mission + Piper 驱动/逆运动学）
```

---

## 1. 定位与建图 —— `S-FAST_LIO`（ROS 包名 `sfast_lio`）

FAST-LIO2 的简化实现，增加了重定位与多雷达型号支持。

- `roslaunch sfast_lio mapping_mid360.launch` —— **建图模式**，运行 `laserMapping` 节点，产出点云地图（`PCD/`）。参数：`rviz`（默认 true）、`enable_waypoint_record` 及一组 `waypoint_*` 参数（开启后内部会拉起 `record_waypoints.py`）。
- `roslaunch sfast_lio mapping_mid360_relocalization.launch rviz:=true` —— **重定位模式**，运行 `laserMapping_re` 节点，加载已建好的地图进行定位，并发布 `camera_init → map → odom → base_link` 的静态 TF。参数：`rviz`、`enable_waypoint_follow`（拉起 `follow_waypoints.py`）、`enable_teb_nav`（拉起 `teb_navigation.launch`）等。

发布的核心话题：`/Odometry`（`nav_msgs/Odometry`，Fixed Frame 为 `camera_init`）、`/cloud_registered`（世界坐标系点云）、`/cloud_registered_body`（车体坐标系点云，供局部避障使用）。

重定位后检查定位质量：

```bash
rostopic hz /Odometry
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /Odometry/pose/pose
```

RViz 中确认：地图与实时点云重合、车辆运动方向正确、Fixed Frame 为 `camera_init`、定位无明显跳变。

---

## 2. 航迹点工具 —— `waypoint_tools`

CSV 航迹格式（`record_waypoints.py` 写出，所有跟踪脚本读入）：

```text
seq,stamp,frame_id,x,y,z,qx,qy,qz,qw,yaw,task,tol
```

`task` 列承载任务标记字符串，`tol` 为到点容差。

### 2.1 录制 —— `record_waypoints.py`

按里程计增量距离/朝向变化自动采样航迹点；也可通过话题强制打点。

主要参数：`_odom_topic`（默认 `/Odometry`）、`_min_distance`（默认 0.30，建议低速录制用 0.15）、`_min_yaw_change`（默认 0.35）、`_file_name`、`_output_dir`、`_frame_id`、`_default_tol`。

手动打点话题：
- `/waypoint_task`（`std_msgs/String`）：写入自定义 `task` 标记，例如 `avoid_start` / `avoid_end` / `stop_30s` / `ext:pick` / `ext:piper_stop_1` 等（见下文任务字符串约定）。
- `/waypoint_stop_seconds`（`std_msgs/Float32`）：自动生成 `stop_Xs` 定时停车标记。

示例：

```bash
rosrun waypoint_tools record_waypoints.py \
  _file_name:=final_route.csv \
  _output_dir:=/home/user/fastlio_ws/src/waypoint_tools/data \
  _min_distance:=0.15 _min_yaw_change:=0.17 _default_tol:=0.25 _frame_id:=camera_init

rostopic pub -1 /waypoint_task std_msgs/String "data: 'avoid_start'"
rostopic pub -1 /waypoint_task std_msgs/String "data: 'avoid_end'"
rostopic pub -1 /waypoint_task std_msgs/String "data: 'ext:pick'"
```

录制结束在录制终端按 `Ctrl+C` 保存 CSV。

### 2.2 任务字符串约定（`task` 列 / `/waypoint_task`）

| 标记 | 含义 | 消费者 |
|---|---|---|
| `avoid_start` / `avoid_end` | 标记随机障碍区起止 | `avoidance_zone_astar_test.py` |
| `stop_Xs` / `hold_Xs` / `pause_Xs` | 定点停车 X 秒 | `follow_waypoints.py` 及其子类 |
| `detect` | 定点短暂停留（识别用） | `follow_waypoints.py` |
| `ext:<name>` | 通用扩展任务：到点后发布 `start:<name>:idx<N>` 到任务事件话题，并阻塞等待完成事件 | `follow_waypoints.py` / `pure_pursuit_follower.py`；`<name>` 可为 `pick`/`place`/`traffic_light`/`finish`（对接 `race_mission`+`piper_mission`）或 `piper_stop_1..7`（对接 `piper_task`） |

### 2.3 纯跟踪（无避障） —— `pure_pursuit_only_follower.py`

单纯 Pure Pursuit 几何跟踪，不处理 `task` 列，不做避障、不做红绿灯/任务点判断。适合场地纯路线联调。

```bash
rosrun waypoint_tools pure_pursuit_only_follower.py \
  _csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/xiaosai.csv
```

### 2.4 纯跟踪 + A* 局部重规划 —— `pure_pursuit_astar_follower.py`

在 Pure Pursuit 基础上叠加基于 `/cloud_registered`（或 `/scan`）的局部占据栅格 + 8 邻域 A* 重规划，并带反应式前/侧向安全停障层。发布 `~planned_path` / `~execute_path`（`nav_msgs/Path`）供 RViz 查看。同样不处理 `task` 列。

```bash
rosrun waypoint_tools pure_pursuit_astar_follower.py \
  _csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/xiaosai.csv
```

### 2.5 随机障碍区专用（分区激活 A*） —— `avoidance_zone_astar_test.py`

继承自 `pure_pursuit_astar_follower.py`，但**只在 CSV 中 `avoid_start`~`avoid_end` 标记的区间内**启用 A* 避障；区间外走平滑重采样的纯 Pure Pursuit 参考线。带定位跳变拒绝（`_odom_jump_threshold`）和速度/角加速度限幅。发布 `~avoidance_zone_active`（`std_msgs/Bool`，latched）。

```bash
rosrun waypoint_tools avoidance_zone_astar_test.py \
  _csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/final_route.csv \
  _obstacle_source:=cloud _cloud_topic:=/cloud_registered \
  _odom_topic:=/Odometry _cmd_topic:=/smoother_cmd_vel _target_speed:=0.50
```

对应竞赛规则第 2.2(3)/(5) 条“随机障碍路段”“静态避障（锥桶）”场景。

### 2.6 带任务点的完整跟踪 —— `follow_waypoints.py` / `pure_pursuit_follower.py`

- `follow_waypoints.py`：P 控制器状态机（`WAIT_ODOM→TRACK→TASK/ALIGN_FINAL→FINISH`），实现上述 `task` 列全部约定，是 `piper_task` 的 `arm_function_test_follower.py` 的父类。
- `pure_pursuit_follower.py`：在同样任务状态机基础上换成自适应前视距离的 Pure Pursuit 跟踪，支持 `_vehicle_model:=ackermann|diff` 输出转向角或角速度，由 `race_mission_manager.py` 以 `rosrun` 子进程方式拉起。

### 2.7 基于 move_base 的跟踪 —— `waypoint_sender.py`

将稀疏化后的 CSV 目标点依次交给 `move_base`（`actionlib`），由 TEB 局部规划器负责避障；同样支持 `stop_Xs` / `ext:` 任务词。对应 `dwa_navigation.launch` / `teb_navigation.launch`。

---

## 3. 局部避障 —— `static_avoid` 与 `hybrid_avoid`

两者都面向 Bunker Mini（车体 0.80×0.70m，MID360 前置），都只在 CSV 的 `avoid_start`/`avoid_end` 区间内生效，且彼此完全独立（互不依赖、互不覆盖），用于同一地图/路线上做方案对比。默认启动时 `enabled:=false`，需通过 Bool 话题（`/static_avoid/enable`、`/hybrid_avoid/enable`）显式开启。

- **`static_avoid`**：参考路径感知的绕行规划。以 CSV 为弧长参数化连续参考线，前瞻 5m 扫描车身矩形是否与障碍碰撞，生成固定横向偏移绕行候选（`[0.65, 0.85, 1.05, 1.20]m`），单一偏移不可行时启用多障碍横向栅格兜底。状态机：`FOLLOW_REFERENCE / FOLLOW_DETOUR / DYNAMIC_WAIT / BLOCKED / SAFETY_STOP / SENSOR_STOP / FINISHED`。主规划点云 `/cloud_registered_body`，可选近距离安全层 `/livox/lidar`。详见 `src/static_avoid/README.md`、`README_WORKFLOW_CN.md`。
- **`hybrid_avoid`**：滚动局部占据栅格（10m×8m@0.10m）+ 8 邻域 A*（禁止穿角）+ Chaikin 平滑重规划（约 3Hz）。障碍膨胀半径 ≈ 车体半对角线 + 0.12m，外加 0.25m 软避让带。状态机：`FOLLOW_REFERENCE / FOLLOW_HYBRID_PATH / BLOCKED / SAFETY_STOP / SENSOR_STOP / DISABLED / FINISHED`。详见 `src/hybrid_avoid/README.md`。

两套方案与 `delivery_final` 的关系：`delivery_final/static_obstacle_component.launch` 是一层薄封装，转发 `static_avoid` 的组件模式启动参数，作为“最终集成”时的单一入口：`/static_avoid/cmd_vel → delivery_final → /smoother_cmd_vel → bunker_bringup`。

---

## 4. 任务总控（车辆层） —— `race_mission` / `delivery_mission` / `delivery_final`

工作区中存在三代/三种任务总控方案，**功能有重叠，建议以其中一套为主线，其余作为参考或降级方案**：

| 包 | 定位 | 特点 |
|---|---|---|
| `delivery_mission` | 较早版本 | 视觉（红旗+红绿灯）+ Pure Pursuit，可选 TEB 兜底（`planner_switch.py` 按激光扫描判断是否切换到 `/teb_cmd_vel`），**不含机械臂任务对接** |
| `race_mission` | 当前主线 | 增加起跑红旗门控、正式状态机（`BOOT→WAIT_START→NAVIGATING→ARM_TASK/WAIT_TRAFFIC_LIGHT→FINISHING→DONE`）、机械臂任务分发（`ext:pick`/`ext:place` → `/arm_task_cmd`，对接 `piper_mission`）、红绿灯任务门控（`ext:traffic_light` → 等待 `/race/traffic_light` 变绿）、终点标记（`ext:finish`） |
| `delivery_final` | 最终集成封装 | 仅包含 `static_obstacle_component.launch`，转发 `static_avoid` 组件模式参数，不含视觉/任务逻辑 |

### 4.1 `race_mission`（推荐的总控入口）

- `start_light_vision_node.py`：按 `/race/vision_control`（`flag`/`light`/`idle`）切换模式，按需开关 RealSense。`flag` 模式做红旗 HSV 检测，连续确认后发布 `/race/start_signal`（Bool，latched）与 `/flag_detected`；`light` 模式加载 YOLO（默认权重 `waypoint_tools/config/traffic_light.pt`）+ HSV 兜底识别红绿灯，发布 `/race/traffic_light`（及兼容话题 `/traffic_light_state`）。对应规则 2.2(1) 起点挥旗起步、2.2(2) 红绿灯路口。
- `race_mission_manager.py`：以 `rosrun` 子进程拉起 `waypoint_tools/pure_pursuit_follower.py` 作为底层跟踪器；监听其 `/waypoint_task_event` 上的 `start:<task>:idxN`；将 `~arm_tasks`（默认 `pick,place`）映射为 `/arm_task_cmd` 并等待 `/arm_task_result`；将 `~traffic_light_tasks`（默认 `traffic_light`）映射为等待 `/race/traffic_light` 进入 `~green_states`；`~finish_task_name`（默认 `finish`）结束任务；对外发布 `/race/mission_state`。

```bash
roslaunch race_mission mission.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/xiaosai.csv \
  target_speed:=0.30 wait_for_start:=true auto_start:=false \
  enable_vision:=true show_image:=true
```

主要参数：`csv_path`、`target_speed`（默认 0.45）、`cmd_topic`（默认 `/cmd_vel`，实车通常改为 `/smoother_cmd_vel`）、`vehicle_model`（`ackermann`/`diff`）、`wheelbase`、`max_steer_angle`、`wait_for_start`、`auto_start`、`enable_vision`、`show_image`、`traffic_weights`。启动前需保证底盘、Livox、S-FAST_LIO 重定位、Piper 机械臂栈均已运行。

`race_mission` 期望的 `pick`/`place` 任务名与 **`piper_mission`** 包对应（见第 6 节），与 `piper_task` 的 `piper_stop_1..7` 体系是两条不同的机械臂对接线，代码层面未打通，使用时需二选一。

---

## 5. 机械臂对接（车辆侧任务名 → 机械臂动作）——`piper_ws`

`piper_ws` 中存在**两套独立**的机械臂任务对接设计，互不依赖，选择其中一套即可：

### 5.1 `piper_mission`（对接 `race_mission` 的 pick/place）

`arm_task_server.py`：订阅 `/arm_task_cmd`（`std_msgs/String`：`pick`/`place`/`stow`），发布 `/arm_task_result`（`done:<task>` / `fail:<task>`）与 `/arm_task_state`。用 YOLO 瓶子检测器（`config/arm_task.yaml`，模型来自 `realsense-D455-YOLOV5/weights/bottle.pt`）+ RealSense 深度定位目标，经手眼标定变换后通过 `/pin_pos_cmd`（`piper_msgs/PosCmd`）驱动平滑笛卡尔抓取/放置动作；订阅 `/end_pose` 获取末端当前位姿。

```bash
roslaunch piper_mission arm_task.launch
rostopic echo /arm_task_state
rostopic echo /arm_task_result
```

### 5.2 `piper_task`（七点位竞赛流程，对应本文最初描述的机械臂测试脚本）

`piper_task_node.py`：既接受直接指令 `/piper_task/command`（`card`/`pick1-3`/`place1-3`/`stow`/`reset`/`status`），也订阅 `/waypoint_task_event`，按 `config/task_config.yaml` 的固定映射自动触发：

| 航迹任务标记 | 机械臂动作 | 对应竞赛环节 |
|---|---|---|
| `ext:piper_stop_1` | `card`：识别裁判指定的目标卡片，锁定目标标签 | 规则 2.2(4) 定点取货——识别“指定抓取物品”卡片 |
| `ext:piper_stop_2/3/4` | `pick1/pick2/pick3`：依次尝试三个抓取候选点，直到抓到目标 | 同上——在桌面 6 个物品中找到并抓取指定物品 |
| `ext:piper_stop_5/6/7` | `place1/place2/place3`：依次尝试三个放置候选点，按标签匹配对应置物框 | 规则 2.2(6) 定点卸货——放入对应类别置物框 |

完成后发布 `done:<task>` 到 `/waypoint_task_done`（与 `follow_waypoints.py`/`arm_function_test_follower.py` 消费的话题一致），并发布 `/piper_task/state`、`/piper_task/result`、`/piper_task/target`（当前锁定/携带的目标标签）、`/piper_task/navigation_skip`（提示跟踪器跳过已完成候选点对应的停靠）。

实际视觉+抓取算法在 `piper_task/src/piper_task/vision_grasp_core.py` 中实现，经比对与 `juesai/grab2016_7_21.py` **逐字节一致**（即该独立测试脚本已被整合进 `piper_task` 正式节点）。

```bash
roslaunch piper_task arm_function_test.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/arm_test_route.csv \
  target_speed:=0.10
```

`arm_function_test.launch` 会自动包含 `piper_task.launch`（拉起 `piper_task_node.py`），并启动 `arm_function_test_follower.py`（`follow_waypoints.py` 的子类，只保留 `ext:piper_stop_1..7` 任务，启动前校验 CSV 中七个点位齐全，等待 `/piper_task/state` 就绪后才开始跟踪）。

### 5.3 独立抓取测试脚本（不接入车辆任务系统）

`juesai/grab2016_7_16_1.py`、`juesai/grab2016_7_21.py`、`xiaosai/grab2016_5_28_3.py` 等为**一次性独立测试脚本**：启动即自行打开 RealSense、加载各自的 YOLO 模型，按固定顺序执行“扫描位→识别目标→深度定位→平滑笛卡尔抓取→放置扫描→识别放置目标→放置→归位”的完整流程，不依赖也不对接 `piper_mission`/`piper_task`。

**注意**：这些脚本会独占 RealSense 设备，与全流程中 `piper_task`/`piper_mission` 按需打开摄像头的方式冲突。全流程运行时不要单独运行 `grab2016_5_28_3.py` 等脚本，否则会自行直接抓取/放置并抢占摄像头。这类脚本仅适合单独调试机械臂抓取动作时使用。

### 5.4 Piper 底层驱动与服务

Piper 官方驱动节点（`piper_ctrl_single_node.py`，随 `roslaunch piper start_single_piper.launch` 启动）提供：

- `enable_srv`（`piper_msgs/Enable`）：使能/失能机械臂。
- `go_zero_srv`（`piper_msgs/GoZero`）：回零。

```bash
rosservice call /go_zero_srv "is_mit_mode: false"
rosservice call /enable_srv "enable_request: false"
```

逆运动学节点：`piper_pinocchio.py`（订阅目标笛卡尔位姿，发布关节命令）。可视化调试：`roslaunch piper_description display_xacro.launch`。

> 注意区分两套“启用/禁用”机制：Piper 机械臂用的是 `piper_msgs` 服务（`enable_srv`/`go_zero_srv`）；`static_avoid`/`hybrid_avoid` 避障模块用的是各自的 Bool 话题（`/static_avoid/enable`、`/hybrid_avoid/enable`），两者互不相关。

---

## 6. 与竞赛规则的对应关系

依据《自主赛道—快递速达4.0 竞赛规则（第一版）》：

| 规则环节 | 对应实现 |
|---|---|
| 起点挥旗起步 | `race_mission/start_light_vision_node.py`（`flag` 模式）+ `race_mission_manager.py` 的 `WAIT_START` |
| 红绿灯路口通行 | `start_light_vision_node.py`（`light` 模式）+ `ext:traffic_light` 任务 |
| 随机障碍路段（动态） | `avoidance_zone_astar_test.py` 或 `hybrid_avoid`/`static_avoid` 在 `avoid_start`~`avoid_end` 区间内避障 |
| 静态避障（锥桶） | 同上，`static_avoid` 的固定横向偏移绕行 / `hybrid_avoid` 的 A* 重规划 |
| 定点取货（卡片识别+抓取） | `ext:piper_stop_1`（`card`）→ `ext:piper_stop_2/3/4`（`pick1/2/3`），或 `ext:pick` → `piper_mission` |
| 定点卸货（放入对应置物框） | `ext:piper_stop_5/6/7`（`place1/2/3`），或 `ext:place` → `piper_mission` |
| 循环取货→避障→卸货→返程 ×2 | 由 CSV 航迹中重复的任务点序列 + 上述避障/取放逻辑自然覆盖，返程车道段 CSV 中不含 `avoid_*`/`piper_stop_*` 标记 |
| 终点停车 | `ext:finish`（`race_mission_manager.py` 的 `FINISHING`）或 CSV 末尾自然到点停车 |

车辆尺寸要求（规则第 3 条：长≤1.2m、宽≤1m、高≤1m，需醒目急停按钮）与车速限制（≤15km/h）为硬件/整车层要求，需在硬件搭建与 `target_speed` 参数设置时留意，不在软件代码中体现。

---

## 7. 完整流程启动顺序（参考）

以下按终端顺序整理自实际调试记录，供参考（先用低速跑通全流程，再逐步提速）：

```bash
# 终端1：roscore
roscore

# 终端2：配置 CAN（底盘 can0，机械臂 can1，视实际接线为准）
sudo ip link set can0 down && sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up
sudo ip link set can1 down && sudo ip link set can1 type can bitrate 1000000 && sudo ip link set can1 up

# 终端3：底盘驱动
roslaunch bunker_bringup bunker_robot_base.launch

# 终端4：Piper 底层驱动
cd ~/piper_ws && conda activate piper
source /opt/ros/noetic/setup.bash && source devel/setup.bash
bash ~/piper_ws/src/piper_ros/can_activate.sh can1 1000000
roslaunch piper start_single_piper.launch can_port:=can1 auto_enable:=true

# 终端5：Piper 逆运动学
python ~/piper_ws/src/piper_ros/src/piper/scripts/piper_pinocchio/piper_pinocchio.py

# 终端6：机械臂任务节点（二选一：piper_mission 或 piper_task，见第5节）
roslaunch piper_mission arm_task.launch
# 或
roslaunch piper_task piper_task.launch

# 终端7：MID360 驱动
cd ~/livox_ws && source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch

# 终端8：S-FAST_LIO 重定位
cd ~/fastlio_ws && source devel/setup.bash
roslaunch sfast_lio mapping_mid360_relocalization.launch rviz:=true
rostopic hz /Odometry   # 确认定位数据正常

# 终端9：比赛总控（低速联调）
roslaunch race_mission mission.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/xiaosai.csv \
  target_speed:=0.30 wait_for_start:=true auto_start:=false \
  enable_vision:=true show_image:=true
```

调试阶段可用不含机械臂/视觉的纯跟踪或跟踪+避障脚本替代终端9（见第2节），逐步叠加功能验证。

---

## 8. 数据目录

`src/waypoint_tools/data/` 下为历次录制的航迹 CSV（按文件名时间戳区分），`arm_test_route.csv` 为机械臂七点位测试专用航迹（含 `piper_stop_1..7` 标记）。检查录制结果常用命令：

```bash
grep -n "piper_stop" ~/fastlio_ws/src/waypoint_tools/data/arm_test_route.csv
grep -c "piper_stop" ~/fastlio_ws/src/waypoint_tools/data/arm_test_route.csv
tail -n 10 ~/fastlio_ws/src/waypoint_tools/data/arm_test_route.csv
```

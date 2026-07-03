# Waypoint Tools 代码结构与逻辑详解

## 目录

1. [整体架构概览](#1-整体架构概览)
2. [record_waypoints.py — 航迹点录制器](#2-record_waypointspy--航迹点录制器)
3. [follow_waypoints.py — 航迹点跟踪器（纯控制）](#3-follow_waypointspy--航迹点跟踪器纯控制)
4. [waypoint_sender.py — 航迹点发送器（move_base）](#4-waypoint_senderpy--航迹点发送器move_base)
5. [两种跟踪方案对比](#5-两种跟踪方案对比)
6. [速度忽快忽慢问题分析](#6-速度忽快忽慢问题分析)

---

## 1. 整体架构概览

本项目提供了一套完整的 ROS 航迹点录制与回放系统，包含三个核心脚本：

```
record_waypoints.py   ──录制──>  CSV 文件  ──回放──>  follow_waypoints.py (纯P控制)
                                                  └──>  waypoint_sender.py  (move_base + TEB)
```

数据流：
- 录制阶段：订阅 `/Odometry`，按距离/航向变化自动采样，输出 CSV
- 回放阶段（方案A）：`follow_waypoints.py` 读取 CSV，自己做 P 控制器直接发 `cmd_vel`
- 回放阶段（方案B）：`waypoint_sender.py` 读取 CSV，通过 `move_base` action 逐点发送目标，由 TEB 规划器负责路径跟踪

CSV 格式：`seq, stamp, frame_id, x, y, z, qx, qy, qz, qw, yaw, task, tol`

---

## 2. record_waypoints.py — 航迹点录制器

### 2.1 类结构

```
WaypointRecorder
├── __init__()          # 初始化参数、打开CSV、订阅话题
├── odom_callback()     # 里程计回调，自动判断是否保存
├── task_callback()     # 手动标记任务点
├── stop_callback()     # 手动标记停车点
├── save_waypoint()     # 实际写入CSV的方法
└── on_shutdown()       # 关闭时flush文件
```

### 2.2 参数列表

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `~odom_topic` | `/Odometry` | 里程计话题 |
| `~task_topic` | `/waypoint_task` | 手动任务标记话题 |
| `~stop_seconds_topic` | `/waypoint_stop_seconds` | 手动停车标记话题 |
| `~min_distance` | `0.30` m | 自动采样最小距离阈值 |
| `~min_yaw_change` | `0.35` rad (~20°) | 自动采样最小航向变化阈值 |
| `~default_task` | `none` | 自动采样点的默认任务 |
| `~default_tol` | `0.30` m | 默认到达容差 |
| `~default_stop_seconds` | `10.0` s | 默认停车时长 |
| `~record_z` | `False` | 是否记录Z轴 |
| `~flush_every` | `1` | 每N个点flush一次文件 |
| `~frame_id` | `""` | 强制覆盖frame_id（空则用odom消息中的） |
| `~file_name` | `""` | 输出文件名（空则自动生成时间戳名） |
| `~output_dir` | `../data` | 输出目录 |

### 2.3 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `~odom_topic` | `nav_msgs/Odometry` | 里程计，用于自动采样 |
| `~task_topic` | `std_msgs/String` | 接收任务名称字符串，强制在当前位置写入一个任务航迹点 |
| `~stop_seconds_topic` | `std_msgs/Float32` | 接收停车秒数，强制写入一个 `stop_Xs` 航迹点 |

### 2.4 自动采样逻辑

在 `odom_callback()` 中，每收到一帧里程计就判断：

```
如果是第一个点 → 直接保存（reason="first_point"）
否则：
  计算与上一个保存点的距离 dist = hypot(dx, dy)
  计算与上一个保存点的航向差 dyaw = |wrap_to_pi(yaw - last_yaw)|
  
  如果 dist >= min_distance (0.30m) → 保存（reason="distance=X.XXXm"）
  否则如果 dyaw >= min_yaw_change (0.35rad) → 保存（reason="yaw_change=X.XXXrad"）
```

这意味着：
- 直线行驶时，大约每 0.3m 记录一个点
- 转弯时，即使距离不够，航向变化超过 ~20° 也会记录
- 录制密度取决于机器人行驶速度和里程计频率

### 2.5 手动标记逻辑

- `task_callback`：收到 String 消息后，在当前位置强制写入一个航迹点，task 字段设为收到的字符串
- `stop_callback`：收到 Float32 消息后，生成 `stop_Xs` 格式的 task 名称，强制写入
  - 整数秒：`stop_10s`
  - 小数秒：`stop_2.5s`

### 2.6 CSV 写入

`save_waypoint()` 方法：
- 如果 `force=False`（自动采样），会额外检查与上一个保存点的距离和航向差是否都极小（<1e-6），避免重复写入
- 如果 `force=True`（手动标记），跳过上述检查，直接写入
- 每写入 `flush_every` 个点就 flush + fsync，确保数据不丢失

---

## 3. follow_waypoints.py — 航迹点跟踪器（纯控制）

### 3.1 类结构

```
WaypointFollower
├── __init__()              # 初始化参数、加载CSV、订阅/发布话题
├── find_latest_csv()       # 自动查找最新CSV
├── load_waypoints()        # 读取CSV为航迹点列表
├── odom_callback()         # 里程计回调，更新当前位姿
├── task_done_callback()    # 外部任务完成回调
├── publish_cmd()           # 发布速度指令
├── stop_robot()            # 停车（发布零速度）
├── publish_task_event()    # 发布任务事件
├── parse_stop_seconds()    # 解析停车任务字符串
├── start_task()            # 启动任务处理
├── handle_task_state()     # TASK状态处理
├── handle_align_final()    # 终点航向对齐
├── control_step()          # 主控制循环（状态机核心）
├── spin()                  # 主循环
└── on_shutdown()           # 关闭回调
```

### 3.2 参数列表

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `~odom_topic` | `/Odometry` | 里程计话题 |
| `~cmd_topic` | `/smoother_cmd_vel` | 速度指令输出话题 |
| `~control_rate` | `10.0` Hz | 控制循环频率 |
| `~max_linear` | `0.6` m/s | 最大线速度 |
| `~max_angular` | `0.60` rad/s | 最大角速度 |
| `~k_linear` | `0.80` | 线速度P增益 |
| `~k_angular` | `1.60` | 角速度P增益 |
| `~final_yaw_k` | `1.50` | 终点航向对齐P增益 |
| `~final_yaw_tolerance` | `0.08` rad (~4.6°) | 终点航向对齐容差 |
| `~goal_tolerance_default` | `0.30` m | 默认到达容差 |
| `~rotate_in_place_angle` | `1.10` rad (~63°) | 航向误差超过此值时原地旋转 |
| `~slowdown_angle` | `0.50` rad (~29°) | 航向误差超过此值时减速 |
| `~finish_stop_time` | `1.0` s | 完成后停车时间 |
| `~detect_pause_time` | `2.0` s | detect任务暂停时间 |
| `~external_task_timeout` | `180.0` s | 外部任务超时 |
| `~unknown_task_policy` | `skip` | 未知任务策略（skip/hold） |
| `~task_event_topic` | `/waypoint_task_event` | 任务事件发布话题 |
| `~task_done_topic` | `/waypoint_task_done` | 任务完成订阅话题 |
| `~csv_path` | `""` | CSV路径（空则自动查找最新） |

### 3.3 话题

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `~odom_topic` | 订阅 | `Odometry` | 里程计 |
| `~task_done_topic` | 订阅 | `String` | 外部任务完成信号 |
| `~cmd_topic` | 发布 | `Twist` | 速度指令 |
| `~task_event_topic` | 发布 | `String` | 任务事件（start/done/skip） |

### 3.4 状态机

```
WAIT_ODOM ──(收到第一帧odom)──> TRACK
                                  │
                    ┌─────────────┤
                    │             │
                    ▼             ▼
              (中间普通点)    (中间任务点/终点)
              到达后直接       到达后停车
              current_index++  ──> TASK
                    │                │
                    │          (任务完成)
                    │                │
                    ▼                ▼
                  TRACK        TRACK 或 ALIGN_FINAL
                                     │
                               (航向对齐完成)
                                     │
                                     ▼
                                  FINISH
```

状态详解：

#### WAIT_ODOM
- 初始状态，等待第一帧里程计
- 收到后自动切换到 TRACK

#### TRACK（核心跟踪状态）
- 每个控制周期（10Hz）执行一次
- 计算当前位置到目标航迹点的距离和方向

**到达判断逻辑：**
```python
distance = hypot(target_x - current_x, target_y - current_y)

# 情况1：中间普通点（无任务），到达容差内
if distance <= tol and (not is_last) and task == "none":
    → 直接 current_index++，不停车，继续追下一个点

# 情况2：中间任务点，到达容差内
if distance <= tol and (not is_last) and task != "none":
    → 停车，进入 TASK 状态执行任务

# 情况3：终点，到达容差内
if distance <= tol and is_last:
    → 停车，如有任务先执行任务再 ALIGN_FINAL，否则直接 ALIGN_FINAL
```

**速度控制逻辑（未到达时）：**
```python
target_heading = atan2(dy, dx)                    # 目标方向
heading_error = wrap_to_pi(target_heading - current_yaw)  # 航向误差

linear_x  = clamp(k_linear * distance, 0, max_linear)    # P控制线速度
angular_z = clamp(k_angular * heading_error, -max_angular, max_angular)  # P控制角速度

# 大角度偏差时的处理：
if |heading_error| > rotate_in_place_angle (1.10 rad ≈ 63°):
    linear_x = 0  → 原地旋转，不前进
elif |heading_error| > slowdown_angle (0.50 rad ≈ 29°):
    linear_x *= 0.35  → 减速到35%
```

**关键特征：这是一个纯 P 控制器，没有前馈、没有轨迹插值、没有速度平滑。**

#### TASK
- 停车状态，等待任务完成
- 支持三种任务类型：
  1. `stop_Xs` / `hold_Xs` / `pause_Xs`：定时停车（支持秒/分钟单位）
  2. `detect`：占位符，暂停 `detect_pause_time` 秒
  3. `ext:xxx`：外部任务，发布事件后等待外部节点回复 `done` 消息
- 未知任务根据 `unknown_task_policy` 决定 skip 或 hold

#### ALIGN_FINAL
- 仅在终点使用
- 纯角速度P控制，对齐到终点航迹点记录的 yaw 值
- 对齐到 `final_yaw_tolerance`（0.08 rad ≈ 4.6°）后进入 FINISH

#### FINISH
- 持续发布零速度
- 程序不退出，保持运行

---

## 4. waypoint_sender.py — 航迹点发送器（move_base）

### 4.1 类结构

```
WaypointSender
├── __init__()          # 初始化参数、加载CSV、稀疏化、连接move_base
├── find_latest_csv()   # 自动查找最新CSV
├── load_waypoints()    # 读取CSV
├── sparsify()          # 航迹点稀疏化
├── task_done_cb()      # 外部任务完成回调
├── handle_task()       # 任务处理
├── send_goal()         # 发送move_base目标
└── run()               # 主循环
```

### 4.2 参数列表

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `~csv_path` | `""` | CSV路径 |
| `~odom_topic` | `/Odometry` | 里程计话题 |
| `~goal_tolerance` | `0.30` m | 到达容差 |
| `~skip_distance` | `1.5` m | 稀疏化最小间距 |
| `~external_task_timeout` | `180.0` s | 外部任务超时 |
| `~unknown_task_policy` | `skip` | 未知任务策略 |
| `~task_event_topic` | `/waypoint_task_event` | 任务事件话题 |
| `~task_done_topic` | `/waypoint_task_done` | 任务完成话题 |

### 4.3 稀疏化逻辑

`sparsify()` 方法对录制的密集航迹点进行稀疏化：

```python
保留规则：
1. 始终保留第一个点和最后一个点
2. 始终保留有任务的点（task != "none"）
3. 中间普通点：只保留与上一个保留点距离 >= skip_distance (1.5m) 的点
```

这是因为 move_base 有自己的全局/局部规划器，不需要密集的航迹点，只需要稀疏的目标点序列。

### 4.4 执行逻辑

```python
for each waypoint:
    1. send_goal(wp)           # 通过 move_base action 发送目标
    2. wait_for_result(120s)   # 等待 move_base 报告到达（最多120秒）
    3. 检查结果：
       - 超时 → cancel_goal，继续下一个
       - 失败 → 继续下一个
       - 成功 → 处理任务
    4. handle_task(task)       # 处理任务（同步阻塞）
```

### 4.5 与 move_base 的交互

- 使用 `actionlib.SimpleActionClient` 连接 `move_base`
- 目标格式：`MoveBaseGoal`，包含位置 (x, y) 和四元数朝向 (qx, qy, qz, qw)
- frame_id 固定为 `"map"`
- move_base 内部使用 TEB 局部规划器进行路径跟踪和避障

---

## 5. 两种跟踪方案对比

| 特性 | follow_waypoints.py | waypoint_sender.py |
|------|---------------------|-------------------|
| 控制方式 | 纯P控制器，直接发 cmd_vel | move_base + TEB 规划器 |
| 避障能力 | **无** | 有（costmap + TEB） |
| 路径规划 | **无**，点到点直线 | 有（global_planner + TEB） |
| 速度平滑 | **无** | TEB 内置加速度约束 |
| 航迹点密度 | 使用原始密集点（~0.3m间距） | 稀疏化后（~1.5m间距） |
| 适用场景 | 简单开阔环境、精确轨迹复现 | 有障碍物的复杂环境 |
| 能否跟踪轨迹 | 逐点追踪，不是真正的轨迹跟踪 | TEB 会规划平滑轨迹 |

---

## 6. 速度忽快忽慢问题分析

### 6.1 问题描述

将 `max_linear` 设为 0.8 m/s 后，机器人在航迹点之间出现速度一快一慢的现象。

### 6.2 根本原因分析

**如果使用的是 `follow_waypoints.py`（纯P控制器）：**

这是问题的核心所在。速度计算公式为：

```python
linear_x = clamp(k_linear * distance, 0.0, max_linear)
```

其中 `k_linear = 0.80`，`max_linear = 0.80`。

**速度变化过程（以两个相邻航迹点为例）：**

```
距离远时：  linear_x = 0.80 * distance → 被 clamp 到 0.80 m/s（全速）
接近航迹点：linear_x = 0.80 * 0.5m = 0.40 m/s（减速）
更近时：    linear_x = 0.80 * 0.35m = 0.28 m/s（继续减速）
到达容差内：distance <= 0.30m → 切换到下一个点
切换瞬间：  distance 突然变大（到下一个点的距离）→ 速度突然跳回 0.80 m/s
```

**这就是"一快一慢"的根源：**

1. **接近当前目标点时**：距离越来越小 → P控制器输出的速度线性下降 → 机器人减速
2. **切换到下一个目标点的瞬间**：距离突然变大 → 速度突然跳到最大值 → 机器人急加速
3. **周而复始**：每经过一个航迹点就重复一次"减速→急加速"的循环

由于录制时航迹点间距约 0.3m（`min_distance = 0.30`），这个减速-加速循环非常频繁。

**加剧因素：**

- 航迹点间距太密（0.3m），导致机器人刚加速就要开始减速
- 纯P控制没有速度平滑/加速度限制，速度变化是瞬时的
- 没有前瞻（lookahead）机制，只看当前目标点，不考虑后续路径
- 航向误差导致的额外减速（`slowdown_angle` 和 `rotate_in_place_angle`）会叠加

**速度曲线示意：**

```
速度
0.8 ┤  ╱╲    ╱╲    ╱╲    ╱╲    ╱╲
    │ ╱  ╲  ╱  ╲  ╱  ╲  ╱  ╲  ╱  ╲
0.4 ┤╱    ╲╱    ╲╱    ╲╱    ╲╱    ╲╱
    │
0.0 ┼──────────────────────────────── 时间
     wp0  wp1  wp2  wp3  wp4  wp5
```

每个航迹点处速度降到最低，然后立刻跳回最高，形成锯齿波。

### 6.3 如果使用的是 `waypoint_sender.py`（move_base + TEB）

TEB 规划器的 `max_vel_x` 在配置中设为 `0.3 m/s`，而你可能在 follow_waypoints.py 中设了 0.8。如果用的是 waypoint_sender 方案，TEB 自身有加速度约束（`acc_lim_x = 0.5`），速度变化会平滑得多，但在航迹点切换时仍然会有停顿（因为 `wait_for_result` 是阻塞的，到达一个点后才发下一个）。

### 6.4 当前跟踪逻辑总结

**`follow_waypoints.py` 的跟踪逻辑本质上不是"轨迹跟踪"，而是"逐点追踪"：**

- 它一次只看一个目标点
- 用 P 控制器计算线速度和角速度
- 到达容差内后立即切换到下一个点
- 没有路径插值、没有前瞻、没有速度规划

**它不能真正跟踪轨迹。** 真正的轨迹跟踪需要：
- 将离散航迹点插值为连续曲线（如样条曲线）
- 使用 Pure Pursuit / Stanley / MPC 等轨迹跟踪算法
- 有前瞻距离（lookahead），根据前方路径曲率调整速度
- 有速度规划，在弯道前提前减速

### 6.5 可能的改进方向（仅列出，不修改代码）

1. **增加前瞻机制**：不追当前最近点，而是追前方一定距离的点（Pure Pursuit 思路）
2. **速度平滑**：加入加速度限制，防止速度突变
3. **增大航迹点间距或稀疏化**：减少减速-加速的频率
4. **使用 move_base + TEB 方案**：TEB 自带速度平滑和轨迹优化
5. **实现真正的轨迹跟踪**：将航迹点插值为平滑曲线，使用 Pure Pursuit 等算法跟踪

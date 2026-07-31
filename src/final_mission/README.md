# final_mission — 决赛总控

2026 第六届智能无人系统应用挑战赛 自主赛道·快递速达 4.0 的决赛集成包。

车：松灵 Bunker Mini（履带差速）。臂：松灵 Piper 六自由度。
传感器：Livox MID360 激光雷达，一台 Intel RealSense D435i（装在机械臂上）。

## 这个包做什么，不做什么

**做**：把已经各自跑通的四块功能串成一条完整赛道流程——分区避障巡航、
任务点精确停靠、机械臂七点位取放、红旗起步与红绿灯放行。

**不做**：不重写任何底层算法。跟踪与避障直接继承
`waypoint_tools/scripts/avoidance_zone_astar_test.py`，机械臂动作完全交给
`piper_task`，视觉抓取算法在 `piper_task/src/piper_task/vision_grasp_core.py`
里原样不动。本包只补三样原来缺的东西：任务点停靠精度、失败重试与放行、
起步与红绿灯闸门。

**没有修改任何原有文件。** `fastlio_ws` 和 `piper_ws` 里所有既有代码保持原样，
本包通过 Python 继承和 ROS 话题接入。唯一还需要你决定的改动见文末
「相机归属」一节。

## 硬件前置进程

六个终端，每个都要先 `source devel/setup.bash`。顺序有讲究：底盘和雷达
要先起，重定位需要雷达数据；机械臂使能要在逆解之前。

| # | 作用 | 命令 |
|---|---|---|
| 1 | 底盘 | `roslaunch bunker_bringup bunker_robot_base.launch` |
| 2 | 雷达 | `roslaunch livox_ros_driver2 msg_MID360.launch` |
| 3 | 重定位 | `roslaunch sfast_lio mapping_mid360_relocalization.launch` |
| 4 | 臂使能 | `roslaunch piper start_single_piper.launch can_port:=can1 auto_enable:=true` |
| 5 | 逆解 | `python piper_pinocchio.py` |
| 6 | 臂任务 | `roslaunch piper_task piper_task.launch enable_camera_relay:=true` |

第 6 步的 `enable_camera_relay:=true` **必须加**：全车只有一台 D435i 且被
`piper_task` 独占，不打开中继，红旗和红绿灯识别拿不到任何图像。忘了加也不会崩
——总控等红旗 10 秒后会报错并提示改用手动起步。

底盘 `port_name` 在 launch 里写死 `value="can0"`，不需要也不能从命令行覆盖。
机械臂走 `can1`，两条 CAN 不冲突。

**不要用 `piper_task` 的 `arm_function_test.launch`**：它自带一个 follower，
会把航迹里的 `avoid_start`/`avoid_end` 当成未知任务抹掉，避障直接失效。
本包的 `arm_only.launch` 是它的替代品。

## 一个硬前提：开机位姿

`S-FAST_LIO/src/laserMapping_re.cpp:372` 直接加载 `PCD/GlobalMap_ikdtree.pcd`，
`:369-381` 是裸的 `ikdtree.Build()`，没有 ICP 也没有 NDT 配准；
`config/mid360.yaml` 里 `init_pos: [0,0,0]`、`init_rot: [0,0,0,1]`，
并且没有订阅 `/initialpose`。

结论：**车必须物理停在建图起点的原点位姿上开机**，否则整条航迹全程平移或
旋转偏移，且没有任何机制能纠正。这是全系统最容易一次性搞砸比赛的地方。

## 怎么运行

### 正式比赛

```bash
roslaunch final_mission final_race.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/final_route.csv
```

起来之后车**不动**，等红旗。看到 `总控状态：BOOT -> WAIT_FLAG` 就是正常的。
裁判挥旗后视觉自动识别放行，全程无需人工干预。

红旗识别不上时手动起步（备用手段，先练熟）：

```bash
rostopic pub -1 /final_mission/command std_msgs/String "data: 'start'"
```

紧急停车：

```bash
rostopic pub -1 /final_mission/command std_msgs/String "data: 'stop'"
```

### 赛前航迹校验

录完航迹先跑这个，不启动任何节点，纯静态检查：

```bash
rosrun final_mission validate_route.py \
  /home/user/fastlio_ws/src/waypoint_tools/data/final_route.csv --rounds 2
```

必须零 ERROR 才能上场。`--rounds 0` 跳过七点位检查（用于校验预赛航迹）。

### 空跑标定（不开机械臂、不开视觉）

```bash
roslaunch final_mission dry_run.launch csv_path:=<csv> task_pause:=3.0
```

车会走完全程，在每个任务点停 3 秒然后自己继续。日志里每个任务点会打一条
`[TASK]`，含位置误差、朝向误差、逼近耗时——这是标定停靠参数的唯一数据来源。

### 只调机械臂（不等红旗、不看红绿灯）

```bash
roslaunch final_mission arm_only.launch csv_path:=<csv>
```

## 系统架构

```
        ┌─────────────────────────────────────────────────┐
        │  final_manager   总控：起步闸门 / 红绿灯 / 终点    │
        └───┬──────────────────────────────┬──────────────┘
   tracker_enable                    vision_control
            │                              │
   ┌────────▼─────────┐          ┌─────────▼──────────┐
   │  final_tracker   │          │   final_vision     │
   │  唯一速度权       │          │   红旗 / 红绿灯     │
   │  继承分区A*避障   │          └────────────────────┘
   └────────┬─────────┘
            │ /smoother_cmd_vel          ┌────────────────┐
            ▼                            │   arm_bridge   │
        底盘（Bunker Mini）                │  重试 / 放行    │
                                         └───────┬────────┘
   /waypoint_task_event ──┬──► piper_task ───────┘
   （一条事件，三方各取所需）  ├──► arm_bridge
                          └──► final_manager
```

### 权责划分

整个设计只有一条主线：**每类资源只有一个主人**。

| 资源 | 唯一主人 | 其他节点怎么办 |
|---|---|---|
| 速度（`/smoother_cmd_vel`） | `final_tracker` | 总控只在停止/终点发零速 |
| 相机（D435i） | `piper_task` | `final_vision` 看话题或分时取用 |
| 机械臂动作 | `piper_task` | `arm_bridge` 只发命令不抢执行 |
| 航迹进度 | `final_tracker` | 无人可改 |

这么切是因为速度权分给两个控制器时，两边各自维护进度状态，切换瞬间会抖。
早期的 `static_avoid` / `hybrid_avoid` 方案就是这个问题（它们在禁用状态下
仍持续发零速，没法共用一个 `cmd` 话题），已经淘汰。

---

# 模块详细逻辑

## 1. final_tracker — 统一跟踪器

`scripts/final_tracker.py`，约 590 行。**唯一往底盘发速度的节点。**

### 继承关系

```
FinalTracker                    本包新增：任务点停靠 + 外部握手 + 起步闸门
  └─ AvoidanceZoneAStarTest     waypoint_tools：分区门控 + 定位跳变保护
       └─ PurePursuitAStarFollower   waypoint_tools：A* + Pure Pursuit + 安全层
```

用 `rospkg` 把 `waypoint_tools/scripts` 加进 `sys.path` 再 import——那两个文件
是脚本不是 Python 包，没有 `__init__.py`，只能这么引。

选这条继承链的理由：`avoidance_zone_astar_test.py` 是你实测跑通的避障方案，
重写等于丢掉已验证的代码。而它的 `load_waypoints` 已经把 CSV 的 `task` 列读进
`AvoidanceWaypoint.task` 却从来不用——这就是现成的挂载点。

### 巡航段：A* 到底有没有用

**有，每个控制周期都在跑，但跑在一张空栅格上。**

`control_step` 无条件调 `replan_to_global_waypoint`；
[pure_pursuit_astar_follower.py:719-720](../waypoint_tools/scripts/pure_pursuit_astar_follower.py#L719-L720)
只在 `planner_enabled` 为假时才跳过 A*，而它默认为真。所以 `:739` 的
`astar_plan` 照常执行，`:584` 取障碍物快照，而
[avoidance_zone_astar_test.py:385-390](../waypoint_tools/scripts/avoidance_zone_astar_test.py#L385-L390)
在区外返回空表。空表进到 `:587` 的膨胀循环等于什么都不做，占据栅格全零，
A* 在全空图上搜出直线，再被视线平滑压成两点。

所以「区外关避障」准确说是**关障碍物，不是关规划器**。代价是每
`replan_min_interval`（0.30s）白建一次栅格，栅格受 `margin`≈0.5m 和
`max_grid_cells` 约束，很小，实测无性能问题。

想真正短路掉，干净做法是按 zone 状态动态改 `planner_enabled`，让区外走
`:720-727` 的直连分支。**不建议赛前动**：收益只有一点 CPU，却要重新验证
整条已跑通的路径。

### 避障段

进入 `avoid_start`~`avoid_end` 区间后 `zone_active` 置真，障碍物快照非空，
A* 和反应式安全层才真正起作用。点云回调在区外也持续缓存，所以入区立刻有
新鲜障碍，不需要等一帧。

反应式安全层（父类默认值）：`safety_slow_dist` 0.60m 减速、
`safety_stop_dist` 0.28m 停车、`safety_emergency_dist` 0.16m 紧急。
这层独立于 A*，是最后一道防线。

### 任务点三段式停靠

这是精度的来源，也是本包最核心的新增逻辑。

```
巡航 ─── s 剩余 < 0.80m ───► SLOWDOWN
                              临时压低 nominal_target_speed，
                              仍走 Pure Pursuit，父类减速斜率生效
        ─── 距离 < 0.45m ───► APPROACH
                              切 P 控制直奔目标坐标，限速 0.10 m/s
                              比 Pure Pursuit 横向外摆小
        ─── 距离 < 0.05m ───► ALIGN_YAW
                              原地转向，误差 > 0.10rad 则跳过
        ─────────────────────► TASK
                              stop_robot() 立即到零，发事件，等 done
```

`approach_switch_dist`(0.45) **必须大于**候选点最大间距(0.40)。三个抓取点和
三个放置点相邻只有 30~40cm，如果切换距离小于间距，车会在 Pure Pursuit 和
P 控制之间反复切换。设成 0.45 后候选点之间全程 P 控制，只有一种控制律。

`task_trigger_margin`(0.80) 必须大于 `approach_switch_dist`，留出从巡航
平滑降速的距离。巡航 0.35m/s 的制动距离约 12cm，蠕行 0.10m/s 约 3cm，
都远小于触发余量，不会冲过点。

停车用 `stop_robot()` 而不是发零速指令：它设 `force_stop_cmd=True`
绕过父类的减速斜率限幅（[:656-661](../waypoint_tools/scripts/avoidance_zone_astar_test.py#L656-L661)），
保证立即到零。

`task_yaw_max_correction`(0.10rad ≈ 5.7°) 是护臂的：机械臂侧伸 0.6m 时，
车转 5.7° 臂尖横移约 6cm。超过这个角度宁可带着朝向误差进任务，也不自转。

### 为什么 ±8cm/±5° 就够

视觉抓取在**机械臂基座坐标系**闭环：

```
T_base_object = T_base_ee @ T_EE_TO_CAM @ T_cam_object_point
```

全程不经过里程计和地图坐标系，所以车的停车误差被视觉吃掉。±8cm/±5° 是
「目标还在相机视野里」的可用性门槛，不是控制目标。而且录航迹和回放用的是
同一套重定位，系统性地图偏差会自相抵消。

30~40cm 的紧间距反而让这个论证更强：相邻视角有约 0.27m 重叠，远大于 8cm 误差。

### 两轮怎么不串

`piper_stop_1` 在两轮 CSV 里出现两次。用**非回退游标** `task_cursor` 而不是
按值删除列表项——`pure_pursuit_follower.py:499` 那种 `task_indices.remove()`
只能跑一轮。

`navigation_skip` 到来时把后续候选点的 `task` 改成 `"none"`，
`next_task_index` 每次重新读 task 字符串，遇到 `"none"` 就跳过。抹除时
撞到下一个 `piper_stop_1` 立即 break，第二轮的点不受影响。

### 起步闸门

`enabled` 默认 **False**（脚本内默认值，不在 launch 里设）：未使能时持续发
零速钉在起点。总控在红旗确认后通过 `/final_mission/tracker_enable`
（锁存 Bool）放行。

默认 False 是安全属性：总控晚起或崩了，车不动；默认 True 则节点一起来就
往前开。锁存意味着即使跟踪器比总控晚起，也能收到之前发布的状态。

## 2. 机械臂七点位：谁在做什么

七个停靠点的分工（`piper_task/config/task_config.yaml` 定义，本包保持一致）：

| 停靠点 | 动作 | 含义 |
|---|---|---|
| `piper_stop_1` | `card` | 读目标卡片，决定这轮抓什么 |
| `piper_stop_2/3/4` | `pick1/2/3` | 三个抓取候选位置 |
| `piper_stop_5/6/7` | `place1/2/3` | 三个放置候选位置 |

### 固定姿态的唯一出处

**所有机械臂固定姿态都在 `piper_task/scripts/piper_task_node.py:32-34`，
是实物调过的，只有那一份。`final_mission` 不定义、不覆盖、不发布任何关节角。**

```python
PICK_SCAN_JOINTS  = [-1.530, 0.446, 0.0, 0.0, -0.115, 0.0, 0.0]
TRANSPORT_JOINTS  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
PLACE_SCAN_JOINTS = [-1.602, 0.641, -0.509, 0.0, 0.324, 0.0, 0.0]
JOINT_MOVE_WAIT   = 8.0
```

本文档提到的角度和秒数都是上面这些值的换算，不是新设的参数：

| 文档里的说法 | 出处 | 换算 |
|---|---|---|
| PICK_SCAN 侧伸约 88° | `PICK_SCAN_JOINTS[0] = -1.530` | -1.530 rad = **-87.66°** |
| PLACE_SCAN 侧伸约 92° | `PLACE_SCAN_JOINTS[0] = -1.602` | -1.602 rad = **-91.79°** |
| 摆一次姿态 8s | `JOINT_MOVE_WAIT = 8.0` | 原样 |
| 零位 / 运输姿态 | `TRANSPORT_JOINTS` 全 0 | 原样 |

唯一**不是**从你代码里来的数是「臂尖伸出约 0.6m」——那是我为了估算
`task_yaw_max_correction` 自己假设的几何量，代码里没有这个值。
它只用来算「转 5.7° 臂尖横移约 6cm」这个上限依据，已在
`config/tracker.yaml` 里标注为估值。实测臂尖伸出距离后可以重算这个上限。

### 机械臂什么时候动、什么时候不动

**这是有意的设计，不要改。**

`PICK_SCAN_JOINTS` 全文只在一处下发：
[piper_task_node.py:314](../../../piper_ws/src/piper_ros/src/piper_task/scripts/piper_task_node.py#L314)，
在 `execute_card` 里，也就是 `piper_stop_1` 那一次。

之后车开到 `piper_stop_2/3/4` 走 `execute_pick_candidate`，这个函数里
**没有任何关节下发**——第一件事就是 `:353` 的 `get_object_pose` 直接读相机。
所以：

- `piper_stop_1`：臂从零位转到 PICK_SCAN（侧伸约 88°），耗时 8s。
- 车开到 `piper_stop_2`：**臂完全不动**，保持侧伸。
- 没认出目标 → `:355-356` `return True, "not_here_continue_to_pick2"`，
  一行都没碰机械臂。
- 车开到 `piper_stop_3`、`piper_stop_4`：**同样不动**。

臂真正再动只有两种情况：某个候选点认出了目标，从 `:360` 开始预抓取；
或三个点全没认出，`:357` 返回 False，臂停在侧伸姿态。

放置侧更明确，`:428` 写着 `if candidate == 1 and not self.move_joint_pose(...)`，
那个 `candidate == 1` 就是「只在 `piper_stop_5` 摆一次」，`stop_6`、`stop_7`
不再摆。原注释也写了这是有意的。

**为什么这样设计**：臂摆一次扫描姿态 8s，三个候选点每次收放要多花 48s，
30 分钟总时长扛不住。保持侧伸让相机视野一直对着桌面，车挪 30~40cm 就换一个
视角，这是最省时间的做法。

对本包的唯一影响是两个跟踪器参数：`approach_max_speed: 0.10`（侧伸时限速）
和 `task_yaw_max_correction: 0.10`（不让车大幅自转）。这两个都在本包的
`config/tracker.yaml` 里，`piper_task` 一个字没动。

## 3. arm_bridge — 重试与放行

`scripts/arm_bridge.py`，约 320 行。**不抢执行权**：`piper_task` 自己订阅
`/waypoint_task_event`，车一停就直接执行动作。本节点只补 `piper_task` 缺的两件事。

### 补的第一件事：原地重试

`piper_task` 已有「换点重试」——`stop_2` 没找到就发 done 让车开去 `stop_3`
再到 `stop_4`。但三个候选点都失败后直接返回 False，不再尝试。

规则允许每轮最多 3 次装货机会（第 2 次成功 90 分，第 3 次 60 分），
所以在最后一个候选点原地重发命令，把机会用足。
`retry_counts` 默认 `{card: 2, pick: 2, place: 2}`，即共 3 次尝试。

只有这五种 reason 才重试（已逐字核对与 `piper_task` 返回值一致）：

```
reference_not_recognized              卡片没认出
reference_scan_failed                 卡片扫描姿态没到位
object_not_found_at_all_pick_points   三个抓取点都没找到
target_not_found_at_all_place_points  三个放置点都没找到
place_scan_failed                     放置扫描姿态没到位
```

其余是逻辑错误或机械故障，重试无意义。

**重发要延迟 1s**。`piper_task` 在 `run()` 里先 `publish_result`（`:266`）
再清 `busy`（`:237`），收到失败结果立刻重发有概率撞上 `busy` 被丢弃，
而那条 `rejected:busy:<动作>` 消息的 command 字段是 `busy` 不是动作名，
不特判就会被忽略，然后白等满 60s——一次重试静默变成跳过。
现在延迟重发 + 识别 `rejected:busy` 重新排队，两条路都堵上了。

### 补的第二件事：失败放行

`piper_task` 只在 `ok==True` 时发 done（`:229`），失败路径不发，
车会一直停到跟踪器的 `external_task_timeout`（150s）。
七点位两轮共 14 个卡点，30 分钟扛不住几次。

重试用尽后本节点代发 `done:<task>`：本环节 0 分，但保住避障、红绿灯、
终点停车的分，也不浪费 150s。

还处理三种 `piper_task` 预处理失败（`action_not_configured` /
`invalid_action` / `arm_busy`），这三种它都不发 done，原来车会一直停着。

### 为什么 piper_task 不用改

关键前提（已核实）：`piper_task` 的 `command_callback` 入队时
`waypoint_task_name` 为 `None`（`:178`）。所以经 `/piper_task/command`
重发的命令**即使成功也不会自动发 done**——放行时机完全由本节点掌握，
不会与 `piper_task` 重复发。

### 兜底

`arm_action_timeout`（60s）内收不到任何结果就认为机械臂进程异常，代发 done。
另外识别 `success:waypoint:<task>:skip_without_arm`（候选点被
`navigation_skip` 跳过，`piper_task` 已自行发 done）并撤销盯守，
否则 60s 后会补发一条过期 done，刷出假的「机械臂异常」报警掩盖真故障。

## 4. final_manager — 总控

`scripts/final_manager.py`，约 245 行。**这就是总控节点**，入口是
`final_race.launch`。职责刻意收窄，只管三件事。

### 状态机

```
BOOT ──► WAIT_FLAG ──红旗/手动──► RUN ──┬──► WAIT_LIGHT ──绿灯/超时──► RUN
                                       └──► FINISHING ──3s──► DONE

任意状态 ──stop──► STOPPED        红绿灯超时且 hold 策略 ──► ERROR
```

- **起步**：压住跟踪器，视觉切 flag 模式，等红旗。
- **红绿灯**：收到 `start:traffic_light` 后视觉切 light 模式，等绿灯。
  等满 `traffic_timeout`(120s) 按 `pass` 策略放行而不是死等——
  宁可闯一次也不要卡死在场上。要死等就改成 `hold`。
- **终点**：收到 `start:finish` 发 done，延迟 3s 后禁用跟踪器并持续发零速。

### 不做的事

不管路径跟踪与避障（`final_tracker`）、不管机械臂动作（`piper_task` 自己订阅
事件）、不管机械臂重试（`arm_bridge`）。

与老的 `race_mission_manager` 最大区别：**不再用 `subprocess` 拉起跟踪器**。
进程生命周期和参数传递都太脆，现在跟踪器由 launch 统一管理，总控只用一个
锁存话题控它起停。

### 红绿灯不需要抬臂

机械臂零位（`piper_task` 的 `TRANSPORT_JOINTS`）下相机前视，看灯不用动臂。

如果实测零位看不到灯，**不要在本包里写关节角**。正确做法是在
`begin_light_wait` 里给 `/piper_task/command` 发一个命令，由 `piper_task`
用它自己实物调过的姿态执行。

## 5. final_vision — 红旗与红绿灯

`scripts/final_vision.py`，约 355 行。检测算法从
`race_mission/scripts/start_light_vision_node.py` 移植。

- **红旗**：HSV 双区间红色掩膜（0-10° 和 170-180°），面积超阈值计数，
  连续 `flag_confirm_frames`(5) 帧确认才发起步信号。防的是裁判走过、
  红色衣服之类的误触发。
- **红绿灯**：优先 YOLO（`ultralytics`），权重缺失或结果不确定时回落到 HSV
  红/绿像素计数。
- 手动解码 `sensor_msgs/Image` 而不用 `cv_bridge`，绕开 ABI 兼容问题。
- 受 `/final_mission/vision_control` 控制在 `flag` / `light` / `idle`
  之间切换，不需要时不跑推理。

## 6. validate_route.py — 赛前校验

`scripts/validate_route.py`，约 240 行。纯静态检查，不启动节点。

**ERROR（必须修）**

1. 缺列、行内容非法
2. `avoid_start`/`avoid_end` 不成对或嵌套
3. **任务点落在避障区内**——A* 重规划会与精确停靠打架
4. 七点位数量与轮次不符（决赛每个应出现 2 次）
5. 同一轮内 `piper_stop` 顺序错乱

**WARN（能跑但有风险）**

6. 相邻任务点间距 < `goal_reached_dist × 2`
7. 任务点朝向与前一点差异过大（对齐时大幅自转，伸臂有碰撞风险）
8. 避障区弧长 < 1m（标记打得太近）
9. **`ext:traffic_light` 排在任何 `piper_stop` 之后**（改动 2026-07-30）

第 9 条的道理：相机装在机械臂上。红绿灯在七点位之前时机械臂在零位、相机前视，
能看到灯。若排到取放段之后，一旦某轮抓取失败，机械臂会停在 `PICK_SCAN`
（侧伸 -87.7°），相机朝侧面，灯根本不在视野里。而且这个失败是静默的：
视觉一直报 `none`，总控等满 120 秒后按 `pass` 策略放行，等于闯红灯。

五个 ERROR 和四个 WARN 都已用真实航迹和合成航迹实测触发过。

## 7. 相机归属：按需中继（已实现）

D435i 装在机械臂上，`pyrealsense2` 对设备**独占**，两个进程不能同时
`wait_for_frames`。而
[vision_grasp_core.py:338-342](../../../piper_ws/src/piper_ros/src/piper_task/src/piper_task/vision_grasp_core.py#L338-L342)
在 `PiperVisionController.__init__` 里就 `pipeline.start()`，
`piper_task` 一构造相机就被占住，直到进程退出（`:950` 才 stop）。
所以 `final_vision` 根本开不了相机。

**采用方案：按需中继。相机永久归 `piper_task`，只在要识别时发帧。**

| | 常开中继 | 分时移交 | **按需中继（采用）** |
|---|---|---|---|
| 相机所有权 | 永久归 piper_task | 交出去再要回来 | **永久归 piper_task** |
| 最坏情况 | 识别失效 | **14 个卡点全废** | 识别失效 |
| 带宽 | 全程 9.2 MB/s | 无 | **占空比约 3%** |
| `pipeline.start()` | 1 次 | 3 次以上 | **1 次** |

不选分时移交的原因：`pipeline.start()` 要 1~3 秒且可能失败（USB 重新枚举、
设备未完全释放），一旦要不回来，14 个机械臂卡点全废。而它唯一的好处
（省掉常开发帧）用"按需"就能拿到，所以那份风险换不到任何东西。

带宽：640×480×3 = 0.92MB/帧。红旗 + 红绿灯合计最多约 1 分钟，
30 分钟赛程占空比约 3%，其余时间完全不取帧。

### 协议

话题 `/final_mission/camera/request`（String）：

- `start` — 开始发帧，需周期性重发当心跳（`relay_keepalive_period` 1.0s）
- `stop` — 停止发帧

`piper_task` 侧 3 秒收不到新的 `start` 就自动停发，所以 `final_vision`
崩了不会留下一个一直发帧的中继。机械臂动作期间由 `set_relay_paused(True)`
强制避让，绝不与 `vision_grasp_core` 的 `wait_for_frames` 并发。

### 三道保险

1. 请求幂等（`piper_task` 侧按状态去重，重复发无副作用）
2. 心跳超时自动停发（请求方异常时兜底）
3. 总控起步前检查图像通路，没通就大声报错并提示手动起步
   （`image_warn_after` 10 秒）

因为设备从不离手，最坏情况只是识别失效，机械臂一定安全。

---

# 航迹格式

CSV 列：`seq,stamp,frame_id,x,y,z,qx,qy,qz,qw,yaw,task,tol`

`task` 列的取值：

| 写法 | 含义 |
|---|---|
| `none` 或空 | 普通巡航点 |
| `avoid_start` / `avoid_end` | 避障区间标记，成对出现，**不是任务点** |
| `ext:piper_stop_1` … `ext:piper_stop_7` | 机械臂停靠点 |
| `ext:traffic_light` | 红绿灯等待点 |
| `ext:finish` | 终点 |
| `stop_10s` / `hold_5s` / `pause_3s` | 定时停车，不需要外部信号 |

决赛航迹需要：1 个避障区间、**14 个** `piper_stop`（两轮各 7 个，
顺序 1→2→3→4→5→6→7）、1 个 `traffic_light`、1 个 `finish`。

**任务点绝对不能落在避障区间内。** A* 会为了绕障改变路径，而精确停靠要求
车直奔一个固定坐标，两者会互相打架。校验脚本把这条列为 ERROR。

现状：`data/final_route.csv` 只有避障区标记，**还没有七点位、红绿灯、终点**，
需要重录。`data/arm_test_route.csv` 是单轮七点位测试航迹，
用 `--rounds 1` 校验通过。

# 话题接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/smoother_cmd_vel` | Twist | tracker → 底盘 | 唯一速度出口 |
| `/Odometry` | Odometry | 定位 → tracker | 世界系 `camera_init` |
| `/cloud_registered` | PointCloud2 | 定位 → tracker | 障碍物来源 |
| `/final_mission/tracker_enable` | Bool(latch) | manager → tracker | 起步闸门 |
| `/final_mission/start_signal` | Bool(latch) | vision → manager | 红旗确认 |
| `/final_mission/traffic_light` | String | vision → manager | `red`/`green`/`none` |
| `/final_mission/vision_control` | String(latch) | manager → vision | `flag`/`light`/`idle` |
| `/final_mission/command` | String | 人 → manager | `start`/`stop`/`resume`/`finish` |
| `/final_mission/state` | String(latch) | manager → 外 | 状态机当前状态 |
| `/final_mission/camera/request` | String | vision → piper_task | `start`/`stop`，按需发帧 |
| `/final_mission/camera/color` | Image | piper_task → vision | 中继图像（bgr8） |
| `/final_mission/vision_status` | String(latch) | vision → manager | `ok:frames=N:age=X` / `no_image:frames=0` |
| `/waypoint_task_event` | String | tracker → 三方 | `start:<name>:idx<N>` |
| `/waypoint_task_done` | String | 三方 → tracker | `done` / `done:<name>` |
| `/piper_task/command` | String | bridge → piper_task | 重试重发 |
| `/piper_task/result` | String | piper_task → bridge | `<status>:<cmd>:<reason>` |
| `/piper_task/navigation_skip` | String | piper_task → tracker | 逗号分隔的跳过点 |

跟踪器发出 `start:<name>:idx<N>` 后**阻塞等待**，收到匹配的 done 才继续。
done 按名字过滤：`done:piper_stop_3` 到达时如果正在等 `piper_stop_5`，
会被正确拒绝，不会误放行。

# 关键参数

`config/tracker.yaml`：

| 参数 | 值 | 说明 |
|---|---|---|
| `target_speed` | 0.35 | 巡航速度 |
| `max_linear` | 0.50 | **必须 ≥ target_speed**，父类默认 0.35 会截断 |
| `task_trigger_margin` | 0.80 | 开始降速的剩余弧长 |
| `approach_switch_dist` | 0.45 | 切 P 控制，**必须 > 候选点间距 0.40** |
| `approach_max_speed` | 0.10 | 侧伸护臂限速 |
| `approach_min_speed` | 0.04 | 履带静摩擦死区，**需实测** |
| `goal_reached_dist` | 0.05 | 到位判据 |
| `task_yaw_tolerance` | 0.05 | ≈2.9° |
| `task_yaw_max_correction` | 0.10 | ≈5.7°，超过则跳过对齐 |
| `task_yaw_min_angular` | 0.08 | 角速度死区，**需实测** |
| `external_task_timeout` | 150.0 | 够 3 次尝试 |

`config/mission.yaml`：`retry_counts` 各 2 次、`arm_action_timeout` 60s、
`traffic_timeout` 120s + `pass` 策略、`release_after_retry_exhausted` true。

**标死参数不要动**：`max_linear` 低于 `target_speed` 会静默截断速度；
`approach_switch_dist` 小于候选点间距会导致控制律来回切换。

# 赛前检查清单

1. `catkin_make --pkg final_mission` 通过
2. 录决赛航迹，`validate_route.py --rounds 2` **零 ERROR**，
   且红绿灯点录在所有 `piper_stop` 之前（否则会有 WARN）
3. 启动 `piper_task` 时确认加了 `enable_camera_relay:=true`，
   用 `rostopic hz /final_mission/camera/color` 确认等红旗时有帧
   （idle 时应无帧，这是按需中继的正常表现）
4. `dry_run.launch` 空跑，收集日志里的 `[TASK]` 误差数据
5. 据误差数据标定 `approach_min_speed` 和 `task_yaw_min_angular`
   （现在的 0.04 / 0.08 是估值，必须实测：发递增速度找轮子起转阈值）
6. 实测确认机械臂零位相机确实前视
7. 实测重定位重复性（你说在 5cm 以内，验证一下）
8. `arm_only.launch` 联调机械臂
9. 全流程 `final_race.launch` 至少完整跑通两遍
10. 练熟手动起步和紧急停车命令

# 故障处置

| 现象 | 原因 | 处置 |
|---|---|---|
| 车原地不动 | 未收到起步信号 | 手动发 `start` |
| 日志报"图像通路没通" | `piper_task` 忘了 `enable_camera_relay:=true` | 重启 piper_task 加该参数，或手动起步 |
| 红绿灯一直报 none | 灯不在视野 / 权重缺失 | 看 `vision_status`；确认红绿灯点在七点位之前 |
| 无显示器时报 cv2.error | 已自动降级不显示 | 无需处理，识别照常；要看画面就开 X11 转发 |
| 整条航迹偏移 | 开机位姿不对 | 停机，摆回建图原点重启 |
| 车在任务点长时间不走 | 机械臂无响应 | 等 `arm_action_timeout` 自动放行 |
| 避障不生效 | 用了 `arm_function_test.launch` | 换 `arm_only.launch` |
| 实际速度上不去 | `max_linear` < `target_speed` | 抬高 `max_linear` |
| 候选点间来回切换 | `approach_switch_dist` < 点间距 | 抬到 0.45 以上 |
| 定位跳变锁死 | 父类跳变保护触发 | 检查 `/Odometry` 发布者 |

# 已知限制

- **全工作空间 `catkin_make` 会失败**，与本包无关：`hybrid_avoid` 有
  `setup.py`，catkin 调 `interrogate_setup_dot_py.py` 时撞上
  `~/.local/lib/python3.8` 里损坏的 setuptools（`importlib_metadata`
  没有 `EntryPoints` 属性）。`hybrid_avoid` 已淘汰，从 `src` 移走即可。
  本包无 `setup.py`，单独构建正常。
- 重定位没有初始位姿配准，见「一个硬前提」。
- 代码里**没有机械臂可达性检查**。如果视觉给出超出 Piper 工作空间的目标，
  逆解会失败，走 `transport_path_failed_after_grasp` 之类的失败路径。
- `config/mission.yaml` 里 `arm_task_names` 当前无代码读取，是备查对照表。
  原先的 `use_look_pose` / `look_light_joints` 已删除——本包不应该出现
  任何关节角，抬臂看灯应改为给 `piper_task` 发命令，由它用自己的姿态执行。

# 改动记录 2026-07-30

| 文件 | 改动 |
|---|---|
| `scripts/final_vision.py` | 修 YOLO 类型冲突（飘黄）；按需请求发帧 + 心跳；`show_image` 无 DISPLAY 时自动降级不崩；上报图像通路状态 |
| `scripts/camera_relay.py` | 常开 10Hz → 按需发帧；加请求话题与心跳超时；默认 `enable_camera_relay=false` |
| `scripts/final_manager.py` | 订阅 `vision_status`；等红旗 10s 无图像则大声报错并提示手动起步 |
| `scripts/validate_route.py` | 新增 WARN：`traffic_light` 排在 `piper_stop` 之后 |
| `config/mission.yaml` | 新增中继与告警参数；`show_image` 保持 true 并注明降级行为 |
| `launch/final_race.launch` | 启动说明补 `enable_camera_relay:=true` |
| **`piper_task_node.py`**（piper_ws） | 三处：导入 mixin（失败则降级空实现）、混入 `CameraRelayMixin`、`run()` 里 busy 联动 `set_relay_paused` |

所有改动都用 `改动 2026-07-30` 注释标出，方便你后续查找和修改。

`piper_task_node.py` 是本次唯一改到 `piper_ws` 的文件，三处都标了
`改动 2026-07-30（N/3）`。**默认 `enable_camera_relay=false`，不打开时行为与
改动前完全一致**；`final_mission` 包不存在时走空实现降级路径（已实测）。
机械臂的三个固定姿态和 `JOINT_MOVE_WAIT` 一个字未动。

# 待你确认的两个尺寸

会影响参数标定，目前用的是保守估值：

1. `piper_stop_1` 到 `piper_stop_2` 的实际间距——决定 `task_trigger_margin`
   能不能维持 0.80m。
2. 车身到桌子的横向余量——决定失败路径要不要先收臂再移动。机械臂在候选点
   之间保持侧伸约 88°（见第 2 节），如果余量不足，就得在每次换点前收臂，
   每次多花 8s。

# MID360 纯手动录制与停靠点跟踪流程

本文档给出一套可直接执行的流程：室外录制轨迹时只手动打点，包含停靠点；后续重定位并按轨迹跟踪，到停靠点自动停车指定时长。

## 1. 前置条件

- 工作区：`/home/user/fastlio_ws`
- 需要已编译：
  ```bash
  cd /home/user/fastlio_ws
  catkin_make --pkg sfast_lio waypoint_tools
  ```

## 2. 终端 A：启动建图 + 录制（纯手动模式）

```bash
cd /home/user/fastlio_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch sfast_lio mapping_mid360.launch \
  enable_waypoint_record:=true \
  waypoint_file_name:=outdoor_manual.csv \
  waypoint_min_distance:=9999 \
  waypoint_min_yaw_change:=9999 \
  waypoint_default_tol:=0.30 \
  waypoint_default_stop_seconds:=180.0
```

说明：
- `waypoint_min_distance:=9999` 与 `waypoint_min_yaw_change:=9999` 用于抑制自动记录，基本只保留手动打点。
- 首个初始点可能仍会被记录，这是正常行为。

## 3. 终端 B：手动打点（你开车时常用）

```bash
cd /home/user/fastlio_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 3.1 记录普通轨迹点

```bash
rostopic pub -1 /waypoint_task std_msgs/String "data: 'none'"
```

### 3.2 记录停靠点（例如停 3 分钟）

```bash
rostopic pub -1 /waypoint_stop_seconds std_msgs/Float32 "data: 180.0"
```

该点会写入 CSV 为 `task=stop_180s`，回放时会到点停车 180 秒。

## 4. 手动获取刚打点的坐标

打点后立即查看 CSV 最后一行：

```bash
tail -n 1 /home/user/fastlio_ws/src/waypoint_tools/data/outdoor_manual.csv
```

只看关键字段：

```bash
tail -n 1 /home/user/fastlio_ws/src/waypoint_tools/data/outdoor_manual.csv | \
awk -F, '{printf("x=%s y=%s yaw=%s task=%s tol=%s\n",$4,$5,$11,$12,$13)}'
```

## 5. 录制结束后检查文件

终端 A 按 `Ctrl+C` 停止后执行：

```bash
wc -l /home/user/fastlio_ws/src/waypoint_tools/data/outdoor_manual.csv
grep -n "stop_" /home/user/fastlio_ws/src/waypoint_tools/data/outdoor_manual.csv
```

## 6. 终端 C：重定位 + 轨迹跟踪

```bash
cd /home/user/fastlio_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch sfast_lio mapping_mid360_relocalization.launch \
  enable_waypoint_follow:=true \
  waypoint_csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/outdoor_manual.csv
```

运行行为：
- `task=none`：通过后继续追踪下一个点
- `task=stop_180s`：到点停车 180 秒，时间到后自动继续

## 7. 后续扩展：机械臂/识别任务

回放时支持 `ext:任务名`，例如 `ext:arm_pick`。外部节点执行完成后发布：

```bash
rostopic pub -1 /waypoint_task_done std_msgs/String "data: 'done:arm_pick'"
```

任务完成后跟踪会继续执行后续点。

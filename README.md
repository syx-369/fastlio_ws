建图命令
bunker底盘
cd ~/bunker_ws
source devel/setup.bash

roslaunch bunker_bringup bunker_robot_base.launch \
  port_name:=can0 \
  publish_tf:=false
雷达驱动
cd ~/livox_ws
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
建图
cd ~/fastlio_ws && source devel/setup.bash
roslaunch sfast_lio mapping_mid360.launch
https://acny7gl98czh.feishu.cn/sync/W6Gxd9sBBsjAOibyxYbch5T0nEf
/home/user/fastlio_ws/src/S-FAST_LIO/config/mid360.yaml
倒数第二行pcd_save_en: false改为true，重定位是需改回false。为保存地图的设置

录制雷达和imu包，后面可以➕/t f
rosbag record -O mid360_test.bag /livox/lidar /livox/imu
查看包信息
rosbag info mid360_test.bag
回放包
rosbag play mid360_test.bag

建新图的时候记得在PCD文件夹下新建文件夹用于保存之前的建图信息，或者建图时录包保存


录制命令
新开一个终端执行
source ~/fastlio_ws/devel/setup.bash
录制航迹点
cd ~/fastlio_ws
source devel/setup.bash

rosrun waypoint_tools record_waypoints.py \
  _odom_topic:=/Odometry \
  _file_name:=arm_test_route.csv \
  _output_dir:=/home/user/fastlio_ws/src/waypoint_tools/data \
  _min_distance:=0.15 \
  _min_yaw_change:=0.17 \
  _default_tol:=0.25 \
  _frame_id:=camera_init
红绿灯检测点
rostopic pub -1 /waypoint_task std_msgs/String "data: 'ext:traffic_light'"
依次记录七个机械臂任务点
点1：目标卡片识别位置
rostopic pub -1 /waypoint_task std_msgs/String "data: 'ext:piper_stop_1'"
点2：第一个抓取识别位置
rostopic pub -1 /waypoint_task std_msgs/String "data: 'ext:piper_stop_2'"
点3：第二个抓取识别位置
rostopic pub -1 /waypoint_task std_msgs/String "data: 'ext:piper_stop_3'"
点4：第三个抓取识别位置
rostopic pub -1 /waypoint_task std_msgs/String "data: 'ext:piper_stop_4'"
点5：第一个放置识别位置
rostopic pub -1 /waypoint_task std_msgs/String "data: 'ext:piper_stop_5'"
点6：第二个放置识别位置
rostopic pub -1 /waypoint_task std_msgs/String "data: 'ext:piper_stop_6'"
点7：第三个放置识别位置
rostopic pub -1 /waypoint_task std_msgs/String "data: 'ext:piper_stop_7'"
随机障碍区起点和终点
避障区起点
在第一个可能出现障碍的位置前至少2米停车。
rostopic pub -1 /waypoint_task std_msgs/String "data: 'avoid_start'"
避障区终点
  通过整个随机障碍区，在最后一个可能出现障碍的位置后至少2米停车。
rostopic pub -1 /waypoint_task std_msgs/String "data: 'avoid_end'"
检查录制结果
检查任务点
grep -n "piper_stop" ~/fastlio_ws/src/waypoint_tools/data/arm_test_route.csv
检查轨迹文件最后十行
tail -n 10 ~/fastlio_ws/src/waypoint_tools/data/arm_test_route.csv
检查任务点数量
grep -c "piper_stop" ~/fastlio_ws/src/waypoint_tools/data/arm_test_route.csv


比赛启动命令
终端 1：roscore
roscore
终端 2：配置 CAN
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
ip -details link show can0 | grep bitrate
ip -details link show can1 | grep bitrate
可选验证：
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 restart-ms 100
sudo ip link set can1 up
timeout 3 candump can0
timeout 3 candump can1
终端 3：启动 bunker底盘
cd ~/bunker_ws
source devel/setup.bash

roslaunch bunker_bringup bunker_robot_base.launch \
  port_name:=can0 \
  publish_tf:=false
终端 4：启动 Piper 底层
cd ~/piper_ws
conda activate piper
source devel/setup.bash

roslaunch piper start_single_piper.launch \
  can_port:=can1 \
  auto_enable:=true
机械臂回零
回零不了，掐断终端强制重启：ctrl+\
conda activate piper
source /opt/ros/noetic/setup.bash
source ~/piper_ws/devel/setup.bash
rossrv show piper_msgs/GoZero
source devel/setup.bash
rosservice call /go_zero_srv "is_mit_mode: false"
机械臂调试
conda activate piper
source devel/setup.bash
roslaunch piper_description display_xacro.launch
终端 5：启动 Piper 运动学
cd ~/piper_ws
conda activate piper
source devel/setup.bash

python ~/piper_ws/src/piper_ros/src/piper/scripts/piper_pinocchio/piper_pinocchio.py
终端 6：启动机械臂任务节点（需更改）
cd ~/fastlio_ws
conda activate piper
source ~/fastlio_ws/devel/setup.bash
source ~/piper_ws/devel/setup.bash

roslaunch piper_task arm_function_test.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/arm_test_route.csv \
  target_speed:=0.10
确认机械臂状态：
rostopic echo /arm_task_state
另开一个看结果：
rostopic echo /arm_task_result
终端 7：启动 MID360
cd ~/livox_ws
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
终端 8：启动 S-FAST_LIO 重定位
cd ~/fastlio_ws
source devel/setup.bash
source ~/livox_ws/devel/setup.bash --extend
roslaunch sfast_lio mapping_mid360_relocalization.launch rviz:=true
确认 /Odometry 有数据：
rostopic echo -n 1 /Odometry
rostopic hz /Odometry
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /Odometry/pose/pose
终端 9：启动比赛总控
先低速跑：
cd ~/fastlio_ws
source devel/setup.bash

roslaunch race_mission mission.launch \
  csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/xiaosai.csv \
  target_speed:=0.30 \
  wait_for_start:=true \
  auto_start:=false \
  enable_vision:=true \
  show_image:=true
避障
rosrun waypoint_tools avoidance_zone_astar_test.py \
  _csv_path:=/home/user/fastlio_ws/src/waypoint_tools/data/final_route.csv \
  _obstacle_source:=cloud \
  _cloud_topic:=/cloud_registered \
  _odom_topic:=/Odometry \
  _cmd_topic:=/smoother_cmd_vel \
  _target_speed:=0.5
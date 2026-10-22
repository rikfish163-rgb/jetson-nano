# U-turn 四段单次调参

在 Nano 端运行。起点由人摆放：前轮压蓝线，车身摆正。测试不识别 UTURN 牌或蓝线，
不运行相机/雷达避障，不增加入场直行，四段结束后停车，不进行出口修正或恢复巡线。

默认读取 src/robot/config/maneuvers.yaml 的四段 uturn_trial_sequence，并保留 uturn_trial_pause_s。
命令行参数只改变本次测试；永久参数仍编辑该 YAML。每次运行都会打印完整动作表。
测试强制入场距离为零，之后即使比赛配置更改，也不会在四段前增加运动。

先在原终端 Ctrl+C 停止整车、键盘或其他底盘控制器。

终端 1：启动测试专用底盘桥接，保持此终端运行。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
roslaunch robocup_competition uturn_test_actuators.launch
```

终端 2：每次把前轮摆在蓝线上后运行。底盘就绪后倒计时 3 秒，随后保留原始
0.7 秒首段暂停，再执行四段；换挡停顿及末尾停车也保留。只执行一次。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
python2 tools/uturn_four_stage_test.py --execute
```

临时调参示例（未指定的字段读取 YAML）：

```bash
python2 tools/uturn_four_stage_test.py --execute \
  --s1-speed 30 --s1-steering 0 --s1-seconds 1.5 \
  --s2-speed 30 --s2-steering 22 --s2-seconds 2.0 \
  --s3-speed -30 --s3-steering -22 --s3-seconds 1.7 \
  --s4-speed 30 --s4-steering -22 --s4-seconds 1.9
```

s1 到 s4 分别表示四段。speed 是带方向的 RAW 整数，绝对值 1..30；
steering 取 -22、0、22，不是角度；seconds 大于 0 且不超过 10 秒。
--pause 调整起始、换挡和末尾停车时长（0.3..3 秒），--countdown 调整启动倒计时（2..10 秒）。
不带 --execute 只打印预览，不创建 ROS 节点。--help 显示全部参数。

SPACE、q、x、Esc 或 Ctrl+C 会停止并退出。重复运行前重新摆车。
底盘状态丢失、控制循环延迟、连接丢失、急停或出现竞争控制器会终止本次测试。
桥接和底盘仍各自保持 0.25 秒命令超时停车。

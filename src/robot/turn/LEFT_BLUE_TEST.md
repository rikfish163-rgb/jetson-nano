# 前轮压蓝线的单次左转测试

正式动作代码：`controller.py` 的 `left_lock_tick`。正式左转先记住路牌，
经过蓝线接近流程，再进入 ENTRY → LOCK → 出口循迹。本测试从摆车位置
直接进入 ENTRY，跳过路牌、蓝线接近和绿灯；不是完整路口流程的回放。
两前轮轴线压蓝线、车身朝原车道前方。entry_m 从这个位置重新计算，
模型坐标仍以此刻后轴为原点，不把前轴误设为后轴。

脚本 `left_blue_test.py`；参数 `left_blue_test.yaml`；launch `left_blue_test.launch`。
只运行一次，视觉完成、定时结束、超时、障碍停车、传感器失效或接管后
保持停车。再次测试先结束 launch、重新摆车，再重新启动。
`lock_seconds: 0` 复用正式视觉出口；正数禁用视觉提前结束，但保留正式
最大转角、雷达、图像时效、总超时保护。计时从 LOCK 指令开始，不是轮角反馈。

先关闭其他整车启动程序。Nano 终端一：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
roslaunch robocup_competition left_blue_test.launch live:=true
```

摆车后在 Nano 终端二使能；传感器就绪后默认等待 3 秒开始运动：

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
rosparam set /competition_controller/enabled true
```

立即停车并锁存（也可以使用现有手动接管）：

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
rostopic pub -1 /competition/estop std_msgs/Bool 'data: true'
```

临时参数示例，将终端一最后一行改为：

```bash
roslaunch robocup_competition left_blue_test.launch live:=true speed_raw:=16 entry_m:=0.12 lock_seconds:=2.0 max_seconds:=10
```

离线验证（不创建 ROS 节点、不发布指令）：

```bash
cd /home/nano/robocup_ws
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src:src/ros/lane/src:src/robot/test python2 -m unittest test_left_blue_test
```

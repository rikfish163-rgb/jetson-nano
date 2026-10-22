# 独立雷达左侧绕障试验

新增的 `lidar_left_pass_node.py` 与旧总控互不调用，不应与 `full.launch`、`./avoid` 或其他 `/control/cmd` 发布者同时运行。

## 行为

- 车辆后轴为原点，`x` 向前、`y` 向左。只选择前方中央 0.45–1.6 米范围的紧凑雷达聚类。
- 每帧重新计算障碍位置，追踪其**左侧**目标点。`clearance_m` 是车身右侧到聚类外缘的期望距离，范围 0.10–0.60 米，默认 0.20 米。
- 当障碍后缘越过后轴后方再额外 0.10 米，停车结束。没有自动回到原车道。
- 扫描过期、目标消失、预计短程车身碰撞或实测侧向净空不足时，输出零速度。
- 发布 `/lidar_left_pass/image` (`sensor_msgs/Image`) 和 `/lidar_left_pass/status`；浏览器预览为 `http://100.86.37.124:8091/`。预览灰点是雷达回波，橙点是聚类，红圈是目标，绿圈是左侧目标点，蓝框是车身。

这是低速、单个紧凑障碍的实验控制器。雷达聚类可能把墙角等当成目标；它不能验证左侧车道边界，也不能保证左侧有足够道路宽度。先在架空车轮与空旷封闭场地逐步验证。参数 `speed_raw` 是底盘原始指令，不是米每秒。

## 在 Nano 上只看雷达图，不动车

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch robocup_competition lidar_left_pass.launch
```

在浏览器打开 `http://100.86.37.124:8091/`。也可查看 `rostopic echo /lidar_left_pass/status`。

## 在 Nano 上启用实车输出（现场人员执行）

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch robocup_competition lidar_left_pass.launch live:=true enabled:=true start_actuators:=true clearance_m:=0.20 speed_raw:=8 max_steer_raw:=12
```

调整距离只改 `clearance_m:=0.20`，例如 `clearance_m:=0.30` 表示期望车身与障碍外缘相距 0.30 米。启动前确认没有其他整车或手动驾驶进程占用底盘。

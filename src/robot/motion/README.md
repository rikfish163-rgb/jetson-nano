# 运动执行

controller.py 执行选定轨迹、停车和转向反馈；tracker.py 跟踪路径。
odometry.py 按底盘命令反馈的源时间戳累计自行车模型，最多外推到 0.25 秒的命令有效期。
控制定时器延迟不会截掉行程；停车反馈按实际反馈时间结束上一条命令的行程。

调参修改本目录 `config.yaml`；允许参数用 `list` 命令查看。

`chassis_calibration` 保存统一的 RAW 到速度、轮角模型。
直行速度使用 2026-10-02 的前后 RAW 15/20/25/30 八组实测平均速度，
其中前进 RAW 15 使用重测 26 cm / 2.05120815 秒，其他七项保留完整标定记录。
`speed_points` 分方向保存各档 m/s，档位之间线性插值；采样范围外按端点比例估算。
`speed_calibration_source` 指向修正数据，原始记录保留。
每档只有一次约 2 秒的测试，长短行程和转弯速度仍需要实车验证。
轮角仍使用历史 RAW 26 标定；前进满舵左、右的后轴半径分别为 .6784646272 m、.9925 m，
倒车半径仍是估计，需要当前车辆实测核对。
正常左右转、直行、UTURN 位姿累计和雷达扫掠使用这套模型。
UTURN 实验配置仍可通过 `uturn_calibration` 显式覆盖。
独立侧方试跑也读取这张速度表：循线找 P1 和盲区估算按实际发送速度查表，
对线后定位按定位档位和方向换算时间；显式命令行速度系数可覆盖查表。

`max_steer=.2` 保留为现有循线调舵尺度；`steering_command_scale_rad=.03`
对应协议 RAW 22。它们不再作为底盘满舵的物理轮角。
`/competition/status` 的 `motion_model` 显示当前使用的模型参数，
位姿来源仍是 `command_estimate`，不是轮速或航向传感器测量。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list motion
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/motion/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test motion --report-dir /tmp/motion-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

独立预览（不启动底盘）：

```bash
roslaunch /home/nano/robocup_ws/src/robot/motion/debug.launch
```

接管与执行桥在 `src/ros/manual`，硬件执行在 `src/drivers/base`。预览输出为 `/modules/motion/output`。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

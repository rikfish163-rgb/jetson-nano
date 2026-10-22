# 前进、后退速度重标定

`speed_calibration.py` 独立测量 raw 15、20、25、30 的前进和后退。
默认每个方向、每档速度测 2 秒，共 8 轮。所有运动段转向为 0，
每轮只执行一段直行，停车后等待输入实测距离，不自动接着动车。
不读取旧速度系数，也不自动覆盖停车或循线配置。

## Nano 端运行

先在原终端停止旧的整车、停车和遥控启动入口，避免重复底盘节点。
以下两个终端均在 Nano 上运行；本地终端先 `ssh nano@100.86.37.124`。

终端 1：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
roslaunch robocup_competition parallel_open_loop_actuators.launch
```

终端 2：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
python2 -B src/robot/parallel_parking/speed_calibration.py \
  --execute --speeds 15 20 25 30 --durations 2
```

顺序为前进 15/20/25/30，再后退 15/20/25/30，每档运行 2 秒。
前进 raw 为正，后退 raw 为负。每轮打印方向、速度和时长。

1. 以当前前轮轴位置作起点标记，车身保持直线方向。
2. 终端出现 `READY` 后按 `g`，倒计时 3 秒后直行，到时自动停车。
   `g` 等待最长 30 秒；超时保存进度退出，可用续测命令继续。
3. 测量本轮停车后前轮轴相对于本轮起点的位移，输入厘米数，例如 `27.5`。
   后退也填写正的距离大小，不用负数；应量本轮单段位移，不是累计距离。
4. 下一轮重新标记起点，再按 `g`。等待输入距离不限时。

输入 `0` 表示确实没动，`r` 重做当前轮，`q` 保存退出。
运动时空格、`x`、`q`、Esc、Ctrl+C 中止本轮；中断轮不参与拟合。
全部试验使用相同地面和电量条件，直行测量时不要同时打方向。

脚本结束或正常保存退出会打印结果目录和完整续测命令。
续测跳过已测轮次；已执行但未输入距离的轮次直接等待输入，不重复运动：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
python2 -B src/robot/parallel_parking/speed_calibration.py \
  --execute --resume /home/nano/robocup_ws/field_data/calibration/这里替换为打印的目录名
```

可用 `--repeats 2` 对每个组合重复两次，或 `--directions reverse` 只测后退。
默认单一时长只计算各档实测平均速度；需要拟合起步、停车综合偏差时，可加 `--durations 1 2`。
去掉 `--execute` 只打印计划，不导入 ROS、不写结果目录、不发布指令。

## 数据与换算

默认结果目录为 `field_data/calibration/speed_时间戳_进程号/`，每次独立创建。
每输入一次距离就保存以下文件：

- `measurements.csv`：方向、raw 速度、计划时长、实际指令时长和实测厘米数。
- `results.json`：完整计划、进度、测量数据及拟合模型。
- `speed_table.yaml`：每个方向、每档速度独立拟合的速度与综合偏差。
- `trial_序号_attempt_次数.json`：单轮实际发布命令及时间记录。

时间取实际发出首个非零命令到首个零命令的单调时钟间隔，
不包含倒计时、等待按键或停车等待。它是指令持续时间，不是编码器测出的行驶时间。
距离取完全停车后的轴位移，因此包含起步和停车过程。

同一方向、同一档位有两种时长时，拟合：

```text
distance_m = max(0, speed_mps × command_seconds + offset_m)
```

`speed_mps` 是该档位拟合速度，`offset_m` 是起步和停车综合偏差。
目标距离在采样范围内时，可由 `(distance_m - offset_m) / speed_mps` 换算指令时长。
单一时长仅给出平均速度，标记 `effective_average`；全程未动或拟合速度不为正时
标记为不可用。两点拟合不构成独立验证，超出采样时长范围的行程仍需验证。
`mps_per_raw` 仅是该档速度除以 raw 的诊断数值，不应跨档共用一个比例系数。

完成后使用实测表更新距离映射；当前脚本仅保存测量结果，现有生产参数不会被覆盖。

## 不动车测试

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
PYTHONPATH="src/robot/test:src:$PYTHONPATH" python2 -B -m unittest \
  test_speed_calibration test_parallel_parking_open_loop
python2 -B src/robot/parallel_parking/speed_calibration.py
```

测试中的 ROS、终端及执行器均为模拟，不启动车辆。

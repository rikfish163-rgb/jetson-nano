# 指定库位与手动侧方开环试验

前后 raw 15/20/25/30 完全重标定使用独立的
[速度标定脚本说明](SPEED_CALIBRATION.md)。每轮停车后输入实测厘米数，分别拟合各档前后速度。

## 指定库位：自动找 P1 前库线，左打死后右打死

`--slot P1`、`--slot P2`、`--slot P3` 选择三个侧方库位。
启动时车身应与右侧库排平行，P1 前库线在车头前方；横向距离与视频起点保持一致。
程序先沿道路循线寻找 P1 的最前横线，连续两张不同源图像识别到同一条线后锁定。
寻找和接近前库线始终属于循线阶段：共用前相机，复用正常车道中心 Pure Pursuit
控制器实时修正方向，默认速度 raw 15；前库线观测只决定纵向停点。
车偏左时按观测到的右侧车道中心向右纠偏，不添加固定右打补偿。
前库线未进入视野或候选宽度不合格时继续循线搜索；搜索不设行程或时限停车点。
漏检、多候选和相机观测中断会清空未锁定的两帧确认；重复源图像不计数。
这条试跑流程不计算光流，不等待地面特征或跟踪稳定，不使用光流有效性作为开车或停车条件。
它按 P3→P2→P1 的已知右侧库排几何识别末端：约 36 cm 横线、右侧外纵线向后延伸、
外纵线前方确有可见地面且没有继续延伸。道路侧允许连接虚线/弯道。
横线完整宽度按道路侧纵线的实际交点计算，并检查整段白线支持，
防止只提取到较短片段时把错误尺度误判为合格库宽。
不会把单根白线或图像边缘截断当成 P1；它不读取地上的 P1 字样。
找线图像覆盖车前 3 m；扩展画布没有改变米/像素比例。
如果已观察到末端几何，但横线宽度不符合 36 cm（允许 28–44 cm），
观测报告 `p1_terminal_width_mismatch`，该候选不作为 P1 锁定，搜索继续。

视觉坐标以后轴为原点，前轴位置为 x=0.26 m。
确认后，只要同一条末端线仍可见，每张新图像都会修正前轴剩余距离。
新测距与按实际发送命令推算的当前距离相差不超过 15 cm、端点横向变化不超过 15 cm
才替换目标；远处其他横线不能替换已锁定的前库线。
现有相机前轮近处不可见。进入盲区后，以最后一次可见线距离及发送的前进指令累计时间
推算剩余距离；默认读取 `motion/config.yaml` 的前进实测速度表。
raw 15 对应修正后的实测平均速度约 0.12675 m/s（26 cm / 2.05120815 秒）。
档位之间插值，范围外按端点比例估算；显式 `--forward-mps-per-raw` 改用指定比例。
循线暂时发送零速度时不累计前进行程；图像延迟和默认 0.10 秒制动延迟也使用该模型补偿。
停车后等待 0.7 秒及新的相机结果；线仍可见时用新测距复核，盲区内的完成依据为估算距离。
默认前轴对线误差目标 ±2 cm，盲区实际停点受速度系数、地面、电量和制动影响，尚未实车验收。
状态 `front_axle_gap_source` 为 `line_image` 或 `timed_last_seen_line`，明确区分图像测距与估算。
停车观测入口默认加载 `reference_camera.yaml` 的专用投影，使用实测 134 cm、107 cm
前轴距离及 36 cm 横线宽度拟合，不依赖 70 cm 库长。车道通用投影没有改变。
这两个位置属于拟合样本；终端每 0.5 秒打印前轴剩余距离、来源及当前转向。
只有对线判断成立，才开始以下库位偏移和倒车动作；锁线后的接近最多估算 3 m/20 秒，
相机结果断流、非法线段几何或估算越线时停车退出。
正常循线横移不受锁线时起点的 5 cm 偏移限制。
入口必须有唯一 `/vision/lane_observation` 发布者，循线感知不另发底盘命令。
车道短时坏帧沿用原循线控制器的 0.25 秒方向保持；持续丢失车道时停止，
锁线后的车道丢失以 `LANE_TRACKING_LOST` 退出，不以看不到前库线作为停车理由。
定位目标统一向前平移 70 cm，库位间距仍为 70 cm；定位和后续倒车状态机默认速度均为 raw 30：

| 库位 | 从共同参考线直行定位 | 后续动作 |
|---|---|---|
| P1 | 前进 85 cm | 左打死倒车 1.6 秒 → 右打死倒车 1.4 秒 → 回正停车 |
| P2 | 前进 15 cm | 同上 |
| P3 | 后退 55 cm | 同上 |

定位时转向为 0，结束后停车 0.7 秒，再进入倒车动作。
左打死输出 raw +22，右打死输出 raw -22，两段之间连续换向、不停车回正。
此顺序固定，不沿用手动模式的 `--side right` 方向，也不执行额外倒车段。
`--first-seconds` 和 `--second-seconds` 可分别覆盖 1.6 和 1.4 秒。

**对线后的库位偏移距离**按 `abs(距离) / (定位 raw 速度 × 对应方向速度系数)` 换算时间，
**没有编码器或视觉距离反馈**。默认使用整车各档实测平均速度，
raw 30 前进约 0.28305 m/s，倒车约 0.28104 m/s；按实际指令时长标定，
P1/P2/P3 定位时间约为 3.003 / 0.530 / 1.957 秒。
每档单个 2 秒样本不能独立确定起步、停车偏差，长短行程仍需实车验证。
定位单段最长 15 秒，超限在启动 ROS 前拒绝；
倒车转弯单段仍最长 5 秒。指定库位状态机已按用户要求设为 raw 30。
时间模型给出的行程仍不是运行时测得的实车行程。

库位几何设置位于 `open_loop_test.yaml` 的 `slot_reference`，
其中速度系数仅作为 RAW 30 备用值；入口默认读取共享速度表。正距离沿车头方向前进：
`distance = reference_offset_m - (库位编号 - 1) × slot_spacing_m`。
如果 P1 对线后的倒车起点偏移不同，修改 `reference_offset_m`。
命令行也可用 `--reference-offset-m`、`--slot-spacing-m`、
`--forward-mps-per-raw`、`--reverse-mps-per-raw` 覆盖。
`--position-speed` 指定定位速度；显式 `--forward-speed` 也可覆盖定位速度。
`--speed` 指定两段转弯的倒车速度；两者缺省使用 YAML 的 `slot_motion`，均为 30。
找线/接近速度仍使用 `reference_search`，默认 15。
指定库位不能与 `--side`、`--forward-only`、`--forward-seconds`、
手动转向幅度及额外倒车段混用，避免参数被悄悄忽略。

不动车预览：

```bash
cd /home/nano/robocup_ws
python2 -B src/robot/parallel_parking/parallel_open_loop_test.py \
  --slot P2 --speed 30 --position-speed 30
```

会打印目标有符号距离、各阶段秒数和 raw 输出，不导入 ROS、不发送指令。

实车终端 1，启动相机观测和专用底层（先停止其他底盘/遥控启动入口）：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
roslaunch robocup_competition parallel_slot_test.launch
```

实时试跑可加 `record:=true record_path:=/home/nano/robocup_ws/field_data/parallel_live/run`，
先创建该目录；rosbag 会以时间戳保存相机图像、前线观测、剩余距离/动作状态及底盘状态。
对线停车后再 Ctrl+C 关闭终端 1，使记录正常结束。

实车终端 2：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
python2 -B src/robot/parallel_parking/parallel_open_loop_test.py \
  --execute --slot P2 --seek-speed 15 --creep-speed 15 --speed 30 --position-speed 30
```

首次核对前轮停车位置可在上面命令添加 `--reference-only`，只找 P1 线、对线停车，
不做库位偏移或倒车。`--seek-speed` 和 `--creep-speed` 控制找线/近线速度，
默认均为 raw 15，与用户最新要求一致；找线及接近均保持此速度。
相机新结果到达时的图像年龄、新结果间隔分别限制为 0.4 秒，
避免将正常处理延迟和帧间隔相加后误判断流；停车预测仍计入两者的完整延迟。
`--line-tolerance-m` 设置对线误差目标；其余设置在 YAML 的 `reference_search`。
`/parallel_parking/open_loop_status` 提供阶段和 `front_axle_gap_m`；
`/parallel_parking/p1_reference_bev` 用红线标出通过校验的 P1 前线，
黄线表示末端几何可见但宽度校验不通过；观测消息中的 `rejected_lines` 给出投影宽度。

相机单独检查（不会启动底盘）：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
roslaunch robocup_competition parallel_reference_camera.launch
```

`--slot` 与 `--execute` 同用时，底盘握手完成即自动倒计时 3 秒，随后执行一次，
不必按 g。仍要求前台交互终端，空格/x/q/Esc/Ctrl+C 可中止。
想保留按 g 开始，添加 `--wait-g`。没有 `--execute` 始终只预览。
手动模式启动方式不变；仅显式 `--auto-start` 时自动倒计时。

## 原手动模式

仅用于现场短时试车。与正式主控、雷达绕障、闭环侧方停车相互独立。
不识别路牌/蓝线/车位，不测量实际位移，不做自动避障，不判断停车是否成功。
图纸不能决定倒车时间；车头朝向、车辆与库口的横向/纵向位置需由现场固定。
每轮结束后人工把车恢复到相同起点再测，不要从上一轮结束位置重复试。

新起点：车位位于车身右前方，车位后边界线的延长线经过车辆前轴，车身与车位长边平行。
新默认：直行前进 0.5 秒，停车 0.7 秒，再向右打方向倒车 0.5 秒，随后停车。
前进时间是未标定的短时试验值，不代表足以到达倒车起点。先用 --forward-only 单独调前进段。
第一轮反打倒车、第二轮右后左后、最后回正倒车默认关闭。--forward-seconds 0 可恢复原来的直接倒车起步。
右侧第一段 raw 转向为 -22，第二段为 +22；以当前底盘协议为准，首次架空确认方向。
参数 raw 不是 m/s 或角度，速度填写正的大小，程序自动发布负速度用于倒车。

## 参数

同目录 `open_loop_test.yaml`，不影响正式 `config.yaml`。
命令行参数覆盖 YAML，不会保存覆盖值。

| 命令参数 | YAML 参数 | 含义 |
|---|---|---|
| `--side right` | side | 车位在车身右侧；left 为左侧 |
| `--forward-speed 12` | forward_speed_raw | 直行前进速度，正整数 1~30，独立于倒车速度 |
| `--forward-seconds 0.5` | forward_seconds | 前进准备时间，0~5 秒；0 跳过 |
| `--forward-only` | forward_only | 只前进后停车，不执行任何倒车段 |
| `--speed 26` | speed_raw | 倒车速度大小；默认 12，上限 30 |
| `--first-steer 22` | first_steering_raw | 第一段向库侧打方向大小 |
| `--first-seconds 0.5` | first_seconds | 第一段倒车转弯时间 |
| `--second-steer 22` | second_steering_raw | 第二段反打方向大小 |
| `--second-seconds 0` | second_seconds | 第二段倒车时间，0 不执行 |
| `--repeat-bay-steer 22` | repeat_bay_steering_raw | 第二轮朝车位侧打方向大小 |
| `--repeat-bay-seconds 0` | repeat_bay_seconds | 第二轮朝车位侧打方向倒车时间；右侧车位即右打 |
| `--repeat-counter-steer 22` | repeat_counter_steering_raw | 第二轮反打方向大小 |
| `--repeat-counter-seconds 0` | repeat_counter_seconds | 第二轮反打方向倒车时间；右侧车位即左打 |
| `--straight-seconds 0` | straight_seconds | 最后回正倒车时间，0 不执行 |
| `--pause-seconds 0.7` | pause_seconds | 运动段之间停车回正时间 |
| `--countdown-seconds 3` | countdown_seconds | 按 g 后停车倒计时 |

每段运动最长 5 秒，不再额外限制各运动段的累计时间。段间至少停车 0.3 秒。
直行前进时车轮回正；前进结束后停车回正，再切换倒车，速度不直接正负跳变。
先固定前进速度，只调前进时间，再固定前进段调倒车。每次从同一标记起点摆车。
回正倒车只沿车身方向后移，不会自行纠正横向位置或车身角度。
这些是试验上限，不是“在此范围内一定安全”。默认速度 12 可能低于电机起转门槛；
如使用已标定的 26，先短时试，不要在车未动时盲目同时增加速度和时长。

## 1. 只预览，不启动车辆

```bash
cd /home/nano/robocup_ws
python2 -B src/robot/parallel_parking/parallel_open_loop_test.py \
  --side right --speed 26 --first-seconds 0.5
```

没有 `--execute` 不创建 ROS 节点、不发布任何指令，也不需要底盘启动。

## 2. 实车：终端 1 启动专用底层

先停止 full.launch、旧 actuators.launch、键盘和手柄程序，确认车辆停止。
现场人员准备物理急停、清空车后/车侧空间，首次可架空驱动轮确认方向。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
roslaunch robocup_competition parallel_open_loop_actuators.launch
```

此入口只接收 `/parallel_parking/open_loop_cmd`，不接收正式主控 `/control/cmd`。
桥接器和底层均保留 0.25 秒无指令超时。不要与其他底盘启动入口重复运行。
不需要重编译；测试脚本用 python2 直接运行，不使用 rosrun。

## 3. 实车：终端 2 先只试前进准备段

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
python2 -B src/robot/parallel_parking/parallel_open_loop_test.py \
  --execute --side right --forward-only --forward-speed 12 --forward-seconds 0.5
```

按 g 后倒计时，直行前进，然后停车，不会倒车。先确认驱动正常，再标定前进时间。
速度 12 可能达不到启动门槛，应使用已实测可稳定运行的速度；不要仅因不动就延长时间。

## 4. 实车：前进准备 + 第一段倒车

以下会允许运动，但先等你按 g，再倒计时 3 秒：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
python2 -B src/robot/parallel_parking/parallel_open_loop_test.py \
  --execute --side right --forward-speed 12 --forward-seconds 0.5 \
  --speed 12 --first-steer 22 --first-seconds 0.5 \
  --second-seconds 0 --straight-seconds 0
```

在该终端按 `g` 开始；空格、x、q、Esc 或 Ctrl+C 中止并退出。
30 秒未按 g 会退出。正常结束自动发送停车，不循环、不自动恢复循迹。
运行中不接受修改参数，改参数前先停止并重新摆车。
停止后程序重复发送零命令；进程失联时依赖桥接器/底盘看门狗。
SSH 断网并不保证远程进程立刻退出：仍可能执行完当前有界流程，不能替代物理急停。

右侧车位的两轮顺序为：第一段右打倒车、`--second-seconds` 左打倒车、
`--repeat-bay-seconds` 再次右打倒车、`--repeat-counter-seconds` 再次左打倒车，
最后 `--straight-seconds` 回正倒车。第二轮方向量由 `--repeat-bay-steer` 和
`--repeat-counter-steer` 独立控制。
以上都是短时试验值；执行组合前将前进速度和时间替换为已标定值。
先调前进段，再调第一段倒车，以短时步进试验；确认第一段终点后再逐段启用，时间需实测。

可另开终端 `rostopic echo /parallel_parking/open_loop_status` 看阶段、输出和剩余秒数。
串口状态只证明指令写入，不证明车轮已按指令运动。

## 离线测试

```bash
cd /home/nano/robocup_ws
python2 -B src/robot/test/test_parallel_parking_open_loop.py
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
```

测试不创建 ROS 节点、不发布运动指令。项目配置 check 不等于实车验收。

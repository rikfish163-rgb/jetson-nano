# P3 单独测试：自动巡线入场停车 + 五段倒车

入场复用整车的前摄车道检测和自动巡线控制器，默认前进速度 30。
P3 允许从弯道起步：首次确认库位线的找线时间默认 15 秒，整个入场阶段最多 30 秒（从倒计时结束后开始计时）。
找线超时 NO_LINES_SEEN 是未识别到库位线的保护停车，不代表到达库位末端；此时不执行倒车。
转角直接使用巡线控制器输出，不叠加硬件转向补偿，也不固定转角直行。
巡线无效时先零速等待；保留原巡线的短暂丢线保持（最多约 0.25 秒）和原入场超时。
单纯丢失车道不会开始倒车。配置仅在测试进程内加载/覆盖，整车源代码、配置和 launch 不修改。
入场优先处理/提供巡线结果，再处理库位线；每帧处理结束后不再额外等待 0.05 秒。
两路视觉合并结果仍用于停车判断，停车的横线与弯道证据来自同一帧，摄像头/底盘保护阈值不放宽。
库位线方向与识别区域随近处车道方向旋转，减少转弯时横线被错误丢弃。
先在连续至少 2 张新鲜图像中看到库位横线。直道几何、近处拟合长度等只用于诊断，不再挡住停车。
第一次横线为零且双侧边线同向弯曲，就立即锁定零速停车。重复使用同一张图像不算新一帧。
停车之后再确认横线为零 + 双边同向弯曲，连续至少 2 帧、持续至少 0.10 秒。
零速保持至少 0.5 秒，且静止确认成功后才进入五段倒车。
确认中出现 0/1 抖动只会重置静止确认票数，永不恢复前进；默认 2 秒内不能确认则保持停车并退出 END_UNCONFIRMED_STOPPED，不执行倒车。
直道/弯道的前轴停止位置仍需要实车检查。
当前仍按可见线消失与道路形状停车，没有实测前轴到目标线的闭环距离；不能保证厘米级前轴对齐。
这是手动放车的 P3 动作试验，不会识别或验证库位编号，也不测量最终是否停正。

倒车五段连续执行，中间不停车：
1. 速度 -30、转角 0、1.5 秒。
2. 速度 -30、转角 +22、0.2 秒。
3. 速度 -30、转角 -22、3.4 秒。
4. 速度 -30、转角 0、0.1 秒。
5. 速度 -30、转角 +22、2.1 秒。
最后发零速并退出，只执行一次。

## 启动

先停止整车控制和其他运动测试。若旧的 parking_line_stop_test.launch 正在运行，
可以直接复用该终端一，只停止旧的 Python 测试，再在终端二执行 P3 命令；不要重复启动底盘或前摄。

Nano 终端一：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
roslaunch robocup_competition p3_parking_test.launch
```

Nano 终端二：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
python2 tools/p3_parking_test.py --execute \
  --speed 30 \
  --seek-seconds 15 \
  --max-seconds 30 \
  --end-zero-frames 2 \
  --end-zero-seconds 0.1 \
  --step1-seconds 1.5 \
  --step2-seconds 0.2 \
  --step3-seconds 3.4 \
  --step4-seconds 0.1 \
  --step5-seconds 2.1
```

启动后停着倒计时 3 秒，再沿车道巡线前进。车放在库位旁车道内，前摄需要看到有效车道线，库位默认在右侧。
每次摆好车后，重新执行终端二命令。空格、x、q、Ctrl+C 会停车。
此测试没有自动避障。底盘反馈、命令独占、急停和相机新鲜度检查贯穿全部阶段；
摄像头/底盘失联等故障会结束测试，不能因为故障停车就自动开始倒车。

## 调参

- --step1-seconds 1.5：第一段直线倒车时间。
- --step2-seconds 0.2：新插入的第二段 +22 转角倒车时间。
- --step3-seconds 3.4：第三段 -22 转角倒车时间。
- --step4-seconds 0.1：第四段直线倒车时间。
- --step5-seconds 2.1：第五段 +22 转角倒车时间。
- 时间可调 0.05～10 秒，互相独立。
- --reverse-speed 30：五段倒车速度的正数幅值，实际输出负数。
- --parking-pause-seconds 0.5：入场停车后的零速保持时间，范围 0.3～3 秒。
- --speed 30：入场巡线前进速度。
- --seek-seconds 15：首次确认看到库位线的最长找线时间；范围 0.5～15 秒。
- --max-seconds 30：整个入场巡线最长时间；范围 1～60 秒，必须不小于 seek-seconds。不计入停车后的倒车阶段。
- --end-zero-frames 2：停车后，连续的横线为零 + 双边同向弯曲确认帧数，范围 2～6。
- --end-zero-seconds 0.1：停车后，上述共同条件最短持续时间，范围 0.05～0.5 秒；不会推迟首次制动。
- P3 停车后确认使用上述 end-zero 参数；共享脚本的 --lost-seconds/--lost-frames 被这两个 P3 参数覆盖。
- --end-verify-seconds 2：静止确认超时；超过后保持停车并退出。
- P3 入场不设置 --steering；非零值会被拒绝，避免旧命令继续施加固定转角。
- 仍支持原停车测试全部视觉参数。
- 默认值在 tools/p3_parking_test.py 的 arguments() 中。

不带 --execute 仅打印动作参数，不发布命令。--observe 只检测图像。
状态话题 /parking_p3/status。巡线入场日志为 P3_LANE_FORWARD_*；巡线输出零速时为 P3_WAIT_LANE。
额外诊断输出 lane_age（巡线图像年龄）、points、confidence、lane_reason、bay_armed（已连续看到库位线）、end_votes、vision_ms。
若再次卡顿，检查 lane_reason=lane_stream_stale / empty_lane_path / lane_unreliable / lane_geometry_invalid 以及 lane_age 是否超过 0.5 秒。
停车后日志阶段：
P3_STOP_HOLD / P3_STOP_VERIFY_END → P3_STEP1_REVERSE_STRAIGHT → P3_STEP2_REVERSE_LEFT →
P3_STEP3_REVERSE_RIGHT → P3_STEP4_REVERSE_STRAIGHT → P3_STEP5_REVERSE_LEFT → P3_COMPLETE。

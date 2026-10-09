# P2 独立测试

复用 P3 的自动巡线、库位线消失与双侧弯曲触发、静止确认和故障停车逻辑。
看到库位线后，首次零库位线且双侧边线同向弯曲立即停车；静止确认两帧及 0.1 秒，至少停车 0.5 秒后执行五段动作：

| 步骤 | 速度 raw | 转角 raw | 时间 |
|---|---:|---:|---:|
| 1 | +30 | 0 | 1.2 秒 |
| 2 | -30 | +22 | 0.2 秒 |
| 3 | -30 | -22 | 3.4 秒 |
| 4 | -30 | 0 | 0.1 秒 |
| 5 | -30 | +22 | 2.1 秒 |

第一段为固定转角前进，后四段倒车，按次序连续切换。最后停车退出。
P3 自己的第一段保持 -30、0、1.5 秒；其余默认值已保存为 0.2、3.4、0.1、2.1 秒。
只修改 tools 测试脚本与说明，不修改正式整车 src、配置或启动文件。
P2 与 P3 名称代表本次人工选择的动作试验，不会识别或验证库位编号；不含自动避障，也不使用新的末端横线距离定位。

## Nano 终端一

已经运行 p3_parking_test.launch 时复用该进程，不重复启动底盘与摄像头。停止其他车辆控制或测试进程。重新启动时执行：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
roslaunch robocup_competition p3_parking_test.launch
```

## Nano 终端二

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME

python2 tools/p2_parking_test.py --execute \
  --speed 30 \
  --side right \
  --step1-speed 30 \
  --reverse-speed 30 \
  --step1-seconds 1.2 \
  --step2-seconds 0.2 \
  --step3-seconds 3.4 \
  --step4-seconds 0.1 \
  --step5-seconds 2.1
```

--speed 是入场巡线前进速度，--step1-speed 是停车确认后的第一段固定直行速度，--reverse-speed 是后四段倒车速度的正数幅值。step1～step5 的时间均可独立调节，范围 0.05～10 秒。
默认值在 /home/nano/robocup_ws/tools/p2_parking_test.py 的 arguments()。
不带 --execute 只预览参数；--observe 只读取相机，不发布运动指令。
空格、x、q 或 Ctrl+C 停车。状态话题 /parking_p2/status。

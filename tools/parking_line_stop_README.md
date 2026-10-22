# 库位横线消失 + 双侧车道弯曲停车测试

独立测试，只用前摄。车头与车道平行，默认库位在右边；允许沿车道前后随机放置。
速度 30，转角可调（默认 -3，用户已确认这一组行驶效果良好）。不循线、不等待绿牌、不使用雷达避障。

## 正常结束条件

连续两张新图像确认看见过库位横线后，第一次同时满足以下条件就立即发送零速，并锁住本次测试的停车状态：

- 库位横线数量为零。
- 同一张图像里左右两侧实际车道边线均明显弯曲、弯向一致，并构成合理宽度的车道。

先显示 `STOP_VERIFY_END`，在静止状态下继续观察。条件连续成立至少 0.3 秒、至少 3 张新图像后，显示 `BAY_END_CONFIRMED`，正常结束。
横线重新出现或任意侧曲线条件不成立，均重置确认计数，但车保持零速，绝不自动恢复前进。
如果两秒内没确认，显示 `END_UNCONFIRMED_STOPPED` 并停止测试，不声称库位末端已确认。
仅横线漏检而双侧弯道条件不成立，继续前进。
斜直线、缺少一侧、可见范围不足，都不视为双侧弯道。
地图用于设计规则，没有地图定位能力，不能证明已经经过某个编号库位。

## 启动

先停止整车控制、其他运动测试和占用前摄的程序。已有本测试终端一可保持运行。
Nano 终端一：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
roslaunch robocup_competition parking_line_stop_test.launch
```

Nano 终端二：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
python2 tools/parking_line_stop_test.py --execute --speed 30 --steering -3 --side right --lost-seconds 0.3 --lost-frames 3
```

取得新鲜图像并完成底盘握手后，静止倒计时 3 秒再前进。空格、x、q、Ctrl+C 均停车。
重试时保留终端一，在终端二重跑。

## 日志

- `FORWARD_SEEN_LINES`：仍有横线，继续前进。
- `FORWARD_WAIT_BOTH_CURVES`：横线为零，但双侧弯道证据不足，继续前进。
- `STOP_VERIFY_END`：第一次横线为零且双侧弯曲，已经停车，在静止状态累计确认。
- `BAY_END_CONFIRMED`：联合条件确认完成，停车退出。
- `END_UNCONFIRMED_STOPPED`：静止验证超时，保持停车退出；需要用户判断当前位置。
- `left/right=STRAIGHT`：边线未达到明显弯曲阈值。
- `left/right=CURVE`：该侧满足曲率、两端方向变化与弯曲量条件。
- `left/right=UNKNOWN`：观测点不足或拟合不可靠，不作为弯道证据。
- `NO_LINES_SEEN`：前进 5 秒仍未确认见线，停止测试。
- `MAX_RUN_TIMEOUT`：前进达到 20 秒仍未正常结束，停止测试。
- `CAMERA_TIMEOUT`：采集或处理结果超过 0.8 秒，停止测试，不算到达库位末端。

## 参数

- `--steering -3`：当前直行修正；停车为速度 0、转角 0。
- `--lost-seconds 0.3`、`--lost-frames 3`：停车后联合条件连续确认时间和新帧数量，不会延迟零速指令。
- `--end-verify-seconds 2`：停车验证最长时间；超时保持停止并提示未确认。
- `--curve-min-curvature 0.45`：边线最小曲率，单位 1/米；调大更严格。
- `--curve-min-turn-deg 10`：边线观测段两端方向至少变化多少度；调大更严格。
- `--white-v-min 190`：白线亮度阈值；减小更容易检出暗线，也更容易误检反光。
- `--acquire-line-m 0.12`：新横线最小可见长度；`--min-line-m 0.08`：跟踪已有线的片段最小长度。
- `--side right` / `--side left`：库位所在侧。
- `--seek-seconds 5`、`--max-seconds 20`：未见线搜索时限、总前进时限。
- `--camera-timeout 0.8`：联合图像处理新鲜度上限；底盘反馈时限与执行器看门狗不变。

横线检测范围为前方 0–3 米、对应侧方横向 0.08–1.30 米。
双侧边线复用现有循线函数，只用可靠实测点，不发布循线控制。每侧至少 5 点、覆盖至少 0.35 米。
实际弯道的可见长度和光照仍需行驶试验验证；本测试不自动修正车头方向。

## 静态观察

不带执行参数只打印设置；`--observe` 只处理图像，不发布运动指令。
只开相机时，终端一加 `start_actuators:=false`。已有独立前摄时可加 `start_camera:=false`。
状态：`/parking_line_stop/status`；横线叠加图：`/parking_line_stop/debug`；车道边线图：`/parking_line_stop/lanes`。
网页预览额外终端（先加载上述 ROS 环境）：

```bash
rosrun yihan_ros_pkg ros_flask_viewer.py __name:=parking_line_viewer _port:=8082 /debug/rear_bev:=/parking_line_stop/debug /debug/lane_tracking:=/parking_line_stop/lanes
```

Windows：`ssh -N -L 8082:127.0.0.1:8082 nano@192.168.1.105`，浏览器打开 http://127.0.0.1:8082 。
“后摄鸟瞰”格显示横线检测图，“白线跟踪与循线目标”格显示双侧车道边线。

完整已确认参数保存在 tools/parking_line_stop_best.yaml；当前脚本默认值与参数记录一致。

# 独立的中轴对正与停车参考线测试

仅测试前进定位，不执行 P3 倒库。正式整车的 src、配置和启动文件保持原样。
复用前摄巡线函数和控制器，两条边线都实际可见时，使用同一帧的两侧边线中点。
缺少一侧时可以按原巡线方式寻找两侧，但不宣布对正成功。无硬件转向补偿。

固定参考：右侧整排库位末端的封口横线。检测器要求完整横线及向后连接、向前终止的外侧库位边线；不能用普通中间分隔线替代。参考需连续三帧唯一可见才能锁定。锁定后看不清或存在多个候选即停车，超过一秒未找回退出。

默认前轴离横线 0.70 m，搜索速度 raw 16，接近目标时 raw 12。接近至 0.74 m 开始制动，停车后检查距离误差不超过 6 cm、横向偏差不超过 4 cm、朝向误差不超过 4 度，三帧及以上且持续 0.4 秒才记为 ALIGNMENT_COMPLETE。未达到这些条件保持停车并报告 ALIGNMENT_UNCONFIRMED_STOPPED，不自动重试或倒车。最小停车等待 0.5 秒。

这些是相机估算阈值，不代表已验证的物理精度。停车后用尺子检查前轴到横线延长线的垂直距离、车身左右间距，并保存实际值。参考摄像头现有投影在 70 cm 附近尚需实车核对。

## Nano 终端一：前摄与底盘

现有 p3_parking_test.launch 仍在运行时复用它，不重复启动。该 launch 本身不启动车辆运动。应退出上一次测试脚本和整车控制程序，避免同时控制底盘。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
roslaunch robocup_competition p3_parking_test.launch
```

## Nano 终端二：先观察

首次选右側整排库位前的无遮挡直道，前轴离选定末端线至少约 1.2 m，车身先大致沿车道方向。终点附近必须有一段直道供对正；该测试不承诺任意偏角和弯道位置都能收敛。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
python2 tools/parking_reference_align_test.py --observe --target-gap-m 0.70
```

此模式不发布运动指令。理想输出 reference=locked_row_terminal、gap_m 有数值、dual_pose=True；找不到参考时 reference=reference_missing，缺少两侧可信边线时 dual_pose=False。需要换无遮挡位置后再尝试。Ctrl+C 退出。

## Nano 终端二：执行前进定位

环境变量延续上面的终端。启动有三秒倒计时；空格、x、q、Ctrl+C 都会停车。

```bash
python2 tools/parking_reference_align_test.py --execute \
  --target-gap-m 0.70 \
  --speed 16 \
  --creep-speed 12 \
  --align-lookahead-m 0.65
```

主要参数：target-gap-m 是前轴目标距离，越小停车越靠前；speed 是搜索时 raw 速度；creep-speed 是靠近目标时 raw 速度；align-lookahead-m 是此测试私有巡线前视距离，缩短能减少远处弯道提前影响，但过短可能导致转向摆动。默认没有 -3/-5 补偿。

日志 lateral_m 是中轴在前轴处的左右偏差（正值表示中轴在左侧），heading_deg 是车道相对车身的朝向差；只有两侧真实边线支持的近处直线段才给出有效姿态。

每次执行后的相机估计、逐帧记录和完成原因自动写入：
/home/nano/robocup_ws/field_data/parking_reference_alignment/trial_*.json

建议固定 70 cm 测三种起点：大致居中、轻微偏左、轻微偏右。分别量实际停止距离和左右间距；这些结果决定后续是否调整摄像头标定、前视距离或停车制动余量。先不要接倒库阶段。

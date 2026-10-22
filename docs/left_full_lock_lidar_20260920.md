# 左转满舵与雷达短暂延迟

## 日志依据

本次 `8a2efad2-c2f3-4014-980d-535c80403224/Pasted text.txt` 中，雷达完成时间的年龄多次达到 0.506–0.548 秒，超过原来的 0.5 秒门槛，引发短暂停车。左转底盘采样为 +13、+17、+19、+17、+11；左转接管后出现 -22。

## 当前实现

- 雷达使用独立参数 `lidar_timeout: 0.8`，ROS 回调接收检查与主控新鲜度检查使用同一值，以最后一束光的时间为基准。视觉 `sensor_timeout` 仍为 0.5 秒。新鲜度超过 0.8 秒、时间戳在未来、有效点数不足仍不能放行。这里容纳的是短暂延迟，没有消除雷达处理延迟本身。
- 正式配置开启 `left_turn_full_lock: true`。保留左转前进入段与出口段，弧线段执行固定模型最大转角，经原有映射输出 raw +22。规划半径同步采用 `wheelbase / tan(max_steer)`，当前模型约 0.52 米。满舵模式下 `left_turn_radius: 0.65` 不再决定实际规划半径；关闭满舵模式可回到原参数规划。
- 只有 LEFT 启用新的路径执行方式；右转、直行、掉头及停车继续使用各自执行逻辑。
- 左转交接循线时启用既有 `lane_recovery` 反向渐变，避免一帧从左舵跳到右满舵。同方向响应保持即时。普通循线此前使用的直接转角增益保持原设置。

全程距离与角度仍依赖当前指令积分位姿；满舵输出通过测试不代表真实转弯半径已完成标定。

## 验证

Nano ROS / Python 2 无动作测试 157 项通过，包括满舵弧线、入弯与出弯模型、主控到 raw +22 的映射、视觉接管反向渐变、雷达 0.55 秒延迟继续 / 超过 0.8 秒停止、ROS 回调拒绝过期数据，以及右转、掉头、停车和蓝线任务序列。

只解析 `full.launch` 参数，确认 `left_turn_full_lock=true`、`lidar_timeout=0.8`、视觉超时 0.5、动作速度 28 生效；未启动任何实车节点。

证据：`field_data/left_full_lock_lidar_validation/`，包含修改前复现、最终测试日志、参数解析结果、源码差异及 SHA256。

备份：`/home/nano/refactor_backups/before-left-lock-lidar-srgtCv/before.tar.gz`。

## Nano 完整命令

```bash
bash /home/nano/robocup_ws/tools/sign_detector_20260916/run_yolo_vehicle.sh \
  lidar_enabled:=true lidar_timeout:=0.8 \
  parking_enabled:=true start_rear:=false \
  wait_green:=true sign_ttl:=0 sign_hz:=5 \
  blue_default_straight:=false blue_align_duration_s:=1.0 \
  blue_aligned_advance_m:=0.36 intersection_wait_s:=1.0 \
  straight_distance:=1.25 \
  lane_speed_raw:=26 action_speed_raw:=28 straight_speed_raw:=26 \
  lookahead:=0.55 steering_command_scale_rad:=0.1 \
  left_turn_full_lock:=true \
  parking_mode:=forward_center parking_entry_speed_raw:=12
```

此命令的 P 牌动作是前进入库。后续停车速度与位置反馈需按用户确认的具体动作另行分析。

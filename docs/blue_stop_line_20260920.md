# 车头到蓝线停车与动作交接

**历史版本：用户随后改为“识别蓝线后先按蓝线摆正车身，再单独前进 0.36 米”。当前入口见 [蓝线对齐后前进](blue_heading_entry_20260920.md)，下文的车头到线停车规则已不再使用。**

本次修改直接写入 Nano 的 `/home/nano/robocup_ws/src/robot`；本机对应 SSHFS 挂载目录 `/home/hetaisheng/jetson-nano`。
修改前备份：`/home/nano/refactor_backups/before-blue-stop-line-bEFr8I/before.tar.gz`。

## 当前任务流程

绿灯起步沿用已有起步直行；正常行驶时识别方向牌只缓存，继续循线。前方横向或斜向长蓝线连续确认后，锁定本次事件，继续循线接近。车头前沿到蓝线后停车回正，等待 1 秒，再执行缓存动作。

| 阶段 | 控制及退出条件 |
| --- | --- |
| LANE / GAP | 维持原循线与短暂丢线处理；路牌缓存，不立即执行 |
| BLUE_APPROACH | 保留原循线转向、近处弯道目标和弯道速度；更新本次蓝线位置；最后 0.20 米速度不高于 gap 配置 |
| BLUE_STOP | 速度、舵角输出均为 0；等待 `intersection_wait_s` 后派发缓存动作 |
| STRAIGHT | 从停车位置计距，至少 1.25 米；沿用右侧纵向蓝线小幅纠偏；到最低距离后确认 3 帧出口路径再交接 |
| LEFT / RIGHT | 保留已有转弯内部逻辑；右转模式中的出口蓝线继续用于结束右转并开始出口直行，不再重复停车派发另一个动作 |
| UTURN | 保留已配置七段动作和入口距离；结束后确认前方出口路径，再恢复正常任务 |
| BYPASS | 雷达独立触发已有定时避障；最后 0.30 秒可提前确认出口，动作结束后交给有效路径；保留已缓存的直行牌 |
| PARKING | 当前 `forward_center` 模式也等待蓝线停车；保留缓存 P 牌的车位锚点，进入原车位识别和前进入库逻辑，识别关联底线后 FINISHED |

路线没有 A–K、固定路口编号、固定牌子顺序或全局地图坐标表。测试使用不同动作顺序，包括连续同类直行牌。

## 蓝线判定与停车参考

- 长度仍使用检测器原门槛 `blue.long_min=0.35m`；短计分线不触发。
- 前方 1.60 米范围内，排除纵向参考蓝线以及完全位于相邻通道的蓝线段；允许斜向横线。
- 3 个不同图像时间戳且跨度至少 0.15 秒，才确认本次事件。重复控制循环不增加确认次数。
- 停车点按蓝线的法向计算车头到线的有符号距离，而非看到蓝线后固定行驶 0.36 米。车头参考为 `wheelbase + front_overhang = 0.33m`，原点是后轴。
- 接近时只更新当前蓝线附近且方向一致的观测；不会用另一条远处蓝线重置目标。
- 当前鸟瞰图最近约 0.50 米，最后一段进入盲区后保留本次观测，用已有位姿估计推进。远处目标消失会暂停等待重见；相机过期、红灯和急停仍优先。
- 一条持续可见的蓝线只消费一次；动作内观测不排队给下一动作。动作结束后，连续至少 3 帧、0.30 秒没有可触发蓝线才重新允许下一事件；单帧漏检不再重置消费状态。
- 普通事件没有恢复旧的“全局坐标距离不足 0.70 米就判重复”规则；右转出口和其他旧专用泊车模式仍保留各自原有判定。
- 无牌自动直行默认关闭。`straight_blue_advance_m` 已删除；旧命令不能继续传这个参数。

## 修复与测试范围

修复了接近阶段因为提前设置 action 而丢失普通循线弯道策略、纵向长蓝线误触发、单帧漏检导致重复触发、前进入库绕过蓝线，以及直行结束无条件交给空路径的问题。

直行搜索复用已有 `straight_search_max_distance=2.20m` 与 `straight_search_timeout=20s`。空路径或偏到相邻通道的路径不能完成动作；到上限停下等待有效出口，不会清空动作冒充完成。曲线出口可以接管，不要求整条路径完全笔直。避障和七段掉头也使用多帧前方路径交接。

Nano 上测试均为 Python 2 离线调用，不创建控制 ROS 节点、不发布底盘命令。测试清单与输出在 `field_data/blue_stop_line_validation/tests.log`；新增停车遥测检查单独记录在 `telemetry_tests.log`。启动验证使用 `roslaunch --dump-params`，不启动节点。

最终主回归 165 项通过；停车遥测新增 1 项及 ROS 适配层复验 16 项通过，共 166 项不同测试。模块归属/配置检查、启动脚本语法检查和完整 launch 参数解析通过。

新增 `test_mission_blue_sequence.py` 用两种不同顺序验证从绿灯起步，经直行/左右转/掉头到前进入库 FINISHED 的状态衔接。输入为合成感知和位姿，不能证明真实转弯半径或整场路线已经实跑成功。

另对已有抓拍进行离线检测：单图检查中横向蓝线可触发、同图纵向蓝线被排除；最新抓拍的 30 帧连续序列中，检测关联达到 22 连续帧，满足三帧条件。该序列只检查视觉确认，未闭环重放车辆运动。

证据目录 `field_data/blue_stop_line_validation/` 包含生产代码差异、SHA-256、单图与序列结果；用于对应这次代码。

## 实车验证边界

当前里程仍是 `command_model`，并非编码器实测；停车精度受相机标定、车头尺寸、速度映射、图像延迟和制动滑移影响。需要实车确认车头到线偏差后才能评价停车精度。

转弯、避障和七段掉头的原有距离/时间标定没有重新实测。避障仍沿用已有“一次完成后不再启用普通行驶雷达判断”的策略，不能据此声称能适应任意障碍数量。蓝线事件重新允许触发依赖动作后明确的无蓝线间隔，密集蓝线赛道仍需验证。

## Nano 完整命令

以下由用户在 Nano 执行，会启动车辆；本次修改和验证未执行它。

```bash
bash /home/nano/robocup_ws/tools/sign_detector_20260916/run_yolo_vehicle.sh \
  lidar_enabled:=true parking_enabled:=true start_rear:=false \
  wait_green:=true sign_ttl:=0 sign_hz:=5 \
  blue_default_straight:=false intersection_wait_s:=1.0 \
  straight_distance:=1.25 \
  lane_speed_raw:=24 action_speed_raw:=24 straight_speed_raw:=24 \
  lookahead:=0.55 steering_command_scale_rad:=0.1 \
  parking_mode:=forward_center parking_entry_speed_raw:=12
```

循线与直行基础速度为 24；弯道和临近停车仍会降速。已有七段掉头/定时避障内部仍使用原标定速度，入库为 12。

现场日志主要看：`state`、`blue_approach.remaining_m`、`observation_age_s`、`wait_remaining`、`straight_search.travelled_m`、`exit_confirmation` 与 `reason`。

# 车辆启动与显式泊车方式

在 Nano 的 `/home/nano/robocup_ws` 中启动。车位由 `parking_slot:=P1` 至 `parking_slot:=P5` 明确选择，启动脚本会打印实际选中的泊车方式。

| 显式车位 | 启动选项 | PARKING 牌确认后的流程 |
| --- | --- | --- |
| P1、P2、P3 | `parking_slot:=Pn` | 各自已保存的侧方流程，分别使用 P1、P2、P3 参数 |
| P4、P5 | `--S parking_slot:=Pn` | 直接进入直行入库任务，根据新鲜白色库线和库底线纠偏、判停 |
| P4、P5 | `--T parking_slot:=Pn` | 继续到蓝线；蓝线动作到位后前进右满舵一秒，完成后停车 |

`--T` 一秒动作结束就是本次入库终点。计时从第一条前进右满舵指令开始，短暂的指令过期停车时间不计入这一秒。`--S` 没有可用的白色库线时先停车等待；有有效 P 空间坐标时保留目标库关联，投影为空时使用任务开始后新鲜、真实的双侧库线和库底线。主入口默认启用泊车；不加车位/方式时选择 P4 与 S。

S 在固定直行期间缓存到 P 后就开启库线处理；入库仍只接受任务开始后的新帧。车身偏斜、双侧线暂不可用时，可先沿与 P 位置匹配的一条真实侧线低速纠偏，沿用最多 6 raw 的入库纠偏舵量；随后用双侧线居中、底线判停。单侧线没有 P 关联、图像超时或所有库线不可用时仍停车。等待日志中的 `line_count`、`line_age_s`、`sign_anchor_local` 用于区分无线、旧帧和目标偏移。

S 入库 APPROACH 阶段的 P 已锁定后，即使该 P 移到新路牌触发区域左侧，仍可更新它的位置：要求图像新鲜、置信度合格、投影未裁切且与旧目标距离小于 0.35 m。该更新不重新计票或产生下一条路牌指令；没有已锁定 P、相邻远处 P 和重复旧帧不能走这条跟踪路径。

当前临时设置 `parking_lidar_enabled=false`，仅在 P4/P5 的 S/T 入库任务中跳过雷达扫描就绪与碰撞检查；白线几何、急停、红灯和动作时效仍按原逻辑判断。传入 `parking_lidar_enabled:=true` 恢复入库雷达。

S 入库双侧线不可用时转为目标库底线识别：按已有库线方向或车前目标走廊筛选横向白线，连续两个不同图像帧位置一致后锁定；随后零舵前进，按车头到库底线的距离减速、停车。邻库横线、纵向侧线和过宽道路横线不能直接作为这条底线；底线也不可用时仍停车等待新观测。

P4/P5 的 S 入库有 1.7 m 总上限，沿用参数名 `parking_blue_max_travel_m`：起点改为最近一次固定 STRAIGHT 开始的位置，蓝线接近不计入，固定直行和后续入库共用预算。正常固定直行 1.25 m 后名义上还剩约 0.45 m；移交 P 或进入入库不重新计距。P 已缓存时就启用限制，到上限锁停，原因为 `parking_straight_distance_limit`。日志中的 `parking_straight_travel` 记录该计数；缺少固定直行起点时停止入库，原因为 `parking_straight_origin_missing`。T 的原有蓝线起点规则保留。

下发前进指令前仍预留其 0.25 秒有效期内的预计行程，因此可能提前少量停车。距离沿用现有位姿来源；生产配置目前为底盘指令估计，不能代替独立实测里程。

直行类雷达由 `straight_lidar_once=true` 控制：第一次绕障成功完成、确认车道并交回循线后，关闭普通循线和 STRAIGHT（含其蓝线接近）的雷达检查，后续不再触发同类绕障。首次完成前的前置直行仍保留雷达。中途拦停不消耗次数，重启主控重新启用。雷达节点继续运行，其他动作和后续专用雷达类别保留各自检查；`straight_lidar_once:=false` 可恢复持续的普通行驶检测。

如果旧程序仍在运行，先在原运行终端按 Ctrl+C，再任选一种命令运行：

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_yolo_vehicle.sh parking_slot:=P3
```

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_yolo_vehicle.sh --S parking_slot:=P4 \
  lidar_enabled:=true straight_lidar_once:=true parking_lidar_enabled:=false \
  parking_entry_speed_raw:=16 parking_final_speed_raw:=12 \
  parking_bottom_clearance_m:=0.09 parking_blue_max_travel_m:=1.7
```

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_yolo_vehicle.sh --T parking_slot:=P5
```

P1/P2 与 P3 使用同一种命令格式，替换车位编号即可；P4/P5 均可选择 S 或 T。`--S` 与 `--T` 不能同时传入，也不能用于 P1–P3。旧命令中的 `parking_mode:=forward_plan` 会与这些明确的选择冲突，启动时会报告错误；新命令直接使用上述参数。

方向路牌可连续相同。第一轮右转完成后，新确认的 RIGHT 可以直接缓存；牌子随后离开画面不会清掉指令，到下一条蓝线时也不要求仍能看见牌。执行期间确认的下一条新指令进入一个后续缓存，完成后移交；这个缓存的计时基准不随蓝线停车切换到实际转弯而重置。执行期间持续可见的当前同名牌不会自行成为第二条指令；同名牌消失超过 `sign_timeout` 后再出现可重新计票。重复来源图像和上一条已消费蓝线不触发第二次动作。P 的日志需要区分模型没有输出、停车关闭、等待路线权限、计票和已缓存。

原定掉头动作执行完就结束 UTURN，沿道路前进等待下一条蓝线确认触发。有两侧新鲜白线时取中间路径纠偏；只有单侧线时向道路内侧偏移半个车道宽；无线时按 `straight_speed_raw` 回正直行。纠偏时减速，检查当前车身及未来0.25秒扫掠，任一侧预测越线均停止。此段不对尚未确认的蓝线提前打舵；路牌继续缓存，蓝线确认后交给原有后续动作。避障完成后恢复这段道路跟随。相机超时、白线越界及急停仍会停车；雷达时效和碰撞检查按上述类别规则启用。

生产右转的定时动作结束后，使用正常道路中心路径确认出口姿态。对近段曲线估计车旁切线，避免把圆弧道路要求成直线；在低速纠偏下，朝向偏差不超过 8 度、横向偏差不超过 0.08 米，并由至少 3 个新帧稳定确认 0.5 秒后交回循迹。旧图像、失效路径和障碍物拦截不能计入完成确认。

排查停在蓝线的问题时，先看 `pending` 或 `next_direction` 是否已有 RIGHT，以及 `last_blue_trigger` 是否记录本次蓝线；当前图像没有右转牌不能证明先前没识别到。`gap_limit_wait_for_lane` 是道路路径丢失后的停车结果，不能代替路牌缓存与蓝线触发的原因分析。重启会清空内存中的路线指令，完整复测应从原起点按原流程开始。

白线扫掠检查保留 0.035 米车身余量，并提供 `footprint_clearance_m`、`sweep_clearance_m`、`guard_travel_m`。显式左白线跟随越界时原因为 `left_boundary_footprint_crossing`；UTURN 完成后的道路跟随检查左右两侧，越界时原因为 `uturn_exit_boundary_crossing`，诊断中的 `guard_side` 指明哪一侧拦停。

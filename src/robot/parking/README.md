# 倒车入库

controller.py 管理车位选择与入库；planner.py 规划；entry.py 执行；其余文件负责检测、布局和跟踪。

调参修改本目录 `config.yaml`。默认 `{}` 沿用整车标定；允许参数用 `list` 命令查看。

## P4 / P5 explicit entry styles

Production keeps `parking_mode=forward_center` and selects the explicit style
with `parking_entry_style`:

S 确认 P 后先执行 `vehicle_tuning.yaml` 中的 `parking_p_reverse_s`
（当前 3 秒，0 关闭）：前摄 P 使用车身倒车，后摄 P 使用车身前进，
均以 `parking_entry_speed_raw`（当前 16）远离 P。退让期间也接收库线，
优先跟双侧库线的中心线，只有单侧时向库内偏移半库宽推算中心线；
退让方向的航向反馈与入库方向相反，横向反馈保持同一目标中心。
没有新鲜、关联到目标库的侧线时才保持直轮退让。
换向前后分别停车至少 0.3 秒；3 秒从首次运动指令计时，重复 P 或后到的
P 位置投影不会重置计时。日志 `parking_p_reverse` 表示退让运动，
`parking_p_reverse_gear_pause` 表示换向停车；诊断包含
`p_reverse_elapsed_s` 和 `p_reverse_duration_s`。换向保留已取得的库位几何，
继续由新库线帧更新，
恢复原来的前摄前进入库或后摄倒车入库、库线纠偏和底线停车。
控制间隔超过 0.5 秒或指令被拦截会停止该计时动作，不把停车时间算成退让时间。
后摄找 P 阶段不触发此动作，T 的蓝线触发程序不受此参数影响。

* `S` is armed by the confirmed `PARKING` sign. It uses
  `ExplicitStraightEntry`, reuses measured white side/bottom-line geometry,
  and continues straight at entry speed when no bay geometry is available.
  Missing geometry never re-enters a white-line waiting stop. When both sides are
  unavailable, it searches for a transverse bottom stripe in the target
  corridor and requires two distinct consistent frames before locking it.
  Bottom-only motion uses zero steering and the existing bumper-clearance stop.
  S prioritizes the center between paired sides, then the center inferred
  half a bay width inward from either associated side (38 cm bay: 19 cm).
  Left paint shifts right; right paint shifts left, normal to the measured
  line rather than along the image's horizontal axis. Paired geometry takes
  priority over single-side geometry and the bottom remains the stop reference.
  P-associated acquisition needs 18 cm of side support; a recently locked
  target can retain a 4 cm segment on either side. Neighbouring paint and
  lines beyond the bottom cannot supply this fallback. Diagnostics report
  `left_seen`, `right_seen`, `side_local`, `centerline_local`,
  `centerline_heading_error_deg`, `centerline_lateral_error_m`, and
  `steering_reference` (`both_sides`, `left_side`, `right_side`, or
  `bottom_only_straight` when neither side remains usable).
  P may stand beside the bay: pair acquisition permits its projected position
  up to max(half the configured bay width, 29 cm) outside a side, measured normal
  to the rails, matching the single-side association tolerance while retaining parallel,
  width, finite overlap and endpoint-distance checks and ranking by P proximity.
  Once acquired, matching side geometry supplies the centerline reference.
  A single-side inferred centre cannot reject a measured pair. Only an earlier
  measured pair may constrain the identity of a new pair. A valid pair replaces
  the single-side centre and supplies left/right identity for later one-side
  fallback, so a mistaken P-based side assignment cannot lock out both rails.
* `T` is armed only after the mission controller confirms a blue line. It uses
  `TimedRightEntry` to emit forward plus right full lock for exactly one
  second, then returns `FINISHED` and a zero command. There is no reverse stage.

The module hooks are `start_explicit_parking(ctx, now, sign_anchor=None,
trigger=None)` and `explicit_parking_tick(ctx, now)`. Production temporarily
sets `parking_lidar_enabled=false` for P4/P5 S/T only, skipping scan readiness
and collision checks during the active entry task. Set the launch argument
`parking_lidar_enabled:=true` to restore both checks. The approach before entry
uses its existing lidar policy. S uses straight entry-speed motion before
initial white bay acquisition or when all side references are lost. It continues
from the last measured geometry and current pose when
ground updates pause, without a `parking_front_stale` stop.
后摄 S（以及关闭 parking_s_blue_distance_m 的前摄 S）在已识别的底线从后续图像中消失后，再沿入库方向走 15 cm 停车。
起点是第一张缺失底线图像对应的车辆位置，不是识别 P 或开始直行的位置；
这段按 `parking_final_speed_raw` 低速行驶，有侧线时继续纠偏。
重新看到底线会取消本次计距；之后再次消失重新起算。未识别过底线、重复帧、
单纯相机没有新帧均不触发。P 后 3 秒退让的位移不计入，换向结束后重新取得底线。
15 cm 使用当前位姿来源累计路径长度（生产为指令运动模型估算，并非轮速实测）。
到位原因为 `parking_bottom_lost_15cm`；诊断包含 `bottom_loss_active`、
`bottom_loss_travel_m`、`bottom_loss_remaining_m`、`bottom_visible_current_frame`。
实时看到底线达到原配置间距时仍可直接停车；旧底线距离外推本身不能判定完成。
`line_age_s` remains visible; stale bottom
distance is labelled `measured_bottom_plus_pose`, not `live_bottom`.
`measured_bumper_clearance_m` is the distance at the last bottom observation,
`bottom_age_s` is its age, and `bottom_arrival_confirmed` is the live-clearance gate.
底线视觉匹配保留实际侧线末端，不按 P 的估算深度截断；被完整侧线覆盖的短重复线
不参与选对，避免把短碎段末端误当库底。已确认双侧时，以车库中心判断对齐，
允许 P 牌立在库外侧。共享横线与两侧实际末端相交后，只保留库内可见部分。
前摄已锁定底线后允许大于 8 cm 的短可见片段续跟踪，重叠门限按片段长度缩放，
短片段不能单独首次选底线；后摄的有限端点匹配不变。
`bottom_detection_reason` 区分 `missing_sign_anchor`、`no_transverse_segments`、
`no_matching_bottom` 及实际匹配来源，`bottom_cross_candidates` 记录横线候选数，
`bottom_measurement_stamp` 记录真正更新的时间。PARKING 控制事件至少每秒留一份
几何快照，不再仅靠动作/停车原因变化打印，便于逐秒定位失配。
If command odometry has drifted, a farther stripe
within 45 cm of the prediction requires two distinct consistent frames before
reacquisition (`reacquired_bottom`); a nearer mouth or adjacent bay is rejected.
T faults when the control gap exceeds 0.5 seconds. A delayed interval
contributes at most 0.25 seconds of valid command time.

前摄 S 默认使用 `vehicle_tuning.yaml` 的 `parking_s_blue_distance_m: 1.40`：
锁定 STRAIGHT 对应的已确认蓝线，以前轴越过该线为计距零点；跨线那次位姿更新
插值分离线前/线后距离。此后沿路径累计，前进加、倒退减；退回蓝线前可出现负值，
不会重新置零。识别 P、退让换向及进入入库状态均保留计距，净路程达到 1.40 m
以 `parking_blue_net_distance_reached` 正常完成。库侧线继续纠偏，底线继续检测与
减速，但不再触发 5 cm 或消失后 15 cm 停车。未取得蓝线原点时停住并报告
`parking_blue_origin_missing`，不把识别 P 的位置当作过线位置。
`parking_straight_travel` 记录 `source=front_axle_blue`、`crossed`、`travelled_m`；
计距来源仍是生产指令运动模型，并非轮速实测。设置该参数为 0 可使用原底线停车。
后摄 S 保留底线停车；`parking_blue_max_travel_m` 仍仅用于 T。
During the preceding STRAIGHT segment, a confirmed queued P for S entry
immediately ends STRAIGHT and dispatches the parking state machine, without
waiting for the straight distance or issuing a lane-speed handoff command.
Parking uses `parking_entry_speed_raw` (currently 16), resumes side-line
correction when geometry is available. Unconfirmed P frames do not interrupt
STRAIGHT. The blue crossing travel origin and accumulated distance remain intact across this
early handoff, and a projected P anchor is passed to the parking task.
T retains its blue-trigger origin and
`parking_blue_distance_limit`/`parking_blue_origin_missing` reasons.
Travel sums accepted pose increments along the path; T's guard reserves a
further 0.25 seconds of commanded motion before allowing a nonzero command.
The distance uses the configured pose source (production uses command estimates).

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list parking
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/parking/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test parking --report-dir /tmp/parking-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

## 左转或右转后循线结束，切换后摄倒入（P4/P5 S）

`run_yolo_vehicle.sh --S` 启动时不开后摄 USB 采集和鸟瞰节点。
`rear_camera_gate` 只监听控制状态；收到新鲜的 `PARKING`、后摄入库、
`action_source=turn_path_loss` 状态后，才启动仅包含后摄 USB 和鸟瞰处理的子 launch。
等待状态日志为 `rear_camera_wait`，实际启动日志为 `rear_camera_start`。
按路牌执行原有左转或右转动作，完成后先取得可靠前摄路径并继续循线；
之后持续收到新的图像且连续 2 秒没有路径，才
进入后摄倒库。首次丢线即停止前进，确认后再保持至少 0.3 秒零速换向。
期间只要重新出现路径，即使置信度不足以循线，也清零丢线计时；低置信度
不等于完全丢线。2 秒以图像源时间戳计算，不能靠重复使用同一帧凑足。
摄像头超时、重复帧、转弯后从未取得可靠路径，都只停车等待，不触发倒车。
转弯动作和后续循线保持原逻辑；S 入库启用时，左/右转及其出弯循线期间的
后续方向牌仍按正常的新帧、置信度、确认票数和动作队列规则处理。
确认后的方向指令优先于丢线倒库；已有待执行或下一方向指令时，清除丢线
倒库计时，不因丢线转入倒库。红灯仍可停车。
直行来路仍使用前摄入库。
换向后默认直轮倒退，等待后摄确认 P 牌才进入视觉入库，不需要直行提示牌。
等待期间不使用蓝线或库线打舵，也不锁定底线；前摄 P 不能触发此阶段切换。

后摄直接复用前摄的 `ForwardParking` 视觉控制，不增加独立搜线阶段：双侧线居中，
前视左侧库线控制映射成后视右侧库线控制，底线距离停车和几何记忆同前摄。
左右均以各自观察方向解释。适配器镜像世界坐标并转换车辆朝向，使同一套
左侧线代码实际处理后视右侧线；速度取负，转向按坐标变换和倒车标定转换，
停车距离改用后保险杠。前摄库线和前摄 P 牌不能刷新或重定向后摄任务。
换向停顿结束即按 `parking_entry_speed_raw` 的负值直轮倒行。后摄 P 确认后
先完成上述 3 秒远离 P 的动作，再调用共享视觉控制器，不经过前摄 S 包装层的
缺线停车等待（前摄也已取消）；退让期间持续使用目标库线，换向后继续更新，蓝线在退让结束后接管其原有朝向回退。当前生产入库速度为 RAW -16，接近底线按前摄同一参数减至
RAW -12。没有有效库线时使用共享控制器的直行回退；有双侧线时居中，只有
后视任一侧线时按半库宽纠偏（优先右侧，只有左侧时使用左侧；必须关联到 P 的目标库，
不要求先看到完整双线）。已锁定的库位几何在图像暂停时可结合当前
位置继续控制，但不能仅靠记忆底线和位移估算判定入库完成。
后摄独立识别白色底库横线，连续两张不同图像的横线位置和方向一致后，
才以 `bottom_source=observed_bottom` 建立停车距离，后续匹配更新为
`tracked_bottom`。两侧线仅用于纠偏，不再通过“两侧末端与横线重合”分支
取得或重设底线；侧线较短或断裂不妨碍独立底线确认。只有两侧线、单侧线或
蓝线，没有确认底库横线时，`bottom_seen=false`，不能判定入库完成。
底线确认后复用前摄的实时 5 厘米间距及底线消失后再走 15 cm 停车；`distance_source` 区分
当前底线测量与 `measured_bottom_plus_pose` 外推，后者不是实时距离测量。
后摄倒库已取消累计搜索距离停车条件，不再因达到 1.9 m 停车。
状态中 `travel_m` 仅记录累计移动距离（包括中途前进），`travel_limit_m=null`。
没有 P、已有 P 但未取得库线、或之后丢失库线，都不再由累计距离触发停车。
前进的 `parking_blue_max_travel_m` 不受影响。等待后摄 P 没有额外限时；
确认 P 后才开始倒库自身默认 35 秒限时，
底线停车和急停保留，不要求前面有固定 STRAIGHT 动作。
近处底线出画后的位移仍采用当前配置的位置来源；
生产配置为命令速度估算，实车停止精度需要验证。

在 Nano 端运行（先在原运行终端 Ctrl+C）：

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_yolo_vehicle.sh \
  --S wait_green:=true startup_follow_lane:=true
```

当前视觉入库不再使用 `parking_slot`，不指定 P4/P5；后摄用 P 牌位置选择并
锁定车库。统一库宽读取 `parking_bay_width_m=0.38`，不再按车位编号读取。
启动脚本拒绝旧 `parking_slot:=...`，请从命令中删除。
旧 `parking_reverse_after_right` 参数也不要传入。
其他定时侧方/规划模式的编号配置仍保留在 launch 中。
直接调用 launch 时需启用 `start_rear:=true`。
状态中 `turn_reverse_watch` 展示路径取得和丢失确认，
`parking_entry.camera_source=rear` 表示后摄已经接管。
`parking_reverse_straight_wait_p` / `phase=REVERSE_STRAIGHT_WAIT_P` 表示直轮
倒退等待后摄 P；`rear_sign.decision=parking_started` 和 `parking_started`
记录后摄 P 已接受及入库计时起点。后摄识牌独立输出到 `/competition/rear_sign`，
总控接受时记录 `competition_rear_sign_decision`。
后摄 P 取消检测框高度、位置和两帧投票门槛，新鲜图像置信度达标即可触发。
前摄 +/-6、后摄 +/-10 的入库额外纠偏限幅已删除；使用底盘标定的
完整舵量范围（当前 +/-22 RAW），不是固定打满，实际舵量取决于中心线误差。
速度和底库线距后保险杠 5 cm 的停车条件保持不变，实际效果需现场验证。
`parking_follow_right_side` / `steering_reference=right_side` 表示后视右库线
纠偏；`parking_reverse_missing_white_straight` 表示暂无可用库线、保持直轮倒行。
库边线控制优先：有双侧线时为 `both_sides`，单侧线为 `right_side` 或
`left_side`。此时蓝线不能覆盖库边线舵量，记录 `bay_edges_priority`。
P 确认后暂时没有可用库边线时，连续两帧确认的前/后摄横向长蓝线可提供
车身朝向，分别记录 `front_blue_heading` / `blue_heading`，优先新鲜前摄蓝线。
蓝线限于对应摄像头前方 0.05–1.4 米、横向中心偏差不超过 0.45 米、
法向角不超过 35 度，并要求当前观测未超过 `sensor_timeout`。
`rear_vehicle_lateral_m` 记录相对目标库中心线的横向误差，正值表示车在后视
视角里偏右，负值表示偏左。找到 P 前仍直轮倒行，不按蓝线打舵。
蓝线不进入白底线距离判断，不会单独触发入库完成。
控制事件保留真实的纠偏原因和 `parking_entry` 几何快照：`pair_seen`、
`pair_width_m`、`blue_used`、`blue_reject_reason` 和 `steering_command_rad`
可区分“没取得双线”“蓝线被拒绝”“已输出纠偏”，不再统一显示 tracking_parking。
诊断中的 `side_local`、`pair_local`、`bottom_local` 使用实际后视坐标
（向后为正 x、后视左方为正 y），`rear_frames` 和 `rear_line_count` 显示后摄
连续帧数与线段数。

独立 P4/P5 停车后端保留在 `src/ros/main`，使用它自己的 `config/parking.yaml`。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

# 倒车入库

controller.py 管理车位选择与入库；planner.py 规划；entry.py 执行；其余文件负责检测、布局和跟踪。

调参修改本目录 `config.yaml`。默认 `{}` 沿用整车标定；允许参数用 `list` 命令查看。

## P4 / P5 explicit entry styles

Production keeps `parking_mode=forward_center` and selects the explicit style
with `parking_entry_style`:

S 确认 P 后先执行 `vehicle_tuning.yaml` 中配置的退让：
前摄使用 `parking_p_reverse_s`（当前 3 秒），后摄使用
`parking_rear_p_reverse_s`（当前 1 秒）；各自设置为 0 可关闭。
前摄 P 使用车身倒车，后摄 P 使用车身前进，
均以 `parking_entry_speed_raw`（当前 16）远离 P。退让期间也接收库线，
优先跟双侧库线的中心线，只有单侧时向库内偏移半库宽推算中心线；
退让方向的航向反馈与入库方向相反，横向反馈保持同一目标中心。
没有新鲜、关联到目标库的侧线时才保持直轮退让。
换向前后分别停车至少 0.3 秒；配置的退让时间从首次运动指令计时，重复 P 或后到的
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
关闭 parking_s_blue_distance_m 的前摄 S 在底线丢失后再行进 15 cm 停车；后摄使用下述视觉参考剩余距离策略。
起点是第一张缺失底线图像对应的车辆位置，不是识别 P 或开始直行的位置；
这段按 `parking_final_speed_raw` 低速行驶，有侧线时继续纠偏。
重新看到底线会取消本次计距；之后再次消失重新起算。未识别过底线、重复帧、
单纯相机没有新帧均不触发。P 后退让的位移不计入，换向结束后重新取得底线。
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
前后摄已锁定底线后都允许大于 8 cm 的短可见片段续跟踪，重叠门限按片段长度缩放，
短片段不能单独首次选底线；后摄仍检查片段位于已锁定底线的横向范围内，拒绝邻库横线。
`bottom_detection_reason` 区分 `missing_sign_anchor`、`no_transverse_segments`、
`no_matching_bottom` 及实际匹配来源，`bottom_cross_candidates` 记录横线候选数，
`bottom_measurement_stamp` 记录真正更新的时间。PARKING 控制事件至少每秒留一份
几何快照，不再仅靠动作/停车原因变化打印，便于逐秒定位失配。
前摄 S 锁定底线后，普通跟踪、双侧库线末端选线和重新捕获都必须保持底线连续性。
沿入库方向行驶时，新底线的相机测距不能增加；转向造成的剩余距离变化也不能加大停车预算。
P 后倒退退让允许按实际估算退行增加距离。同一底线仍需满足角度和片段重叠条件。
更远横线被拒绝时记录 `bottom_identity_rejected`，不更新底线时间，也不重置丢线后
15 cm 的停车计距。其他前进模式保留原来的两帧远线重新捕获逻辑。
T faults when the control gap exceeds 0.5 seconds. A delayed interval
contributes at most 0.25 seconds of valid command time.

当前 `vehicle_tuning.yaml` 的 `parking_s_blue_distance_m: 0.0`，使用底线停车。
以下蓝线距离模式仅在该参数设置为正数时启用（例如 1.40）：
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

`runall.sh` 默认使用 S 入库，启动时不开后摄 USB 采集和鸟瞰节点。
`rear_camera_gate` 监听控制状态；S 入库启用时，在新鲜的左/右转 `MANEUVER`
状态提前启动后摄 USB 和鸟瞰处理，避免已经倒行才开始等待摄像头冷启动。
收到新鲜的 `PARKING`、后摄入库、`action_source=turn_path_loss` 状态也可触发启动。
摄像头预热不触发倒车，倒车仍由下述丢线确认流程决定。
等待状态日志为 `rear_camera_wait`，实际启动日志为 `rear_camera_start`。
按路牌执行原有左转或右转动作；固定转弯结束后，有可靠前摄路径就继续循线，
没有路径时持续收到新的图像且连续 2 秒没有路径，才
进入后摄倒库。首次丢线即停止前进，确认后再保持至少 0.3 秒零速换向。
期间只要重新出现路径，即使置信度不足以循线，也清零丢线计时；低置信度
不等于完全丢线。2 秒以图像源时间戳计算，不能靠重复使用同一帧凑足。
摄像头超时和重复帧只停车等待，不触发倒车；固定转弯结束后始终没有路径，
也可由连续 2 秒的新空路径帧触发，不再要求先接到一次车道。
左、右转都只接收固定动作结束后的新帧，转弯过程中的丢线不计入。
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
先完成上述 1 秒远离 P 的动作，再调用共享视觉控制器，不经过前摄 S 包装层的
缺线停车等待（前摄也已取消）；退让期间持续使用目标库线，换向后继续更新，蓝线在退让结束后接管其原有朝向回退。当前生产入库速度为 RAW -16，接近底线按前摄同一参数减至
RAW -12。没有有效库线时使用共享控制器的直行回退；有双侧线时居中，只有
后视任一侧线时按半库宽纠偏（优先右侧，只有左侧时使用左侧；必须关联到 P 的目标库，
不要求先看到完整双线）。已锁定的库位几何在图像暂停时可结合当前
位置继续控制；底线停车按下述视觉参考和带符号净路程规则执行。
后摄在 `command_model` 模式下，库线纠偏使用图像采集时的中心和角度，
新图像到来后更新；避免指令估算转角在两帧之间把同一观测的纠偏方向反过来。
`steering_pose_source=rear_image` 表示使用此反馈。前摄和真实里程计模式保持原逻辑。
选线关联与舵量反馈分开：跟踪新帧时先按前摄方式，用当前车辆位姿比较已确认库位，
避免真实转动后仍拿旧图像角度卡住双侧。命令里程计模式还允许上一图像坐标中的
一致观测匹配，避免仅由估算姿态漂移丢失目标；库宽、平行度和邻库排除仍生效。
`pair_tracking_rejected` 记录当前帧通过几何筛选但未匹配历史目标的双侧候选数。
后摄可见侧线延伸到鸟瞰图远边界时，允许关联到图外更远的 P，
仍保留库宽、平行度和横向位置筛选。有效双侧优先于单侧。
仅对鸟瞰图远边界裁断的后摄侧线，可用至少 8 cm 的共同可见段配双侧；
近处短碎线仍不能按此规则组成车库。单侧退化时，符合目标关联的长线（至少 18 cm）
优先于短碎线，包括另一侧的长线；左右偏好不能盖过可见线段的长度证据。
白线提取前按新鲜 P 检测框排除牌面投影；霍夫线先去除重叠重复段，
再应用 80 条上限，避免同一条粗线耗尽名额而漏掉另一侧。
后摄复用前摄最新的双侧末端与底线匹配：P 在库外侧、侧线短重复段、
共享底线、已锁定底线的短片段，都使用共享算法。真实横线与两条实际侧线末端
相接可直接建立底线测量；只有侧线而没有横线不能取得底线，更不能宣布到位。
后摄用本帧实测双侧优先定位目标库；双侧不完整时，可用与目标关联的本帧长单侧
向库内偏移半个库宽，避免旧双侧坐标覆盖新单侧观测。没有任何目标侧线依据时，
孤立横线不能初始化底线。独立底线判定复用前摄 S 的 30–60 cm 横向跨度、
中心与方向检查、侧线远端前方最多 25 cm 的容差；该容差是识别筛选量，不是停车距离。
不再额外要求后摄横线两端各自贴合固定库宽至 8 cm。近处库口和邻库横线仍须排除。
连续两张不同图像的位置和方向一致后，
才以 `bottom_source=observed_bottom` 建立停车距离，后续匹配更新为
`tracked_bottom`。已确认的后摄底线不会被更近的侧线末端重设；
命令里程计模式下，首次两帧确认也允许图像坐标中的一致性，避免估算位姿漂移
反复清空确认票。`bottom_corridor_source` 显示本帧双侧、单侧或历史双侧来源，
`bottom_rejection_counts` 记录横线被宽度/跨度、方向/中心、侧线远端或深度检查拒绝的数量。
侧线较短或断裂不妨碍独立底线确认。只有两侧线、单侧线或
蓝线，没有确认底库横线时，`bottom_seen=false`，不能判定入库完成。
共享底线延伸到相邻库时，跟踪阶段截取它与已确认目标库的实际重叠部分，
继续刷新同一底线；仅位于邻库的片段不刷新。近处库口不能先取得底线再裁掉真实双侧。
底线确认后的停车使用下述第 318 行视觉参考，旧 5 cm / 消失后固定 15 cm 规则不适用。
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
bash runall.sh \
  --S wait_green:=true startup_follow_lane:=false parking_entry_speed_raw:=16
```

同一条命令兼顾三种来路：直行牌→前摄 P→正向入库；左转／右转完成→
连续 2 秒无路径→后摄找 P→倒车入库。绿牌起步沿用当前不要求先有车道的流程。
P 后退让方向相反：前摄倒退 3 秒、后摄前进 1 秒，分别由上述两个参数配置。
前摄 1.70 m 净距离停车已关闭，保留前摄距离不增限制。
后摄恢复视觉参考停车：底线位于鸟瞰图第 318 行时剩余 20 cm；
有效视觉可修正剩余距离，倒车入库扣净路程、前进退让加距离。
后摄底线在命令里程计模式下还按相邻图像中的深度、角度和横向重叠跟踪，
避免估算位姿漂移后拒绝仍然可见的底线。旧底线未在当前帧确认时，不再裁掉
当前双侧；同一目标库的双侧末端和闭合横线连续两帧一致，可以重新确认底线，
记录 `paired_bottom_reconfirming` / `paired_bottom_reacquired` 并更新视觉参考。
后摄保留原来的单条远横线拒绝和双侧连续两帧重新确认，不套用前摄的距离不增限制。
后摄底线丢失后扣减最后视觉参考剩余距离，不重新起算固定 15 cm。
只有最后有效观测已经到达近处参考位置，才允许扣到零后完成；
远处底线丢失时继续等待视觉确认。停车原因为 `parking_rear_visual_reference_reached`。
前后摄 P 已确认但位置投影暂缺时，都可以用当前可见双侧库线居中；
后摄日志 `parking_started_unprojected` 表示已进入入库、等待补充 P 位置，
不会因缺少位置投影而完全屏蔽双侧纠偏。已有目标的相邻库排除继续保留。

当前视觉入库不再使用 `parking_slot`，不指定 P4/P5；后摄用 P 牌位置选择并
选择车库。后摄首帧 P 触发入库，后续同一块 P 的有效投影持续更新位置，
不冻结第一次小检测框得到的坐标。沿用前摄目标跟踪的位置/尺寸连续性识别
同一块牌；不会切换到图像位置明显不同的另一个 P。投影暂缺时保留上次有效位置。
更新 P 不重启退让和入库计时，不重置白线观测边界、已取得的库边线或底线。
当前双侧库线优先于单侧，当前可用库边线优先于 P 投影用于纠偏。
日志 `rear_sign.decision=parking_p_position_updated`、`position_stamp` 和
`position_updates` 表示有效位置刷新；`parking_p_observed_unprojected` 表示
这次看到 P 但未获得新投影，继续使用上次位置。`target_locked` 表示已选定目标 P，
不表示 P 坐标停止刷新。统一库宽读取 `parking_bay_width_m=0.38`。
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
后摄参考位置和剩余距离由 `rear_bev.bottom_reference_v` / `bottom_reference_remaining_m`
配置（当前 318 行 / 20 cm）；实际停止精度仍需现场验证。
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

## 巡线 P 牌侧方停车

正常整车启动默认启用 `parking_side_lidar_enabled`。只有在 `LANE` 且没有
执行动作时新确认的前摄 P 牌才进入 `TIMED_PARKING`。转弯/直行动作中缓存的
P 牌、丢线与后摄 P 继续走原有 S/T 流程；红灯、急停等停止条件仍有效。
手动 `parking_slot:=P1/P2/P3` 的单库测试仍按指定动作表执行。

流程：巡线新确认前摄 P → 最高 RAW 16 沿道路巡线 → 前摄识牌的新画面
中没有任何 P 候选 → 立即下发零速，同一控制周期进入 `SELECT_SLOT` 雷达选库 →
锁定库号 → 执行原有 P1/P2/P3 五段动作 → 整车 `FINISHED`。

P 的可见性直接读取 `/competition/sign` 的完整 `detections`，接受 `P`、
`park`、`PARKING`。首次进入仍用原有确认置信度和路线入口；进入以后，只要
模型检测中仍有 P，就视为 P 仍在画面中，不因低置信度、尺寸变小、路线区域
筛选拒绝或另一标志更大而判消失。多个 P 全部不再被检测到时才触发。

只接受本次任务开始以后、源时间戳递增且在 `sign_timeout` 内的新鲜前摄
检测结果。重复、旧帧、未来帧、后摄消息、缺少/损坏完整检测列表以及识牌报错
不构成 P 消失。识牌消息超时记录 `P_SIGN_TIMEOUT` 并停车报错，不启动选库；
接近阶段总时限沿用 `parking_timed.max_seconds`（30 秒）。前摄车道仍须新鲜，
在等待 P 消失时接管巡线；无有效路径/图像按原控制器停车处理。

自动侧方不再需要横白线、道路判弯、P 位置投影或位姿对齐。自动模式下
`parking_timed_vision` 保持空闲，不启动额外白线工作线程；普通巡线和识牌继续。
第一张有效的无 P 图像立即停车选库，没有停车后二次确认。
`parking_pause_seconds` 保留最短 0.5 秒停车，与雷达选库并行；选库和动作表
阶段不要求前摄或识牌继续更新，P 重现也不撤销已开始的选库。
手动单库测试仍用原白线末端确认，动作参数不变。

`parking_p_visibility` 记录 P 消失的新帧和任务时间；
`parking_timed.stop_mode=p_sign_disappearance`、`parking_timed.p_sign`
记录可见性、源时间、年龄和原因。标志牌检测消失位置与最终物理停位需驶入实测。

雷达使用 `lidar_web_test.py` 的右前 1 米参考直线及框 1/2，几何代码共用
`robot/lidar/side_bays.py`；两个框只平移、不旋转，共同边界属于框 1。
默认框内至少 10 点为大量点，连续 3 张不同的新鲜雷达帧确认同一结果：
框 1 达阈值、框 2 未达阈值选 P1；框 2 达阈值、框 1 未达阈值选 P2；
两框均未达阈值选 P3；两框均达阈值保持停车、清空确认票。
两框均有少量点仍按阈值判断，不能把“雷达未收到/未找到参考直线”当成 P3。

雷达几何计算由独立、只发布观测的 `parking_side_lidar` 节点完成，只有
`SELECT_SLOT` 时计算，采集回调只缓存最新帧。只接受选库阶段开始以后
采集且未超过 2 秒的 `/scan`，参考线和至少 30 个有效距离点均须有效。
等待超过 15 秒进入 `FAULT` 并持续停车；两框冲突期间如果后来得到明确
且连续确认的结果，可在超时前继续选库。选库后库号锁定，执行期间不再重选。

参数在 `/home/nano/robocup_ws/src/robot/config/vehicle_tuning.yaml`：
`parking_side_lidar` 配置点数、帧数、等待时间、有效点数与处理频率；
动作仍修改 `parking_timed.sequences.P1/P2/P3`。
关闭侧方入口时给整车启动命令追加 `parking_side_lidar_enabled:=false`。
启动参数覆盖 YAML 中的同名开关。

Nano 端整车命令（会驱动车辆，齿轮修复后由操作员执行）：

```bash
cd /home/nano/robocup_ws
bash runall.sh
```

`/competition/status` 的 `parking_timed.selection` 记录两框点数、票数、
选库原因和采集时间；`/competition/parking_side_bays` 提供当前几何观测。
白线/选库节点的调度使用独立的 `/competition/parking_hint` 小消息，
每个控制周期捕获阶段，由遥测线程在尝试整车状态锁前发布；整车诊断忙时
不会因 `/competition/status` 更新间隔而暂停停车传感器。
本次只做无底盘命令的离线回放及启动文件检查，停车位置和点数阈值需实车验证。

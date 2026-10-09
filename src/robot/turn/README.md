# 左右转

## 共用图像蓝线接近（当前默认）

LEFT、RIGHT、STRAIGHT、UTURN 确认路牌后，先将循迹速度限制到
`align_speed_raw`，保留原循迹方向。第一帧经过长度、角度和车道范围
筛选的路口蓝线候选出现后，就调用 `blue_timed.py` 修正方向；不用再等蓝线
连续3帧确认完才开始打舵。候选确认期间不触发动作、不开始直行计时。
正式确认同一蓝线后，沿用候选期间的对齐帧数与时间，不重置对齐进度。
远处按 `align_tolerance_deg`（当前10度）修正方向。进入触发带后，
按 `heading_tolerance_deg`（当前15度）同时确认方向与位置，仍要求连续3帧。
超出15度时回正停车，最多用 `align_recheck_s`（当前1秒）等待新的图像复核；
这不是FAULT，期间不计直行时间。必须新图像连续确认合格才继续，否则超时报错。
当前触发带为70%～90%，合格后舵机回正、raw 20继续直行1秒后停车，等待 `intersection_wait_s`
后分派原动作。PARKING 仍走原专用分支。正式与独立测试共同读取
`blue_stop_test.yaml`，后续改该文件会同时改变两者。
方向修正复用 camera/alignment.py 的函数；此模式不使用旧的固定1秒对齐、
按轴距停车、固定0.36米进入流程。新增对齐参数与原计时参数在同一文件。
正式蓝线提前对齐、接近和停车后分派统一使用 `ground_timeout`（当前1.25秒）
检查前相机源时间戳，不再额外使用独立测试的0.6秒门槛。相同图像的重复控制
周期不会增加确认计数。独立蓝线停车测试仍使用 `camera_timeout_s`（当前0.6秒）。
相机数据超过1.25秒、雷达中断、触发带越过、姿态不合格或碰撞保护仍会锁存
FAULT，已开始计时的接近被保护中断后不会自动续跑。
蓝线接近和提前对齐允许不超过0.5秒的控制调度间隔；底层0.25秒无新指令
停车保护仍生效。触发后的直行计时扣除每个间隔中超过0.25秒的时间，
恢复时仍检查相机和障碍保护。间隔超过0.5秒或控制时钟倒退才报
`blue_timed_control_gap`（提前对齐为 `blue_prealign_control_gap`）。
计时日志的 `elapsed_s` 为累计有效指令时间，`paused_s` 为累计扣除时间，
`control_gap_s` 为当前控制间隔；`trigger` 保留原始蓝线触发时间。
`blue_timed_enabled: false` 可恢复历史接近分支。
日志 `direction_sign_slow_approach` 表示路牌确认后的减速循迹，
`straight_blue_prealign`（或其他动作前缀）表示确认蓝线期间已开始修正。
`blue_prealign.sequence.debug` 显示此阶段的画面位置和角度；正式接近中的
`prealigned_during_confirmation: true` 表示已接续提前对齐结果。
`straight_blue_recheck` 表示暂时停车复核，`blue_timed_alignment_recheck_timeout`
才表示复核超时后的锁定停车。长时间断流、障碍保护和触发带越界仍会锁定停车。

正式 RIGHT 的定时动作依次为：回正倒车1.1秒（raw -30）、前进右转4.7秒
（raw 30，转向 -22）、回正倒车2.5秒（raw -30）、回正停车2秒。
停车阶段 `EXIT_SETTLE` / `right_timed_exit_settle` 始终输出零速度、零转向，
满2秒后同一控制周期直接恢复普通巡线，不再执行出口找线、持续对齐或额外限速。
最后倒车和停车时间分别由 `right_timed_exit_reverse_s`、`right_timed_exit_stop_s` 控制。
恢复后的速度和转向由普通巡线决定；没有有效车道时仍按普通巡线规则停车。
已缓存的下一路牌交回正常分派流程，不再要求先完成右转专属出口对齐。

正式 RIGHT 由新鲜画面中右转牌消失来解除旧牌锁定，不再按转向时间释放。
完整 YOLO 候选列表没有 RIGHT 的首帧立即清空旧牌投票，允许下一块 RIGHT
按原来的置信度0.80、连续2帧条件确认，无须额外等待 `sign_timeout`。
置信度下降但候选框仍在、相机断流或延迟旧帧均不能解除锁定；即使动作已
结束，只要同一块牌连续可见，也不能重复确认。日志 `right_sign_disappeared`
表示空画面解除了锁定，`right_visible` 表示完整候选列表中是否仍有右转牌。
这里没有跨帧物体跟踪，单帧完全漏检也会视为消失。
已经确认的下一条指令继续保留；最后动作和出口循线交接完成后，将它交给
正常蓝线触发流程，不提前打断当前右转。


当前正式 LEFT 默认采用 `timed_left.py` 两段定时动作，参数集中在 `config.yaml` 的
`left_timed_*`。确认左转牌后接近蓝线，前轴到线处停车等待，然后 raw 30/0
执行 1.5 秒，raw 30/+22 执行 3.5 秒。没有原来的额外 0.12 米进入段，
没有视觉提前结束转向。两段结束回正停车，确认新鲜出口车道后恢复循迹。
保护停车或控制时钟断续会锁存 FAULT，防止中断后自动续跑。
脚本中的 action_speed_raw 不覆盖上述独立速度。
以下旧满舵视觉说明仅适用于 `left_timed_enabled: false`。

controller.py 管理转弯及路口直行；planner.py 生成轨迹；geometry.py 判断转弯出口。

调参修改本目录 `config.yaml`；允许参数用 `list` 命令查看。

当前整车启动脚本使用 `max_steer=0.2`、命令尺度 `0.03`、左右转动作速度
`20`。显式传入 `action_speed_raw` 时，入口和满舵阶段使用该速度。

- 左转先直行 `left_turn_entry=0.12 m`，再保持当前满舵（RAW `+22`）。
  估算转角至少 `left_lock_min_angle_deg=30` 后，出口路径须在车辆前方、
  横向误差小于 `exit_lateral_tolerance=0.18 m`、近端朝向误差小于
  `left_lock_heading_deg=20`，并由 `exit_frames=3` 张新图像连续确认。
  随后立即交给循线，不因规划圆弧结束先回正。超过目标转角 30 度或动作
  超时仍未找到出口时停车。估算角度和距离仍依赖车辆模型，需实车确认。
- 右转保留倒退 `0.25 m` 后满舵找新蓝线的流程。舵量限制在当前命令尺度
  内（尺度 `0.03` 对应 RAW `-22`）；符合位置、新鲜度、去重条件的蓝线
  必须连续出现于 `exit_frames=3` 张新图像，才开始独立直行 `1.25 m`。
  相同图像的重复控制周期不增加确认计数。
- 规划与位姿累计使用 `motion/config.yaml` 的独立底盘模型，前进满舵左、
  右后轴半径分别约 `.678 m`、`.993 m`；不再用循线的 `max_steer=.2`
  推算成 `1.283 m`。这些是已有标定记录，当前实车仍需核对。

直行牌的流程是：缓存 STRAIGHT → 横向蓝线触发并停车 → 从该动作起点
按 `straight_distance` 前进 → 继续循线。正式直行牌动作使用本目录
`config.yaml` 的 `straight_steering_raw: -2`，整段保持轻微右偏，不叠加右侧蓝线纠偏；
负数向右、正数向左、0回正，单位为RAW，不是角度。该参数不用于绿牌启动或普通巡线。
行程累计使用命令反馈的源时间戳与统一速度映射，不使用控制回调次数。

UTURN 的固定动作表和普通循线参数由各自模块管理。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list turn
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/turn/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test turn --report-dir /tmp/turn-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

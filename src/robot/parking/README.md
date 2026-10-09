# 倒车入库

controller.py 管理车位选择与入库；planner.py 规划；entry.py 执行；其余文件负责检测、布局和跟踪。

调参修改本目录 `config.yaml`。默认 `{}` 沿用整车标定；允许参数用 `list` 命令查看。

## P4 / P5 explicit entry styles

Production keeps `parking_mode=forward_center` and selects the explicit style
with `parking_entry_style`:

* `S` is armed by the confirmed `PARKING` sign. It uses
  `ExplicitStraightEntry`, reuses measured white side/bottom-line geometry,
  and stays stopped until fresh bay geometry is visible. When both sides are
  unavailable, it searches for a transverse bottom stripe in the target
  corridor and requires two distinct consistent frames before locking it.
  Bottom-only motion uses zero steering and the existing bumper-clearance stop.
* `T` is armed only after the mission controller confirms a blue line. It uses
  `TimedRightEntry` to emit forward plus right full lock for exactly one
  second, then returns `FINISHED` and a zero command. There is no reverse stage.

The module hooks are `start_explicit_parking(ctx, now, sign_anchor=None,
trigger=None)` and `explicit_parking_tick(ctx, now)`. Production temporarily
sets `parking_lidar_enabled=false` for P4/P5 S/T only, skipping scan readiness
and collision checks during the active entry task. Set the launch argument
`parking_lidar_enabled:=true` to restore both checks. The approach before entry
uses its existing lidar policy; S still requires fresh white bay geometry.
T faults when the control gap exceeds 0.5 seconds. A delayed interval
contributes at most 0.25 seconds of valid command time.

The S-style `parking_blue_max_travel_m=1.7` bound now starts at the preceding
fixed STRAIGHT entry, excluding blue approach. `parking_straight_travel` persists
across lane handoff and parking entry; the nominal 1.25 m straight leaves 0.45 m
for entry. A queued P instruction already activates the bound.
`parking_straight_distance_limit` latches a stop; missing fixed-straight history
stops entry with `parking_straight_origin_missing`. T retains its blue-trigger
origin and `parking_blue_distance_limit`/`parking_blue_origin_missing` reasons.
Travel sums accepted pose increments along the path, and the guard reserves a
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

独立 P4/P5 停车后端保留在 `src/ros/main`，使用它自己的 `config/parking.yaml`。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

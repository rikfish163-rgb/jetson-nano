# 避障

controller.py 检查障碍并选择停车或绕行；planner.py 生成绕行路径。

调参修改本目录 `config.yaml`；未覆盖参数沿用整车标定。

在 Nano 的 `/home/nano/robocup_ws` 执行 `./avoid` 会启动车辆。
当前定时避障：前方目标进入后轴 0.85 米范围，先进入 `SETTLE_LEFT`，
下发零速度、左满舵并等待 `timed_bypass_settle_s: 0.4` 秒，再以 raw 26 左满舵 1.5 秒、
右满舵至少完成 4.0 秒（`timed_bypass_right_s`），安全停车时暂停计时。
此后连续 3 帧、跨至少 0.15 秒确认可接管车道才交回原循线，
日志为 `bypass_visual_lane_handoff`；一帧有效或航向估算不代表动作完成。
检查近处车道的方向、位置、点间连贯性，并在车身运动补偿后检查跨帧一致性。
车道可有横向偏移，交接后由原循线恢复到中心，交接当次速度不超过 RAW 26。
未找到可接管车道时最多右转到 6 秒（`timed_bypass_right_max_s`），然后停车等待，
日志为 `bypass_wait_exit_lane`；等待时确认连续新画面后仍可恢复循线。
避障期间直行牌可缓存，退出后交给原状态机按蓝线条件执行，不直接跳入直行动作。
当前循线不区分左侧或右侧目标车道。0.85 米约等于车头前方
0.52 米，可用 `timed_bypass_trigger_distance_m` 调整。形状筛选为几何判断，
不是视觉路锥分类；固定转弯时长也不代表已经测量确认绕过路锥。
交接还要求估算的触发目标已位于车身后方，不能替代独立里程或目标测量。
准备阶段不计入左转行驶时间；红灯、急停或雷达超时使其停车回正后，
重新开始准备计时。0.4 秒是待标定的机械换向余量，不是轮角到位反馈；
底盘是否支持静止打舵仍需先架空验证。它不提供绕锥净空保证。

绕障成功确认车道并交回循线后设置 `timed_bypass_completed=true`。
生产配置 `straight_lidar_once=true`：首次成功完成前，保留触发、扫描时效和碰撞检查；
完成后关闭普通循线和 STRAIGHT 动作的直行类雷达检查，也不再触发同类绕障。
中途拦停、扫描过期或等待出口车道都不消耗这一次机会；重启主控重新启用。
雷达节点继续运行，LEFT、RIGHT、UTURN 和其他专用流程仍按各自规则使用雷达。
P4/P5 的 S/T 入库由独立的 `parking_lidar_enabled` 控制，当前临时关闭。
设 `straight_lidar_once:=false` 可恢复绕障后的普通行驶检查和不同目标触发；
在该配置下仍记录已完成目标，避免同一目标重复触发。

定时避障期间检查近场车身碰撞和未知空间；雷达超时、红灯及急停暂停动作计时。
侧方净空不足时可短暂采用已检查的较小舵角绕开，期间不计入满舵动作时间。
左转结束前检查右弧线入口，必要时延长左转最多 1.5 秒，仍受碰撞保护约束。
旧日志的 `bypass_wait_forward_lane` 等待分支已移除；
`gap_limit_wait_for_lane` 表示循线阶段丢线超过允许距离或时间。
固定路锥回归测试覆盖触发、绕行和控制权回切，但假定相机提供有效车道，
不能代替实车车道重捕获验证。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list obstacle
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/obstacle/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test obstacle --report-dir /tmp/obstacle-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

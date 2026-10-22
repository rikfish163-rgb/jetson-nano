# 避障

controller.py 检查障碍并选择停车或绕行；planner.py 生成绕行路径。

调参修改本目录 `config.yaml`；未覆盖参数沿用整车标定。

在 Nano 的 `/home/nano/robocup_ws` 执行 `./avoid` 会启动车辆。
当前定时避障：前方目标进入后轴 0.85 米范围，先进入 `SETTLE_LEFT`，
下发零速度、左满舵并等待 `timed_bypass_settle_s: 0.4` 秒，再以 raw 26 左满舵 2 秒、
右满舵必须完成 5 秒；期间即使发现有效车道也不提前切回循线，安全停车时暂停计时。
满 5 秒后，有可用车道则交回循线，日志为 `bypass_timed_lane_handoff`；
没有可用车道则停车等待，日志为 `bypass_wait_exit_lane`，不继续盲目右转。
避障期间直行牌可缓存，退出后交给原状态机按蓝线条件执行，不直接跳入直行动作。
当前循线不区分左侧或右侧目标车道。0.85 米约等于车头前方
0.52 米，可用 `timed_bypass_trigger_distance_m` 调整。形状筛选为几何判断，
不是视觉路锥分类；左转 2 秒、右转 5 秒也不代表已经测量确认绕过路锥。
准备阶段不计入左转行驶时间；红灯、急停或雷达超时使其停车回正后，
重新开始准备计时。0.4 秒是待标定的机械换向余量，不是轮角到位反馈；
底盘是否支持静止打舵仍需先架空验证。它不提供绕锥净空保证。

定时转弯结束后设置 `timed_bypass_completed=true`，记录本次目标的位置避免重复触发。
后续不同障碍仍可触发避障；雷达碰撞检查和扫描超时停车继续生效。

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

# 避障

controller.py 检查障碍并选择停车或绕行；planner.py 生成绕行路径。

调参修改本目录 `config.yaml`；未覆盖参数沿用整车标定。

在 Nano 的 `/home/nano/robocup_ws` 执行 `./avoid` 会启动车辆。
当前定时避障：前方目标进入后轴 0.85 米范围，先进入 `SETTLE_LEFT`，
下发零速度、左满舵并等待 `timed_bypass_settle_s: 0.4` 秒，再以 raw 26 左满舵 2 秒、
右满舵必须完成 6 秒；期间即使发现有效车道也不提前切回循线，安全停车时暂停计时。
满 6 秒立即退出避障、交回循线，不再卡在避障等待车道状态；无车道时沿用普通循线保护。
避障期间直行牌可缓存，退出后交给原状态机按蓝线条件执行，不直接跳入直行动作。
当前循线不区分左侧或右侧目标车道。0.85 米约等于车头前方
0.52 米，可用 `timed_bypass_trigger_distance_m` 调整。形状筛选为几何判断，
不是视觉路锥分类；左转 2 秒、右转 6 秒也不代表已经测量确认绕过路锥。
准备阶段不计入左转行驶时间；红灯、急停或雷达超时使其停车回正后，
重新开始准备计时。0.4 秒是待标定的机械换向余量，不是轮角到位反馈；
底盘是否支持静止打舵仍需先架空验证。它不提供绕锥净空保证。

定时转弯结束后设置 `timed_bypass_completed=true`：本次运行不再触发第二次避障，
普通行驶不再因雷达障碍或扫描超时停车，雷达节点仍可采集数据。
这也意味着后续真实障碍不会自动停车；急停、红灯、车道保护及需要雷达的停车流程保留。
重启主控后该标记清零。直线点云过滤与不规则聚类识别逻辑保持不变。

定时避障期间不再使用预测车身碰撞停车，诊断显示 `timed_bypass_sweep_disabled`。
即使点云位于车身前方也会继续转弯，存在实际碰撞风险；雷达超时、红灯及急停仍暂停动作。
其他仍启用雷达的流程保留 `lidar_obstacle_in_sweep` 碰撞停车。
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

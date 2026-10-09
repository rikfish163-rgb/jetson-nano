# 左右转

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
- `left_turn_full_lock:=false` 仍可选择规划路径跟踪。以当前车辆模型计算，
  最小半径约 `1.283 m`；`left_turn_radius=0.65` 不代表模型能生成该半径。

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

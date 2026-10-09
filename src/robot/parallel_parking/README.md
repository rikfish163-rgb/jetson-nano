# 测量闭环停车

`scene.py` 将前相机车位观测和雷达证据组成固定车位坐标下的场景；
`controller.py` 管理输入门控及停车阶段；`planner.py` 规划并执行两类车位停车。

默认 `parking_mode:=forward_plan`：P1–P5 共用前相机测量、前进规划和闭环跟踪。
`parallel_reverse` 保留为侧方倒车的显式选择。调参修改本目录 `config.yaml`；
允许参数用 `list` 命令查看。

输入必须明确车位编号、测量来源、时间戳和可行驶区域。生产控制需要
`ready=true` 且 `occupancy=FREE`，新帧 UNKNOWN／OCCUPIED、缺少门控或失效时停止。
相机只输出几何，没有语义编号；显式 P1/P2/P3 的无编号关联依赖配置排序，
必须满足完整候选数。AUTO 需要观测提供指定目标编号。
`command_model` 不能当作测量位姿；默认通过相机相对车位反算车辆位姿。

可行驶区域保持名义道路及车位的精确尺寸。相邻同类车位只有在完整图像中
测到边界、且当前雷达确认 FREE 时才加入；后续 UNKNOWN／OCCUPIED 会立即
移除该区域并检查剩余路径。借用相邻空位可能跨白线，按赛规仍可能扣分。
路径全部前进，找不到路径或测量失效时停车；不会自动改成倒车或开环。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list parallel_parking
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/parallel_parking/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test parallel_parking --report-dir /tmp/parallel_parking-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

# 运动执行

controller.py 执行选定轨迹、停车和转向反馈；tracker.py 跟踪路径。

调参修改本目录 `config.yaml`。默认 `{}` 沿用整车标定；允许参数用 `list` 命令查看。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list motion
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/motion/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test motion --report-dir /tmp/motion-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

独立预览（不启动底盘）：

```bash
roslaunch /home/nano/robocup_ws/src/robot/motion/debug.launch
```

接管与执行桥在 `src/ros/manual`，硬件执行在 `src/drivers/base`。预览输出为 `/modules/motion/output`。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

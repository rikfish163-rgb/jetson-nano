# 雷达

scan.py 处理扫描、占用证据和障碍分类；lidar_preview_node.py 发布独立预览。

调参修改本目录 `config.yaml`。默认 `{}` 沿用整车标定；允许参数用 `list` 命令查看。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list lidar
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/lidar/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test lidar --report-dir /tmp/lidar-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

独立预览（不启动底盘）：

```bash
roslaunch /home/nano/robocup_ws/src/robot/lidar/debug.launch
```

预览默认复用已启动的传感器；需要本入口启动传感器时加 `start_devices:=true`。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

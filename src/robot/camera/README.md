# 摄像头

vision.py 处理图像；observations.py 整理车道/地面观测；perception_node.py 接入 ROS。

调参修改本目录 `config.yaml`。默认 `{}` 沿用整车标定；允许参数用 `list` 命令查看。

前摄像头蓝线不再按角度过滤路口标记；颜色、厚度、长度筛选不变。
当前 `blue.long_min=0.35` 米，短线仍为 tick，不触发方向动作。
方向缓存及主控 0.36 米触发距离条件不变。纵向长蓝线也可能成为路口标记，
存在误触发风险，需现场验证；后摄像头保留原角度过滤，蓝线朝向测量仍保留。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list camera
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/camera/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test camera --report-dir /tmp/camera-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

独立预览（不启动底盘）：

```bash
roslaunch /home/nano/robocup_ws/src/robot/camera/debug.launch
```

预览默认复用已启动的传感器；需要本入口启动传感器时加 `start_devices:=true`。

已有相机 ROS 节点和标定在 `src/ros/camera`，设备驱动在 `src/drivers/camera`。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

# 识别

sign_node.py 调用模型；decisions.py 处理识别决策；range.py 计算距离。

正式定时右转在前进右满舵段执行约1秒后，清除当前路牌的投票缓存并重新开放识别。
两张新的合格 RIGHT 图像可确认下一条右转指令，允许连续两个相同路牌，
不再额外等待牌子消失。早于缓存释放时刻的延迟图像会被丢弃；已确认的下一条
指令不会被清除，当前动作交接结束后仍需正常蓝线触发。

调参修改本目录 `config.yaml`。默认 `{}` 沿用整车标定；允许参数用 `list` 命令查看。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list signs
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/signs/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test signs --report-dir /tmp/signs-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

独立预览（不启动底盘）：

```bash
roslaunch /home/nano/robocup_ws/src/robot/signs/debug.launch
```

预览默认复用已启动的传感器；需要本入口启动传感器时加 `start_devices:=true`。

模型推理实现和模型资源在 `src/ros/signs`。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

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


## 路牌抓拍调试

整车脚本默认 `sign_capture:=true`。每次运行保存到
`/home/nano/robocup_ws/field_data/sign_capture/时间_PID/`。
`*_frame.png` 是实际推理的原始完整图，`*_crop.png` 是实际路牌裁剪，
同名 JSON 包含相机源时间戳、最终标签、模型首选标签、置信度、全部候选和拒绝原因。
模型检测到候选不等于总控确认了指令，请同时查看 `competition_sign_decision`。

同一有效标签每秒最多保存一帧，标签变化立即保存；被拒绝/无候选的画面每两秒最多一帧，
每次运行最多保存 60 帧此类证据。后台有界队列写盘，不在推理回调内等待磁盘。
单次抓拍上限 128 MiB，并保留至少 256 MiB 可用空间；达到限制停止抓拍并告警，
不删除旧证据，不中断识别或控制。队列满会跳过当前抓拍并告警。

查看最近一次抓拍及同次运行的总控事件（不启动车辆）：

```bash
cd /home/nano/robocup_ws
python2 tools/sign_detector_20260916/inspect_sign_captures.py
python2 tools/sign_detector_20260916/inspect_sign_captures.py --label STRAIGHT
```

可用 `--directory /绝对路径/时间_PID --limit 30` 指定历史记录。
`decision=--` 表示该图像源时间戳没有对应的总控决策事件，不能据此判定总控接受了路牌。
需要关闭抓拍时，在整车启动命令末尾加 `sign_capture:=false`。


## 靠右行驶的路牌候选过滤

`sign_route_gate` 默认启用。方向牌和 P 牌先按检测框中心过滤：640px 原图中
中心 x < 192px 属于明显左侧，不能成为动作候选；192px 及右侧保留。
这保留了前方偏左余量：实拍有效 LEFT 中心约243px、有效P候选约286px，
不能把画面左半幅全部裁掉。红/绿牌继续使用现有信号逻辑。

45px 是普通目标的分界，不再是绝对拒绝门槛。更小目标至少占图高4%、图宽1%，
置信度至少0.90，连续3张不同源图像关联同一目标；普通目标连续2帧。
相邻帧超过0.75秒、类别变化、目标位置/尺寸跳变或低置信度会重置候选。
候选通过后仍经过总控原有投票、路线阶段和动作触发条件。

先过滤明显左侧再选择目标，避免左侧大牌遮蔽前方小牌。
JSON 保留全部模型检测 `detections` 和可接受区域内候选 `route_detections`，
`route_gate` 记录目标ID、帧数、门槛；`range_reason` 可以是
`outside_route_region`、`candidate_tracking`、`low_confidence`、`route_confirmed`。
这是一项按实拍设置的图像区域启发规则，不等于地图定位或车道空间关联。
弯道时合法牌可能横向移动，需要实车抓拍继续校准 left_center_ratio。
暂时恢复旧45px逻辑，可将本模块配置 `sign_route_gate.enabled` 设为 false。

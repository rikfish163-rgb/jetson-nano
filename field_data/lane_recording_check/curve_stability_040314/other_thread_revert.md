# 本对话巡线改动撤回记录

收到用户经巡线故障修复对话转达的“禁止那个对话改循线，撤回”要求后，停止本对话所有巡线算法、参数与相关测试修改。

已撤回，仅涉及本对话引入的内容：

- `src/robot/lane/controller.py`：撤回 Pure Pursuit 普通巡线替换、恢复原右弯锁定函数；同时撤回本对话新增的置信度和跨度门槛。
- `src/robot/lane/README.md`、`src/robot/test/test_topmost_lane.py`、`src/robot/test/test_lane_loss_transition.py`：恢复修改前内容。
- `src/robot/test/test_metric_lane_tracking.py`：删除本对话新建的米制巡线测试。
- `src/robot/test/test_competition_input_gates.py`：移除本对话新加的两项巡线门槛测试；保留动作交接、停车及总控扫描时效测试。
- `src/robot/turn/controller.py`：撤回本对话新加的直行期间白线跟踪逻辑，恢复原右蓝线/零舵策略。独立的直行出口多帧确认仍保留。
- `src/robot/master/controller_node.py`：撤回巡线输入注释的修改；此文件的停车场景集成继续保留。
- 根 README 的巡线门槛说明撤回，明确巡线由修车对话负责。

前四个既有文件已与本轮修改前备份逐字节核对相同。基准来自 `/home/nano/robocup_cleanup_backups/20260929/competition_implementation_before/src_tools.tgz`，本地核对目录为 `/tmp/competition-implementation-baseline`。

此次撤回没有覆盖 `src/robot/common/geometry.py`、`src/robot/obstacle/controller.py`、`src/robot/lane/record_run.py`、`test_lane_recorder.py` 或 `test_vector_collision.py`。本对话先前的障碍保护修复仍在；收到该要求后停止编辑 obstacle，以免与修车对话冲突。停车和离线赛道仿真继续推进，仿真使用当前实际源码并诚实记录失败位置，不通过调换巡线策略制造通过结果。

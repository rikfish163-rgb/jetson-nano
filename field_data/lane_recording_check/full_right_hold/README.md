# 普通右弯保持满右舵修复与验证

## 已确认的问题

旧代码只保持右转方向，锁定期间仍用当前目标点比例舵量更新 `lock['steer']`。最新记录 `20260930_021005_24181` 中，锁定未解除时输出曾从 `-11` 收小到 `-5`；出口候选确认阶段也可能回到零。这与要求的弯中保持 `-22` 不符。

## 修改

- 在现有几何条件确认普通右弯后，命令固定为 `-steering_command_scale_rad`，经现有编码器输出 raw `-22`。
- 锁定期间，目标点偏差缩小、短时 GAP 和出口候选确认阶段均保持满右舵。
- 保留既有出口几何判定、至少 5 个连续有效的新源帧、至少 0.4 秒的确认条件；确认成功后恢复普通目标点纠偏。
- 保留图像超时、持续丢线和雷达等停车条件。停车指令仍可覆盖满右舵保持。
- 有效目标且右弯锁定时，调试选择标记为 `right_curve_full_lock`。

生产文件：`/home/nano/robocup_ws/src/robot/lane/controller.py`；同步更新同目录 README 和相关回归测试。修改前备份位于 `/home/nano/robocup_cleanup_backups/20260930/full_right_hold_021258/`。

## 验证结果

- 修改前的新回归测试复现：刚识别右弯输出 `-12`，目标偏差缩小后输出 `-6`，均未达到要求的 `-22`。
- 修改后针对性测试：`Ran 88 tests ... OK`。包括映射比例为 0.01、0.03、0.1 时均输出满右舵，出口确认、短时丢线和雷达停止覆盖。
- 模块检查：`PASS 452 source/config files assigned; module overrides valid`。
- 历史固定观测回放：13 次有效记录，7550 次普通 LANE/GAP 控制输出；`moving_locked_not_full` 全部为 0，`positive_while_locked` 全部为 0。
- 最新记录：382 次右弯锁定且继续行驶的输出全部为 `-22`，其中 28 次原有较小舵量被修正。保留 1 次原始 `scan_missing_or_stale` 停车。
- 此前 `011909` 记录的错误停车窗口：238 次控制输出，回放中停车次数仍为 0。
- 5 个已知错误左打角案例仍受保护，回放输出均为 `-22`。

具体输出见本目录各记录 JSONL 和 `summary.json`。

## 复现方式

在 Nano 上执行（只离线回放，不驱动车辆）：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=$PWD/src:$PYTHONPATH \
LANE_REPLAY_OUTPUT=$PWD/field_data/lane_recording_check/full_right_hold \
LANE_REPLAY_REQUIRE_FULL_RIGHT=1 \
python2 -u field_data/lane_recording_check/replay_exit_correction.py
```

## 证据边界

回放使用已记录、已补偿到车体坐标系的路径，回放位姿设为零以避免重复补偿；保留原始外部安全停车。它验证控制输出，不模拟改变舵量后的新行车轨迹，也不验证真实前轮角度或新一轮雷达数据。程序重启后加载修改，实际过弯效果尚待跑车验证。

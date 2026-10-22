# 远端仍向右弯时禁止提前解除满右舵

## 可复现问题与修改

取历史 `20260930_021005_24181` 记录中的路径几何（坐标保留四位小数）：

```text
[(0.5601,0.0274),(0.6116,0.0291),(0.7031,0.0264),(0.7680,0.0201),
 (0.8381,0.0092),(0.9066,-0.0055),(1.0064,-0.0341)]
```

整段回归方向为 -7.6017 度，残差约 1.30 cm，近处横偏约 4.65 cm，满足旧出口条件。
但从最远端起、满足至少三点和 20 cm 跨度的最短后缀包含四点，其方向为 -12.8987 度，
仍明显朝右。将这个实际几何连续输入五个不同源帧可以暴露错误出口确认。

新增约束：该远端后缀的回归方向必须不再右偏超过 8 度，才能进行出口确认。
原有整段拟合、近处横偏、残差、至少五个不同源帧及至少 0.4 秒条件继续生效。
未满足出口条件时继续满右舵 raw -22；原有丢线、图像超时和雷达停车仍可覆盖。

修改仅涉及：

- `src/robot/lane/controller.py`
- `src/robot/test/test_lane_right_curve_guard.py`
- `src/robot/lane/README.md`

修改前文件已逐一校验备份到 `/home/nano/robocup_ws/field_data/code_backups/20260930_tail_exit_guard/`。

## 验证

- 新增测试在修改前失败：第一次观察错误累加 `exit_frames=1`，预期为 0。
- 修改后 19 项右弯保护测试通过，包含上述五帧保持 -22 和既有真实出口样例。
- 主线程在 Nano 独立执行 89 项相关测试：`Ran 89 tests in 23.604s / OK`，完整输出见 `tests.log`。
- 模块检查：`PASS 452 source/config files assigned; module overrides valid`。
- 13 次历史固定观测回放，共 7,550 次普通 LANE/GAP 输出：继续行驶且右弯锁存在时，非满右舵次数为 0。
- 5 个已知错误左目标均仍输出 -22。
- 与上一版 `full_right_hold` 汇总相比，仅 `225212` 的保护次数从 0 变为 1；各记录的停车、解锁次数没有变化。
- `011909` 原错误停车窗口 238 次输出仍无停车；`021005` 原始一次雷达安全停车仍保留。

数据见 `summary.json`、`comparison.json` 和各运行 JSONL。

在 Nano 复现（纯离线，无车辆启动）：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src:$PWD/src/robot/test:$PWD/src/ros/camera/test:$PWD/src/ros/lane/test:$PYTHONPATH"
python2 -m unittest test_lane_right_curve_guard test_topmost_lane test_lane_loss_transition test_startup_curve_direction test_module_boundaries test_direct_lane_points.DirectLanePointsTests test_dashed_left_boundary test_lane_transport test_ros_adapter
LANE_REPLAY_OUTPUT="$PWD/field_data/lane_recording_check/tail_exit_guard" LANE_REPLAY_REQUIRE_FULL_RIGHT=1 python2 -u field_data/lane_recording_check/replay_exit_correction.py
```

## 现场证据边界

用户确认现场是“车还在前进时，前轮先回正或减小角度”。最新 `022714` 记录因低磁盘
在等待绿牌时停止，无法提供该现象发生时的同步路径和逐周期底盘数据。现存约每秒一次
的底盘日志曾持续显示 -22，但不足以排除采样之间的短暂减角，也不是实际前轮角度反馈。

本次新增回归是将一帧真实路径几何重复输入不同时间戳，验证该类路径不能被误当作出口；
历史回放固定原有车体坐标观测，不模拟修改命令后的新轨迹。不能据此宣布最新现场问题
已解决。需要重启后的一次完整同步记录验证实车效果。

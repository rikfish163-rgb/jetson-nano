# UTURN 牌缓存与蓝线距离接管修复

## 日志结论

来源：`/home/hetaisheng/.codex/attachments/f4440f68-5c5c-4876-bcdf-436588939df0/Pasted text.txt`。

- 第二个 STRAIGHT 已触发并执行；1790887069.050516 开始独立直行。
- UTURN 在直行期间识别到，置信度 0.81–0.87，却被 `action_sign_ignored` 丢弃；全程没有进入 UTURN。
- 直行进度约 0.699 m 时，蓝线前向距离约 0.613 m。两项数据有异步观测和速度积分误差，不能视为实车厘米级测量。曾据此增加 UTURN 提前接管，用户明确重申直行必须走满 1.25 m 后恢复循线，因此已撤回提前接管。
- 最后持续零速：1790887085.663705 的 `gap_limit_wait_for_lane`；随后车道置信度 0.1146。中间 1790887083.947598 的雷达过期停车约 0.046 s 后恢复。

## 修改

- STRAIGHT 执行期间允许连续两个新帧缓存 UTURN；允许其在 0.75 m 以前出现。重复 STRAIGHT 保持原来的后段确认门槛。
- 下一条蓝线仍须为连续确认、与已消费蓝线相距至少 `marker_rearm_distance` 的横向蓝线。旧线、纵向蓝线、牌子单独出现均不能接管。
- 严格直行流程：缓存 STRAIGHT → 横向蓝线触发 → 独立直行满 1.25 m → 恢复循线。UTURN 只缓存，不能打断本次直行。到达距离后直接交还现有循线控制，不再留在 MANEUVER 等待三帧直道确认。
- 下一条蓝线由恢复循线后的正常状态机触发；不在直行内部直接派发下一动作。
- 绿牌 STARTUP_STRAIGHT 单独执行，仍须走到 1.25 m 才交还循线。
- UTURN 前蓝线停车沿用已有距离公式：蓝线法向距离减去前轴投影；保留停车后 1 s 等待。七段掉头内部序列未修改。
- 避障左右弧线速度独立设为 RAW 26，见相邻 `20261001_bypass_speed26` 备份。

## 验证

- 修复前新增 6 项 UTURN 用例：4 失败、2 通过，复现缓存和接管错误。
- 初次修复后 6 项通过，补充绿牌 1.25 m 用例也通过。随后按用户明确的状态机更正 UTURN 提前接管，并更新对应回归；最终结果以 `strict_straight_verified_tests.txt` 为准。
- 严格直行更正前两项新流程断言都失败，复现 UTURN 提前中断和在直行内部直接派发下一动作的问题，见 `strict_straight_before_tests.txt`。
- 严格直行更正后，37 项针对直行、UTURN 缓存、蓝线、动作锁定和绿牌停车门禁的回归全部通过：`Ran 37 tests ... OK`。
- 初次修复的相关集合执行 71 项：70 通过，1 项历史测试速度断言失败。该项 `test_blue_before_minimum_is_remembered_but_does_not_end_straight` 用旧 RAW 26 断言，实际为 RAW 12；加载修改前四个模块也相同失败，见 `startup_legacy_baseline.txt`。未改变起步速度实现。
- Python2 语法检查 8 个文件通过。模块检查：`PASS 507 source/config files assigned; module overrides valid`。
- 两个真实启动脚本的参数经 `roslaunch --dump-params` 检查，避障速度均为 26；全图普通动作仍为 20，循线专用脚本普通动作仍为 24。
- 8 个受保护源码/配置/脚本哈希未变，包括循线实现和掉头内部序列；修改前备份与差异保存在本目录。
- 未启动 ROS 控制节点，未发底盘命令，未进行实车验证。蓝线进入近处盲区后仍使用命令积分位姿估计，停车精度需现场确认。

## Nano 完整启动命令

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_yolo_vehicle.sh
```

脚本默认纯跑，不录制。已经运行的进程需在原终端 Ctrl+C 后重新启动才能加载新代码。

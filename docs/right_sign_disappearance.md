# 连续右转牌按画面消失解除锁定

第一块 RIGHT 已确认后，保留待执行/执行中动作。新鲜 YOLO 帧的完整候选列表不再含 RIGHT 时，立即解除旧牌锁定并清空投票，允许下一块 RIGHT 按 0.80 置信度、连续 2 帧确认。删除了前进右满舵约 1 秒后强行解除锁定的逻辑。

低于接受阈值但仍有 RIGHT 候选框时不释放。相机断流、重复或过期帧不释放。旧牌持续可见时，动作结束回循线后也不会重新确认同一块牌。下一条指令已确认后，即使它的牌也消失，指令仍保留；第一轮动作完成后交接给正常蓝线触发流程。

这采用候选类别是否可见，没有跨帧物体跟踪；单帧完全漏检也视为消失。如果两块 RIGHT 同时连续出现在画面中，候选类别不会消失，不能仅凭此规则区分两块牌。

## 不动车验证

Nano 的 ROS Melodic / Python 2 环境运行以下 8 个测试模块，共 83 项通过：

`test_right_final_sign_rearm`、`test_action_sign_lock`、`test_route_phase_handoff`、`test_sign_range`、`test_all_sign_threshold`、`test_route_obstacle_regression`、`test_right_sign_disappearance`、`test_sign_callback_scheduling`。

新增消失回归在修复前有 6 项断言失败、2 项缺少新接口的错误；修复后通过。模块目录检查通过：584 个源文件/配置文件均有归属，模块覆盖配置有效。

使用手推记录 `manual-right-sign-20261008_133023/detections.jsonl` 中第一条 RIGHT 起的片段，在纯 Controller 对象上模拟第一轮处于 MANEUVER（没有 ROS 控制输出），得到：

| 源时间戳 | 结果 |
| --- | --- |
| 1791437503.306986 | 第一块 RIGHT 确认：stored_pending |
| 1791437509.9741368 | 第一块消失：right_sign_disappeared |
| 1791437518.6406374 | 下一块 RIGHT 确认：stored_next_direction，置信度 0.9308 |

调用动作结束交接后，pending 保留为 RIGHT。这证明新逻辑能够接收这段真实检测序列，不代表复现了当时的真实蓝线触发/车辆姿态，也未进行自动行驶验证。

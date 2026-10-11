# 指定功能恢复（2026-10-09）

此记录描述上一轮局部恢复，已被后续用户确认的严格 17:00 状态机恢复取代；当前行为见 `state_machine_restore_1700_20261009.md`。

用户限定只恢复路牌防误识别、定时避障、正式 RIGHT，其余使用本轮开始时工作区代码。
RIGHT 行为由用户明确确认：倒车 1.1 秒、前进右满舵 4.7 秒、倒车 2.5 秒、回正停车 2 秒、立即交回当前普通循线。
没有将整套历史目录覆盖回来，也没有恢复历史右转后持续对齐代码。

- 路牌先选检测框面积最大者，再应用现有置信度阈值；最大候选不足阈值不改选小牌。
- LEFT/RIGHT/STRAIGHT/UTURN/PARKING 检测框高度至少为图像高度的 0.125（360 高时为 45 像素），RED/GREEN 不加高度限制；保留完整候选诊断。
- 避障距离沿用后轴参考 0.85 米，横向走廊和连续雷达帧确认保留。新增显式角度参数 `timed_bypass_front_half_angle_deg=20`，此值是本轮对“前方一定角度”的默认选择，不声称是已考证的 17:00 历史角度。
- 避障前现有 0.4 秒零速打舵等待保留；运动阶段 raw 26，左转 1.5 秒后切右转，右转至少 4 秒且需车道确认，上限 6 秒；未确认时停车等待。
- 左转时间达到后若右转路径被原有雷达检查否决，停车等待，不再额外延长左转。保留当前其他雷达检查及视觉/急停保护。
- 右转速度/舵量为 -30/0、30/-22、-30/0、0/0。停车期间遭外部保护停止会锁定 FAULT。
- LEFT 仍为 2.2 秒前进、3.0 秒左转、1.0 秒倒车；未修改掉头、停车、普通循线、启动脚本。

## 验证

全部行为测试在 SSH Nano 上的纯 Python Controller/模拟图像中运行，没有发布底盘命令或启动实车节点。
修改前新增用例复现四项失败：高度门槛缺失、小牌替代低分大牌、侧前方目标触发、避障时长不符。
`roslaunch --dump-params robocup_competition full.launch` 确认实际加载上述参数，仅展开配置。
模块检查通过：566 source/config files assigned，module overrides valid。
最终针对性回归 59 项全部通过，覆盖最大候选、高度边界、红牌高度豁免、前方扇区、左转 1.5 秒切换、右转 4 秒下限及 6 秒上限、完整 RIGHT 序列、停车期间保护、模块参数归属。
本轮文件 `git diff --check` 通过（Nano 未安装 git-lfs，因此仅此次只读 diff 命令禁用 LFS filter）。

扩展回归 90 项，85 通过、5 失败。以下五项均在修改前代码副本 `/tmp/requested-restore-baseline` 上复现，未为通过测试修改其他现有行为：

- `test_gap_fault_never_restarts`
- `test_third_stage_gap_never_resumes`
- `test_safety_stop_pauses_timer_and_resumes_remaining_right_time`
- `test_six_second_cap_keeps_existing_partial_lane_handoff`
- `test_completed_entity_is_not_triggered_again_after_protection_reopens`

前两项仍期待 0.4 秒控制间隔触发故障，而现有实现阈值是 2 秒；后三项涉及既有绕障车道确认和已绕过目标记忆。
保留本轮开始前文件于 `backups/requested_restore_20261010/`。该备份是局部文件备份，不是整车快照。
日志归档在该备份目录下。未提交、推送或重启运行程序；实车轨迹尚未验证。

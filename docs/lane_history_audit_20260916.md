# 巡线历史核对（2026-09-16）

## 本次修改与验证

- 仅将 `src/robot/config/competition.yaml` 的 `lane_min_confidence` 从 0.25 恢复为 0.35。
- SSH 核对 Nano 文件为 0.35；`roslaunch --dump-params robocup_competition full.launch` 解析结果也是 0.35，未启动节点。
- Nano 离线控制器检查：0.25、0.338 被拒绝，0.35、0.385 通过。未发布运动命令。

## 比较依据

Nano 上的备份：

- `/home/nano/robocup_cleanup_backups/20260911-production/source_before.tar`
- `/home/nano/robocup_cleanup_backups/20260911-stutter/before.tar`
- `/home/nano/refactor_backups/20260914/source-before.tar.gz`
- `/home/nano/refactor_backups/20260915-structure/before.tar.gz`

另核对 Codex 任务“排查绿牌延迟循线摇摆停车”和“重构项目架构与代码”的历史记录。当前仓库没有 Git 提交，不能以 Git 重建完整历史。

## 已确认的差异

1. 当前 `camera_yihan_web.py` 与 9 月 15 日结构调整前备份逐字相同。相对 9 月 11 日备份，新增真实边界导出、消息字段和重复发布节点检查；白线提取、置信度计算、中心路径筛选未变化。9 月 14 日到当前的视觉差异是给边界导出补上 `window_index`。
2. 当前普通巡线模块相对 9 月 15 日 `controls/lane.py` 只修改了导入路径，普通巡线算法未变化。
3. `full.launch` 和 `stack.launch` 的转向尺度默认值从空覆盖改为 0.1；历史任务证实曾按用户要求试 0.2，随后按“还是得0.1”恢复。当前用户启动命令显式传 0.1，因此默认值改变不能解释这条命令的新差异。
4. `full.launch` 的 `debug_view` 默认 false 改为 true；路牌节点增加模型、裁剪模式和调试图设置。新增负载可能影响时延，但没有对照运行证明它导致本次丢线。
5. 直行动作接管循线的逻辑改变：历史任务按用户要求增加至少 1.5 米后接管，再改为 1.6 米。该改动会改变进入循线时的位置，未证明它是此次失效的唯一原因。

## 默认参数与本次命令不能混为一谈

9 月 15 日备份的基础配置：lookahead=0.30、speed_raw.lane=20、lane_min_confidence=0.35；模块 lane.yaml 为空覆盖。当前基础配置仍为这些值。

本次用户命令显式覆盖 lookahead=0.55、lane_speed_raw=26、steering_command_scale_rad=0.1、lane_hz=12、lane_window_height=40、lane_min_span=0.15。9 月 15 日晚的历史回复已经提供过同样的巡线参数。不能把基础默认值当成用户此前成功实跑的参数，尚未锁定最后一次成功内道巡线记录。

## 本次失败记录能证明什么

用户附件 `6350d487-3b23-46cf-bcb1-db1ed3c8fe6b/pasted-text.txt` 中，GAP 同时包括低于 0.35 的非空路径和 no_trusted_centers 空路径；所列 GAP 样本的数据年龄均小于 0.5 秒。存在右满舵后变为左满舵、丢线保持转向的行为，不能据此证明实际轮角或具体选错哪条白线。

普通中心巡线直接将计算角送入 `steer / 0.1 * 22` 编码；左边界跟随先做物理角到命令角的换算。这一差异在旧备份中已存在，不应说成此次刚引入的回归。

剩余限制：没有最后一次成功跑内道的精确命令和同步视觉帧，尚不能把退化归因于单个参数或一次修改。此次未回滚其他算法、模型或直行逻辑。

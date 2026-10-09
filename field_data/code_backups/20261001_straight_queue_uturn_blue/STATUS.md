# 连续直行与 UTURN 蓝线停车修复

## 状态

修改已保存到 Nano `/home/nano/robocup_ws`；本地工作目录为该目录的 SSHFS 挂载。
没有启动车辆节点或发送底盘命令。

本次输入：`/home/hetaisheng/.codex/attachments/7a88abd4-4377-469b-900c-0bd19bf33750/Pasted text.txt`。
完整 ROS 日志另位于 Nano `/home/nano/.ros/log/0d803b62-bdce-11f1-a256-380025ec2b29/`。

## 日志证据

| 时间戳 | 事实 |
| --- | --- |
| 1790882869.937 | 第一段路口 STRAIGHT 开始，独立直行 1.25 m |
| 1790882871.787 | STRAIGHT 置信度约 .842，decision=action_sign_ignored |
| 1790882877.977 | 已估算直行 1.101 m；STRAIGHT 置信度 .93055，仍 action_sign_ignored |
| 1790882879.052 | 第一段完成并交回 LANE，pending=None |
| 1790882886.278 | 主控以 gap_limit_wait_for_lane 输出停车 |
| 1790882886.576 | 路径置信度 .1146，车道/雷达/前摄流仍新鲜；持续停车原因是有效路径缺失 |
| 1790882918.308 | UTURN 入口仍走固定 ALIGN 阶段 |
| 1790882919.349 | 定时对齐结束，执行固定 .36 m ADVANCE |
| 1790882922.005 | 固定距离完成并停车，最后一次有效入口蓝线观测约 2.89 秒前 |
| 1790882941.930 | 七段与出口修正后恢复 LANE；日志中的转角为命令估算 |

路牌标签不能单独证明两条识别属于两块不同实体牌；根据用户描述与直行末段的重复识别，新增下一块直行牌的缓存衔接。可解析的原始状态与事件摘要见 `run_evidence.json`。

UTURN 原停车状态的命令模型按蓝线法向计算，前轴距目标平面约 .004 m；这不证明实车停准，现场反馈不能被估算坐标替代。原固定距离策略本身也不保证不同入口距离下前轮在线。

## 修改内容

1. 第一段 STRAIGHT 的后 40% 距离允许确认下一块 STRAIGHT：源帧新鲜且不同、连续两票，保留到当前动作结束。早期重复牌、单票、间断票不会排入队列；当前动作继续完成 1.25 m。其他正在执行的动作继续锁定。
2. 排队的直行牌只能与距离上一条已消费蓝线至少既有 marker_rearm_distance 的新横向长蓝线配对，仍需三张不同图像确认。单帧、旧线、纵向蓝线不触发。
3. 当前直行达到 1.25 m 时，若下一条蓝线已经确认且仍新鲜，直接交给下一次蓝线接近；若先交给白线循线，则将队列提升为 pending=STRAIGHT，等待它自己的蓝线。
4. UTURN 入口使用 STOP_LINE，持续更新与已锁定蓝线匹配的观测。停车参考为前轴中心：蓝线法向距离减去 wheelbase*cos(航向误差)，不使用车头保险杠和固定 .36 m 行程。
5. UTURN 车身对齐误差超过既有 8 度容差时，不从蓝线停车直接开始掉头，报 uturn_blue_heading_not_aligned；相机观测流过期仍停车。
6. 控制事件日志增加 blue_approach 和 next_direction，便于把实际零速决定与阶段对齐。

保持入口速度 RAW 20、UTURN 七段 RAW ±26 与舵 RAW ±22、七段时长、普通循线控制和全部启动参数。本次没有调速度、max_steer、相机标定或丢线保护时限。

## 验证

- 修改前七项新回归中四项失败，见 `regression_before.txt`。
- 修复阶段 61 项相关检查通过，见 `regression_after.txt`。
- 最后调整后 31 项针对衔接、蓝线事件、右蓝线直行及 ROS 事件日志的检查全部通过，无跳过；其中九项是本次新增回归，见 `handoff_final.txt`。
- Python 2 语法检查七个文件通过。
- `tools/module_workspace.py check`：502 个源代码/配置文件归属检查通过。
- 八个受保护文件散列一致，见 `protected_check.json`。

上述测试均在 Nano 上执行，未驱动车辆。它们证明软件行为符合修复要求，不是完整一圈的实车验收。持续没有可信车道且没有已确认的下一路口时，现有丢线停车仍会生效；未通过继续盲行掩盖这个问题。

## 9 月 20 日对照

找到 17 个当日历史归档，部分只备份某个模块，见 `historical_20260920.json`。
其中 03:45 的整套源代码归档已读：旧版也存在 action_sign_ignored 与固定蓝线前进流程。其基础配置为 lane RAW 20、lookahead .3、max_steer .46275、straight RAW 26；这些是该归档的配置值，不代表用户成功场次实际启动参数。
当前整车脚本的有效参数包含 lane RAW 30、lookahead 1.0、max_steer .2、steering_command_scale_rad .03、straight RAW 20。相机和动作逻辑后续也有修改，所以只还原单个历史文件不能代表恢复了整套成功配置。
现有记录尚不能把某一份归档认定为用户确认跑完一圈的那次版本。对照来源见 `historical_snapshot_comparison.json`；没有将不明场次的历史包直接覆盖当前文件。

## 纯跑命令（Nano）

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_yolo_vehicle.sh
```

默认不录制。重新启动后加载此次修改。改前文件见 `before/`；此次源代码差异见 `changes.patch`。

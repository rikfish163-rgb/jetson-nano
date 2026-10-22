# 最靠上目标点横向比例控制

2026-09-29：修改 Nano 实际工作区的普通循线控制函数，应用于所有后续普通循线帧。

选择 x_forward 最大的前方目标路径点，即鸟瞰图最靠上的路径点。
车体中线为 BEV u=240 px；e=(240-u)/400 米，左正右负。
raw=round(22*clamp(e/0.30,-1,1))。0.30m 是本次采用的初始比例参数，
可通过 competition.yaml 的 lane_lateral_full_scale_m 调整，并非实车标定结论。
普通循线不再使用前瞻交点、PurePursuit 曲率、路径最低置信度或至少两点门槛，
右转退出遗留的左边线参考不能覆盖当前目标路径。
有一个前方路径点也可选点；相机过期和无路径处理仍生效。
专门的路口、停车、倒库动作不属于这条普通循线控制律。

速度保持20，左单边路径偏移保持右侧40cm。
普通循线灵敏度由30cm满舵偏差决定，旧lookahead不再参与选点；
steering_command_scale_rad只负责将这一比例指令编码给底盘。

## 无运动验证

- 72项针对性测试通过，模块归属检查450项通过。
- 隔离ROS：相机、路径观测、主控选点、录制贯通；live=false，actuator_publishers=[]。
- 对 field_data/lane_runs 下本次快照的全部5段录制逐帧重跑图像处理和普通循线控制。
- 2193个front文件：2076张可读，117张不可读。
- 1024帧生成目标路径，均选中最远前方点且最终raw与横向偏移比例公式一致，0处不一致。
- 1052帧未生成可用目标路径，单独统计，未计为选点成功。
- 每段使用其录制的K/D/H；没有重新执行录制中的路口/停车状态机，未验证实车运动。

结果：topmost_replay.json；逐帧点、选择及指令：topmost_frames.jsonl。
复现脚本：replay_topmost.py。
备份：/home/nano/robocup_cleanup_backups/20260929/before_topmost_control/。

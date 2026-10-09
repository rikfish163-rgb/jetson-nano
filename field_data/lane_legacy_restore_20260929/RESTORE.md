# 旧版循线恢复记录

恢复来源：

- 视觉：`/home/nano/robocup_cleanup_backups/20260911/retired/competition_left_backup.rQbRa2/camera_yihan_web.py`。
- 路径控制：直接导入 `/home/nano/robocup_ws/src/ros/lane/src/vehicle_control/pure_pursuit.py`。
- 普通循线输出：沿用旧 `lane_main_adapter.py` 的 `angle / 0.1 * 22` 增益与 raw ±22 限幅。
- 前瞻保留用户最后指定的 0.45 米，完整脚本速度为 raw 20。

41 个原视觉函数与备份的 AST 一致。路径选择只附加诊断字段，追踪时间接受源帧时间戳；
标定 H 保留当前配置。取最新图像、同帧观测与边线发布、运行标定与主函数沿用当前 ROS 接口。
普通循线已移除新增的拟合转向、起步方向锁、拟合降速和丢线恢复滤波。
路口与停车的独立动作控制仍由完整任务主控执行。

验证结果：

- 旧原文件生成的 11 次图像观测结果与恢复后一致（模式、置信度、目标路径）。
- 31 项旧版一致性、Pure Pursuit、相机接口、启动接管和丢线测试通过。
- 55 项 ROS 适配、任务蓝线序列、动作参数与边线接口测试通过。
- `module_workspace.py check`：443 个源代码/配置文件归属和覆盖项有效。
- 独立 ROS master 下启动实际相机节点与禁用输出的主控，收到标定、路径观测和状态；
  `/control/cmd` 与 `/ackermann_cmd` 没有发布者。见 `shadow_result.json` 和相邻日志。
- 完整脚本只解析参数，确认绿牌、reverse_plan 倒库、雷达、定时 U-turn、0.45 米前瞻。
- Nano 源文件与挂载文件 SHA256 相同；未执行实车运动。

限制：六张分别冷启动回放的保存图像中，旧算法在 `recorded_curve_corridor.png`、
`turn_01.png`、`turn_02.png` 输出空路径。因此一致性通过不能解释为当前弯道已经实车通过。
此前针对后加边线关联、弯道记忆和新中心线拟合的测试不是此旧算法的验收标准。

改动前可恢复备份：
`/home/nano/robocup_cleanup_backups/20260929/lane_before_legacy_restore_184741`。

恢复后 SHA256：

| 文件 | SHA256 |
| --- | --- |
| camera_yihan_web.py | 0413466a2683017ec9e5768989232446f50b5f6cf30983a9f85c6e816ae539a3 |
| robot/lane/controller.py | d06e17a4662a7654ce8a61283373cab551c9012b9f3fa8d3dc2f9139bcfc1f79 |
| vehicle_control/pure_pursuit.py | b961b127ccbe189bd231e047923796483d5259878e0e128b5cd0a673b89cdbba |

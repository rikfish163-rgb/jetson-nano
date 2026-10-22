# 040314 录制：压外线与停顿修复

## 现场证据

- 输入：用户上传 `Pasted text.txt`；Nano 录制 `field_data/lane_runs/20260930_040314_16505`，以及 ROS 日志 `c784d7b4-bc40-11f1-acc0-380025ec2b29`。
- target 数值记录共 1770 行；行驶段反复出现 `scan_missing_or_stale`，最终出现 `lane_unreliable`。
- 录制器遇到没有 `path` 的无效选择报文后异常退出：`write_error: 'path'`。终端继续有输出，不能把录制结束当作车辆停止。
- 源帧 1790712275.7770731 已有向右弯的曲率，但最远点横偏仅 +0.002449 m。记录命令 raw=+1，不能根据这一帧的最远点判断道路仍直。

## 本次修改

1. 在原最远点横偏控制上叠加当前观测路径的曲率前馈；一维卡尔曼滤波只对不同源帧更新，源帧间隔过长重置。没有预测出未观测的车道点。
2. 整车脚本显式开启 `lane_curvature_preview:=true`。弯道、右弯锁及其短 GAP 的速度上限 raw=12，直线 raw=20。新模式拒绝低置信度、单点和过短路径；已确认右弯遇到不可靠图像停车时保持舵角。
3. 原始雷达点的车体碰撞检查改为数组运算，每次扫掠只转换一次。保护范围、所有原始点、覆盖检查和超时阈值保留。
4. 录制器兼容无效选择报文。全部数值事件继续写入；只对有路径的不同源帧渲染目标图。
5. 已按用户要求撤回另一个对话的巡线替换，见 `other_thread_revert.md`。本次自己的改动在其撤回后实施。

## Nano 不动车验证

| 检查 | 结果 | 文件 |
| --- | --- | --- |
| 录制器故障复现 | 修改前无效选择触发 KeyError | tests_before.log |
| 真实入弯路径复现 | 修改前 raw=+1，预判测试失败 | preview_before.log |
| 避障耗时对比，3000 条射线、30 次相同命令 | 平均 79.33 ms → 33.26 ms；输出均为 [20,-0.03] | benchmark.json / benchmark.py |
| 相关测试 | 61 项通过；最后新增两项滤波边界检查后，11 项预判测试通过（合计 63 个不同相关检查） | tests_final.log / preview_final.log |
| 配置、文件归属 | PASS，463 个源码/配置文件 | module_check.log |
| 启动入口 | bash -n 通过；ROS dump 参数验证预判 true、弯道速度 12，无节点启动 | launch_params.yaml |
| 原录制本地路径回放 | 395 条命令；上述入弯帧 raw=+1 → -10，速度 20 → 12 | replay_summary.json / replay.jsonl |
| 目标图渲染负担 | 此录制由 1770 张降至最多 318 张，数值事件保留 | replay_summary.json |

额外运行的旧 `test_lane_command_units` 有一项期望直接物理角度 0.1974 rad，原最远点控制输出命令角度 0.1 rad；该失败在切换到修改前备份后同样复现，见 `legacy_units_baseline.log`。未修改该历史测试制造通过结果。

## 空间与变更审查

清理五次较早录制的 JPG，保留全部数值日志和 summary；最近 `025754`、`040314` 两次完整录制保留。清理后可用空间约 804 MiB，详见 `cleanup.json`。

修改前源码备份位于 `field_data/code_backups/20260930_curve_stability/src/robot`。本次差异见 `changes.diff`；新文件含 `src/robot/lane/preview.py` 和三份回归测试。仓库既有大量暂存删除和未追踪的重组源码，未暂存或提交任何变更。

## 实车验证命令（Nano 端）

```bash
bash /home/nano/robocup_ws/tools/sign_detector_20260916/run_yolo_vehicle.sh \
  wait_green:=true parking_mode:=reverse_plan parking_slot:=AUTO \
  start_rear:=true lidar_enabled:=true \
  steering_command_scale_rad:=0.03 startup_steering_raw:=0 \
  lane_speed_raw:=20 action_speed_raw:=20 straight_speed_raw:=20 \
  lane_curvature_preview:=true lane_curve_speed_raw:=12 record_lane:=true
```

本次只验证了相同图像路径的控制输出与离线计算耗时，没有重建真实车体轨迹，也没有证明整车运行下雷达超时已经消失。实际车速、物理舵角和转弯半径没有编码器/舵角测量；是否不再压线仍需下一次实车记录确认。

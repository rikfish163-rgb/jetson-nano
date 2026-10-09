# 每帧循线记录

车辆启动脚本已默认传 `record_lane:=true`。不启动车辆的验证由
`verify_recording.py` 在独立 localhost ROS master 完成。

实车输出：`/home/nano/robocup_ws/field_data/lane_runs/YYYYmmdd_HHMMSS_PID/`

- `源时间戳_front.jpg`：视觉节点实际处理的原始图像。
- `源时间戳_bev.jpg`：同源帧的鸟瞰边界和路径图。
- `源时间戳_控制时间戳_target.jpg`：该次控制计算的路径点、前瞻线、交点、目标点和输出舵量。
  这是控制时刻后轴坐标下的独立示意图，未把运动补偿后的点错误叠加到过去的原始图像。
- `target.jsonl`：每个收到的控制周期，保存实际选点数据与最终编码指令；无新选点时 selection 为 null。
- `observation.jsonl`：每个收到的视觉观测，含帧序号、原始路径、置信度和诊断。
- `calibration.jsonl`：运行时相机标定与源码哈希。
- `chassis.jsonl`：底盘串口输出状态。该状态不含真实前轮角度。
- `summary.json`：录制数量、队列丢弃数量、图像缺帧数量及是否因容量/错误停止。

“逐帧”指视觉节点处理并发布的帧（配置 12 Hz），不是 USB 相机全部曝光帧。
控制器每次计算都会记录，因此一张源图像可能有多张 target 图。
时间戳统一按 ROS 浮点秒转纳秒整数命名，用文件名前缀关联。
磁盘写入和 JPEG 编码在独立录制节点工作线程完成。限制为单次约 512 MiB，
保留至少 300 MiB 系统空间；超限报警并停止录制，已录数据保留，不停止车辆。
队列溢出会计数并报警；最终须检查 summary，不能把有缺帧的记录说成完整。

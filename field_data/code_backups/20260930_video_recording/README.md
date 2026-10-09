# 同步录制：前摄原画、BEV、控制状态

此次只修改录制与离线导出，不调整循线、避障或底盘控制。

## Nano 上的运行命令（会启动车辆）

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_lane_green_obstacle.sh \
  record_lane:=true record_max_mb:=160 \
  record_duration_s:=180 record_start_on_motion:=true
```

前摄与 BEV 图像从第一条非零速度控制命令开始保存，最长 180 秒、160 MiB，磁盘保留 300 MiB。时间结束只停止记录，不停止车辆；试跑结束仍需 Ctrl+C。绿牌等待阶段保留数值日志与标定信息。

输出目录：`field_data/lane_runs/<时间_PID>/`。图像文件名是摄像头源时间戳；target、observation、status、chassis JSONL 保存选点、视觉观测、避障阶段和底盘输出。BEV 是车载相机变换后的鸟瞰画面，不能代替从赛道上方拍摄整辆车的实拍视频。实拍可用固定在上方的手机拍摄，并以绿牌放行或车辆起步对齐时间。

## 停车后导出（不会启动车辆）

```bash
cd /home/nano/robocup_ws
python3 tools/export_lane_videos.py field_data/lane_runs --latest
```

输出 `videos/front.mp4`、`bev.mp4`、`comparison.mp4`（两路拼接及控制字幕）与 `manifest.json`。两路仅使用同源时间戳配对的帧；丢帧数量、长间隔见 manifest。Nano 旧版 FFmpeg 的播放时间戳取整到 40 ms，原始精确源时间保存在 JPEG 文件名和日志中；间隔缺帧时保留上一帧，不代表车辆真实停顿。字幕显示图像源时刻之前最近的控制记录，视觉处理与指令产生之间的延迟需结合 JSONL 分析。

## 修改与验证

- `before/` 保留本次修改前的三个已有文件；`changes.diff` 保存差异。
- 录制回归测试先复现运动后开始、限时结束的缺失，再修改；Nano 上 5 项测试通过。
- 在 Nano 使用 3 对合成 JPEG（含 0.6 秒间隔）验证 FFmpeg 导出，不启动车辆。
- 清理两个旧 run 的 JPEG 共 172,420,948 字节，保留其数值日志，保留较新的录制；清理记录在 `field_data/lane_recording_check/video_space_cleanup_20260930.json`。
- 本次尚未获得新的实车录制；合成片段仅验证导出能力。

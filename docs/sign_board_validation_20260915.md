# 路牌紧裁剪验证（2026-09-15）

## 本次改动

模型权重不变，使用 `signs_20260914` 与补充集训练出的八分类模型。
`board_roi.py` 将红、绿、蓝分开寻找候选，蓝色候选要求内部浅色字符；
按有效候选面积排序，并只加约 10% 边缘送入模型，减少地板、手臂和锥桶底座的干扰。
面积排序不是距离测量，不能保证选择物理距离最近的路牌。

正式节点支持 `sign_roi_mode:=board`、模型路径和标签文件。
八分类输出明确保留 BACKGROUND 分数，背景胜出时路牌 label 为空，不发布停车类别。
用户确认预览效果后，full.launch 和 stack.launch 已默认使用本次八分类模型、
对应 labels.json 和 board 紧裁剪。旧模型文件保留，不再是这两个入口的默认模型。
运动控制、绿牌启动、蓝线触发和转向逻辑未改动。

## 已验证

4090 从完整原图重新提取候选，再用固定模型回放 2000 张 val 和 1000 张 test，阈值 0.8。

| test 类别 | 原保存 ROI 正确且通过阈值 | 新完整原图裁剪正确且通过阈值 |
|---|---:|---:|
| right | 135/150 | 150/150 |
| park | 104/150 | 149/150 |
| background 高置信度误报 | 5/160 | 0/160 |

新 test 的其余五种路牌全部正确且通过阈值；park 剩余一张低于阈值。
整个 val+test 中没有高置信度错误类别。val 有 2 张 red、1 张 uturn 低于阈值。
480 张背景通过无候选、低置信度或 background 类被拒绝。

**裁剪规则使用这批数据调过两轮，因此这不是新的独立测试成绩，不能当成实车准确率。**
20 Hz 连拍也存在相邻帧高度相似的问题。需要新的摆放位置、距离、光照和多路牌场景验证。

Nano Python2 / OpenCV ONNX 复核 46 张，包括低分、无候选和各类别样本：
46/46 的最终接受/拒绝及类别与 4090 一致。
裁剪中位耗时 12.62 ms；完整推理中位耗时 693.46 ms（离线单独运行）。
摄像头、网页与循线同时运行时还可能更慢，尚未完成行驶中的延迟测试。

7 项候选裁剪测试、2 项分类输出测试、8 项正式节点测试、4 项旧裁剪测试、
11 项旧识别测试和 1 项截图保存测试通过。
正式启动参数已用 roslaunch --nodes 解析，未启动执行器。

生产裁剪与 4090 回放裁剪的 Python AST 相同（忽略模块说明文字）。
本地完整回放报告：`/home/hetaisheng/robodata/runs/20260915_10000/board_replay_v2.json`。
Nano 复核结果：`field_data/board_validation_20260915/nano_results.json`。

## 当前只读预览

只运行前摄、查看器和 `sign_board_preview`；识别结果在隔离话题 `/competition/sign_preview`。
预览使用私有配置，避免覆盖正式控制配置。没有控制器或执行器节点。

- 带框及结果：http://localhost:8080/stream/sign_board_frame
- 实际模型输入：http://localhost:8080/stream/sign_board_crop
- 前摄：http://localhost:8080/stream/front_camera

实景抽查时画面是锥桶及倒放的路牌，结果为 BACKGROUND，label 为空。
这只能说明这一帧未误触发，不能证明正常竖立的 parking/right 已通过新场景验证。

## 实车命令（由用户在 Nano 运行）

以下沿用此前用户给出的行驶参数，仅加入新模型与裁剪参数。先结束当前只读节点以释放前摄。
运行后车辆会按原有绿牌等待逻辑开始动作。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
rosnode kill /front/sign_capture_camera /sign_board_preview /competition_viewer
roslaunch robocup_competition full.launch \
  live:=true enabled:=true start_actuators:=true \
  start_rear:=false lidar_enabled:=false parking_enabled:=true wait_green:=true \
  lane_hz:=12 lane_window_height:=40 lane_min_span:=0.15 \
  lane_speed_raw:=26 action_speed_raw:=28 \
  lookahead:=0.55 action_lookahead:=0.30 steering_command_scale_rad:=0.1 \
  sign_ttl:=0 blue_default_straight:=true marker_trigger_x:=0.36 \
  intersection_wait_s:=1.0 \
  left_turn_entry:=0.12 left_turn_radius:=0.65 left_turn_exit:=0.30 \
  right_turn_full_lock:=true right_exit_on_blue:=true right_lock_min_angle_deg:=30 \
  right_turn_entry:=0.0 right_reverse_entry_m:=0.25 \
  right_turn_radius:=0.55 right_turn_exit:=0.25 \
  sign_model:=/home/nano/robodata/models/20260915_10000/resnet18_candidate.onnx \
  sign_labels:=/home/nano/robodata/models/20260915_10000/labels.json \
  sign_roi_mode:=board
```

正式节点仍保存每次接受路牌时的原图、裁剪和分数，目录由启动日志中的
`Sign inference captures` 指明。先观察竖立的 right/park 在新距离和多牌场景下的
识别框、结果与延迟，再判断是否需要补图或重新训练。

# 2026-09-16 视觉延迟优化

已写入 Nano 工作区。模型仍为 YOLOv5s 640 TensorRT FP16；分类阈值、框选择、运动状态机未改。

## 修改

- `src/robot/signs/yolo_detector.py`：使用 OpenCV 原生 RGB/CHW 浮点转换；先按 objectness 筛掉不可能达到最终 0.5 阈值的候选，再解码坐标与类别。保留候选原顺序及原 NMS；完整 `predict_raw()` 默认输出仍兼容离线工具。
- `src/ros/camera/scripts/camera_yihan_web.py`：连通域标签重建由 NumPy 高级索引改为 `np.take`；标签由 connectedComponents 生成，均在查表范围内。
- `src/ros/camera/scripts/ros_flask_viewer.py`：只订阅 HTTP 正在请求的图像话题，停止访问约 3–4 秒后释放订阅；状态查询不激活图像。保留源时间戳与过期画面检查。
- `src/robot/signs/sign_node.py`：无人订阅的标框预览、裁剪预览不再绘制和发布。识别帧留存逻辑保留。

## 验证

Nano 同一批 20 张历史实拍图片对照：

| 项目 | 修改前中位数 | 修改后中位数 |
|---|---:|---:|
| 路牌完整处理 | 190.9 ms | 86.6 ms |
| 白线掩膜处理 | 36.4 ms | 22.3 ms |

20 张检测类别、框位置一致，置信度差异均小于 0.0001；白线掩膜逐像素相同。
原始结果见工作区 `field_data/vision_latency_20260916.json` 和 `field_data/vision_lane_latency_20260916.json`。

前摄实时测试开启路牌预览，设置 lane_hz=12、sign_hz=5：

| 数据 | 实际频率 | 拍摄到订阅端收到结果，中位数 / P95 |
|---|---:|---:|
| 摄像头原图 | 15.00 Hz | 7.1 / 11.8 ms |
| 循线 | 12.00 Hz | 80.2 / 110.6 ms |
| 路牌 | 5.00 Hz | 152.5 / 159.0 ms |

预览首个请求等首帧返回 503，后续 35 次均返回 200/JPEG；查看路牌时仅订阅该图像及状态，停止访问后仅保留状态订阅。
原始结果：`field_data/vision_latency_live_final_20260916.json`。

测试使用 stack.launch，live=false、enabled=false、start_controller=false、start_actuators=false；未启动车辆。当前静止画面耗时不能代替整条赛道、多路预览和停车阶段的运行性能。

91 项测试通过：test_yolo_detector、test_yolo_sign_node、test_sign_capture、test_image_scheduling、test_competition_viewer、test_lane_transport、test_lane_center_geometry、test_lane_nearest_boundaries。

## 实车启动（由用户在 Nano 执行）

先结束旧的 roslaunch。脚本自带工作目录、ROS 环境和完整行驶参数：

```bash
bash /home/nano/robocup_ws/tools/sign_detector_20260916/run_yolo_vehicle.sh \
  lidar_enabled:=false \
  parking_entry_speed_raw:=12 \
  blue_default_straight:=false \
  sign_hz:=5
```

sign_hz 的配置默认仍是 3；上面的显式参数使用已测的 5 Hz。车速和转向比例沿用脚本设置，启动直行距离仍为 1.25 m。

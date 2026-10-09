# YOLOv5s Nano 部署

## 模型与接入

- 使用 20260916_detector_v1/yolo_fast/weights/best.pt 导出，七类顺序为 red、green、straight、left、right、uturn、park。
- Nano 目标目录：/home/nano/robodata/models/20260916_yolov5s。
- 输入为整幅相机图像，经保持比例缩放和灰色补边到 640×640；RGB、除以 255。取消原颜色 ROI 对画面位置的限制。
- TensorRT 7.1 本机生成 FP16 引擎，输入/输出绑定仍为 FP32。GPU 输出三层原始检测头，CPU 按各 anchor 尺寸解码并去重。Python 2 ROS 通过现有 TensorRT 和 CUDA runtime 推理，无需安装 PyTorch/PyCUDA。
- 使用验证集选出的权重；没有在测试集上调阈值。候选检测阈值 0.5，控制器接受阈值沿用 0.8；停车仍需原有连续帧确认。
- 多牌时从达到接受阈值的检测中优先选择画面面积最大的牌。仅为接近程度的启发式，不是距离测量，不能保证任何多牌布局都正确。
- 正式识别继续发布 /competition/sign，保留原始图像时间戳、类别、置信度，并增加全部 detections。状态机、直行距离和转向逻辑没有在本次部署中改动。
- full.launch 和 stack.launch 增加 sign_backend 参数，正式默认 yolo_trt；旧分类器可指定 sign_backend:=classifier 并传原 ONNX 路径。

## 运行

需要车动时，由用户在 Nano 上执行：

```bash
bash /home/nano/robocup_ws/tools/sign_detector_20260916/run_yolo_vehicle.sh
```

脚本包含完整 source、ROS 环境和此前车辆参数：转向比例 0.1、直行距离 1.6m、循线速度 26、动作速度 28、右转倒车 0.25m 等。已有摄像头或控制器运行时会拒绝重复启动，请先在原启动终端 Ctrl+C。

只看识别、复用已打开的前摄且不控制车辆：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
roslaunch robocup_competition yolo_preview.launch
```

预览消息只发 /competition/sign_preview，调试画面发 /debug/sign_board_frame，兼容现有网页 /stream/sign_board_frame。预览结束请 Ctrl+C，避免与正式节点同时向调试画面发布。

## 回退

旧模型 /home/nano/robodata/models/20260915_10000/resnet18_candidate.onnx 保留。
在原完整 roslaunch 命令中显式加：

```text
sign_backend:=classifier
sign_model:=/home/nano/robodata/models/20260915_10000/resnet18_candidate.onnx
```

以上是两项参数，实际命令中按 shell 续行规则连接。sign_labels、sign_roi_mode 仍兼容旧模型。
修改前 sign_node.py、full.launch、stack.launch 的备份在模型目录 backup/。

## 验证记录与边界

- Nano Python 2：各 anchor 独立解码、坐标还原、画面底部目标、补边剔除、NMS、非法数值、较近候选选择与 ROS 消息回归共 29 项通过。
- 原路牌接受阈值/连续帧行为 11 项通过；截图保存 2 项通过。
- 扩展相机调度测试原本 5 项失败，原因是旧测试对象未初始化生产节点已有的 uturn_vision 字段。仅在测试进程临时补 None 后 6 项通过；生产相机代码与原测试文件均未修改。
- 40 张部署对照图（七类各 5 张，背景 5 张）是测试集中的小样本，用于发现预处理、TensorRT 导出和消息接入错误，不能替代完整独立赛道测试。
- 首版含框解码的 TensorRT 引擎出现嵌套重复框，未通过检查，已隔离为 decoded_head_rejected.engine。实际部署采用 raw-head ONNX + CPU 解码，不使用该失败引擎。

## 已完成的本机验收

- 最终 FP16 引擎：40 张对照图的检测数和类别与 PyTorch 全部一致；置信度最大差 0.0007303，框坐标最大差 2 像素。该组图片预处理、推理、后处理平均 281ms、P95 342ms，前摄同时运行。
- 独立前摄 ROS 预览：30 条有效消息，约 3.02Hz，平均推理处理 285ms，最大来源帧龄 0.527s。
- 完整静止流程（循线、地面感知、YOLO 同时运行；live=false、enabled=false、start_actuators=false）：30 条有效消息，约 3.00Hz，平均推理处理 206ms，P95 214ms，最大来源帧龄 0.250s。两组实时消息中均未接受路牌，这项验证只证明链路和延迟，不证明当前画面上有已正确识别的实物路牌。
- 正式 full.launch 参数解析确认 sign_backend=yolo_trt、目标 engine 路径正确、sign_hz=3.0。未修改行驶状态机或原路牌接受阈值。
- 自己启动的测试 roslaunch 已确认不再运行。没有通过测试命令启动车辆执行器。
- 保存记录时发现完整静止测试的原 JSON/JPEG 文件为空；上述完整流程数值来自本次工具执行时已经返回的成功报告。恢复报告单独标记 recovered_from_execution_output；不使用空图或预览旧图作为实物识别证据。
- 本机测速不等于移动实战验收；仍须检验运动模糊、远处小牌、多牌干扰，以及连续赛道控制表现。

## 证据文件

- 模型目录 nano_benchmark.json、ros_preview_report.json：Nano 对照与独立预览。
- ros_shadow_report_recovered.json：从已返回的成功执行输出恢复的完整静止流程统计。
- 模型目录 build_raw.log、deployment_manifest.json、test_cleanup.json：构建、文件校验与测试进程清理。
- 模型目录 backup/：部署前代码和启动配置。

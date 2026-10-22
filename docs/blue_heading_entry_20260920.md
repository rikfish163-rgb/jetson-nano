# 蓝线定时纠偏 1 秒后，单独前进 0.36 米

代码直接修改于 Nano `/home/nano/robocup_ws/src/robot`。本次备份：`/home/nano/refactor_backups/before-blue-heading-align-qI642o/before.tar.gz`。

## 当前顺序

1. 方向牌连续两帧确认后缓存，正常循线；确认本次长蓝线后立即开始蓝线方向控制。直行的右侧蓝线距离控制及缓存更正见 [右蓝线 30 厘米控制](straight_right_blue_20260920.md)。
2. `blue_approach.phase=ALIGN`：以前摄蓝线的法向为目标，让车身与横向蓝线垂直。车身左偏则右纠、右偏则左纠；用既有 `steering_for_heading` 和 gap 速度低速前进纠偏。舵轮回正不等于车身摆正。
3. `ALIGN` 是固定 `blue_align_duration_s=1.0` 秒窗口，从确认蓝线、进入 ALIGN 时计时，不随新图像重置。仍能匹配本次蓝线时按测得方向纠偏，蓝线离开视野或本帧被拒绝后使用最后接受的方向。误差在 5 度内回正前进；满 1 秒无论是否达到角度要求，都直接进入 `ADVANCE`，不停车等待。
4. `ADVANCE`：到时回正，以当前姿态作为 `advance_origin`，单独前进 `blue_aligned_advance_m=0.36` 米。保持进入本阶段时的朝向，不延续未完成的蓝线纠偏，不追随横穿白线路径。纠偏窗口内路程不计入这 0.36 米。
5. `STOP`：速度、舵角输出为 0，停车 `intersection_wait_s=1` 秒，然后执行缓存动作。
6. 直行牌从该停车位置起重新计距，直行 `straight_distance=1.25` 米（下限 1.25 米），到距离立即结束直行动作、交回普通循线。没有额外出口确认、延长搜索或 2.2 米退出阶段；交回后缺少路径由普通循线处理。其他动作内部逻辑沿用上一版。

原“车头前沿恰好到蓝线时停车”的规则已替换。本流程不保证停车位置恰好在线上，因为停车位置由对齐完成的位置加 0.36 米决定。

## 感知与动作独立计距

蓝线事件仍要求足够长度、横向/斜向几何、三帧确认；只有线中心而没有测得的方向，不能启动车身对齐。ALIGN 只能使用匹配到本次蓝线的新角度，突变或另一条蓝线不能反向接管。

对齐时空蓝线观测不再停车等待。目标方向保留自已确认的本次蓝线，不能用新线或被拒绝的角度替换。`alignment_source` 区分 `visual` 与 `latched_pose`；后者依赖姿态估算。`alignment_completion=timed` 表示时间到，`heading_within_tolerance` 单独记录当时是否达到角度要求，不伪称已摆正。前摄观测流中断、红灯、急停和适用的雷达检查仍有效。

观测中的 `yaw` 是线的法向；模 pi 处理保证蓝线端点顺序反转不会改变纠偏方向。底盘不能在原地瞬间改变车身朝向，所以“识别后立即对齐”落实为立即开始边前进边纠偏。

日志看 `blue_approach.phase`、`heading_error_deg`、`align_frames`、`advance_origin`、`advance_travelled_m`、`remaining_m` 与 `wait_remaining`。动作直行距离另在 `straight_search.travelled_m`。

## 验证与限制

### 当前固定 1 秒窗口

`blue_align_duration_s` 已接入整车脚本和 full/stack launch，默认 1.0 秒。角度达标、角度未达标、蓝线消失都不会改变窗口长度，到时回正进入独立 0.36 米前进。时间从本次蓝线事件开始计算，后续图像不重置计时。

两个定时回归在修改前失败，修改后 Nano Python 2 上的 135 项相关测试通过，模块归属、脚本语法及 launch 参数解析通过。记录见 `field_data/blue_align_timer_validation/`。其余阶段测试用压缩的 0.1 秒窗口提高测试速度，定时回归及自行车模型使用生产 1 秒设置。仅完成不动车验证，未宣称实车已走准。

### 2026-09-20 蓝线丢失停车修复

最新日志在 `1789895907.514451` 确认蓝线，随后 `1789895910.31549` 因单帧目标丢失输出停车，尚未进入 0.36 米 ADVANCE。原分支已删除，改用最后接受的蓝线朝向和当前姿态完成对齐。旧代码在三项新增/修正回归上失败；修改后 Nano Python 2 的 127 项相关测试通过，模块归属检查通过。证据位于 `field_data/blue_align_latch_validation/`（原始失败字段、代码差异、测试输出）。

按日志剩余偏差约 7.76 度构造的自行车模型测试，覆盖后续无蓝线、完成对齐、独立前进 0.36 米、停车 1 秒和进入直行；另覆盖错误角度不能反向接管、前摄流中断仍停车。本次没有实车运行；模型完成对齐不代表实测车身方向或距离准确。

### 之前版本的验证记录

不动车测试在 Nano Python 2 上运行。测试输出存于 `field_data/blue_heading_entry_validation/`；覆盖左右纠偏、蓝线法向端点翻转、空观测、角度突变、三个阶段分别计距、红灯/急停、连续路口及不同顺序到入库的状态流。

主回归 174 项通过；增加角度突变检查后，最终入口/流程/ROS 适配层 55 项复验通过，共 175 项不同测试。配置归属检查、脚本语法检查和含 `blue_aligned_advance_m:=0.36` 的 launch 参数解析通过。生产代码差异与 SHA-256 同时保存在证据目录。

自行车模型闭环测试分别从左右 0.3 弧度朝向误差开始，模拟观测反馈、完成摆正，再前进 0.36 米停车。它验证控制方向和计距衔接，不替代实车校准。位姿仍为 command_model；摄像头标定、转向映射及速度映射误差都会影响实车结果。

沿用此前后续规则：停车 1 秒，直行状态机至少 1.25 米；本次没有改成总共只走 0.36 米。没有启动任何实车控制节点。

## Nano 完整命令

```bash
bash /home/nano/robocup_ws/tools/sign_detector_20260916/run_yolo_vehicle.sh \
  lidar_enabled:=true parking_enabled:=true start_rear:=false \
  wait_green:=true sign_ttl:=0 sign_hz:=5 \
  blue_default_straight:=false blue_align_duration_s:=1.0 blue_aligned_advance_m:=0.36 \
  intersection_wait_s:=1.0 straight_distance:=1.25 \
  lane_speed_raw:=26 action_speed_raw:=28 straight_speed_raw:=26 \
  lookahead:=0.55 steering_command_scale_rad:=0.1 \
  parking_mode:=forward_center parking_entry_speed_raw:=12
```

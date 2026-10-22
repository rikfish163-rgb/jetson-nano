# P4/P5 倒车入库与出库标定手册

这份手册对应 `parking_closed_loop.launch` 的当前实现。它把“视觉能看到”、
“状态机能决策”、“命令到达底盘”和“车辆确实运动”分成四类证据；只有最后
一类也被记录和人工确认，才可以把一次运行称为实车成功。

## 0. 不变的安全约束

- `src/ros/main/config/parking.yaml` 默认 `calibration_complete: false`。
- `parking_closed_loop.launch` 默认 `start_actuators: false`。
- 任何视觉丢失、雷达不明确、库位不一致或状态超时都必须输出零命令。
- 预触发前进只在 `calibration_complete=true` 且 `/parking/lidar` 为有效、
  `base_link`、无障碍的 `parking_lidar_v1` 时允许，并由主控制器以
  `steering_raw=0` 直行；停车请求后仍由 FSM 继续执行同一安全门。
- `fault` 只能通过人工确认后发送 `/parking/reset` 或重启控制节点解除。
- 第一次运动测试必须轮子架空、急停可触达，并且现场只能有一个命令发布者。
- `/control/cmd`、`/ackermann_cmd` 和串口状态不能互相替代：它们分别证明
  控制器决策、ROS 执行接口和本机串口写入，不直接证明车轮位移。

## 1. 当前设备和默认映射

当前 Jetson 已发现：

| 用途 | 默认设备 |
| --- | --- |
| 前视相机 | `/dev/v4l/by-path/platform-70090000.xusb-usb-0:2.3:1.0-video-index0` |
| 后视相机 | `/dev/v4l/by-path/platform-70090000.xusb-usb-0:2.4:1.0-video-index0` |
| 底盘串口 | `/dev/ttyACM0` |

前后相机的物理朝向仍需用画面人工确认一次。若相机互换，用 launch 参数覆盖，
不要只把 `video0` 和 `video1` 互换后忘记更新记录：

```text
front_device:=<前视设备路径>
rear_device:=<后视设备路径>
```

## 2. 分阶段流程

### A. 源码和配置验证

这一阶段不需要 ROS master，也不接触设备。应先通过：

```text
python2 -m unittest discover -s src/ros/main/test -p 'test_parking*.py'
python2 -m unittest discover -s src/ros/camera/test -p 'test_parking*.py'
catkin_make --pkg main yihan_ros_pkg ackermann_drive_teleop base_controller
```

完成现场参数填写后，先运行离线解锁检查。它不会连接 ROS、打开相机或
发布底盘命令；默认配置应该明确报告 `NOT_READY`：

```text
python2 src/ros/main/scripts/parking_calibration_check.py
```

例如使用“前视候选 affine + 出库候选 affine”作为两套动态视觉源时，只有
在 P4/P5 profile、`calibration_complete`、前视 `derive_pose_from_candidate`
和出库 `use_candidate` 都已经按实测配置后，下面检查才可能通过：

```text
python2 src/ros/main/scripts/parking_calibration_check.py \
  --front-pose-mode candidate_affine \
  --exit-pose-mode candidate_affine
```

检查输出中的 `motion_ready` 只表示静态配置完整，不表示车辆已经运动，也
不表示现场线条已经被视觉正确识别。

### B. 视觉-only 接入

执行器保持关闭：

```text
roslaunch main parking_closed_loop.launch \
  start_actuators:=false \
  start_cameras:=true \
  start_lane_nodes:=true \
  start_lane_pose:=true \
  start_lidar_adapter:=true \
  slot_id:=P4
```

如果还需要由本仓库的旧 LS01B 发布端提供原始 JSON，可额外打开
`start_lidar_source:=true`；否则只要外部已经提供 `lidar_input_topic`，适配器
就可以直接接入回放/桥接流。上述参数仍不会打开执行器。

检查以下数据，不允许手工给控制器补字段：

1. 前视原图、蓝色起始线 mask/overlay、白色库线 mask/overlay 和 BEV 都有效；
   `/perception/parking_slot_candidate` 能稳定给出白色库线候选，而
   `/perception/parking_start_line` 只在蓝色起始线出现时置 `start_line_visible=true`；
2. `/perception/front_parking_metric` 只在目标已经选定后转发蓝线的
   `turn_distance_m`/`parking_trigger_visible`；
3. 前视姿态源能给出 `vehicle_yaw_error_rad`、`lateral_error_m` 和
   `reverse_ready`；
4. `/perception/rear_parking_metric` 能同时看到两条库线和停车横线；
5. `/perception/exit_metric` 能给出出库距离、横向误差、航向误差；
6. `/parking/lidar` 的 schema 为 `parking_lidar_v1`，`valid` 和
   `obstacle_detected` 均为明确布尔值；原始 `/lidar/receive` 只作为适配器
   调试证据，不直接作为状态机安全输入；
7. `/parking/observation` 的 `frame_id` 为 `base_link`，slot_id 始终一致，
   当前阶段的 `source_confidence` 不低于主配置阈值；
8. `parking_acceptance` 所需的配置 hash 和运行参数已写入日志。

为了自动留存现场图像和视觉证据，可以在上述视觉-only launch 运行期间打开
一次性采集器；它只订阅图像/JSON，不发布任何控制命令：

```text
rosrun yihan_ros_pkg parking_field_capture.py \
  _slot_id:=P4 _sample_count:=8 \
  _output_dir:=/tmp/parking_field_captures
```

采集器会为每次运行创建唯一目录，保存前后原图、前视候选 overlay、前后 BEV、
各 metric、融合 observation、ROS 图像时间戳，以及 white/blue 两套后视检测
overlay。它输出的
`manifest.jsonl` 是后续现场分析和生成 profile 的原始证据；它不是完整运动日志，
不能直接冒充 `parking_autotune` 的成功运行输入。即使检测无效，也会保留图像和
无效结果，不能把“没有检测到”静默丢掉。`parking_closed_loop.launch` 也提供了
`start_field_capture`、`field_capture_output_dir` 和 `field_capture_samples` 参数，
便于由上层脚本自动启动这次无运动采样。

此阶段最容易暴露的结构性问题是：把蓝色起始线误接成库位候选、白色库线无法
稳定形成 P4/P5 目标、前视只有蓝线距离而没有库位姿态，或者后视 BEV 只有
标定板而没有真实 P4/P5 白线。这些情况都不能解锁运动。

### C. 底盘命令链路验证

只在轮子架空且 `calibration_complete` 仍为 false 时打开执行器：

```text
roslaunch main parking_closed_loop.launch \
  start_actuators:=true \
  start_cameras:=false \
  slot_id:=P4
```

确认：

- `/control/cmd` 在校准锁下只有零命令；
- `/ackermann_cmd` 能收到桥接器发布的零命令；
- `/base_controller/status` 能报告串口写入字节数；
- 日志中的 `actuator_command`、`base_controller_status` 和 `control` 时间顺序
  正确。

此阶段不以车是否移动为目标。若校准锁为 false 仍出现非零命令，应先修复，
不得进入下一阶段。

### D. 车辆和场地标定

对 P4、P5 分开记录以下量，单位必须写清楚：

| 参数 | 含义 | 记录方式 |
| --- | --- | --- |
| `wheelbase_m` | 前后轴距离 | 卷尺实测 |
| `vehicle_length_m/width_m` | 车体外廓 | 卷尺实测 |
| `front_overhang_m/rear_overhang_m` | 轴到车体边缘 | 卷尺实测 |
| `camera_to_rear_axle_m` | 后视相机参考点到后轴 | 卷尺实测 |
| `turn_start_distance_m` | 蓝色起始线释放 FSM 后的前进转向入口点 | 场地标线/BEV 测量 |
| `reverse_steer_switch_distance_m` | 倒车换舵点 | 车辆后轴参考 |
| `park_stop_distance_m` | 停车边界 | 停车线到后轴 |
| `exit_straighten_distance_m` | 出库回舵点 | 场地标线测量 |
| `exit_complete_distance_m` | 出库完成线 | 车体完全脱离库位 |
| `forward_speed_raw` | 前进原始速度 | 架空后再低速确认 |
| `reverse_speed_raw` | 倒车原始速度 | 架空后再低速确认 |
| `entry_steering_raw` | 前进入口名义舵角（按库位侧向镜像） | 实车左右方向确认 |
| `straighten_steering_raw` | 回正舵角 | 实车确认 |
| `exit_steering_raw` | 出库转向舵角 | 实车确认 |
| `lane_yaw_sign` | 车道线拟合角到车辆航向误差的符号 | 前进架空/静态姿态确认 |
| `lane_slot_yaw_bias_rad` | 车道方向到 P4/P5 库位轴的角度偏置 | P4/P5 BEV 测量 |
| `lane_slot_lateral_offset_m` | 车道中心到对应库位中心线的横向偏置 | P4/P5 BEV 测量 |
| `candidate_*` / `exit_candidate_*` | 前视触发线/出库线的 affine 映射 | 各库位多帧拟合 |

每次只改一组参数。P4/P5 的场地量写入 `slot_profiles.P4` 和
`slot_profiles.P5`；车辆尺寸、底盘限幅、雷达策略和校准锁保持全局。
视觉映射写入 `yihan_ros_pkg/config/parking_visual_profiles.yaml` 的对应
profile，不要把 P4 的横向偏置复制给 P5。
实车解锁前两个 profile 必须把这些场地/路线字段全部显式写出，不能只创建
空 profile 继承 `parking.yaml` 的占位值；校验器会把缺失字段列为阻塞项。

### E. 低速闭环顺序

按以下顺序逐步放开，不要一开始就运行完整自动流程：

1. P 标志 arm 后，确认白色库线目标已锁存，但蓝线未出现时仍保持直行且
   ParkingController 尚未接管；
2. 识别蓝色起始线后才进入 `approach`，确认 `turn_distance_m` 随车辆前进
   单调减少；随后验证 `forward_right_turn`，确认实际左/右方向由
   `parking_side` 决定且 yaw 误差朝目标方向变化；
3. 验证 `forward_straighten` 和 `reverse_ready`；
4. 验证 `reverse_steer_in`，确认 `rear_distance_m` 随倒车减少；
5. 验证换舵、回正和 `reverse_align`，确认横向/航向误差收敛；
6. 验证 `parked` 停稳；
7. 最后验证 `exit_turn`、`exit_straighten` 和 `exit_complete`。

每一步都要有一个可执行的中止条件：视觉源过期、距离反向跳变、误差超过
限制、雷达不明确、车辆方向与命令符号不一致时立即急停，并保留日志。

### F. 离线自动调参

至少积累多次同一库位的完整成功日志后，再离线生成候选 profile：

```text
rosrun main parking_autotune.py run_p4_1.jsonl run_p4_2.jsonl \
  --min-runs 2 --out p4_profile.yaml
```

工具只允许完整经过 `approach -> ... -> done`、无 `fault`、已打开校准锁的
P4/P5 运行进入候选；`--allow-failed` 只把失败样本列入诊断，不会让它们参与
候选计算。它会复用运行时的 profile 校验，拒绝违反停车/出库阈值顺序的结果。
其中 `exit_complete_distance_m` 的安全裕量会向更小距离收缩，避免提前宣布
出库完成。输出文件是建议片段，不会自动覆盖 `parking.yaml`；人工核对场地
几何和日志后，再把对应 profile 合并进去。

## 3. P4/P5 运行和记录

P4：

```text
roslaunch main parking_closed_loop.launch \
  slot_id:=P4 \
  start_cameras:=true \
  start_lidar_source:=true \
  start_lidar_adapter:=true \
  start_actuators:=true
```

P5：

```text
roslaunch main parking_closed_loop.launch \
  slot_id:=P5 \
  start_cameras:=true \
  start_lidar_source:=true \
  start_lidar_adapter:=true \
  start_actuators:=true
```

如果雷达由其他 launch 或外部节点提供，只打开
`start_lidar_adapter:=true` 并将 `lidar_input_topic` 指向那个原始 String
话题；不能省略适配器而把旧的 `angle/distance` JSON 直接接入状态机。

实车运行前必须把 `calibration_complete` 改为 true，并且在日志中保留：

- 主配置和所有视觉/标定文件 SHA-256；
- `slot_id`、前后相机设备路径、触发策略和执行器开关；
- `/parking/status` 的完整状态序列；
- `/control/cmd`、`/ackermann_cmd`、`/base_controller/status`；
- 规范化 `/parking/lidar` 以及可选的原始 `/lidar/receive`；
- 前/后/出库观测及其 confidence、slot consistency；
- 如有里程计/编码器，配置 `odometry_topic` 并记录它。

运行结束后：

```text
rosrun main parking_replay.py <run.jsonl>
rosrun main parking_acceptance.py <run.jsonl> --config <parking.yaml>
```

上面的默认验收是软件与命令链模式；只有报告中的
`physical_feedback.available` 为 true 才有运动反馈记录。配置了真实里程计/编码器
后，实车闭环验收还要使用：

```text
rosrun main parking_acceptance.py <run.jsonl> --config <parking.yaml> \
  --require-physical-feedback
```

该模式会额外要求有效里程计记录、正向和反向运动样本以及最小位移。无论哪种
报告通过，都还要现场确认车辆实际完成入库和出库；命令到串口的证据不能替代
车辆位姿/运动证据。

## 4. 当前尚未具备的真实证据

当前仓库已经具备状态机和接口，但现有保存图像主要是室内环境和后视标定板，
还没有一组 P4/P5 实际运行的完整前视、后视、出库图像与车辆位姿日志。因此：

- 前视姿态映射的 bias/gain 仍需现场确认；
- 后视 BEV 的车辆外参和停车线偏置仍需现场确认；
- 出库姿态源仍需真实出库线数据确认；
- 当前没有现成编码器/里程计话题，`physical_feedback` 默认不可用。

这些不是可以通过把阈值调大来替代的项目，必须产生新的现场观测和验收日志。

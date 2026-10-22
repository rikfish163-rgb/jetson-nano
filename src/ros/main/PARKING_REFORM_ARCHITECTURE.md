# P4/P5 倒车入库闭环架构

本文记录当前实现的接口、坐标约定、状态机和现场标定边界。它描述的是
“视觉/雷达选择目标 -> 蓝线触发 -> 前向摆位 -> 后视闭环倒入 -> 安全出库”
的统一流程，不把 P4/P5 或右转写死。

## 1. 总体数据流

```text
P 标志(/camera_hts/receive) ─────────────────────────────┐
前摄白色库线候选(/perception/parking_slot_candidate) ───┤
前摄蓝色起始线(/perception/parking_start_line) ────────┤
原始雷达(/scan) ─> parking_target_selector ─────────────┤
                                                        v
                                      /parking/observation
                                                        |
                         main.py + ParkingController    |
                                                        v
                                               /control/cmd

前摄目标/摆位姿态 -> parking_front_metric ──────────────┘
后摄鸟瞰图 -> parking_rear_metric ────────────────────┘
出库线/前视出库观测 -> parking_exit_metric ─────────────┘
```

各模块只通过结构化 JSON 字段交换事实；控制器是唯一的停车动作命令拥有者。
目标选择器不发车速/舵机，视觉模块不直接发车速/舵机，避免多个节点抢占控制话题。

## 2. 坐标系和符号

- `base_link`：车体坐标系，`x` 向车头前方，`y` 向车辆左侧；单位米。
- 雷达点先由 `laser_frame` 通过 `lidar_to_base_x_m`、
  `lidar_to_base_y_m`、`lidar_yaw_rad` 转到 `base_link`，再做车库矩形区域
  的占用判断。不能直接把雷达原始 `x/y` 当成车体坐标。
- 前/后摄像头先完成像素到地面的鸟瞰/度量变换，再输出车体坐标中的
  `turn_distance_m`、`lateral_error_m`、`vehicle_yaw_error_rad`、
  `rear_distance_m` 等量。
- 本工程约定：正舵机 raw 为左打，负舵机 raw 为右打；正速度为前进，
  负速度为倒车。最终发送给底盘的舵机值是 `-22..22` 的整数。
- `parking_side=left` 对应 `turn_sign=+1`，`parking_side=right` 对应
  `turn_sign=-1`。控制器在任务开始时镜像入口转向、回正、出库转向；
  任务中途不允许目标侧变化。

## 3. 目标选择逻辑

`parking_target_selector.py` 接收白色库线候选、独立蓝色起始线和同一时刻的
LaserScan：

1. 过滤过期、非法或没有车体坐标的白色库线候选；蓝色起始线不会进入这一组
   候选，也不会参与 P4/P5 近远映射。
2. 按白色库线候选距离把最近库映射为 P5、远端库映射为 P4；有明确 `slot_id` 时保留
   明确 ID。距离分带和近远映射只是当前地图先验，现场应以实测值替换。
3. 在每个候选库的 `base_link` 矩形区域中检查雷达点，得到
   `slot_obstacle_detected`。
4. 请求的 P4/P5 只是偏好；偏好库有障碍物时，可以自动选择另一处清空的库。
   两处均有障碍物、雷达过期、区域未标定或转向侧不明确时，
   `selection_ready=false`，车辆不能进入停车动作。
5. 独立蓝线只输出 `start_line_visible` 和 `start_line_distance_m`；只有在已经
   选出、雷达确认清空且经过标定的目标后，才合成为
   `blue_trigger_visible=true`。因此“库没有蓝线”不会影响库位检测。
6. `parking_side` 的优先级为：候选显式侧向标注、对应库位配置、全局配置、
   候选的带符号 `lateral_m`。`base_link.y>0` 视为左侧，`base_link.y<0`
   视为右侧；落在 `side_inference_deadband_m` 内拒绝启动，避免把噪声当成转向方向。

输出主题默认为 `/perception/parking_target`，关键字段为：

```text
selected_slot_id       P4/P5 或空
slot_selection_ready   是否允许绑定目标
slot_obstacle_detected 选中库是否被雷达判为占用
parking_side           left/right 或空
turn_sign              +1/-1 或空
start_line_visible     蓝色起始线原始检测是否可见
start_line_distance_m  蓝线到车体参考点的 base_link 距离
blue_trigger_visible   目标已选定后蓝线是否允许释放 FSM
candidate_source       白色库线候选来自哪一帧/哪一个候选
```

## 4. 状态机

```text
IDLE / PREPARKING（主控制器保持直行，FSM 尚未接管）
  └─ P 标志识别 -> 锁存目标库 -> 雷达确认目标库清空
       └─ 继续直行，直到看见已选目标对应的蓝色起始线
              └─ blue_trigger_visible
       v
APPROACH  --低速前进，保持直行，直到入口点--
       v
FORWARD_TURN       --按 parking_side 左/右镜像打角--
       v
FORWARD_STRAIGHTEN --根据前视角度/横向误差回正--
       v
SETTLE
       v
REVERSE_STEER_IN   --后摄实时误差闭环倒车入库--
       v
REVERSE_STRAIGHTEN --后摄角度接近零，减小/反向修正--
       v
REVERSE_ALIGN      --横向、角度、库深同时满足--
       v
PARKED
       v
EXIT_FORWARD_CLEARANCE --先前进脱离库尾/安全距离--
       v
EXIT_TURN            --按入库侧的反向路线驶出--
       v
EXIT_STRAIGHTEN -> DONE
```

任一运动状态发生急停、全局雷达障碍、选中库被占用、目标库位变化、目标侧变化、
必需视觉源过期或置信度不足，控制器输出零速；需要人工复位后才重新开始。
`forward_right_turn` 是历史状态名，为保持已有日志和回放兼容暂不改名；实际方向由
`parking_side/turn_sign` 决定，不能从状态名推断为右转。

## 5. 任务的锁存规则

- 看到 P 只负责 `parking_armed=true`，不立即转向；前摄白色库线和雷达负责
  选出一个清空的 P4/P5，然后锁存 `selected_slot_id` 和 `parking_side`。
- 只有看见已选库的蓝线，主控制器才从预触发直行切入停车 FSM 的
  `APPROACH`；`WAIT_BLUE` 仅保留给旧的直接控制器 API/回放兼容路径。
- 蓝线只释放已经选好的目标，不参与库位编号、库内白线拟合或雷达占用判断。
- 运动期间不接受新的 P4/P5 或另一侧结果；新结果与锁存值不一致直接安全停车/故障。
- 后视相机的 `lateral_error_m`、`vehicle_yaw_error_rad`、库深距离和线可见性
  是倒车阶段的反馈；它们不改变目标库，只改变本周期速度和舵机命令。

## 6. 仍需现场确定的参数

代码框架已经具备，但 `parking_target.yaml` 中 `zone_calibrated: false` 是有意的
安全锁。真正上车前至少要测量：

1. 雷达到 `base_link` 的三项外参，以及 P4/P5 的矩形占用区起止距离、宽度和安全边界。
2. 前摄白色库线候选在实际地图中的距离分带，确认最近/远端库的 P5/P4 映射；
   另外测量蓝色起始线与 P4/P5 的近/远关联规则和触发距离。
3. 每个库在当前车辆路线中的 `parking_side`；如果两库同一侧，可在配置中固定，
   否则保留候选的带符号横向判断。
4. 前进摆位的入口距离、目标偏航角、最大转角和回正点。
5. 后摄鸟瞰坐标的横向零点、角度零点、库深零点，以及倒车速度、角度/横向误差增益、
   误差死区和限幅。
6. 出库前进安全距离、出库转向角、回正条件和完成条件。

这些量集中在 `main/config/parking.yaml`、`yihan_ros_pkg/config/parking_target.yaml`、
`parking_front.yaml`、`rear_parking.yaml` 和 `parking_exit.yaml`；应先低速、单阶段
验证并记录日志，再把 `target_zone_calibrated` 打开。仅通过单元测试不能证明实体车辆
已经成功倒库。

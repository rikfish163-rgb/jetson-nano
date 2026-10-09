# 倒库控制接口与必须标定量

## 1. 坐标系

倒库控制只使用车辆局部坐标，不直接把像素坐标送给控制器：

- frame_id: base_link
- x 轴指向车辆前方，y 轴指向车辆左侧
- vehicle_yaw_error_rad：车辆车身方向相对库位中心线的误差；正值表示车头偏向库位中心线左侧
- lateral_error_m：后轴中心相对库位中心线的横向误差；正值表示后轴在中心线左侧
- 所有距离均为米，时间戳为 Unix 秒

视觉节点必须先完成相机标定和 BEV/地面坐标换算，再发布下面的观测。控制器不使用“像素偏差”直接闭环。

规则图中 P4/P5 标注的 450 mm、380 mm 已作为地图先验放入
`main/config/parking.yaml` 和后视检测器的期望库宽；实际比赛场地尺寸和
线宽仍以现场测量为准。规则还规定车道线、标记线宽度均约 20 mm，且
P4/P5 是倒车入库位。

## 2. /parking/observation 观测格式

停车专用 launch 使用 `/parking/observation`，确保旧的 `/image/send -> /image/receive` 桥接链路不会混入第二套观测。`main.py` 的参数 `~parking_observation_topic` 仍默认兼容旧的 `/image/receive`。

停车 launch 将 `front_image_topic`、`rear_image_topic`、`rear_bev_topic`、白色库线候选、
蓝色起始线、姿态/metric 话题和车道姿态话题作为显式参数传给各节点；默认值仍分别是
当前 Jetson 的前后相机、`/debug/rear_bev` 和 `/perception/*` 话题。更换相机、接入回放或
使用外部姿态源时，应从 launch 参数改图，而不是修改某个脚本里的隐含默认值。session
logger 会把这些话题名写入 `logger_start.topics`，并额外记录白色库线候选、蓝色起始线、
前视姿态和出库姿态的实际 JSON 流。

`parking_closed_loop.launch` 还会把
`yihan_ros_pkg/config/parking_visual_profiles.yaml` 加载到前视 metric、车道姿态和
出库 metric。该文件按 `slot_id` 给出 P4/P5 的车道 yaw/lateral 偏置、白色库线
affine 映射和出库 affine 映射；蓝色起始线只负责触发，不负责库位身份。它只负责
视觉坐标换算，不负责速度、舵角或状态阈值。实际标定时只修改对应库位 profile，并把
该文件保留在 logger 的配置哈希中。

每一帧发布一个 std_msgs/String，内容为：

    {
      "schema": "parking_observation_v1",
      "stamp": 1720000000.123,
      "frame_id": "base_link",
      "valid": true,
      "confidence": 0.92,
      "source_confidence": {"front": 0.92, "rear": 0.88, "exit": 0.90},
      "source_status": {
        "front": {"present": true, "fresh": true, "valid": true},
        "rear": {"present": true, "fresh": true, "valid": true},
        "exit": {"present": true, "fresh": true, "valid": true}
      },
      "parking_armed": true,
      "parking_request": true,
      "selected_slot_id": "P4",
      "slot_selection_ready": true,
      "slot_obstacle_detected": false,
      "parking_side": "left",
      "start_line_visible": true,
      "start_line_distance_m": 0.82,
      "blue_trigger_visible": true,
      "parking_trigger_visible": true,
      "slot_id": "P4",
      "slot_consistent": true,
      "slot_visible": true,
      "rear_visible": true,
      "exit_visible": true,
      "front_clear": true,
      "rear_clear": true,
      "exit_clear": true,
      "turn_distance_m": 0.82,
      "vehicle_yaw_error_rad": -0.12,
      "lateral_error_m": 0.10,
      "rear_distance_m": 0.95,
      "reverse_ready": false,
      "exit_distance_m": 0.80,
      "exit_yaw_error_rad": 0.15,
      "exit_lateral_error_m": 0.12,
      "exit_complete": false
    }

其中：

- parking_armed：P/停车标志识别后的任务臂；它只表示“后面要执行倒库”，不立即
  转向。当前 reform launch 以 P 标志为权威来源，`request_from_front_slot` 默认
  为 false。
- selected_slot_id/slot_selection_ready：由白色库线候选和 LaserScan 选出的具体
  P4/P5。蓝色起始线不参与编号、近远映射或障碍物判断。
- start_line_visible/start_line_distance_m：独立蓝线检测器的原始事实；蓝线不在
  库内，`start_line_distance_m` 也不是库深距离。
- blue_trigger_visible：只有在目标已经选定、雷达确认清空、目标经过标定，并且
  当前看到了蓝色起始线时才为 true。
- parking_trigger_visible：P 已经 arm 且上述目标和蓝线条件同时满足的最终 FSM
  释放门。普通车道姿态源不能单独触发倒库。
- slot_consistent：前/后/出口新鲜视觉源必须指向同一个 P4/P5；为 false 或缺失时控制器停车
- source_confidence：分别表示前视、后视、出口 metric 的置信度；控制器按当前状态只使用必需源（前进看 front、倒车/停车确认看 rear、出库看 exit）
- source_status：每个视觉源的 `present`、`fresh`、`valid` 状态；当前状态的必需源三者都必须为 true，置信度不能替代新鲜度声明
- turn_distance_m：蓝色起始线触发事实对应的剩余距离；状态机真正的左/右转向由
  `parking_side` 决定，不能从字段名推断为右转
- reverse_ready：前进摆位已经完成、允许开始倒车
- rear_distance_m：后轴/车尾到库内切换线或停车线的剩余距离
- exit_distance_m：出库时到车道重新对齐线的剩余距离
- exit_complete：视觉确认车辆已回到安全车道后置为 true

front_clear、rear_clear、exit_clear 必须是明确的布尔值；缺失或不是 true 时，控制器会保持零速。

## 3. 视觉源的职责

观测融合节点只做“新鲜数据拼接”，不做像素到米的猜测。各源如下：

- `/perception/parking_slot_candidate`：由前摄已标定的白色 metric BEV 提取库线
  候选，输出候选的 `distance_m/lateral_m/angle_deg`；它只用于 P4/P5 选择和
  库位几何，不会触发 FSM。
- `/perception/parking_start_line`：由前摄原图的蓝色检测器输出蓝色起始线，使用
  `parking_start_line_v1`。它只输出 `start_line_visible`、距离和诊断几何，不携带
  库位身份。
- `/perception/parking_target`：目标选择器将白色库线候选、P 偏好、LaserScan 和
  独立蓝线合成为一个目标。`blue_trigger_visible` 只有在目标就绪后才会释放。
- `/perception/front_parking_metric`：消费上面的目标结果和可选的独立
  `parking_front_pose_v1`。它转发蓝线的 `turn_distance_m`/触发门，同时保留
  白色库位身份；不把蓝线角度当库位航向。
- `/perception/rear_parking_metric`：输入已经标定的 `/debug/rear_bev`，只有
  同时看到两条近似平行库线和一条横向停车线时才置 `valid=true`，输出倒车
  阶段的 `rear_distance_m/lateral_error_m/vehicle_yaw_error_rad`。K/D/P
  与 H 属于车辆级的后视几何标定；后视检测器还会按 P4/P5 读取
  `parking_visual_profiles.yaml` 中的线宽先验、BEV 横向原点、停车线偏置、yaw
  符号和线颜色。`camera_to_rear_axle_m` 则是 H 的相机地面原点到后轴的
  安装外参，不能从 K/D/H 自动推出。
- `/perception/exit_metric`：默认只接受显式的
  `/perception/exit_pose` 或出库白线姿态。蓝色起始线不作为出库线；只有现场
  明确接入了另一个独立、已标定的出库候选源，才允许打开
  `parking_exit.yaml` 的 `use_candidate=true`，输出随车辆移动的
  `exit_distance_m`；不能用固定计时替代视觉距离。
- 原始 `/lidar/receive` 是现有 `lidar_broadcast` 的兼容输入，通常只有
  `angle`（度）和 `distance`（米），不能直接交给状态机。停车闭环应先由
  `parking_lidar_adapter.py` 转成 `/parking/lidar` 的
  `parking_lidar_v1`；没有分区雷达时，融合器将这个全局无障碍结论保守地
  复用到前/后/出口三个 clear 门控。`lidar_broadcast/scripts/just_msg.py`
  必须对每个有效 `/scan` 发布心跳：近距离回波保留最近距离，只有远距离有效
  回波时发布 `valid=true` 的上限距离；全扫描无效时发布 `valid=false`，不能
  静默不发，否则适配器无法区分“空旷”和“雷达失联”。

## 4. 坐标转换和数据流

```text
相机原图
  -> 去畸变/地面 BEV（各视觉节点自己的标定）
  -> base_link 米制误差（前/后/出口 metric source）
  -> parking_observation_node 拼接新鲜源
  -> main.py 的 ParkingController 状态机
  -> /control/cmd 原始速度/舵角
  -> ackermann_control_bridge
  -> /ackermann_cmd -> base_controller -> 串口底盘
       |                  |
       |                  +-> /base_controller/status（串口写入证据）
       +-> parking_session_logger（执行命令证据）
```

`/scan -> lidar_broadcast -> /lidar/receive -> parking_lidar_adapter
-> /parking/lidar` 是独立的安全输入链路；适配器只做消息契约和阈值判断，
不拥有车辆控制权。

`parking_closed_loop.launch` 默认使用 `parking_only=false`、
`require_calibrated_parking=true`：标定未完成时仍只发零速；标定完成后，
`main.py` 才允许在雷达安全门打开时以 `steering_raw=0` 直行，直到统一观测中的
`parking_request=true`，再把控制权交给倒库 FSM；预触发阶段不再采用前视车道
曲率命令。预触发阶段与倒库阶段使用
同一套 `/parking/lidar` 硬闸门；雷达缺失、无效、坐标系不是 `base_link` 或检测到
障碍物时，模式为 `parking_lidar_wait` 并发零速。若确实要保留旧的“只等停车请求”
行为，可显式传 `parking_only:=true`。默认 `calibration_complete=false`，
即使收到停车请求也会锁到安全停止。

## 5. /parking/lidar 格式

    {
      "schema": "parking_lidar_v1",
      "stamp": 1720000000.123,
      "frame_id": "base_link",
      "valid": true,
      "obstacle_detected": false,
      "emergency_stop": false,
      "nearest_distance_m": 1.2,
      "nearest_angle_deg": 12.0,
      "threshold_distance_m": 0.35
    }

`valid=true`、`frame_id=base_link` 和布尔型 `obstacle_detected` 缺一不可。
`parking_lidar.yaml` 中的距离/角度阈值是待实测的全局安全参数，不是最终
比赛参数。默认要求适配后的雷达数据存在且没有障碍物；障碍物一旦触发，FSM
锁定到 fault；确认现场安全后，可重启节点，或向 `/parking/reset` 发布一次
`std_msgs/Bool(data=true)` 再重新触发停车。

## 6. 状态和控制方向

    idle / preparking（P 已 arm，目标未到蓝线前保持直行）
      -> approach（蓝色起始线释放 FSM）
      -> forward_right_turn（历史状态名，实际按 parking_side 镜像）
      -> forward_straighten
      -> settle
      -> reverse_steer_in
      -> reverse_straighten
      -> reverse_align
      -> parked
      -> exit_turn
      -> exit_straighten
      -> done

任何观测超时、坐标系错误、置信度过低、视野丢失或安全距离不明确，当前周期均输出零速；到达停车线但姿态仍不合格则进入 fault，不会继续硬倒。

## 7. 代码架构与信息边界

倒库不是“相机直接控制舵机”，而是四层闭环。每层只交换约定的数据：

```text
相机/雷达原始数据
        -> metric perception（像素/BEV -> base_link 米制误差）
        -> observation builder（新鲜度、置信度、clear 门控、slot_id）
        -> ParkingController FSM（只读观测，不读像素，不改地图）
        -> command gate（限幅/安全停）
        -> bridge/base_controller（raw speed/steering -> 底盘）
                 ^
                 +---- status/logger/replay（只记录和复盘，不抢控制权）
```

坐标边界必须保持清楚：

- 场地/map 坐标只存在于视觉节点的标定和点位配置中，用来定义入口触发线、换舵线、停车线、出库对齐线；它不直接进入底盘控制。
- 相机像素和 BEV 像素只能留在 metric perception 内。控制器收到的 `frame_id` 必须是 `base_link`，x 为车头方向、y 为车辆左侧。
- `lateral_error_m`、`vehicle_yaw_error_rad` 是相对当前目标库位中心线的误差；距离字段是从车辆当前参考点到对应场地点位的剩余米数。误差正负由车辆实测确认。
- `/control/cmd` 只使用当前底盘约定的 raw 整数：速度负值为倒车，舵角正负按实车标定。整个系统只能由 `main.py` 的 parking 分支拥有这个控制出口。

人工接管使用独立的 `/keyboard/control_cmd` JSON 话题，由
`ackermann_control_bridge` 统一转成唯一的 `/ackermann_cmd` 发布。键盘节点
不能直接发布 `/ackermann_cmd`；键盘消息带 `enabled` 和 `0.30 s` 超时，超时后
桥接器锁定为零命令，必须收到明确的 `enabled=false` 才能释放。推荐使用
`keyboard_override.py`，不要把旧版 `keyop.py` 和自动控制桥同时启动。

每个状态只有一个职责，状态机只沿下面的有向路径前进：

| 状态 | 车辆动作 | 必须的视觉/安全输入 | 进入下一状态 |
| --- | --- | --- | --- |
| `approach` | 低速直行 | 目标已锁存、前方 clear、蓝线 `turn_distance_m` | 到蓝色起始线/入口点，连续帧确认 |
| `forward_right_turn` | 按 `parking_side` 左/右打舵；可叠加前视 yaw/lateral 反馈 | 前视库位、前方 clear、航向误差 | 航向达到目标，连续帧确认 |
| `forward_straighten` | 前进回舵；可叠加前视反馈 | 前视库位、航向、`reverse_ready` | 车身回正且允许倒车 |
| `settle` | 刹停短暂稳定 | 前/后 clear | 稳定时间结束 |
| `reverse_steer_in` | 倒车保持入口舵角；可叠加后视反馈 | 后视库线、后方 clear、后距/横向/航向 | 到换舵线 |
| `reverse_straighten` | 倒车回舵 | 后视库线、后距/航向 | 航向进入容差 |
| `reverse_align` | 倒车闭环修正横向和航向 | 后视库线、后距/横向/航向 | 到停车边界且姿态合格 |
| `parked` | 停车保持 | 后视停车确认、前/出口 clear | 保持时间结束后进入出库 |
| `exit_turn` | 前进出库打舵；可叠加出口反馈 | 出口线、前/出口 clear | 到出库回舵线 |
| `exit_straighten` | 前进回正 | 出口距离/横向/航向 | 视觉确认回到安全车道 |
| `done` | 零速；本次请求结束后释放 parking 控制权 | 无 | 下一次请求前保持完成 |

状态转换、`state_age_s`、状态进入原因、最后一条命令原因和当前观测源状态统一发布在 `/parking/status` 的 `parking_status_v1` 中。这样调参时能区分“状态没到点”“视觉丢失”“安全门没开”和“命令被限幅”，不会只看车最后停在哪里猜原因。

在 `parking_only`、标定未完成或 `parking_lidar_wait` 时，`main.py` 会持续发布
零速心跳和安全原因，便于区分“尚未触发”“尚未标定”和“雷达闸门关闭”，而不是
误以为节点没有工作。进入 `lane_following` 后，仍只有一条 `/control/cmd` 出口；
触发 `parking_request` 后，车道控制不再与 FSM 并行发命令。

前进/倒车/出库的反馈增益都放在 `main/config/parking.yaml`，默认是零，先保留名义点位动作；某一项增益打开后，如果对应横向误差不新鲜，控制器会安全停车，不会用旧值或像素值补齐。

即使在 `approach` 阶段，蓝线距离也不能单独放行车辆：每个前进控制周期都必须同时有
有效的前视航向误差和横向误差。白色库线只负责目标选择/几何；蓝色起始线只负责
释放 FSM；没有经过标定的前视位姿时，控制器保持零命令等待。

## 8. 必须实测的参数

下面这些不能从现有代码可靠推出：

1. 车辆轴距、车宽、车长、前后悬，以及相机到后轴的安装外参
2. speed_raw 到实际车速的映射、死区、最小可动车速、正负方向
3. steering_raw 到实际舵角的映射、左右符号、最大舵角、舵机响应延迟。
   当前已测得满舵幅值为 `raw=±22` 对应 `±26.515°` / `±0.46275 rad`，
   所以每个 raw 单位对应 `1.205227273°` / `0.021034091 rad`。物理角度
   转 raw 时按 `sign(angle) * ceil(abs(angle) / 1.205227273)`，再限幅到
   `[-22, +22]`；`main.py` 的 `/control/cmd` 只允许这个范围内的整数。
   当前实车方向已确认：正 raw 为左转，负 raw 为右转；最终符号由选中库位的
   `parking_side` 镜像决定，不能把 P4/P5 永久写成右转。
4. P4/P5 每个库位的入口方向、库位宽度/深度、前进转弯触发点、倒车换舵点、停车线和出库对齐线
5. 前视/后视相机内参、畸变参数、相对车辆坐标外参、地面 BEV 标定；当前
   后视 K/D/P 和 H 文件已经存在，但后视相机到后轴的安装外参仍需现场量取
6. `parking_visual_profiles.yaml` 中 P4/P5 各自的 lane yaw/lateral 偏置、
   白色库线/姿态 affine、出库 affine，以及后视 BEV 的车位宽度/原点/停车线偏置/
   线颜色参数
7. 视觉输出的置信度阈值、丢帧超时、障碍物阈值，以及每个误差字段的正负方向

后视 `rear_parking.yaml` 中的 `camera_to_rear_axle_calibrated` 和
`line_color_calibrated` 是安全凭证，不是算法参数。两者没有在实际车上确认前，
即使 K/D/H 文件存在，静态检查也不会把后视链路判为可解锁。

YAML 中的数值只是安全的起始占位值。calibration_complete 默认是 false，未完成上述实测前不会发出运动命令。

`supported_slot_ids` 默认只允许 `P4`、`P5` 进入这套倒车入库 FSM；P1/P2/P3 是侧方停车位，视觉即使误报这些编号也只会得到无效观测/安全停止。当前 FSM 根据 `parking_side` 在任务开始时镜像左/右路线；两侧都必须经过各自的舵角符号和现场点位标定。`slot_profiles.P4` 和 `slot_profiles.P5` 只覆盖场地/库位几何、各阶段阈值、名义舵角和反馈增益；车辆尺寸、底盘 raw 限幅、雷达策略和校准锁是全局参数，不能随库位悄悄变化。

当 `calibration_complete=true` 时，P4 和 P5 profile 不能只是空对象或只写一两个阈值；每个 profile 必须显式包含该库位的尺寸、入口触发、前进回正、倒车换舵、停车边界、出库边界和名义舵角字段。这样全局 YAML 中的起始占位值不会因为漏填而被误当成实测值。离线回放可用 `--allow-uncalibrated-replay` 临时覆盖，但该选项不会影响实车运行时的校准锁。

## 9. 调试与回放

先用 `parking_closed_loop.launch` 在轮子架空/急停可触及时检查：

1. `/debug/front_raw`、`/debug/metric_bev`、`/debug/rear_bev`、
   `/debug/rear_parking_metric` 是否有图像；
2. `/perception/front_parking_metric`、`/perception/rear_parking_metric`、
   `/perception/exit_metric` 是否按 `base_link` 契约输出；
3. `/parking/observation` 的 `source_status` 是否 fresh/valid；
4. `/parking/status` 状态变化是否严格经过上面的状态序列，
   `/control/cmd` 在未解锁前是否始终为零。

`parking_session_logger` 会把 observation、三个 metric 源、规范化 lidar（以及
可选的原始 lidar）、标志和
控制命令、`/ackermann_cmd`、`/base_controller/status` 写成 JSONL，并在
`logger_start` 记录主控制参数、视觉参数、话题配置、slot 选择、触发策略、
相机设备路径、前视/出库候选模式、相机/车道节点开关、执行器开关、闭环
launch 文件和 BEV 标定文件的 SHA-256。`/ackermann_cmd` 代表桥接器已经发布了命令，
`/base_controller/status` 代表底盘节点完成了一次串口写入及写入字节数；
这两者都不是轮速/位移测量。里程计或编码器可通过 `odometry_topic` 可选接入，
当前工程没有现成的该类话题，默认关闭。用
`rosrun main parking_replay.py <log.jsonl>` 可离线
重放状态机；该工具不会连接 ROS，也不会发布任何底盘命令。现场每次只改
一组参数，再用日志比较首次丢检、越界或超时状态。

还可以运行只读预检：`rosrun main parking_stack_check.py --duration 5`。
它会汇总各话题是否有数据，并同时检查 `/control/cmd` 和 typed 的
`/ackermann_cmd` 是否出现非零命令；它本身不发布 `/control/cmd`。

在启用 `calibration_complete` 之前，可先运行
`python2 src/ros/main/scripts/parking_calibration_check.py`。这是无 ROS 的静态
检查，会列出 P4/P5 profile、相机/BEV 文件以及前视/出库动态视觉源是否已经
声明完整；`motion_ready` 仍然不等同于车辆已经运动或场地视觉已经验收。

完成一次实际运行后，用
`rosrun main parking_acceptance.py <run.jsonl> --config <parking.yaml>`
生成 `parking_acceptance_v1` 验收报告。报告必须同时通过状态顺序、配置哈希、
控制器命令限幅/校准锁、阶段视觉源、状态超时、`/ackermann_cmd` 双向命令和
底盘串口写入证据，才算一次可用于调参的成功样本。报告中的
`physical_feedback` 在配置真实里程计/编码器后才会有有效记录；当前默认为
`available=false`。这条默认命令是“软件和命令链验收”，不是实车运动验收。
有真实里程计/编码器后，使用
`rosrun main parking_acceptance.py <run.jsonl> --config <parking.yaml> --require-physical-feedback`；
此模式还要求有效里程计记录中出现正、反两个方向的运动和足够的位移。
里程计位置应注明 `frame_id`（通常为 `odom`/`map`），速度应注明
`child_frame_id`（通常为 `base_link`）；验收只统计 `approach` 到 `done`
窗口内、且速度方向与状态机前进/倒车阶段一致的样本。
即使该项通过，也只是补足车辆运动证据，最终仍需结合视觉位姿变化、停车位置
和现场安全验收，不能把串口写入等同于车辆已经按命令运动。

# 架构与接口

```text
摄像头 / 雷达 / 路牌识别
            ↓ 观测
          master 总控
            ↓ 选择行为
lane / obstacle / turn / uturn / parking / parallel_parking
            ↓ 速度与转向建议
motion 跟踪及停车 → ROS 执行桥与接管 → 底盘驱动
```

## 三个源码入口

- `src/robot`：比赛业务和直接配套的 ROS 适配节点，按功能组织。
- `src/ros`：既有相机/识别/雷达节点、基础寻线、接管、消息桥、独立停车后端。
- `src/drivers`：设备驱动及串口库。

保留 ROS 包的逻辑名称以维持话题、模型、标定和包依赖。物理目录已更名并分组，
Python 业务包改为 `robot`；不保留旧 Python 模块转发文件。
既有 ROS 支撑包也包含各自的处理算法，其维护归属可通过 `tools/module_workspace.py list <模块>` 查看。

| 物理目录 | ROS 包名 |
| --- | --- |
| `src/robot` | `robocup_competition` |
| `src/ros/camera` | `yihan_ros_pkg` |
| `src/ros/signs` | `hts_ros_pkg` |
| `src/ros/main` | `main`（独立停车及旧后端） |
| `src/ros/lane` | `vehicle_control` |
| `src/ros/lidar` | `lidar_broadcast` |
| `src/ros/manual` | `ackermann_drive_teleop` |
| `src/ros/bridge` | `msg_bridge` |
| `src/ros/keyboard` | `teleop_twist_keyboard` |
| `src/drivers/base` | `base_controller` |
| `src/drivers/camera` | `usb_cam` |
| `src/drivers/lidar` | `ls01b_v2` |
| `src/drivers/serial` | `serial` |

## 模块协议

业务函数接收 `ctx`，通过 `FIELDS` 声明所需字段，`CALLS` 声明跨模块操作。
`master/runtime.py` 返回 `ModuleResult(value, updates)`，由总控应用状态更新。
传感器观测的字典、列表、元组按调用复制；规划器、跟踪器、Future 等持久对象保留身份，
仍由 ROS 调用锁保护。这里是进程内模块化，不是多个进程。

`common` 只放共享协议、坐标、配置加载、异步执行和通用搜索。
转弯、绕障、入库、掉头的专用规划器放在对应功能目录。
控制模块不创建电机发布器；硬件输出由执行桥统一仲裁。

## 参数与协作

参数加载顺序：`config/competition.yaml` → `config/maneuvers.yaml` → 各功能 `config.yaml`
→ launch 显式参数。保留原 launch 默认覆盖，不改既有标定含义。
`config/workspace_modules.yaml` 定义参数与源码归属；集成前必须运行 `module_workspace.py check`。
独立 P4/P5 后端及相机标定仍使用支撑包内部的配置。

当前本地项目是 Nano 工作区的 SSHFS 挂载。负责人修改自己的目录；
公共协议、总控和整车 launch 由集成人员协调。接口变化需要同步测试和回放样例。

## 构建

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
catkin_make -j2
source devel/setup.bash
```

主控现在由 `src/robot/master/main.py` 启动。直接运行主控时使用
`rosrun robocup_competition main.py`；完整实车命令见根目录 README。

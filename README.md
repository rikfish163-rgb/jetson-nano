# 小车项目

开发从 `src/robot` 进入。每个功能目录包含自己的实现、`config.yaml` 和 `example.json`。
ROS 包名保留 `robocup_competition`；Python 导入统一使用 `robot`。

```text
src/
├── robot/                 比赛程序
│   ├── master/            总控
│   ├── camera/            摄像头输入
│   ├── lidar/             雷达输入
│   ├── signs/             路牌识别
│   ├── motion/            运动执行与轨迹跟踪
│   ├── lane/              寻线
│   ├── obstacle/          避障
│   ├── turn/              左右转、路口直行
│   ├── uturn/             掉头
│   ├── parking/           白线停车与旧入库方式
│   ├── parallel_parking/  测量车位规划与闭环跟踪
│   ├── common/            共享坐标、协议、配置加载与搜索器
│   ├── config/            整车基础参数、参数归属
│   ├── launch/            整车启动
│   └── test/              离线回归测试
├── ros/                   已有 ROS 输入、接管及停车支撑包
└── drivers/               相机、雷达、底盘、串口驱动
```

| 功能目录 | 负责人 |
| --- | --- |
| [master](src/robot/master/README.md) | 总控 |
| [camera](src/robot/camera/README.md) | 摄像头 |
| [lidar](src/robot/lidar/README.md) | 雷达 |
| [signs](src/robot/signs/README.md) | 识别 |
| [motion](src/robot/motion/README.md) | 运动执行 |
| [lane](src/robot/lane/README.md) | 寻线 |
| [obstacle](src/robot/obstacle/README.md) | 避障 |
| [turn](src/robot/turn/README.md) | 左右转 |
| [uturn](src/robot/uturn/README.md) | 掉头 |
| [parking](src/robot/parking/README.md) | 显式选择的原停车模式 |
| [parallel_parking](src/robot/parallel_parking/README.md) | P1–P5 测量闭环停车 |

## 日常调试

以下在 Nano 执行，以寻线为例：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list lane
```

修改 `src/robot/lane/controller.py`，参数写入同目录 `config.yaml`，然后：

```bash
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/lane/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test lane --report-dir /tmp/lane-tests
```

这些命令不启动底盘。独立 ROS 预览见各模块 README。

## 整车启动

统一使用 `tools/sign_detector_20260916/run_yolo_vehicle.sh`。默认 raw 20，
前瞻 1.0 m，命令转向映射 0.03；实际参数以脚本和 launch 为准。

以下会启动车辆，须由现场人员在 Nano 执行。先腾出赛道，准备接管，
不得同时运行其他底盘控制器；出现异常轨迹立即急停并在启动终端 Ctrl+C。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check &&
bash tools/sign_detector_20260916/run_yolo_vehicle.sh \
  parking_mode:=forward_plan parking_slot:=P4 start_rear:=false
```

P1–P5 统一优先使用 `forward_plan`，将 `parking_slot` 改为现场确认的具体编号。
前相机测量指定车位，雷达确认空间，规划器检查整车路径，执行中持续根据观测修正。
测量缺失、目标不明确或空间未知时停止，不自动切换成定时盲走。
当前相机只输出车位几何，AUTO 需要输入明确目标编号，不能自动识别 P1–P5。
无编号时按同类车位完整排序关联：侧方需三个，垂直需两个；驶入后保持同一目标。
后相机不参与前进停车。需要原有后摄掉头的路线可显式设置 `start_rear:=true`。
原 `reverse_plan`、`parallel_reverse`、`forward_white`、`forward_center` 保留为显式单项模式。

雷达启用后持续检查观测时效及车身运动扫掠，包括绕障期间及绕障完成后。
无法确认空间时停车。直行距离达到 1.25 m 后停车等待多个不同图像帧确认出口，
缺少出口时不会直接清除动作。巡线算法和参数由正在修车的另一对话维护。

绿灯、方向牌和蓝线负责触发动作；各行为返回速度/转向建议，
只有总控和执行桥输出底盘命令。当前 `pose_mode=command_model` 与
速度换算仍是配置估算，仿真通过不能替代实际里程、转角和停车验证。

三条路线的米制场景与离线仿真见 [本轮实现与验证](docs/competition_implementation_20260929.md)。

## 其他目录

- `tools/`：检查、回放、标定和采集工具。
- `docs/`：当前架构与验收记录。
- `field_data/`：实车数据，保留供标定、训练和回放使用。
- `build/`、`devel/`：构建自动生成，开发时不用修改。

旧目录中的历史文档、临时图片、实验输出和代码副本已移至
`/home/nano/refactor_backups/20260915-structure/history/`，归档清单为同级 `archive.json`。
源码迁移前快照为 `before.tar.gz`。

[架构及接口](docs/ARCHITECTURE.md) · [验证记录](docs/VALIDATION.md)

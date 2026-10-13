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

统一使用主目录的 `runall.sh`。日常参数只改 `src/robot/config/vehicle_tuning.yaml`。
开机自启只需在 Nano 设置一次（提示时输入 Nano 的 sudo 密码）：

```bash
cd /home/nano/robocup_ws
bash runall.sh --enable true
```

此命令只安装并启用 `robocup-vehicle.service`，不会立即启动程序或动车。
以后上电自动运行整车程序，等待绿牌确认后按现有起步流程行驶，无需电脑输入命令。
启动时读取同一份 YAML；开机服务固定附加 `wait_green:=true`，保证开机等待绿牌。
`--enable` 管理开机服务，不是 YAML 中控制输出的 `launch.enabled`。
关闭以后开机自启：`bash runall.sh --enable false`（不会停止当前运行）。
开关不能和临时行驶参数混用；开机使用的长期参数请修改 YAML。
查看状态：`systemctl status robocup-vehicle.service`；查看日志：`journalctl -u robocup-vehicle.service -b`。
开机服务运行期间，不要再手动启动第二份程序；如需改为手动运行，先在现场停止服务：
`sudo systemctl stop robocup-vehicle.service`。服务退出后不会自动重启。

默认巡线 raw 40、前瞻 1.0 m、转向映射 0.03；入库按当前状态机自动分流，不需要写 `--S`。
速度、动作时长、开关在 YAML 主体；相机帧率、模型路径等在同一文件的 `launch` 小节。
参数优先级：模块默认值 < 此 YAML < 显式命令行参数。保存后重启整车生效。
`runall.sh` 不再硬编码调参值；直接调用底层 launch 的历史调试命令不属于统一入口。

以下会启动车辆，须由现场人员在 Nano 执行。先腾出赛道，准备接管，
不得同时运行其他底盘控制器；出现异常轨迹立即急停并在启动终端 Ctrl+C。

```bash
cd /home/nano/robocup_ws
bash runall.sh
```

例如巡线速度修改 `speed_raw.lane`，直行牌速度修改 `straight_speed_raw`，
左右转时间修改 `left_timed_*_s` / `right_timed_*_s`，掉头修改 `uturn_trial_sequence`，
避障修改 `timed_bypass_*`，侧方开关修改 `parking_side_lidar_enabled`。
临时覆盖示例：`bash runall.sh lane_speed_raw:=30`，不会改写 YAML。
直行后的所有动作牌（左转、右转、直行、掉头、P）：直行期间只记为候选，不进入下一动作缓存。
即使提前交回循线，仍需从直行起点推进到 `straight_distance`（当前 1.4 米），
越过门槛前记录的蓝线，并由至少 3 个连续新画面、跨度至少 0.3 秒确认旧线离开。
之后必须重新识别对应牌子，方向动作才能在新蓝线上触发；P 重新确认后按原停车流程处理。
红牌停车、绿牌放行不受此门槛限制；距离与越线位置仍采用当前位姿估算。
不动车检查最终参数：`bash runall.sh --dump-params`；只检查节点清单：`bash runall.sh --nodes`。
配置中 `parking_mode` / `parking_entry_style` 保留当前共用视觉控制设置；
前摄 P、后摄 P 和侧方分流由状态机决定，本次统一参数不改变触发条件。

雷达仅参与绕障目标检测；雷达缺失、过期、有效点不足、障碍和未知区域不再触发停车，
绕障也不再先零速等待打舵。已删除雷达扫掠停车和入库雷达门槛，车辆不再具备雷达自动刹停保护。
红灯、急停、视觉观测和动作完成停车继续生效。直行距离达到 1.25 m 后停车等待多个不同图像帧确认出口，
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

## Vehicle action tuning

Edit `src/robot/config/vehicle_tuning.yaml` for LEFT, RIGHT, STRAIGHT, UTURN, bypass and P1/P2/P3 parking stages. This file is loaded after module defaults and before explicit launch arguments. Save and restart the vehicle; no live reload. Speed/steering are RAW units; duration is seconds. Visual/distance end conditions are documented in the file.

```bash
cd /home/nano/robocup_ws
nano src/robot/config/vehicle_tuning.yaml
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
```

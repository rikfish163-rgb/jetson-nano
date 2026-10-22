# 摄像头

vision.py 处理图像；observations.py 整理车道/地面观测；perception_node.py 接入 ROS。

S 入库的后摄由 `deferred_rear_node.py` 延迟启动：整车启动时只启动监听节点，
不启动后摄 USB 驱动/鸟瞰节点。仅在总控确认连续 2 秒丢线并发布
`action_source=turn_path_loss` 的后摄倒库状态后启动一次。
前摄识别到牌子、短暂丢线、转弯中、过期状态均不能启动后摄。

调参修改本目录 `config.yaml`；未覆盖的参数沿用整车标定，允许参数用 `list` 命令查看。

当前后摄 `rear_bev.yaml` 的实际单应矩阵会横向翻转原图，因此在归属总控的
`master/config.yaml` 中覆盖
`rear_bev.y_sign=-1`，将鸟瞰像素正确转换为车体坐标（车头为正 x、车体左为正 y）。
后视右侧对应车体左侧，也对应当前 BEV 的左侧。白库线和蓝线共用此转换，
倒车控制器继续使用原有转向标定，不再额外翻转舵量。前摄映射不变。
此符号与当前标定矩阵绑定；更换后摄鸟瞰标定时需运行
`test_rear_bev_direction`，检查真实单应变换、左右偏移、航向和单侧库线纠偏。

2026-10-11 后摄重新标定：640x480，20 mm 棋盘格；地面图中为横向 8、
纵向 11 个内角点。最近一排内角点距镜头地面投影 0.285 m，镜头高度
实测 0.17 m；镜头在后轴后方约 0.05 m，镜头和棋盘均对准车身中心。
`master/config.yaml` 覆盖 `rear_bev.axle_x=-0.05`，继续使用 `y_sign=-1`。
新内参与鸟瞰必须配套加载；重启相机/鸟瞰和总控后生效。原始图、49 张内参
样本、新内参、候选预览及旧标定备份在
`artifacts/rear_calibration_20261011T115527Z/`。角点拟合误差只说明棋盘区域
拟合情况，远处地面距离和实际 5 cm 停车仍需现场核对。

前摄像头蓝线不再按角度过滤路口标记；颜色、厚度、长度筛选不变。
当前 `blue.long_min=0.35` 米，短线仍为 tick，不触发方向动作。
方向缓存及主控 0.36 米触发距离条件不变。纵向长蓝线也可能成为路口标记，
存在误触发风险，需现场验证；后摄像头保留原角度过滤，蓝线朝向测量仍保留。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list camera
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/camera/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test camera --report-dir /tmp/camera-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

独立预览（不启动底盘）：

```bash
roslaunch /home/nano/robocup_ws/src/robot/camera/debug.launch
```

预览默认复用已启动的传感器；需要本入口启动传感器时加 `start_devices:=true`。

已有相机 ROS 节点和标定在 `src/ros/camera`，设备驱动在 `src/drivers/camera`。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

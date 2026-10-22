# 普通整圈与侧方模式隔离

本次以 2026-09-29 Nano 实际代码为准。当前启动脚本默认 `forward_center`；9月22日的 `auto_p` / `parallel_second_p_enabled` 视觉三库入口已经不在当前主流程，本次没有将旧工作副本覆盖回去。

## 模式边界

| 项目 | 普通整圈：forward_center / reverse_plan / forward_white | 侧方：parallel_reverse |
| --- | --- | --- |
| 侧方 Python 模块、规划器导入 | 不导入 | 控制器创建时导入 |
| parallel_parking/config.yaml | 不打开，不合并参数 | 显式 launch 参数选择模式后加载 |
| 侧方参数校验 | 不参与启动校验 | 保留原有范围校验 |
| /competition/parallel_parking_scene | 不订阅，直接调用回调也忽略 | 订阅并检查新鲜度 |
| 状态机 | 原循线、方向牌、左转、右转、普通停车入口 | 原侧方入口 |

侧方参数已从公共 `config/maneuvers.yaml` 移到 `parallel_parking/config.yaml`，数值不变。显式通过 `parking_mode:=parallel_reverse` 启动才加载该文件。仅在自定义 YAML 内写侧方模式仍可选择控制器，但不会自动加载该模块覆盖文件；推荐统一使用明确的 launch 参数。切换模式需要退出后重新启动，不支持在同一控制器实例上改 parking_mode 热切换。

普通整圈继续使用原脚本；明确保持当前默认停车方式的 Nano 命令：

```bash
bash /home/nano/robocup_ws/tools/sign_detector_20260916/run_yolo_vehicle.sh parking_mode:=forward_center
```

`forward_center` 是当前前进入库方式，不是倒车。如选择原普通倒车规划入口，可显式使用：

```bash
bash /home/nano/robocup_ws/tools/sign_detector_20260916/run_yolo_vehicle.sh parking_mode:=reverse_plan parking_slot:=AUTO start_rear:=true
```

以上是会动车的命令，本次未执行。未修改现有启动脚本、循线算法、左/右转参数或停车动作参数。侧方仍需要自身已验证的场景输入；本次不是侧方实车入库验收。

## 验证

- 用只解析配置、不启动节点的 `roslaunch --dump-params` 对比：普通整圈全部非侧方参数与修改前完全一致；侧方参数只在显式侧方模式出现。
- 注入损坏的侧方 YAML：普通 forward_center 和 reverse_plan 仍可解析配置；parallel_reverse 报侧方配置错误。注意 ROS Melodic 的 dump-params 即使报告加载失败也可能退出码为0，测试同时检查错误输出和实际参数。
- 独立 Python 子进程证明普通控制器不导入任何 `robot.parallel_parking` 模块。关闭侧方后，无效侧方参数和无效侧方消息不影响普通入口。
- 相同输入序列下，普通模式的控制命令、状态、原因和动作与保留旧侧方模块的对照运行一致。
- 95项针对性回归通过（隔离、ROS适配、普通停车、模块边界、侧方规划）；另补一项损坏侧方配置的解析回归。
- 扩大回归192项中15个失败、1个错误。恢复旧的始终加载侧方运行时作对照，188项共同用例出现完全相同的16个失败项，涉及路牌确认、蓝线、速度预期等。没有为使测试通过而修改这些公共动作算法，不能将此结果表述为全赛道所有测试已通过。

隔离的范围是侧方代码、配置、数据入口和任务运行。整车仍共用底盘、相机和基础算法；修改这些公共部分仍需回归。未试车，不承诺实车“绝对零影响”。

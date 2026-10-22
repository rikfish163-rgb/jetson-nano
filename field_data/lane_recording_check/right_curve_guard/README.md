# 普通右弯方向保护：修复与验证

修改直接写入 Nano `/home/nano/robocup_ws`（本机挂载 `/home/hetaisheng/jetson-nano`）。
修改前文件保存于 `/home/nano/robocup_cleanup_backups/20260929/right_curve_guard`。

## 修复内容

- 第一帧具有至少三个点、纵向跨度至少 20 cm、整体朝右至少 10 度、远点右偏至少 3 cm，且点列方向一致时，立即记住普通右弯方向。
- 锁定后不接受突然要求左舵的目标；用最近接受的非正舵量，以 GAP 速度 raw 16 过渡。空路径也不复制可能错误的正向 applied steering。
- 被拒绝的目标不刷新 GAP 起点。继续保留 4 s / 0.8 m 的丢线限制和 0.5 s 图像超时停车。
- 至少五个不同源帧、持续至少 0.4 s 确认出弯直线后释放：至少三个点、跨度 20 cm、拟合方向偏差不超过 8 度、残差不超过 2.5 cm，所有点横向偏差在 15 cm 内。确认期间允许回正；释放后恢复双向纠偏。
- 普通锁与路口动作右转锁分开；起步、路口、泊车、实际避障动作入口清除普通锁。
- 调试图的橙色圈表示拒绝的左侧目标，红圈仅表示采用的目标。
- 相机由至少三个实测边线点生成路径，改为两个点也可线性估计法向；单点仍不臆造边线方向。

主要源文件：`src/robot/lane/controller.py`、`src/robot/master/state_machine.py`、`src/robot/obstacle/controller.py`、`src/ros/camera/scripts/camera_yihan_web.py`。遥测和录制同步更新。

## 历史控制记录回放

入口脚本：`../replay_right_curve_guard.py`。结果：`summary.json` 及逐趟 JSONL。

10 趟记录共 5051 次普通 LANE/GAP 控制输出；锁定 15 次、确认释放 5 次、拒绝反向目标 266 次。
锁定期间正舵为 0 次，锁定后 GAP 正舵也为 0 次。

| 记录 | 源图时间戳 | 原始 raw | 修复后 raw | 修复后速度 |
| --- | --- | --- | --- | --- |
| 230116 | 1790694132.4520724 | +22 | -22 | 16 |
| 232406 | 1790695509.9184332 | +22 | -22 | 16 |
| 232406 | 1790695510.1848896 | +18 | -22 | 16 |
| 232406 | 1790695537.3839903 | +22 | -22 | 16 |
| 232406 后续空路径 | 1790695537.451351 | +22 | -22 | 16 |

这些值已逐项读取并断言。负号为右舵，正号为左舵；raw 22 不代表已测得前轮物理角度 22 度。

回放输入是原记录中已补偿到当时车辆坐标系的路径，回放 pose 固定为零，以免重复补偿。
该方法验证同样视觉输入下的指令变化，不能生成改变指令后的新车辆轨迹，也不验证实际轮角或转弯半径。
回放仍有旧观测超时/持续丢线触发停车；并未取消停车限制。距离上限另有单元测试。

## 最新一趟全部相机图像回放

入口脚本：`camera_replay.py`；结果：`camera_summary.json`、`camera_frames.jsonl`。

按顺序读取 `20260929_232406_4508` 的全部 1029 张前摄 JPEG，使用录制 K/D/H。
修改前后相机模块维护独立的历史跟踪状态，同一图像使用同一白线 mask。

- 1029 张全部可读。
- 无路径帧：修改前 144，修改后 132。
- 恢复 12 帧路径；原来有路径、修改后丢失的帧为 0。
- 关键帧 `1790695536524874752_front.jpg`：原先 `NO_LANE / 0` 点，现在 `LEFT_ONLY / 1` 个可信区域内目标点。

仍有 132 帧无路径，方向保护不等于视觉误识别已经全部修好。
JPEG 重处理与原始运行图像会有压缩差异，也不等于实际车辆重新行驶。

## 自动检查

以下 79 项针对性测试全部通过，模块边界检查通过（452 个源码/配置文件）：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=$PWD/src:$PWD/src/robot/test:$PWD/src/ros/camera/test:$PWD/src/ros/lane/test:$PYTHONPATH
python2 -m unittest test_lane_right_curve_guard test_topmost_lane test_lane_loss_transition test_startup_curve_direction test_module_boundaries test_direct_lane_points.DirectLanePointsTests test_dashed_left_boundary test_lane_transport test_ros_adapter
python2 tools/module_workspace.py check
```

更广检查中 `test_right_line_lock` 的三项、`test_route_obstacle_regression` 的两项旧断言失败。
将 lane/controller、obstacle/controller、state_machine 换回本次修改前备份，复现相同五项失败：

- `test_left_boundary_aligns_and_overrides_wrong_center_then_releases`
- `test_left_reference_keeps_right_active_until_stably_aligned`
- `test_wait_and_next_sign_stay_integrated`
- `test_right_seen_during_left_needs_new_votes_after_handoff`
- `test_uturn_rejects_below_090_but_parking_keeps_global_threshold`

未据此宣称全仓库测试通过，也未扩展修改这些既有行为。

隔离 ROS 集成由 `../verify_recording.py` 完成：`passed=true`，`enabled=false`，`live=false`，底盘命令发布者为空。
相机、观测、控制诊断及录制链路连通；录制前摄 4、BEV 4、观测 4、目标诊断 31，缺失前摄/BEV 和队列丢弃均为 0。
详见 `../shadow_result.json`。相机源码 SHA256 为 `21319287f3f01c1cdf6fc4073699b94843a5a146fa5de5de86ad9c2ed99c8eea`。

## 使用与边界

需要停止旧 launch 并重新启动，正在运行的 Python 进程不会自动加载新源码。
普通循线速度 20、映射尺度 0.03、起步中位 0、起步估算距离 1.25 m、左边线向右法向偏移 30 cm 均保持现有设置。
普通循线满舵横向偏差仍为当前配置的 7.5 cm；此项本次未调整。
这次验证未启动车辆；两个实际弯道是否通过、机械舵角和抓地条件仍待实车验证。

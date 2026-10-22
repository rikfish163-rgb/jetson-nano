# 出弯纠偏误停车修复

## 修改

修改直接位于 Nano `/home/nano/robocup_ws/src/robot/lane/controller.py`。
修改前备份：`/home/nano/robocup_cleanup_backups/20260930/exit_correction_014731`。

真实记录 `20260930_011909_18373` 的正常出弯路径相对车身朝左约 13–16 度，
旧逻辑却要求先对齐到 8 度内、所有远点偏差小于 15 cm，才允许左纠偏，
因此有路径仍进入 GAP，4 秒后停车，直到用户手动摆正。

新条件保持三个点、20 cm 跨度、2.5 cm 直线拟合残差、五个不同源帧和 0.4 秒确认：

- 允许出口直线相对车身朝左至 25 度；仍朝右超过 8 度的路径不作为已出弯。
- 检查拟合线在后轴前方 0.50 m 的横向偏差不超过 15 cm，允许远点因车身姿态而偏左。
- 确认期间允许回正并继续接受该路径，确认完成后恢复现有的左右比例控制。
- 保留明显错误车道、突发错误左目标、真实丢线、图像超时和特殊动作的处理。

普通循线速度 20、raw 饱和 ±22、映射 0.03、满舵偏差 7.5 cm 和运动补偿模型均未修改。

## 复现与验证

新增两个直接使用实车路径坐标的测试，旧代码均失败：一个仍返回 GAP `(16,-22)`，
另一个仍以 GAP 速度 16 行驶。修改后两个通过。
另加仍朝右的直线不能提前解锁、急左路径不能当作出口纠偏、真实空路径必须
中断出口连续确认三项测试。有效出口候选保持 LANE；真正丢线仍进入 GAP 并重置确认。

Nano 上以下 84 项针对性测试通过，模块边界检查通过（452 个源码/配置文件）：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=$PWD/src:$PWD/src/robot/test:$PWD/src/ros/camera/test:$PWD/src/ros/lane/test:$PYTHONPATH
python2 -m unittest test_lane_right_curve_guard test_topmost_lane test_lane_loss_transition test_startup_curve_direction test_module_boundaries test_direct_lane_points.DirectLanePointsTests test_dashed_left_boundary test_lane_transport test_ros_adapter
python2 tools/module_workspace.py check
python2 field_data/lane_recording_check/replay_exit_correction.py
```

回放全部 12 趟、7097 次普通循线控制输出，结果见 `summary.json` 和逐趟 JSONL。

- 最新一趟全部 1087 次 LANE/GAP 输出，停车次数为 0。
- 原误拒绝到手动恢复窗口内共 238 次输出，停车次数为 0。
- 源时间 `1790702407.1940708` 的路径在控制时间 `1790702407.398318` 完成出口确认，
  开始正常左向纠偏 raw `+4`、速度 20；原记录此时仍为 GAP、raw `-4`、速度 16。
- 历史 5 个已确认误识别样例仍为 raw `-22`、速度 16，未恢复错误左转。
- 所有回放中，仍处于右弯锁定状态时正舵次数为 0；确认退出后的正常左纠偏允许执行。

## 验证边界

回放固定使用原记录中已换算到当时车体坐标系的路径，回放 pose 为零，
防止重复进行运动补偿。它验证相同视觉输入下的新命令，不生成新车身轨迹。
旧记录中用户手动摆正后的画面也仍存在于完整回放中；238 次窗口单独覆盖摆正前的误停车问题。
没有启动底盘或发布 ROS 控制命令，尚未实车验证两个弯道均通过。
持续输出 raw `-22` 仍靠外的问题，还需要实际前轮角度/运动证据，不能用命令回显代替。

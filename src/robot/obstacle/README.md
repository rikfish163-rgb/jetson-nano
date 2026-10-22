# 避障

正式整车的避障动作参数统一在 `../config/vehicle_tuning.yaml` 的“六、定时避障”中。
保存后重启整车生效。速度、舵量是 RAW 控制值；+22 左满舵，-22 右满舵。

当前固定顺序：

1. LEFT：速度30、左满舵，1.5秒。
2. RIGHT：速度30、右满舵，3秒。
3. RETURN_LEFT：速度30、左满舵，1秒。
4. 立即调用正常巡线控制，使用原巡线速度和视觉转向；没有额外定时接线段。

每段速度/舵量/时间分别使用 `timed_bypass_left_*`、`timed_bypass_right_*`、
`timed_bypass_return_left_*`。`timed_bypass_settle_s=0`，直接开始第一段。
旧的右弧3至6秒视觉确认退出流程和 `timed_bypass_right_max_s` 已移除。
即使提前看见车道，三段也必须完成；缺少车道时仍完成固定三段，交回巡线后
遵守正常巡线的丢线/过期停车规则。完成日志为 `bypass_timed_lane_handoff`，
如果巡线输出为停车则保留其具体停车原因。

触发：巡线且没有正在执行的任务、有效雷达及车道；前方目标到后轴不超过0.85米、
横向不超过车宽一半加0.10米；同一目标确认3次新扫描、跨度至少0.15秒。
full 默认启用雷达形状筛选。现有 `straight_lidar_once=true` 保留：
固定动作表完成后消费本次启动的避障机会。预先缓存的方向牌在交回巡线时保留。

红牌/急停暂停计时；控制时钟异常停车。当前左右弧的雷达预测碰撞拦停仍未启用。
这些是软件指令时长，无轮角到位反馈，也不是实测行驶距离。

无车辆运动的回归检查：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src:src/robot/test:$PYTHONPATH python2 -m unittest -v test_fixed_bypass_sequence test_timed_bypass_speed
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
```

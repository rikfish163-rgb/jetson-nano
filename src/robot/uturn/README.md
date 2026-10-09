# 掉头

controller.py 管理阶段；planner.py、relative.py、timed.py 实现已有掉头策略；vision.py 整理视觉场景。

调参修改本目录 `config.yaml`；动作表在 `../config/maneuvers.yaml`。
当前五段动作的最后一段为回正倒车（RAW -30）2秒；进入此段前停车0.7秒，
倒车结束后再回正停车0.7秒，然后沿道路前进，等待下一条蓝线触发。
当前 `uturn_trial_exit_max_s: 0.0`：动作表和原有换挡/收尾停顿执行完，
同一个控制周期结束掉头并直行，不再等待出口角度或车道确认帧数。
此段优先参考当前两侧白线的中间路径纠偏；只有一侧线时，参考向道路内侧偏移半个车道宽的路径。
没有新鲜白线时按 `straight_speed_raw` 回正直行，也不对尚未确认的蓝线提前打舵。
居中纠偏时减速，并检查当前车身和未来0.25秒扫掠，防止压到任一侧白线。
此检查使用相机实际观测到的有限白线段，不把前方白线向车身下方无限延长；
真实白线进入车身扫掠范围仍会停车，缺少白线的区域保留直行回退。
路牌继续缓存，下一条蓝线确认后交给原有后续动作；避障完成后继续直行等待蓝线。
定时掉头期间，下一路牌的投票窗口在动作表最后一个带转向的阶段完成后才打开。
当前五段动作在进入第5段前的回正换挡停顿时打开，后面的回正倒车和收尾停顿
都可以确认下一路牌。之前旋转过程中看到的方向牌不占缓存，日志为
`uturn_wait_final_heading`；延迟送达、但拍摄于窗口打开前的图像也不参与投票。
窗口按动作表计算，不限定下一块必须是RIGHT；红绿灯仍按原规则处理。
前方相机处理必须新鲜；画面没有白线或蓝线仍可直行，相机/雷达超时、白线越界、障碍物与急停仍会停车。
若以后显式设置大于0的 `uturn_trial_exit_max_s`，才启用有时间上限的出口修正。
`uturn_trial_exit_min_turn_deg` 和 `uturn_trial_exit_heading_deg` 仅在该修正启用时控制交接条件。

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 tools/module_workspace.py list uturn
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py check
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py replay src/robot/uturn/example.json
PYTHONDONTWRITEBYTECODE=1 python2 tools/module_workspace.py test uturn --report-dir /tmp/uturn-tests
```

回放与测试不启动车辆；样例只验证接口，不代表完整实车动作完成。

模块协议见 [架构](../../../docs/ARCHITECTURE.md)。

[Four-stage one-shot tuning commands](FOUR_STAGE_TEST.md)

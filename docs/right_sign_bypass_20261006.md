# 本次 RIGHT 日志与避障下限

来源：用户附件 `0f5a3b88-3c70-401e-8501-365e9828556a/Pasted text.txt`。

## 日志能确认的事情

| 源/控制时间戳 | 事件 |
| --- | --- |
| 1791268093.952 | 避障后 RIGHT 仅 1 票；下一帧空标签，没有确认指令 |
| 1791268138.605 | RIGHT 达到 2 票，`stored_next_direction` |
| 1791268161.268 | 再看到 RIGHT 时已有待执行指令，旧提示为 `action_sign_ignored` |
| 1791268162.733 | 蓝线以 `cached_sign` 触发 RIGHT |
| 1791268166.696 | 开始 `right_timed_reverse` |
| 1791268167.643 | 开始 `right_timed_turn` |
| 1791268171.955 | 开始 `right_timed_exit_reverse` |
| 1791268176.624 | `right_timed_exit_aligned`，交回循迹 |
| 1791268185.319 | `gap_limit_wait_for_lane` 停车，无下一条待执行 RIGHT |

日志确认了一轮完整右转，没有第二轮右转触发证据。记录没有实体牌身份关联，不能把两次检测出现直接等同于两块实体牌各被确认。当前图片中没有牌也不能作为历史漏检证据。

## 本次修改

- 避障参数 `src/robot/obstacle/config.yaml`：`timed_bypass_right_s` 从 3.0 改为 4.0 秒。原日志实际右满舵约 3.064 秒即进入车道确认；新配置不足 4 秒不能因车道可见提前结束。上限仍为 6 秒，速度 RAW 26；停止时间不计入右转运动时长。达到下限后仍需车道确认，不能只靠计时证明回到原车道。
- 路牌诊断 `src/robot/signs/decisions.py`：同名牌已有 `pending` 时显示 `pending_already_stored`，替代容易误解的 `action_sign_ignored`。不改变缓存或蓝线执行条件。
- 路口 RIGHT 的右转段仍为 4.3 秒，和避障右转下限是不同参数。

## 非运动验证

在 Nano 的 Python 2 / ROS Melodic 环境执行，未启动实车或发布控制指令。

- 新增两个下限回归修改前均失败，修改后通过；覆盖日志的 3.064 秒交接点、3.9 秒仍保持右转、4.0 秒才允许转入确认。
- 避障回归 41 项通过，输出 `/tmp/bypass-minimum-four-tests.txt`（Nano）。
- 路牌/路由/右转回归 45 项通过，输出 `/tmp/right-retained-sign-tests.txt`（Nano）。
- 576 个源文件/配置归属及配置验证通过。

新下限仍需实际跑车验证回原车道的效果，不宣称实车已验证。启动示例（Nano，P3 侧方；完整流程从原起点开始）：

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_yolo_vehicle.sh parking_slot:=P3
```

# 避障右转执行满 6 秒

## 现场日志结论

来源：Nano `/home/nano/.ros/log/6626d80c-bddd-11f1-bae2-380025ec2b29/competition_controller-7.log`。

- 1790889440.213972：`bypass_right_6s`，输出 `[26, -0.03]`。
- 1790889442.586078：`bypass_visual_lane_handoff`，输出 `[30, -0.02386363636363636]`，右转只执行约 2.372106 秒。
- 1790889443.736803：`gap_limit_wait_for_lane`，输出 `[0, 0]`，提前交接后约 1.150725 秒因丢线停车。
- 本次上述停车由循线丢线保护触发。六秒参数未变，但视觉提前交接绕过了时长条件。

## 修改

- 移除视觉车道和估算航向提前结束右转的分支，只按 `timed_bypass_right_s` 完成右转。
- 默认右转速度仍为 RAW 26，满舵命令仍为 `-0.03`（编码 RAW -22），时长仍为 6 秒。
- 控制器安全停车及侧方临时净空调整期间暂停动作计时；恢复后继续剩余时长。
- 满六秒有可用车道才交回循线，日志为 `bypass_timed_lane_handoff`；没有可用车道仍停车等待。
- 修正文档中过时的“禁用避障碰撞保护”和“完成一次后关闭雷达停车”描述。此次没有改变这些保护逻辑。
- 循线算法、普通左右转、UTURN、启动脚本和它们的参数未在此次修改。

## 验证

- 修改生产代码前，新的十二项右转时长回归中，三项失败、三项因提前清除避障任务报错；见 `before_tests.txt`。
- Nano Python 2：右转时长、左右弧线入口、侧方净空、持续障碍保护共 30 项通过，无跳过；见 `final_tests.txt`。
- 模块归属与参数检查：`PASS 514 source/config files assigned; module overrides valid`；见 `module_check.txt`。
- 代码通过 SSHFS 直接保存至 Nano；本地和 Nano 的生产代码 SHA256 一致。
- 实车未由本次检查启动。六秒是控制器动作计时，未获得轮角到位或实际运动反馈来验证机械执行时间和赛道轨迹。

## Nano 运行命令

已有运行需先在原终端 Ctrl+C，然后重新启动以加载代码。

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_yolo_vehicle.sh
```

此命令默认纯跑，不记录。

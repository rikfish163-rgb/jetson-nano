# 默认纯跑与 UTURN 参数核对（2026-10-01）

## 修改

- 整车脚本默认 `record_lane:=false`，不启动巡线录制节点。
- 新增并传递 `sign_capture` 启动参数；整车脚本设置为 false，通过空的
  `capture_dir` 关闭路牌图片保存，路牌推理继续运行。
- full/stack 的 `sign_capture` 默认仍为 true，其他启动入口保持原有行为。
- 当前左右转修复、循线控制、UTURN 七段动作表和各速度参数未修改。

## Nano 上的验证

没有启动车辆节点或底盘执行器。

- 启动脚本 bash 语法和两个 launch 的 XML 解析通过。
- 对整车脚本的实际参数运行 ROS `--dump-params` 与 `--nodes`：录制节点
  不在启动列表，路牌 capture_dir 为空；显式开启两个参数可恢复保存。
- 实际合并配置为 max_steer=0.2，UTURN 前进和倒退专用 RAW 均为 20，
  七段定时模式启用。
- `test_uturn_production` 三项通过，覆盖入口、七段指令和完成后接回循线。
- 比较 max_steer=0.46275 / 0.2、循线 RAW=20 / 30、动作 RAW=20 / 24 / 30：
  UTURN 每段编码后的速度、舵量、持续时间完全相同。满舵仍为 RAW ±22。

结果分别保存在 launch_checks.txt、uturn_tests.txt 和
uturn_parameter_comparison.txt；本次差异保存在 changes.patch，原文件在 before/。

## 参数影响

当前七段执行器直接使用固定 RAW 指令与时间；普通循线/动作速度以及
max_steer 不改变这份动作表。max_steer 仍影响位姿估算和障碍物预测，
掉头完成后接管的循线使用当前循线速度。物理轮角没有重新测量，因此
软件结果不能替代实车掉头轨迹验证。没有依据提高 UTURN 专用速度；
如果日后提高专用速度，七段时长也需按实际运动重新标定。

## Nano 运行命令

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_yolo_vehicle.sh
```

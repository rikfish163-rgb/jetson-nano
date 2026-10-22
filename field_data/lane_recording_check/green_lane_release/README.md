# 1.25 m 直行后的巡线和转向执行核对

用户纠正后保留原要求：绿牌放行先直行 1.25 m，再接管巡线。
本轮没有修改 state_machine、巡线控制算法、配置或车辆运行入口。
目录名来自早先的排查假设，并不表示已改成绿牌立即巡线。

## 本次运行证据

来源：附件 882b0142-ac05-4455-bbd7-40eec11267c5/Pasted text.txt。
附件包含多趟运行；这里只分析 1790717812 之后的一趟。

| 时间戳 | 状态/底盘串口输出 |
| --- | --- |
| 1790717901.563623 | STARTUP_STRAIGHT，命令 [20,0] |
| 1790717910.964149 | LANE，tracking_lane，命令 [20,-0.003279] |
| 1790717910.924222 | chassis_output：速度12，舵量-2 |
| 1790717911.974204 | 速度12，舵量-3 |
| 1790717913.024190 | 速度12，舵量-4 |
| 1790717918.174237 | 速度12，舵量-12 |
| 1790717921.224244 | 速度12，舵量-15 |
| 1790717926.365364 | GAP，lane_unreliable，置信度0.312，速度0 |

上述底盘行均为 command_received=true、timed_out=false、serial_bytes=11。
这证明本机生成并写出了带转向值的指令，不证明 STM32 接受或舵机实际转动。
用户现场确认：前轮一直完全回正。结合此观察，需要验证串口之后的指令执行；
尚不能从这些日志判定是舵机死区、供电/结构、底层程序或协议执行问题。

## 无运动验证

- Nano 上 21 项测试通过，见 tests.log。
- test_startup_lane_handoff 验证起步 1.25 m 前零舵，达到距离后新鲜左右弯路径当周期产生对应非零舵量。
- serial_encoding_check.py 抽取实际 C++ writeSpeed，用内存串口代替硬件编译执行。
  6 组编码验证通过，见 serial_packets.json 和 serial_check.log；不连接 /dev/ttyACM0。
- 相机回放 camera_replay.json：使用本趟保存的稀疏路牌抓拍、当前标定与固定回放位姿。
  它不是完整相机帧序列，也不是修改后的实车轨迹验证。
  1790717912.1505487 帧可见有效右弯中心路径，前瞻0.8产生raw -4。
- roslaunch --nodes 验证转向测试仅含 bridge、base_controller 两个节点，见 probe_nodes.log；没有启动硬件节点。
- tools/steering_output_probe.sh 通过 bash -n；实际打角留给用户在 Nano 执行。

## 测试工具修复及现场下一步

bench_command.py 以前缺少 stamp，当前 require_stamp=true 桥接器会拒绝它。
bench_before.log 记录修复前真实 send 函数生成的 raw -3 指令被桥接为0。
补上当前 ROS 时间戳后，真实函数生成的正负打角、停止和 seq 回绕均通过桥接验证。
此工具不参与用户前一趟巡线运行，所以它不是那趟故障的根因。

先 Ctrl+C 结束原车辆程序，在 Nano 执行：

```bash
cd /home/nano/robocup_ws
bash tools/steering_output_probe.sh
```

脚本请求速度始终为0，依次测试 -3/-8/-15/-22/+3/+8/+15/+22，各2秒后回正，
结束后停止自建 ROS 运行程序。拒绝与已运行的底盘/控制器并行。
保存底盘状态和启动日志到 field_data/steering_probe_*。
观察是否执行、从哪档开始动作、左右是否一致；若静止全档不动，还须确认底层是否支持零速打角，
不能直接据此宣判机械故障。未在本轮自动执行任何真实打角或车辆移动。

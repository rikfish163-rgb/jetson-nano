# 绿牌＋巡线＋避障修复验证

## 当前恢复记录

用户要求按加绿牌和避障之前的巡线逻辑运行。已恢复最远点横偏、卡尔曼曲率
转角预判、右弯锁舵和原 GAP 预算；消息参数缓存和独立遥测修复保留。
绿牌后先零舵直行 1.25 m，再巡线；雷达避障开启。启动入口不变。
本次只在 Nano 检查原录制入弯和 1.25 m 后左右弯接管两项，均通过；
参数展开确认上述流程。见 ../green_lane_release/restored_logic_check.log、
restored_params.yaml、restored_logic.diff。未执行实车运动。
下面保留之前 Pure Pursuit 版本的验证记录，不代表当前转向策略。

## 已完成

- 预览模式使用测量中心路径的插值目标和 Pure Pursuit，不再叠加曲率转角或强制满右舵。卡尔曼曲率仍用于降速。
- 轻微置信度波动允许最多 0.25 秒降速保持可信角度；不刷新 GAP 预算。空路径最多 0.35 秒；过期相机、严重低置信度仍停车。
- enabled 参数改为启动时订阅缓存；状态/轨迹发布独立计时器，发布不持有控制锁。底盘和桥接超时仍为 0.25 秒。
- 新 Nano 入口 run_lane_green_obstacle.sh 打开绿牌和雷达避障、关闭泊车/后摄像头/JPEG 录制/图像查看。lookahead=0.8 m，直道 raw20、弯道上限 raw12。

## 证据

- 修改前 27 项测试：6 项失败、2 项错误，复现锁舵、偏内不能纠正、轻微置信度即停车以及控制周期中的阻塞调用。
- Nano 定向测试 75 项通过，另加避障完成后恢复连续巡线的回归。见 tests_targeted.log 和 tests_bypass.log。
- Nano 独立 ROS master 11329，真实 ROS 发布订阅，控制输出仅 /competition/control_preview；未启动相机、雷达驱动、桥接或底盘。
- 人为让状态输出每次阻塞 350 ms，350 条控制指令最大间隔 120.512 ms，95% 分位 59.050 ms，无超过 250 ms 的间隔。绿牌前零速、绿牌后起步、障碍零速、无效扫描零速、参数禁用零速、恢复巡线均验证。
- 启动参数展开：wait_green=true、lidar_enabled=true、timed_bypass_enabled=true、parking_enabled=false。节点列表只有前摄像头、巡线、雷达、地面检测、路牌、中央控制、桥接和底盘。
- module_workspace.py check 468 个文件归属通过。只是静态展开，没有启动实车节点。
- 更宽的旧测试还存在 3 项预先失败：旧起步接管舵量断言、旧方向牌单帧断言、旧泊车牌断言；用本次修改前源码在内存重跑，仍全部失败，见 preexisting_failures_baseline.log。没有宣称全仓库测试通过。

## 实车范围

消息验证使用合成视觉/扫描，未同时运行 TensorRT 与真实相机，也未测量实际舵机角、轮速或车道中心偏差。因此仍需实车确认压线与停顿是否消失。
绿牌后保持原 1.25 m 起步直行，起点需要在直道上。

## Nano 完整启动入口

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_lane_green_obstacle.sh
```

脚本自身加载 ROS 和工作区环境，设置 localhost ROS 网络，检查 TensorRT 引擎以及重复摄像头/控制器，再执行完整 roslaunch。

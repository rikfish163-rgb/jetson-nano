# P4 独立正入库测试

前轴由操作员对齐蓝线，车头朝向 P4 库内。程序不识别绿牌、停车牌或车位编号，
也不做自动避障。需手动选好 P4，前摄安装位置与现有标定保持一致。

## 流程

1. 相机/底盘就绪后零速倒计时 3 秒。
2. 速度 30、转角 0，前进 2 秒。
3. 停车寻找两条库位侧线；连续 3 张新鲜图像确认平行、宽度和方向后继续。
   此时不要求看到库底线。10 秒未确认两条边线则停车退出。
4. SIDE_ALIGN：只根据两侧线中心及方向，以速度 15 纠偏前进，转角限制 ±22。
   最长 15 秒还没有确认库底线则停车退出；边线丢失或目标跳到邻库立即停车。
   库底实线连续 3 张新鲜图像确认后切换 ALIGN，按 bottom_weight（默认 0.3）
   将库底线中点加入纠偏，速度仍为 15。未确认底线短暂出现又消失时，继续边线引导。
   使用当前 competition.yaml 的前摄 K/D/H，在标定地面平面上判断平行/垂直，
   不把透视图中的像素线角度直接当作实际车身方向。
5. 库底中点逐渐接近去畸变图像下沿（默认行比例 ≥0.86、至少 2 帧，且相比
   初次对齐观测向下移动至少 2 像素）后，才允许判断正常出画。
   初次确认侧线需长 ≥0.12m，临近下沿允许跟踪 ≥0.025m 的短侧线。
   若侧线先出画而库底线仍可见，仅在最近 0.5 秒有真实侧线的条件下继续观察
   实际库底线，保持最后有效舵角、速度 15；不会把侧线消失当作库底线消失。
6. 首次库底线消失立即停车，锁定最后有效舵角；连续至少 3 帧且持续 0.2 秒
   确认消失后，以速度 30、锁定舵角前进 1 秒，最后停车退出。
   库底线与两侧线可以一起正常出画；需有此前稳定几何及接近下沿的证据。
   确认期间库底线重新出现则停车退出，不自动再次前进。

中途目标突然换位、提前丢失库底线、相机/底盘异常、命令冲突、控制循环超时
或急停都会终止并发零速。最后一段以时间控制，没有厘米级定位保证。

## 启动（Nano 两个终端）

先在原终端 Ctrl+C 停止整车、旧停车测试和遥控节点。每次只能运行一个控制测试。

终端一：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
roslaunch robocup_competition p4_forward_test.launch
```

终端二（摆好车后运行，每次只执行一次）：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
python2 tools/p4_forward_test.py --execute
```

空格、x、q、Esc、Ctrl+C 停车。终端一只启动相机/底盘/测试桥，
终端二倒计时结束后才发非零速度。终端一可以保留，每次重新摆车再运行终端二。

只观察图像、不运行运动逻辑：将终端二命令改为
`python2 tools/p4_forward_test.py --observe`。
不加 `--execute` 或 `--observe` 只打印参数，不创建 ROS 节点、不发命令。

## 参数与观察

独立参数文件：`src/robot/config/p4_forward_test.yaml`，下一次启动生效。
不修改整车 `vehicle_tuning.yaml`；本测试速度/舵角/时间全部来自独立文件。
不接受整车或其他测试的 `--speed` 等参数，避免参数覆盖关系不明确。

- `prepare_speed_raw / prepare_steering_raw / prepare_s`：起步段 30 / 0 / 2 秒。
- `align_speed_raw`：纠偏段速度 15。
- `side_align_timeout_s`：边线引导最长前进时间，默认 15 秒。
- `bottom_seen_frames`：切换底线纠偏前的连续确认帧数，默认 3。
- `position_gain / heading_gain / lookahead_m / bottom_weight`：横向与方向纠偏。
- `final_speed_raw / final_s`：最后一段 30 / 1 秒；舵角固定为丢线前的值。
- `bottom_near_row`：库底线接近图像下沿的阈值，默认 0.86。
- `bay_width_m / width_tolerance_m / white_v_min`：库宽、容差、白线亮度。
- `seen_frames / lost_frames / lost_s`：识别与消失确认。

状态：`/parking_p4_forward/status`；识别鸟瞰图：`/parking_line_stop/debug`。
日志给出库底中点（米）、中心线横向偏差、方向偏差、库底图像行比例、确认票数。
本测试复用 parking_line_stop_test 的前台生命周期和命令独占检查、测试桥话题，
但不使用其巡线、侧方停车或旧 P1 试验投影。

不动车验证：

```bash
cd /home/nano/robocup_ws
python2 tools/test_p4_forward.py -v
python2 tools/p4_forward_test.py
```

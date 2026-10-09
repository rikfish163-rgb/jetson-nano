# 先对齐蓝线、固定画面位置触发、固定时间直行的停车测试

脚本：blue_stop_test.py；参数：blue_stop_test.yaml；启动：blue_stop_test.launch。
放车位置在蓝线前方，车头朝蓝线，可有少量倾斜。
无需测量起步距离，但需给车留下行进对齐空间，蓝线须在配置的触发位置之前。
此参数文件现为测试与正式 LEFT/RIGHT/STRAIGHT/UTURN 共用；PARKING不使用。
修改参数会同时改变四个正式动作的接近流程。

整车入口额外在方向路牌确认后就将循迹速度限制到 align_speed_raw。
第一帧合格的蓝线候选出现时便开始对齐，与路口蓝线的3帧确认同时进行；
正式接近沿用该期间的对齐结果。独立测试不识别路牌，本来就在第一帧蓝线时开始对齐。

程序复用现有蓝色检测，将检测线还原到俯视图的图像行坐标，使用蓝线与
中央竖线的交点高度触发，不按前轴距离计算停车时间。
俯视图话题为 /competition/debug/front_bev，不是原始相机画面。
1. 看见蓝线后，低速前进并根据蓝线方向修正舵机。
2. 远处以 align_tolerance_deg（当前10度）为目标修正方向。
3. 到70%～90%触发带后，角度允许放宽到15度，连续3帧确认后开始计时。
4. 舵机回正、raw 20直行1秒后停车。正式程序等待后执行对应动作；测试保持停车。

共用参数：align_speed_raw（对齐速度，当前30）、align_max_steering_raw（最大修正量22）、
align_tolerance_deg（远处对齐目标10度）、align_confirm_frames（连续确认3帧）、
align_timeout_s（对齐超时8秒）。参数以 YAML 实际值为准。
heading_tolerance_deg=15 是触发区接受的角度范围；超过时回正停车，
最多等待 align_recheck_s=1 秒重新确认。期间保持零速度、零转向，不计直行时间。
只使用复核停车之后的新图像，角度与位置连续确认都满足才重新前进。
复核超时、超出触发带、图像断流或障碍保护仍会锁定停车。
计时前蓝线丢失则停车；计时后允许蓝线离开画面，但相机必须持续更新。

先退出其他车辆程序。Nano 终端一：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source /home/nano/robocup_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
roslaunch robocup_competition blue_stop_test.launch live:=true
```

Nano 终端二，准备好后使能（无倒计时；相机和雷达就绪即运动）：

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
rosparam set /competition_controller/enabled true
```

急停：

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
rostopic pub -1 /competition/estop std_msgs/Bool 'data: true'
```

每次结束都保持停车，退出 launch 后重新摆车、启动。
先固定 speed_raw、trigger_row_ratio、confirm_frames，再调整 forward_seconds。
只有日志显示计时已完成（blue_test_complete）后，停在线前才增加时间，越线才减少时间。
如果显示 FAULT 或对齐不足，则直行计时可能根本没开始，不应通过增加时间处理。
改变相机安装、触发位置或速度后须重新调时间。
当前配置为 raw 20、触发后1秒；触发容差放宽后的停车位置仍需复测。
允许启动参数覆盖，例如终端一最后一行使用：

```bash
roslaunch robocup_competition blue_stop_test.launch live:=true speed_raw:=12 trigger_row_ratio:=0.65 forward_seconds:=1.5
```

控制台 image 字段显示 phase、row_ratio、heading_deg、align_frames、aligned 和触发后的 elapsed_s。
blue_test_past_trigger_band：已越过触发带，重新摆远，或确认带过窄/相机太慢。
blue_test_alignment_distance_insufficient：到触发位置仍未完成对齐，请摆远些或调整对齐参数。
blue_test_alignment_timeout：对齐未在规定时间完成。
blue_test_alignment_recheck_timeout：触发区短暂停车后，未在复核时间内通过新图像确认。
blue_test_blue_lost_before_trigger：计时前丢失蓝线，停车，避免沿用旧方向。
blue_test_sensor_lost / blue_test_guard：相机、雷达或碰撞保护停止，锁存不自动重启。
blue_test_search_timeout：达到未触发的搜索时间上限。

离线测试（无车辆运动）：

```bash
cd /home/nano/robocup_ws
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src:src/ros/lane/src:src/robot/test python2 -m unittest test_blue_stop_test test_blue_timed_production test_blue_prealign
```

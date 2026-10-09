# P1 / P2 / P3 normal launch

The saved one-shot tests remain in tools and are unchanged by this integration.
Normal full/stack launch selects `parking_mode=timed_sequence` when an explicit
`parking_slot=P1`, `P2` or `P3` is supplied. AUTO/P4/P5 retain their prior mode.
An explicit `parking_mode` launch override still takes precedence.

Stop any standalone parking-test launch and test process before starting full.
Run on Nano:

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
roslaunch robocup_competition full.launch live:=true enabled:=true start_actuators:=true parking_slot:=P3
```

Replace only P3 with P1 or P2. Green-start and the existing route gate remain:
parking signs cannot arm during WAIT_GREEN, the initial startup straight, or
before the first completed route action. In normal lane driving, three fresh
accepted PARKING observations latch parking. This mode dispatches immediately
after confirmation and does not wait for the parking blue line.

1. Automatic road lane steering at raw speed 30, without steering trim.
2. First see bay transverse lines in two fresh frames.
3. First same-frame ZERO bay lines + BOTH lane boundaries curved the same way
   brakes immediately. While stopped, require two matching fresh frames over
   0.1 seconds; hold zero for at least 0.5 seconds.
4. Perform the selected five stages. Timing starts when each stage's outgoing
   command is first selected; a delayed cycle cannot skip the next stage.
5. Stay stopped after completion. An ambiguous end remains stopped and never
   starts the timed maneuver.

| Stage | Raw speed | Raw steering | P1 seconds | P2 seconds | P3 seconds |
|---|---:|---:|---:|---:|---:|
| 1 | P1/P2: +30; P3: -30 | 0 | 4.0 | 1.2 | 1.5 |
| 2 | -30 | +22 | 0.2 | 0.2 | 0.2 |
| 3 | -30 | -22 | 3.4 | 3.4 | 3.4 |
| 4 | -30 | 0 | 0.1 | 0.1 | 0.1 |
| 5 | -30 | +22 | 2.1 | 2.1 | 2.1 |

Settings: `src/robot/config/maneuvers.yaml`, key `parking_timed`, including
`sequences/P1`, `sequences/P2`, `sequences/P3`. Steering is in protocol units.
Selection is manual: this mode does not identify a numbered bay or verify its
occupancy/final pose; it reproduces the saved tests. Normal lidar collision
checks still apply, without inserting a bypass during parking. Fixed stages
use the same 0.08 m near-field sweep cap as existing parking entry.

Only the normal controller publishes motor commands. The additional
`parking_timed_vision` node publishes `/competition/parking_timed_scene`, pairing
raw front images with already-computed lane paths/boundaries by exact capture
timestamp. It avoids running a second lane detector. The debug image topic is
`/competition/debug/parking_timed`; status is in `/competition/status` under
`parking_timed`. Production scene reception, frame pairing and motion use the
shared `ground_timeout` camera budget (currently 1.25 s); there is no additional
0.8 s parking timeout. Estop/red,
missing lidar, a collision stop, disabled automatic output or a control fault
abort and latch zero; restart the run after correcting the cause.

Offline checks (no motion):

```bash
PYTHONPATH=src:$PYTHONPATH python2 -m unittest discover -s src/robot/test -p test_timed_parking.py -v
roslaunch --dump-params robocup_competition full.launch parking_slot:=P3
```

# Single U-turn action test

Start with the front axle on the blue line and the vehicle aligned by the operator.
Stop other vehicle/controller/teleoperation launches before starting this test.
This test executes the independent YAML once, then stops. It does not run cameras,
recognize signs/blue lines, avoid obstacles with lidar, or resume lane following.

Terminal 1 on Nano (keep running):

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
roslaunch robocup_competition uturn_test_actuators.launch
```

Terminal 2 on Nano, after placing the vehicle:

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
export ROS_IP=127.0.0.1
python2 tools/uturn_action_test.py --execute
```

Edit `tools/uturn_action_test.yaml` with `nano` to save independent tuning.
The file is reloaded each invocation; the bridge does not need restarting.
Default movement rows were copied from current production tuning (five segments).
They do not track later production changes, and edits do not modify production.
Positive RAW speed is forward; negative is reverse. Steering is positive left,
negative right; integer range -22..22. Movement speed magnitude is 1..30.
Seconds must be >0 and <=10. Pause is 0.3..3 seconds; countdown is 2..10 seconds.
The production TimedUturn duration bound still applies.

Temporary overrides, without changing YAML:

```bash
python2 tools/uturn_action_test.py --execute --s2-seconds 3.0 --s4-seconds 3.8
python2 tools/uturn_action_test.py --execute --stop-after 2
```

Omit `--execute` for an offline preview with no ROS node or commands.
Space, q, x, Escape or Ctrl+C stops and exits the test.
Replace the vehicle at the start position before every new invocation.
Dedicated ownership, base status, disconnect, emergency stop and command watchdog
checks are retained from the established one-shot runner.

Offline validation: `PYTHONDONTWRITEBYTECODE=1 python2 tools/test_uturn_action.py`.

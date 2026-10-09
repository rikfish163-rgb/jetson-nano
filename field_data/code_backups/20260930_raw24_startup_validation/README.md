# Raw 24 startup validation repair

User log 02b95622 shows ground, sign, and controller nodes exiting because
lane_curve_speed_raw=24 was rejected by the obsolete upper bound of 20.
contracts.py in this directory is the pre-edit backup.

The running validator now bounds curve speed by speed_raw_limit, preserving
positive integer and finite number validation. No lane tracking or bypass
behavior was changed in this repair.

Verified over SSH on Nano:
- Before repair: test_lane_speed_validation failed at speed 24 with the same
  exception as the user log.
- After repair: all 3 focused tests passed.
- Current run_lane_green_obstacle.sh --dump-params configuration passed
  validate_config: lane=24, curve=24, straight=24, action=24.
- check_shadow_startup.py initialized the actual ROS controller and ran its
  timers on an isolated ROS master with live=false. Only the preview output
  was registered; no /control/cmd or /ackermann_cmd publishers existed.

Physical driving was not started. Restart on Nano:

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_lane_green_obstacle.sh
```

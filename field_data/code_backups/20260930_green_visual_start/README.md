# Green release to camera lane control

The user observed physically overshooting the estimated 1.25 m startup.
Read-only Nano inspection found /odom has no publishers. Base serial code
writes commands and does not read encoder feedback. Controller command_model
integrates acknowledged command speed with raw_to_mps.forward=0.008 and caps
each integration dt at 0.10 s. This is an estimate, not a physical distance
measurement; no guarantee of actual 1.25 m is possible from these inputs.

The current run_lane_green_obstacle.sh enables startup_follow_lane=true.
After GREEN, begin_startup enters LANE directly and clears startup/gap state.
Camera center geometry sets steering immediately at lane speed 12. Curvature
detection and command pose reaching 1.25 m do not gate steering. Missing or
unreliable initial lane input cannot command blind forward motion. Existing
red/estop, lane freshness, and obstacle scan/collision checks remain active.
Avoidance still uses action speed 24 with the existing 2 s left / 6 s right.

The new boolean is validated and passed through full.launch and stack.launch.
The default is false for entry points explicitly using distance-based startup.
The current vehicle wrapper overrides it to true. Signed STRAIGHT maneuvers
retain their existing distance setting. No lane steering algorithm changes.

Verified over SSH on Nano, without vehicle command publishers:
- New tests reproduced 4 failures before implementation.
- 14 targeted visual startup, distance-start compatibility, and continuous
  obstacle tests passed after implementation.
- module_workspace.py check: PASS 475 source/config files assigned.
- Current common + wrapper arguments were parsed and roslaunch --dump-params
  was invoked only to inspect configuration, without starting vehicle nodes.
  validate_config passed; green visual startup=true, lane/curve12, bypass24.
- With actual launch configuration (only lidar input disabled for pure core
  simulation), GREEN produced speed12 and steering with the correct sign for
  left and right paths at estimated x=0. WAIT_GREEN produced speed0.
- Actual ROS controller initialized and timers ran on an isolated ROS master
  with live=false, no /control/cmd or /ackermann_cmd publisher registered.
- Shell syntax passed. Code changes reviewed against pre-edit backup files.

The physical vehicle was not moved. Camera accuracy, steering calibration,
traction, and actual turn tracking still require field verification. Existing
vehicle processes must be restarted by the user to use the new mode.

# Startup 24 -> lane 12

The former entry retained lane_speed_raw=24 and only reduced it to 12 when
filtered curvature exceeded 0.4. This did not guarantee slowing at 1.25 m.

Changed the vehicle entry to lane_speed_raw=12. Startup now reads
straight_speed_raw=24 separately. Lane gap commands are bounded by the lane
speed, preventing the default gap speed 16 from accelerating after handoff.
Avoidance action speed remains 24. No steering algorithm changes.

Nano verification over SSH, without vehicle commands:
- The new handoff regression initially failed because startup used lane speed.
- 13 targeted lane preview and startup handoff tests passed after repair.
- Expanded startup suite: 23 passed, one pre-existing steering sign assertion
  failed. Replacing startup_tick with its saved pre-change implementation
  reproduced that same negative-steer assertion failure. Not fixed here.
- Parsed both current entry scripts into roslaunch arguments and dumped
  parameters without starting nodes (the normal entry refused a duplicate
  launch because the user's camera/controller was already running).
- Actual arguments passed validate_config. Direct core input simulation gave:
  0.00 m startup 24; 1.24 m startup 24; 1.25 m lane 12;
  1.30 m straight-looking lane 12; 1.35 m short gap 12; 1.40 m curve lane 12.
- Vehicle motion was not started. Restart the user's running process to apply.

The 1.25 m threshold remains based on the existing command pose estimate in
command_model mode. Physical distance and lane tracking still need field
verification.

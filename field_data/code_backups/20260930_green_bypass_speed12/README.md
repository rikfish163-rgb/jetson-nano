# Remove startup bypass speed jump

User log f101d2a4:
- 1790723557: lane GAP command12, steering0.
- 1790723558: raw collision sweep stopped on points near x=0.304 m.
- 1790723559: TIMED_BYPASS SETTLE_LEFT trigger at approximately x=0.253 m.
- 1790723559.535 onwards: chassis output24, steering22, then -22.
- 1790723567.685: lane speed12 after bypass.

This was the preserved action_speed_raw=24, not lane speed exceeding its 12
setting. Logs do not identify the physical object producing the scan return.

Only the current low-speed vehicle wrapper changed: action_speed_raw and
straight_speed_raw are now12 alongside lane/curve12. Green visual startup
remains enabled. Bypass durations stay left2s/right6s; a lower speed therefore
changes their physical travel and requires field verification.

Verified on Nano over SSH without vehicle commands:
- bash syntax passed.
- Parsed actual current common/wrapper launch arguments; roslaunch parameter
  dump and validate_config passed with all four speed settings12.
- Pure core with fresh synthetic lane/scan inputs completed bypass. Outputs:
  LANE (12,0), SETTLE_LEFT (0,22), LEFT (12,22), RIGHT (12,-22), then LANE.
  All 200 command ticks had speed0..12. No physical vehicle test performed.
- Reviewed the change against the wrapper backup in this directory.

The running user's process must be restarted to apply the new parameters.

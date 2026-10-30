# Phase 4: follower diagnosis and check runs

- `diag_before_fix_v2a2_timeseries.csv`: old time-based tracker, 2.0 m/s,
  2 m/s^2. Per control step: reference, estimate, raw/shaped command, wheel
  commands and speeds, ground truth, map->odom. Shows the reference running
  ahead (0.36 m lag) and the position feedback pushing the command to 3.35 m/s
  (true 2.4 m/s), then 0.26 m overshoot at the zigzag corner -> wall contact.
- `diag_after_fix_v2a2_timeseries.csv`: new follower (shortcut + arcs, speed
  plan, projection-based feed-forward, capped feedback), same settings.
- `check_runs.csv`: one run each at 1.0 m/s and 2.0 m/s (2 m/s^2), nominal
  start, `mission_trial` columns incl. overshoot and settling time per stop/turn.

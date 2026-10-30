# Phase 4: path clearance margin and return turn mode

2.0 m/s, 2 m/s^2, lateral 2 m/s^2, nominal start, one run each:
path_clearance_margin 0.06 / 0.10 / 0.15 m with the 90 -> 180 deg turn in
place, and 0.10 m with the turn done while moving.

- `runs.csv`: mission_trial rows (times, stop errors, overshoot, settling,
  localization error peaks, min wall clearance).
- `timeseries_*.csv`: per control step control pose (what the mission used)
  vs ground truth interpolated at the same time.

Waypoints were moved to the middle of the free space before these runs
(graph clearance >= 0.22 m except leaving area 1 from the fixed start, 0.10 m).

# local/ — path planner + robot drivers

## Files
- `setup_robots.py` — interactive BLE pairing. Run once per session, writes addresses into `robot_info/run_config_<RID>.json`.
- `compute_path.py` — path planner. Reads `ROBOTS` dict at the top, writes `path_<RID>.csv`, `path_<RID>_robot.csv`, `run_config_<RID>.json`.
- `run_robot.py` — single-robot driver. `ROBOT_ID` defaults to `"A"`, override with env var.
- `run_both.py` — drives A + B together. Both finish, process exits cleanly (exit 0).
- `interactivepage.html` + `style.css` — old browser UI (Pyodide). Superseded by the website at `tangle-7f181.web.app`.

## Workflow (per session)
1. `python3 setup_robots.py` — assign A / B / skip. just run this once so the bluetooth got setup
2. `python3 compute_path.py` — run this to get the path (or use the interactive.html website by hitt the apply and build button).
3. `python3 run_both.py` (both paired) or `python3 run_robot.py` to test one (on the top you can change A or B to test differnt one).

## What's new — please test tomorrow
**Transit segmentation.** Long straight pulls were drifting because the robot works bad over a single huge `navigate_to`. Now `compute_path.py` slices long transit lines into chunks so the robot re-aims periodically.

Change the MAX_TRANSIT_SEGMENT_CM  around the top of the `compute_path.py` to test different parameter
During the test on Thursday we test on 1,3,2 and it bumped into 2 so make sure this got test, maybe also try 1,3,2,4 to see how it preform on long distant 
```python
MAX_TRANSIT_SEGMENT_CM = 50 
#   0  -> no segmenting (old behavior)
#   50 -> default; start here
#   smaller -> tighter tracking, slower
#   larger  -> faster, more drift
```

## Outputs (regenerated; do not hand-edit)
- `robot_info/path_<RID>.csv` — full waypoints, world frame.
- `robot_info/path_<RID>_robot.csv` — reduced waypoints, robot's local frame (driver loads this).
- `robot_info/run_config_<RID>.json` — direction, lights, BLE address, etc.

## SDKs
- `irobot-edu-python-sdk/` — iRobot Edu SDK (also on PyPI: `pip install irobot_edu_sdk`).

# RobotFinal

Path planner and driver for two iRobot Create 3 robots (A and B) that circle a set of poles, with optional Vicon pose correction.

## Files

**Planner**
- `compute_path.py` — single source of truth for world config, pole layout, per-robot constants, and path generation. Produces `path_<RID>.csv`, `path_<RID>_robot.csv`, and `run_config_<RID>.json`.
- `interactivepage.html` + `style.css` — browser UI that runs `compute_path.py` via Pyodide when you press **Apply & Build**.

**Outputs (regenerated; do not hand-edit)**
- `path_A.csv`, `path_B.csv` — full waypoints in world frame (for debugging).
- `path_A_robot.csv`, `path_B_robot.csv` — reduced waypoints in the robot's local frame (what the driver loads).
- `run_config_A.json`, `run_config_B.json` — per-robot constants (direction, arc radius, lights, Bluetooth address, etc.).

**Drivers**
- `get_name.py` — BLE scan for nearby Create 3 robots, prints name + Bluetooth address.
- `test_connection.py` / `test_2_connection.py` — connect to one/both robots, flash the configured color, and play 3 notes to verify the Bluetooth address in `run_config_<RID>.json` points at the right robot.
- `run_robot_noVicon.py` — pure-odometry driver, no Vicon dependency. Known-good baseline.
- `run_robot.py` — Vicon-corrected driver (between-wrap pose correction).
- `run_robot.sh` — launcher for `run_robot.py` on macOS (sets `DYLD_LIBRARY_PATH` so the Vicon dylibs load).
- `mock_robot.py` — fake Create 3 used when `MOCK_ROBOT = True` in `run_robot.py`.

**Config / SDKs / misc**
- `vicon_config.json` — Vicon host + subject names (hand-maintained).
- `irobot-edu-python-sdk/` — iRobot Edu Python SDK.
- `vicon-py-sdk/` — Vicon DataStream Python SDK.
- `sdk_commands.pdf` — SDK reference.
- `legacy_version/` — older scripts kept for reference.

## Quick start (no Vicon)

1. **Find the robots.** Run `python get_name.py` to list every Create 3 in range with its Bluetooth address. Paste the addresses into the `bluetooth_addresses` dict at the top of `compute_path.py`.
2. **Build the paths.** Either run `python compute_path.py`, or open `interactivepage.html` in a browser and press **Apply & Build** (the page runs `compute_path.py` under the hood).
3. **Test the connection.** Run `python test_connection.py A` (and `B`). The correct robot should light up green (A) or blue (B) and play 3 notes. If the wrong robot lights up, the address in `compute_path.py` is wrong — fix it and rebuild.
4. **Drive.** Once the addresses are verified, run `python run_robot_noVicon.py` (set `ROBOT_ID` at the top of the file to pick A or B).

## Adding Vicon

Follow the Vicon setup link Souju posted in the GitHub repo. If the build fails, delete `vicon-py-sdk/` and re-clone the repo for a clean setup. Once Vicon is working, use `./run_robot.sh` to launch `run_robot.py`.

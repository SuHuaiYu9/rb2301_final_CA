# Claude's test tools (2D simulator and test scripts)

These tools let Claude run the user's `obstacle_course.py` offline, without ROS or Gazebo, to answer "why" questions with evidence and to check code for crashes. **They are not part of the user's project.** They live only on the Claude branch, and the user's own code is never changed.

## `sim/sim2d.py`: the 2D course
`World(np.random.default_rng(seed))` builds a random layout with the same shape as the Gazebo course:
- a start corridor; a step (jog) down towards −y; a second corridor;
- a door wall with a 0.30–0.60 m gap;
- two swinging thin boxes past the door;
- 2–8 static obstacles.

The robot starts at (0, 0) facing +x, as in Gazebo.

| Part | What it gives |
|---|---|
| `w.lidar(x, y, th, t, n=360)` | ranges like the Gazebo LiDAR: mounted 0.114 m ahead of the pose point, index i ≈ i degrees anticlockwise from the front, `inf` = no hit, 0.05–10 m |
| `w.collides(x, y, th, t)` | `'static'`, `'swing'` or `None`, using the nanocar footprint (`ROBOT_BODY`) |
| `w.swingers` | list of dicts (`x`, `hx`, `hy`, `lo`, `hi`, `half`, ...). Set it to `[]` to remove the swinging obstacles |
| `w.swinger_y(s, t)` | where swinging obstacle `s` is (its y) at time `t` |
| `w.door` | `(x, y_lo, y_hi)` of the door gap |
| `w.geom` | dict of course measurements: `xb`, `x1e`, `xj`, `c2t`, `c2b`, `xd`, `xs`, `xe`, `w1` |

- Some seeds can't build a layout and raise `RuntimeError`. Seeds 1 and 6 fail; 0, 2, 3, 4, 5, 7, 8 and 9–13 work.
- The 2D layouts vary more than the user's Gazebo world: y can go down to about −4.1. So scripts that depend on the map **override the node's grid to cover the layout**: `node.origin`, `node.grid_n_x`, `node.grid_n_y` and `node.occupancy_grid`. This is test setup only.

## `harness/`: running the user's node
Every script:
1. stubs out the `rclpy` and message modules;
2. imports `obstacle_course` from `<code_dir>` and builds `ObstacleCourseNode(is_simulation=...)`;
3. runs ticks of 0.05 s. Every second tick it feeds a scan through `sub_scan_callback`. Every tick it sets `node.pose = (x, y, heading in degrees)`, calls `timer_callback()`, captures the published `Twist`, and moves the robot by that command, read in the robot's frame (as Gazebo does).

`<code_dir>` is the user's package folder, for example `<clone>/src/rb2301_fp/rb2301_fp`. `<sim_dir>` is `claude_tools/sim`.

| Script | Arguments | What it reports |
|---|---|---|
| `drive_user_node.py` | `<code_dir> <sim_dir> <secs>` | Crash check in Gazebo mode, seed 0 |
| `drive_timed.py` | `<code_dir> <sim_dir> <secs> [seed] [noswing]` | Same, plus tick times, counts of each log message, and the final pose |
| `real_mode_run_phi.py`, `real_timed.py` | `<code_dir> <sim_dir> <secs> <phi_deg>` | Real-robot mode: 720-reading scans, and maze-frame poses starting at (2.82, 2.94), with the course rotated by phi. Needs `optitrack_variables.config` next to `obstacle_course.py` (it's in the user's package folder) |
| `target_study.py` | `<code_dir> <sim_dir> <secs> <seed> <frontier\|nearest_free>` | Runs until the robot passes the door, gets stuck (moves under 5 cm in 15 s) or times out. Prints contacts and the last target |
| `diff_study.py`, `diff_study2.py` | `<code_dir> <sim_dir> <seed>` | Cells that change between consecutive grids, on static walls vs swinging obstacles, with the robot still vs moving |
| `blob_track.py` | `<code_dir> <sim_dir> <seed>` | Speed of each swinging obstacle estimated from the centre of its occupied cells, from a standing robot |

**`target_study.py` needs some setup:**
- It sets `node.global_goal` past the swinging obstacles.
- It expects a copy of the code where the door threshold is replaced by a module variable `DOOR_X`, which the script sets past the door. Make that copy with:
  ```bash
  mkdir -p "$SP/tv" && sed 's/if cx>=self.door_x:/if cx>=DOOR_X:/' <code_dir>/obstacle_course.py > "$SP/tv/obstacle_course.py"
  sed -i '0,/^import numpy as np/s//import numpy as np\nDOOR_X = 99.0/' "$SP/tv/obstacle_course.py"
  ```
- A reactive version with no door x or goal won't need this, so adapt the script instead.

Example:
```bash
cd claude_tools/harness && python3 drive_timed.py "$SP/user_repo/src/rb2301_fp/rb2301_fp" ../sim 30 0
```

Claude's own navigator (`course_navigator.py` / `nav.py`) is deliberately **not** included. The user rejected it, so never suggest it.

# Claude session context: RB2301 final project (read this first after a context reset)

Saved 2026-11-03, updated 2026-11-04, so work can continue seamlessly after context compaction. This lives on branch
`claude/final-project-review-3m8r3v` only. The user's own code is on `main`, which Claude must never change.

## 0. First steps after a context reset
1. **Read this whole file, then the report log** `/home/user/rb2301_final_CA/rb2301_final_report_log.md`, so you know every logged entry.
2. **Bring the clone of the user's code up to date:**
   ```bash
   SP=/tmp/claude-0/-home-user-rb2301-final-CA/be22f863-093c-51e5-b646-f1aab5ca3d5f/scratchpad
   cd "$SP/user_repo" && git fetch -q --depth 10 origin main && git checkout -q FETCH_HEAD && git log --oneline -3
   ```
   If the scratchpad is gone (environment reset): `git clone --depth 10 https://github.com/SuHuaiYu9/rb2301_final_CA.git "$SP/user_repo"`.
   If the 2D simulator and harness in `/tmp/fp_work` and `$SP/harness` are gone, they can't be recovered. Say so, and rebuild them only if an investigation needs them.
3. **Re-read section 1 (ground rules)** before answering. The most common way to upset this user is giving advice they didn't ask for.
4. **Continue from section 2, "Last thing discussed".**

## 0b. How to update the report log (whenever the user says "log this")
- **File:** `/home/user/rb2301_final_CA/rb2301_final_report_log.md`, on branch `claude/final-project-review-3m8r3v`.
- **Entry types:**
  - `D#` = a decision the user made;
  - `M#` = a problem or mistake observed;
  - `R#` = a real-robot run;
  - `E#` = an error or crash;
  - `S#` = a milestone or success.
  Next free numbers: **D12, M4, R2, E2, S2**.
- **Fields:** Decision/Observed, Why, Potential errors (only if they ask), Observed, Fix. Leave Fix blank unless they say what they did. Add a "Later:" line to an older entry when a value changes, as with D5.
- **Record the user's own remarks for the report faithfully,** as with R1's "Remark for the report".
- **Commit and push:** `git add rb2301_final_report_log.md && git commit -m "Log ..."` with the Co-Authored-By and Claude-Session trailer lines, then `git push origin claude/final-project-review-3m8r3v`. Push access works.
- **Also offer to log** things that are clearly report-worthy, but only offer; don't log without being asked. Currently offered and unanswered: the no-path investigation results (section 5).

---

## 1. Ground rules from the user (follow strictly)
1. **The user writes the project themselves.** Claude's only job is to translate the user's ideas and algorithms into correct, efficient Python when asked.
2. **Do NOT correct their high-level logic, algorithm choices, edge cases or drift** unless they explicitly ask (for example "why does X happen?", "how should I fix X?", "investigate X"). They want to make mistakes and find them, so they have material for their report.
3. **DO flag code-level problems:** syntax errors, crashes, and code that doesn't do what they described. These count as translation errors.
4. **One exception:** if something on the real robot could hurt someone or damage hardware, say so once.
5. **Never suggest** heading hold, or rotating world-frame velocity into the robot frame. D1 and D2 are deliberate learning choices.
6. **When the user is just "updating" you on a plan,** acknowledge briefly and don't advise. They got angry about unsolicited advice.
7. **Never modify their codebase or `main`.** Claude maintains only `rb2301_final_report_log.md` (and this file) on the Claude branch. Log only what they ask to log, and record their remarks faithfully.
8. **They prefer short, minimal answers** when they ask for "only the necessary things".
9. **To check their code:** they push to GitHub `main`. Fetch it into the scratchpad clone (section 6), run `py_compile`, run the stubbed harness, and report only code-level issues unless asked for more.
10. **Answer "why" questions with evidence:** use the 2D simulator harness and real measurements, not guesses. Say when something is unverified.
11. **Git:** commit messages end with the session's Co-Authored-By and Claude-Session lines. Push to `claude/final-project-review-3m8r3v` (push works now; GitHub was connected mid-session). A stop hook demands that uncommitted or untracked files in `/home/user/rb2301_final_CA` be committed and pushed.

## 2. Status right now (updated 2026-11-04)
- **Milestone S1:** in Gazebo, the robot ran the whole course for the first time (commit `676268b`), with the swinging obstacles on. Logged as D9, D10, D11 and S1.
- **Known code bug (reported, the user hasn't fixed it yet):** in the after-door branch, the stop at the goal is overridden in the same tick by `move_2D(0.5, 0, 0)`. Suggested fix: `return` after stopping.
- **Still unhandled:** E1. `select_target2` (now used for every Gazebo target) raises `ValueError` from `np.argmin` when there's no free, seen cell outside the buffer.
- **Real robot:** no access at the moment.
  - The real `self.origin` and `self.global_goal` are **placeholders** (the user said so). The real door threshold is still unknown; the user's comment asks whether it changes on the real course.
  - The real branch uses 0.5 cm cells and the shared 6.02 × 4.5 m size, so with origin (−1.5, −1.5) the grid only reaches y = 3.0.
  - At the measured start (2.82, 2.94, 96.3°), the centre `cy` = 3.012 is outside the grid, so A* raises `IndexError` on tick 1. Reported; whether it happens depends on the final numbers.
  - Tick time at 0.5 cm is roughly 240–310 ms median (rough measurement).
  - The real branch has no after-door stop-and-wait logic yet (see section 3).
- **M3 (gap too narrow):** the user moved to 1 cm cells. The effect on the gap isn't recorded.
- **Dynamic obstacles:** handled in Gazebo by D10 (stop and wait). The user's first idea, comparing grids, got an evidence-based critique (logged under D10) and was dropped.
- **Critique permission:** the user allowed high-level critique for ONE question ("you are allowed to give high-level logic critique now"). The default rule (section 1) still applies.
- **Last thing discussed (2026-11-04):** the user asked to log D9–D11 and the milestone; that's done and pushed. Likely next: fixing the goal-stop bug, adapting the real robot (measuring origin/goal/door, porting the after-door logic), or writing the report.
- **About the user:** casual and blunt (swears), learns by doing. Sometimes sends code as screenshots, sometimes pushes to `main` and says "check it". If they say "check" without attaching code, look on GitHub `main`.

## 3. User's current algorithm (`src/rb2301_fp/rb2301_fp/obstacle_course.py` on `main`, commit `676268b`)
- **Settings** in `__init__` by `is_simulation`:

  | Setting | Gazebo | Real robot |
  |---|---|---|
  | origin | (−0.5, −3.3) | (−1.5, −1.5), placeholder |
  | goal | (4.7, −3.5) | (0.102, 0.758), placeholder |
  | robot_length × width | 0.22 × 0.24 | 0.3 × 0.2 |
  | lookahead / speed / tolerance | 0.05 / 0.3 / 0.05 | same |
  | resolution | 0.01 | 0.005 |
  | door_x | 3.4 | None (the real branch still hard-codes 2.9) |

  Gazebo also has `self.reached_right_wall = False`, which is unused.
- **Grid:** `self.map_size_x = 6.02` and `self.map_size_y = 4.5` are set after the if/else, so both branches share them. `grid_n_x` / `grid_n_y` = `round(size / res)`, and the grid is `np.full((grid_n_x, grid_n_y), -1, int8)`. All bounds checks use `grid_n_x` / `grid_n_y`.
  - Gazebo: 602 × 450 cells, covering x from −0.5 to 5.52 and y from −3.3 to 1.2.
  - Real: 1204 × 900 cells.
- **Origin:** `self.origin` is the lowest-x, lowest-y corner of cell `[0, 0]`. Index `i` is x and `j` is y.
- **Mapping:** both branches now let scans clear occupied cells (the `!= 99` check is commented out).
  - Gazebo: LiDAR at `pose + 0.114`; `theta = heading + idx * 6.28 / 359`.
  - Real: LiDAR at `pose + 0.045`; `theta = heading + idx * 2π / len`.
- **Centre:** both branches use `cx, cy = pose + 0.0725` along the heading. The real branch copied this from Gazebo.
- **Buffer:** `inflate_square(occ, round((L / 2) / res), round((W / 2) / res))`, with length along axis 0 (x) and width along axis 1 (y). Gazebo: 11 × 12 cells. Real: 30 × 20 cells.
- **Gazebo branch flow:**
  - If `self.passed_door`:
    1. goal check: stop if within tolerance (no `return`, which is the bug);
    2. `if self.last_scan[270] >= 0.15 and False:` move right (switched off);
    3. otherwise: if `last_scan[:91]` and `last_scan[270:]` are all > 0.25, `move_2D(0.5, 0, 0)`; else stop.
  - Otherwise, on odd counter ticks:
    - if `cx >= self.door_x`: set `passed_door = True`, then `select_target2` + A*;
    - else: `select_target2` + A*, logging "no path found bro" on failure;
    - then `pure_pursuit(path, cx, cy, ...)` → `move_2D(vx, vy, 0)`, or stop if there's no path.
- **Real branch flow:** on odd counter ticks:
  - if `cx >= 2.9`: print "passed door", then `select_target2` + A*;
  - else: `select_target` (frontier) + A*, with the `select_target2` fallback;
  - then `pure_pursuit(path, cx, cy)`. There's no `passed_door` logic.
- **Unchanged helpers:** A* (Python lists, 8-connected, diagonal cost 1.414, Euclidean h, returns `[]`), `pure_pursuit`, `select_target2`, `frontier_cells`, `world_to_grid` (floor), `grid_to_world` (+0.5).
- **A* start cell outside the grid:** past the high edge → `IndexError`; past the low edge → Python wraps the negative index, then returns `[]`.

## 4. Facts
**Gazebo world** (`rb2301_gz/worlds/obstacle_course_world_fp_original.sdf`, world name `empty`):
- **Walls:** 1.18 m panels placed 1.2 m apart, so there are **2 cm gaps between panels**. Each wall model sits 0.6 m to the left of its `<pose>`.
- **Layout (no randomisation):**
  - start corridor: y −0.6 to 0.6, x −0.6 to 3.0;
  - jog: x 1.8 to 3.0, towards −y;
  - second corridor: y −3.4 to −1.8, x 1.8 to the end wall at 5.4;
  - door wall at x = 3.6, gap 0.42 m (y −2.47 to −2.05);
  - swinging actors `obstacle_1` / `obstacle_2` at x 4.2 / 4.8, moving between y −2.0 and −3.2. Boxes 0.05 × 0.3 × 0.3 m, no collision.
- **`obstacle_modifier.py` randomisation:**
  - walls whose pose has x > 1 shift by x_off ∈ (−0.8, 0];
  - walls whose pose has y < −2.6 shift by y_off ∈ (−0.6, 0];
  - the door's y position shifts by up to 0.5 m;
  - the actors shift by the same offsets;
  - the goal moves to about (5.1 + x_off, −2.6 + y_off / 2).
- **Running without the swinging obstacles:** copy the generated world, delete the `<actor>` blocks, rebuild, then `ros2 launch rb2301_gz fp_gazebo.launch.py world:=<copy>.sdf x:=0.0 y:=0.0`.

**Gazebo robot (nanocar URDF):**
- Footprint relative to the pose point: x from −0.035 to +0.18, y ±0.11. The centre is 0.0725 ahead of the pose point.
- LiDAR 0.114 ahead, mounted yaw π, 360 readings from −3.14 to 3.14. Index i ≈ i degrees anticlockwise from the front.
- Moves sideways freely (VelocityControl); `cmd_vel` is in the robot frame; Gazebo speed cap 1.4.
- Odom pose is ground truth; spawns at (0, 0) at heading 0.

**Real robot:**
- `self.pose` is in the CA2 maze frame (the Optitrack config converts it automatically).
- Robot 0.23 long × 0.20 wide; LiDAR 4.5 cm ahead of the centre.
- 720 raw readings, 360 after `[::2]`. LiDAR direction verified correct.
- Course about 3 × 3 m. Start corner reads (2.82, 2.94, 96.3°); goal reads (0.102, 0.758).
- Speed cap 0.3.

## 5. Findings from investigations (2D harness), useful for future "why" questions
- **Causes of "no path":**
  1. **Swinging-obstacle trails** when 99 stays 99: the band they sweep becomes a permanent wall at the door, so targets beyond it can't be reached.
  2. **Frontier targets outside the course:** the LiDAR sees out through gaps (Gazebo's 2 cm panel gaps; vertex slips in the 2D simulator). This happens even without swinging obstacles and with clearing; 100% of failed A* calls had outside targets in 2 runs.
  3. **M2:** the robot's centre cell is inside the buffer of real obstacles.
  - Clearing removes cause 1 and most stray cells, but not causes 2 or 3. Without swinging obstacles, the old version still got stuck in 2 of 4 layouts.
- **Reaching the door** (2D, swinging obstacles on, 99 stays 99, 8 layouts):

  | Version | Reached the door | Contacts |
  |---|---|---|
  | pose point + 8 cells | 2 / 8 | 3 |
  | pose point + 6 cells | 4 / 8 | 21 |
  | centre + 6 cells | 3 / 8 | 0 |

  The centre offset removes contacts. In Gazebo that breaks the chain contact → heading change (D1) → wrong direction (D2) → stuck in buffer (M2). The keep-99 runs got stuck at the door itself.
- **Smallest usable gap** = (2k + 1)·res to (2k + 2)·res for a k-cell buffer:

  | Cells | Buffer | Gap never usable below | Always usable from |
  |---|---|---|---|
  | 2 cm | 6 cells | 0.26 | 0.28 |
  | 1 cm | 11 cells (0.11) | 0.23 | 0.24 |
  | 1 cm | 12 cells | 0.25 | 0.26 |

  Stray cells (about 4 cm from real surfaces) make gaps smaller still.
- **1 cm `ValueError`:** at 1 cm, a fixed 301 × 301 grid covers only 3.01 m (y −5.41 to −2.40), so the robot at (0, 0) is outside the grid, nothing gets mapped, and the frontier is empty. Fix: `self.grid_n = int(round(6.02 / res))`, used in `np.full` and the four bounds checks. At 1 cm, `round((0.22 / 2) / 0.01)` = 11 cells, which is exactly the 0.11 m half-width (zero margin).
- **M1 (earlier):** corner cutting with the 0.3 m lookahead, stale paths with halfway replanning, and planning from the pose point rather than the centre.
- **Timings on this machine:**
  - A* detour: 149 ms (Python lists), 379 ms (numpy arrays), 423 ms (`EachNode` objects);
  - map update: 7–26 ms per scan;
  - square inflation: 0.5 ms (numpy) vs 3.6 ms (loop).

## 6. Tools and paths
- **Scratchpad:** `/tmp/claude-0/-home-user-rb2301-final-CA/be22f863-093c-51e5-b646-f1aab5ca3d5f/scratchpad`
  - `user_repo/`: shallow clone of the user's repo. Update with `git fetch --depth 10 origin main && git checkout FETCH_HEAD`.
  - `harness/`, stubbed-ROS runners. Arguments are usually `<code_dir> /tmp/fp_work <seconds> <seed> [swing|noswing]`:
    - `drive_user_node.py`: crash check, Gazebo mode;
    - `real_mode_run_phi.py`: real-robot mode;
    - `contact_probe3.py`: classifies obstacle contacts;
    - `nopath_study2.py`: no-path causes and outside-course targets;
    - `door_study2.py`: whether the robot reaches the door.
  - Test copies of user versions: `prev/` (387d9f7), `np_sticky/`, `np_clearing/`, `A_pose8/`, `B_pose6/`, `C_centre6/`.
- **2D simulator:** `/tmp/fp_work/sim2d.py`, with `World(rng, mode, swing_model)`, `.lidar()`, `.collides()`, `.swingers` (set to `[]` to remove them) and `run()`.
- **Claude's own navigator:** `/tmp/fp_work/course_navigator.py`. **The user rejected it. Never suggest it.**
- **Don't use `pkill -f <name>`** where the pattern matches this shell's own command line, or the shell kills itself (exit 144). Use the `[x]` bracket trick instead.
- **When deleting:** use `"${VAR:?}"` paths. A safety check blocks `rm -rf $VAR`.

## 7. Report log (`rb2301_final_report_log.md`, this branch)
Entries:
- **D1** no heading control; **D2** world-frame velocity sent straight to `move_2D`; **M1** brushing obstacles; **M2** no path because the start cell is in the buffer;
- **D3** fallback `select_target2`; **D4** replanning every other tick; **D5** buffer from `robot_length` (plus later notes); **D6** lookahead 0.05;
- **D7** planning from the robot's centre; **D8** clearing occupied cells; **M3** wide-enough gap rejected (plus a later note about 1 cm cells);
- **R1** first real-robot runs, plus the user's report remark linking D1/D2 and M2 with R1; **E1** empty-target crash;
- **D9** every target from `select_target2`, with the 2D frontier-vs-free test; **D10** stop and wait after the door, including the dropped grid-differencing idea and the goal-stop bug from the code check;
- **D11** separate buffer sizes along x and y; **S1** first full run of the course in Gazebo (commit `676268b`), with the user's remark.

Most "Fix" fields are blank for the user to fill in.

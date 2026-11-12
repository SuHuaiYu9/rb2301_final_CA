# RB2301 Final Project: decisions and mistakes log

One entry per decision or mistake. Fill in **Observed** and **Fix** when you find out what actually happened.

> **This is the version 2 (reactive) log, a continuation of the version 1 log.**
> - **Version 1** (the original code, `obstacle_course.py`): entries D1–D11, M1–M3, R1, E1 and S1 below are copied unchanged from branch `claude/final-project-review-3m8r3v` as of 2026-11-05 (commit `329a919`). Any later version 1 entries are logged on that branch, not here.
> - **Version 2** (the reactive code, `obstacle_course2.py`): logging starts at the heading **"Version 2 (reactive) starts here"** at the end of this file. Version 2 entries use a `V2-` prefix (V2-D1, V2-M1, ...) so their numbers never clash with version 1.

---

# Version 1 (original code: `obstacle_course.py`)

## D1: No heading control (2026-11-03)
- **Decision:** the robot never commands a rotation. Every `move_2D` call uses `turn = 0`, with no correction to hold the heading.
- **Why:** the robot can move sideways, so it only needs to translate in x and y. Leaving drift unhandled is deliberate, so it can be observed and fixed later.
- **Potential errors:**
  - Nothing returns the heading to its starting value if it changes (wheel slip, bumping into things, mecanum wheels on the real robot), so the robot may slowly rotate over a run.
  - With D2, any heading change makes the robot drive in the wrong direction.
  - The square obstacle buffer assumes the robot stays square to the grid.
- **Observed (Gazebo, 2026-11-03):** the robot's sides often brush against obstacles (see M1). Each contact pushes the robot and turns its heading, and nothing turns it back.
- **Fix (proposed, not yet implemented):** heading correction/hold.

## D2: World-frame velocity sent straight to `move_2D` (2026-11-03)
- **Decision:** pure pursuit computes the velocity `(vx, vy)` in the world frame and passes it directly to `move_2D`. `move_2D` actually expects velocities in the robot's own frame.
- **Why:** Gazebo spawns the robot at heading 0°, where the world frame and the robot's frame are identical.
- **Potential errors:**
  - Whenever the heading isn't 0°, the robot moves in a direction rotated by its heading. For example, at 90° a +x command moves it along +y.
  - On the real robot the pose is in the CA2 maze frame, and the starting heading is whatever direction the robot was placed in, so it's unlikely to be 0°.
  - Heading drift (D1) makes this worse during a run, even in Gazebo.
- **Observed (Gazebo, 2026-11-03):** once a contact has turned the heading, the robot can no longer follow its path. Since control relies on heading = 0°, every command after that sends it in the wrong direction.
- **Fix (proposed, not yet implemented):** heading correction/hold (as in D1).

## M1: Robot brushes obstacles despite the buffered map (2026-11-03)
- **Observed (Gazebo):** the robot's sides often brush against obstacles while it follows the A* path. This then triggers the heading problems in D1 and D2.
- **Tried:** increasing the obstacle buffer from 8 to 9 cells (0.16 m to 0.18 m). The robot still hit obstacles.
- **Why (investigated with the same code in a 2D test: 6 contacts across 11 layouts):**
  1. **Pure pursuit cuts across bends.** All 6 contacts were at a bend in the path, with the robot 2–11.5 cm off the path. With a 0.3 m lookahead, the robot drives across the inside of a bend, and A* paths already pass as close to obstacles as the buffer allows on the inside of bends.
  2. **The path goes stale.** In 5 of 6 contacts, the path next to the robot ran through buffered cells at the moment of contact, even though it was clear when planned. Between planning and contact, 90 to 640 new cells had been marked occupied. Obstacles are mapped more completely as the robot gets closer, but the path is only replanned at halfway.
  3. **The buffer is centred on the pose point, not the robot.** The robot reaches 0.18 m ahead of `self.pose` (0.035 m behind, 0.11 m to each side), but the buffer is the same in every direction. That leaves zero margin at the front with 9 cells and negative margin with 8, before grid rounding of up to 2 cm.
  - The 2 cm gained by going from 8 to 9 cells is small compared with the corner cutting (up to about 11.5 cm) and the growth from stale paths.
- **Fix:**

## M2: "no path found bro" even though a feasible path clearly exists (2026-11-03)
- **Observed (Gazebo):** many runs print "no path found bro", even though there is clearly a route through the course.
- **Why (investigated with the same code, buffer 9, in a 2D test across 8 layouts; same pattern in all 7 that didn't crash):**
  1. **The robot's own cell is inside the buffer.** At the first "no path", the robot's cell was within 0.18 m of an occupied cell, with 0 of its 8 neighbours free. A* only steps into unblocked cells, so it can't leave the start cell and returns `[]` immediately. The target itself was not in the buffer. The robot gets that close for the reasons in M1 (cutting across bends, obstacles filling into the map as it approaches).
  2. **It never recovers.** With no path, the robot is sent zero velocity, so it stays inside the buffer. The bottom `if not self.path:` block reruns A* every tick from the same start cell and fails every time. The halfway branch, the only place a new target is chosen, needs a non-empty path, so it never runs again. In the test this lasted the rest of the run: 1,300–1,700 ticks in a row (about 65–85 s).
  3. **Two errors on the first tick:** the halfway replan fails, then the bottom block immediately retries A* with the same inputs.
- **Fix:**

## D3: Fallback target when A* finds no path (2026-11-03)
- **Decision:** if A* to the frontier target fails, choose a fallback target with `select_target2` and run A* again. The fallback is the free, seen, non-buffered cell closest to the goal, and doesn't have to be a frontier cell. If that A* also fails, log "still no path found bro".
- **Why:** the diagnosis at the time was that "no path found bro" happens because there are no suitable frontier cells to pick as a target. See also M2.
- **Observed:**
- **Fix:**

## D4: Replan and re-choose the target every other tick (2026-11-03)
- **Decision:** instead of replanning at halfway along the path, choose a new frontier target and rerun A* whenever `self.counter % 2` is true. That's every other timer tick (every 0.1 s). The halfway version is still in the file, commented out.
- **Why:** the intent was to replan and re-choose the target every tick. As written, the code does it every other tick.
- **Observed:**
- **Fix:**

## D5: Obstacle buffer calculated from the robot's size (2026-11-03)
- **Decision:** buffer radius = `round((robot_length / 2) / occupancy_grid_resolution)`, with `robot_length = 0.3` m. That's `round(7.5)` = 8 cells (0.16 m), replacing the 9 cells tried in M1. `robot_width = 0.2` m is defined but not used.
- **Later (real robot, 2026-11-03):** `robot_length` changed to 0.2 m, so 5 cells (0.10 m). See R1.
- **Later (2026-11-04):** the buffer now has separate sizes along x and y, from `robot_length` and `robot_width`. See D11.
- **Why:** (not recorded)
- **Observed:**
- **Fix:**

## D6: Lookahead distance changed from 0.3 m to 0.05 m (2026-11-03)
- **Decision:** `lookahead_distance` changed from 0.3 to 0.05. `goal_tolerance` is also 0.05.
- **Why:** (not recorded)
- **Observed:**
- **Fix:**

## D7: Plan and follow from the robot's centre, not the pose point (Gazebo) (2026-11-03)
- **Decision:** in the simulation branch, compute the robot's centre, which is 0.0725 m ahead of the pose point along the heading:
  ```python
  cx = self.pose[0] + 0.0725 * np.cos(heading)
  cy = self.pose[1] + 0.0725 * np.sin(heading)
  ```
  Use `(cx, cy)` instead of `self.pose` for A*'s start cell and in `pure_pursuit`. The LiDAR position is still calculated from `self.pose`. The real-robot branch is unchanged so far: the offset between the real robot's pose point and its centre still needs finding.
- **Why:** after replanning every other tick and the 0.05 m lookahead, the remaining contacts in a 2D test were all at the robot's front, while it was right on its path. In Gazebo the pose point is near the back of the robot: the body reaches 0.18 m ahead of it but only 0.035 m behind. The buffer is centred on whatever point is planned for, so the front stuck out past it.
- **Also changed (simulation settings):** `robot_length` 0.22 m and `robot_width` 0.21 m. That gives a buffer of `round(5.5)` = 6 cells (0.12 m). Measured from the centre, the Gazebo robot reaches about 0.11 m in each direction.
- **Observed (Gazebo):** avoids obstacles much better.
- **Fix:**

## D8: LiDAR scans can now turn occupied cells back to free (Gazebo) (2026-11-03)
- **Decision:** removed the rule that kept a cell at 99 once it was marked occupied (`if self.occupancy_grid[i,j] != 99:`). Any cell a LiDAR ray passes through is now set to free (0), so the map trusts the latest scan. Simulation branch only: the real-robot branch still keeps 99 cells as 99.
- **Why:** suspected to be causing the very frequent "no path found bro", without being sure of the mechanism at the time.
- **Observed (Gazebo):** far fewer "no path found bro" messages.
- **Fix:**

## M3: Robot won't use a gap that is physically wide enough (Gazebo) (2026-11-03)
- **Observed (Gazebo; 2 cm cells, 6-cell buffer, planning from the robot centre, 99 cells stay 99):** the robot gets stuck in front of three obstacles, even though the rightmost gap is physically wide enough for it.
- **Why:**
  1. **Not the square buffer.** The Gazebo robot model is about 0.215 m long × 0.22 m wide, essentially square. The 6-cell buffer (0.12 m) closely matches half its width (0.11 m). It's the real robot (0.20 × 0.23 m) that isn't square.
  2. **The grid makes gaps look smaller.** A LiDAR hit marks the whole 2 cm cell it lands in. A* needs every path cell to be at least 7 cells from an occupied cell (the buffer covers 6), so the two obstacles' occupied cells must be at least 14 cells apart. Gaps under 0.26 m are never usable, 0.26–0.28 m sometimes, and 0.28 m or more always. A gap that is "technically big enough" for a 0.22 m robot (0.22–0.27 m) is rejected.
  3. **Obstacles look thicker in the map than they are.** In 2D tests, stray occupied cells sat typically about 4 cm (2 cells) from the real surface, and there are more of them when 99 cells are never cleared. Each extra cell on an edge facing the gap adds 2 cm to the smallest usable gap.
- **Options and their trade-offs:**

  | Change | Smallest gap A* uses | Trade-off |
  |---|---|---|
  | Current: 2 cm cells, 6-cell buffer (0.12 m) | 0.26–0.28 m | — |
  | 2 cm cells, 5-cell buffer (0.10 m) | about 0.22–0.24 m | 0.10 m is less than the 0.11 m half-width, so it scrapes |
  | 1 cm cells, 11-cell buffer (0.11 m; what `robot_length` 0.22 gives at 1 cm) | 0.23–0.24 m | Zero margin over the half-width; 4× as many cells. The grid size must be set in metres: a fixed 301 × 301 grid at 1 cm only covers 3 m, the robot starts outside it, and `select_target` raises a `ValueError` (seen when 1 cm was first tried) |
  | 1 cm cells, 12-cell buffer (0.12 m) | 0.25–0.26 m | Same safety margin as now; only 1–2 cm better |
- **Planned fix:** try a finer grid (1 cm cells).
- **Later (2026-11-04):** Gazebo changed to 1 cm cells (real robot 0.5 cm), with the grid size set in metres: `map_size_x` 6.02 m × `map_size_y` 4.5 m. Effect on this gap not recorded.
- **Fix:**

## R1: First runs on the real robot (2026-11-03)
- **Setup:**
  - Real course is about 3 m × 3 m, with the same general shape as the Gazebo course. Coordinates are measured in the CA2 maze frame with `./fp.sh --calibrate`.
  - `origin = (-1.5, -1.5)`, so the 301 × 301 grid covers x and y from −1.5 to 4.52 m.
  - `global_goal = (0.102, 0.758)`, the measured goal.
  - Start pose at the start corner: maze (2.82, 2.94), heading **+96.3°**.
  - LiDAR offset 0.045 m: the LiDAR is 4.5 cm ahead of the robot's centre, assuming the Optitrack pose point is the centre.
  - The real LiDAR sends 720 readings, which become 360 after the template's `[::2]`, so the angle per reading is `2π / len(scan)`.
- **LiDAR direction check:** box in front of the robot and to its left. The direction was correct, so the LiDAR is not the cause.
- **Observed:**
  - The robot moves haphazardly for a short while after starting.
  - It soon stops, the terminal prints "no path found bro", and it stays stopped.
- **Tried:** `robot_length` from 0.3 to 0.2 (buffer from 8 to 5 cells, 0.16 m to 0.10 m). Same problem.
- **Possibly related (not yet confirmed on the real robot):**
  - D2: the start heading is +96.3°, not the 0° the velocity commands assume.
  - M2: the robot's own cell ending up inside the buffer, so A* can't take a step and no path is ever found again.
- **Remark for the report:** a potential error that was noticed in Gazebo but ignored because simulation conditions are ideal is likely to become a real error on the robot. Point out the connection between D1/D2 and M2 with R1.
- **Fix:**

## E1: `select_target` crashes when there are no frontier cells
- **Observed (2D tests):** `ValueError: attempt to get argmin of an empty sequence` from `np.argmin(d)` in `select_target`, raised inside `timer_callback`. Seen in 1 of 11 layouts, and again in 1 of 8 in the M2 test.
- **Fix:**

## D9: Every target is now the free cell closest to the goal (2026-11-04)
- **Decision:** `select_target` (the frontier cell closest to the goal) is no longer used. Every target, before and after the door, comes from `select_target2`: the seen, free cell outside the buffer that is closest to the goal in a straight line. If A* finds no path, the robot logs "no path found bro" and stops; there is no second fallback any more.
- **Why:** sometimes the robot got through the door gap, but before its centre passed the x threshold for "passed the door", it saw the swinging obstacles. The frontier cell closest to the goal then switched to a cell behind it.
- **Tested before the change (2D, 8 layouts, swinging obstacles on, success = reaching the door):**
  - frontier targets reached the door in 4 of 8 layouts; closest-free-cell targets in 6 of 8;
  - in 6 layouts the two versions behaved identically or almost identically;
  - in the other 2, the frontier version stopped next to a target by a wall and stayed there.
- **Observed (Gazebo):** part of the first full run of the course (S1).
- **Fix:**

## D10: After the door: drive straight ahead, and stop and wait while anything is within 0.25 m in front (Gazebo) (2026-11-04)
- **Decision:**
  - "Passed the door" means the robot's centre has `cx >= self.door_x`, hard-coded as 3.4 in Gazebo (previously 2.9). It's checked on planning ticks; once true, `self.passed_door` stays true.
  - After that, A* and pure pursuit are no longer used. Every tick:
    - if the centre is within `goal_tolerance` of the goal, stop;
    - if every LiDAR reading in the front-left quarter (indices 0–90) and the front-right quarter (indices 270–359) is more than 0.25 m, drive straight forward with `move_2D(0.5, 0, 0)`;
    - otherwise, stop and wait until the swinging obstacle has gone past.
  - A first move to the right (−y) until the right-hand wall is close is written but switched off (`and False`).
- **Why:**
  - The robot only moves forward when nothing is close in front of it, so a swinging obstacle can't hit it while it moves.
  - 0.25 m was calculated to give the robot enough clearance from the swinging obstacles. The previous plan used 0.22 m, the front-left quarter only and a move to the right-hand wall first; 0.22 m was worked out from the obstacles' simulated speed and the robot's length.
- **Earlier idea, dropped:** compare consecutive occupancy grids to find which obstacle cells moved, matching freed cells to newly occupied neighbours (nearest neighbour), using the obstacles' known size. A 2D test (4 layouts, 1 cm cells) showed:
  - **It works while the robot stands still:** about 1 wall cell changes per scan, against 10–13 obstacle cells.
  - **While the robot moves,** 200–280 wall cells change per scan, against 33–46 obstacle cells.
  - **The neighbour matching doesn't fit this motion.** Each box slides along its own 0.3 m face, so the freed cells and the newly occupied cells end up about 0.3 m apart, not next to each other.
- **Observed (Gazebo):** "works perfectly". This gave the first full run of the course (S1).
- **Code check (2026-11-04):** at the goal, the stop is overridden in the same tick by the forward command (`(0, 0)` then `(0.5, 0)` are published), so the robot doesn't stop at the goal. Suggested fix: `return` after stopping.
- **Fix:**

## D11: Separate buffer sizes along x and y (2026-11-04)
- **Decision:** `inflate_square(occ, length, width)` grows obstacles by `length` cells along x and `width` cells along y, where `length = round((robot_length / 2) / res)` and `width = round((robot_width / 2) / res)`.
  - Gazebo: `robot_width` changed from 0.21 to 0.24 m. With 1 cm cells, the buffer is 11 cells (0.11 m) along x and 12 cells (0.12 m) along y.
  - Real robot: 30 × 20 cells at 0.5 cm.
- **Why:** (not recorded)
- **Observed:**
- **Fix:**

## S1: First full run of the course in Gazebo (2026-11-04)
- **Code:** commit `676268b`, "added dynamic obstacle avoidance logic and made inflate_square logic independent for width and length".
- **Setup (Gazebo, swinging obstacles on):**
  - 1 cm cells; grid 6.02 × 4.5 m from origin (−0.5, −3.3); goal (4.7, −3.5);
  - scans can clear occupied cells (D8); planning and following from the robot's centre (D7);
  - every target from `select_target2` (D9); stop-and-wait after the door (D10); buffer 11 × 12 cells (D11).
- **Result:** the robot ran the whole course: past the static obstacles, through the door, and past the swinging obstacles.
- **Remark from the user:** "this marks our actual first success of the robot running the full course in sim".

---

### Path-following setup (for reference)
Pure pursuit with the robot treated as a point that can move in any direction: each tick it drives at a constant speed straight towards the path point `lookahead_distance` ahead of it, and it stops within `goal_tolerance` of the last path point. Starting values: lookahead 0.3 m, speed 0.3 m/s, tolerance 0.05 m. Current values: lookahead 0.05 m (see D6), speed 0.3 m/s, tolerance 0.05 m.

---
---

# Version 2 (reactive) starts here (2026-11-05)
**Everything above this line is version 1. Everything below it is version 2.**

- **Code:** `src/rb2301_fp/rb2301_fp/obstacle_course2.py` on `main`, a new file for version 2. Version 1 stays in `obstacle_course.py`. Starting point: version 1 at commit `676268b` (S1).
- **Goal:** no hard-coded door x-value and no hard-coded global goal.
- **Why (the user's words):** "the prof just said that he would prefer if the whole project is reactive instead of harcoded."
- Entries are numbered V2-D#, V2-M#, V2-R#, V2-E#, V2-S#.

## V2-M1: The dynamic state switches on before any swinging obstacle is seen (Gazebo) (2026-11-05)
- **Setup (commit `27822e7`):** dynamic-obstacle detection. Every `compare_every` = 4 timer callbacks, the grid cells saved at the last comparison are checked again. These cells form a window fixed in the world: 0–0.2 m ahead of the robot centre along the heading, 1 m wide. If `dynamic_threshold` = 25 or more of them have gone from occupied (99) to free (0), `dynamic_state` becomes true. In that state the robot drives straight forward, and stops while anything in the front quadrants is within 0.22 m. The threshold 25 was a placeholder from a 2D test: per comparison, up to 9–75 cells were freed with no swinging obstacles and up to 24–111 near them, and no single value separated the two.
- **Observed (Gazebo):** the dynamic state switches on very early and very often, before the swinging obstacles are seen at all. Many cells change from occupied to free.
- **Why (2D test: 7 layouts, swinging obstacles removed, detection switched off so it only counts, 40 s each):**
  - **Standing still:** 0 cells freed per comparison in every layout.
  - **Moving:** up to 9–67 cells freed per comparison, and 25 or more in 3 of the 7 layouts.
  - **Where:** 85–100% of the freed cells were within 1.5 cm of a real wall or obstacle surface. So they are static surfaces, not sensor noise (the Gazebo LiDAR noise is set to 0).
  1. **The viewing angle changes.** A hit marks the whole 1 cm cell it lands in, and most surface cells lie partly in front of the surface. From a few centimetres further on, a ray at a slightly different angle passes through the front part of that cell and hits the surface in the next cell. Every cell before a hit is set to free, and scans are allowed to clear occupied cells (D8), so the cell flips from 99 to 0.
  2. **The same scan is used twice.** A scan arrives every 2nd tick, but the grid is updated from `last_scan` every tick, using the newer pose. At 0.3 m/s the hit points move about 1.5 cm deeper into the surface, so the previous hit cells are freed. Roughly half or more of the freed cells were freed on these in-between ticks. With a new scan every tick the counts were about the same (up to 11–74 per comparison), so cause 1 alone is enough.
  - **Why surfaces are in the window:** a 1 m wide window contains wall cells whenever the robot is within 0.5 m of a side wall, or within 0.2 m of anything ahead.
  - **Not tested (Gazebo only):** the 2 cm gaps between wall panels, and the pose used for a scan being the latest odometry rather than the pose when the scan was taken.
  - **Related:** version 1's dropped grid-comparison idea (D10) saw the same effect: 200–280 wall cells changed per scan while the robot moved.
- **Tried (commit `af71c1b`):** a false alarm. In the user's words: the robot waits in place for a while, because its front quadrants are blocked most of the time, and "if the window does not change sufficiently in the time that the robot is still, it knows it is a false alarm and go back to dynamic_state = False". In the code, on every tick with the front blocked: stop; count the saved window cells that went 99 → 0; if 5 or fewer, `dynamic_state = False`; then save a new window. Also changed: `window_width` 0.5, `compare_every` 2, `dynamic_threshold` 15. Gazebo branch only.
  - **Code check (2D trace):** the decision is made on the first blocked tick, not after a wait.
    - The window is saved again on every blocked tick, so each check covers only the last tick.
    - The first blocked tick compares against a window saved when the state switched on, possibly far behind. In one trace, it was 29 ticks earlier, while the robot was driving.
    - If the front is blocked on the tick the state switches on, the window was saved earlier in that same tick, so 0 cells are freed and the state switches off in that tick.
    - On ticks with no new scan, a still robot's grid doesn't change, so the count is 0.
- **Fix:**


## V2-D1: Switch to purely reactive control, with no occupancy grid (2026-11-05)
- **Decision:** drop the occupancy grid, A*, pure pursuit and the grid-window detection, and look for a purely reactive controller instead.
- **Why:**
  - A prof said the course can be solved with reactive control alone and no occupancy grid ("nested if...else blocks", a bias towards the goal's general direction, and a bias towards places not visited or seen before).
  - The user felt the grid version was over-complicated and not robust, and expected LiDAR noise and localisation noise to make it worse on the real robot. V2-M1 measured this in a 2D test: with 1 cm LiDAR noise and 3 mm pose jitter, a robot standing still in front of a wall saw 23–26 cells change per comparison at 1 cm cells, and 98 at 0.5 cm cells.
- **Known course facts to use:**
  - The shape is fixed: a straight corridor, a right turn into a second straight corridor, then a left turn through the door into the swinging-obstacle region, with the goal wall at the back.
  - The robot starts near the back of the first corridor, facing its far end.
  - Optitrack gives the pose.
- **Rejected:** wall following, which Claude proposed first. The prof does not want wall following ("too simplistic").

## V2-REF: Claude's reactive reference code (written by Claude at the user's request, 2026-11-05 to 06)
**Not the user's code.** Kept for the report as a record of a working approach, its design rationale, and the problems found while building it. File: `claude_tools/reference/obstacle_course_reactive.py` on branch `claude/reactive-version`. Unless stated otherwise, all test results are from Claude's 2D simulator (`claude_tools/`), not Gazebo.

### How it works, and why each part is there
1. **Everything happens in the robot's own frame:** x forward, y left, measured from the robot's centre.
   - **Why:** the robot never turns (D1), so this frame stays lined up with the course for the whole run. No map or coordinates are needed. Every command is worked out in the robot's frame, so it means the same thing whatever the start heading (the real robot starts at about 96°).
2. **Scan to points** (`scan_to_points`): each LiDAR reading becomes an (x, y) point measured from the robot's centre, allowing for the LiDAR being ahead of the centre. Readings that aren't finite or are under 0.05 m are dropped, and so are points inside the robot's own outline.
   - **Why:** obstacle distances must be measured from where the robot's body actually is. Points inside the robot can't be obstacles; they're glitches, or the robot seeing itself.
3. **"Can I go that way?"** (`free_distance`): how far the robot can slide in a direction before a LiDAR point enters the strip it would sweep. The strip is the robot's outline projected sideways, plus a side margin, so it's narrower when moving forward (width 0.24 m) and wider diagonally. A direction is blocked if the robot can't slide at least 0.10 m.
   - **Why:** a gap narrower than the robot automatically counts as blocked, with no special cases.
4. **Choosing a direction:** each tick, score 16 directions, every 22.5°. The score has four parts:
   - **Goal bias:** cos(angle between the direction and 45° right of forward). The goal is generally forward-right of the start, as the prof suggested.
   - **Room bonus:** 1.0 × (free distance, up to 1 m). This makes a long view through a gap beat a short slide along a wall. Without it, the robot slid past the door gap (problem P5).
   - **Novelty bonus:** 1.5 × (how far the point it's looking at is from anywhere the robot has been, up to 1). The point is 0.4 m along the direction, or less if the direction is blocked sooner. This is the prof's "bias towards places not visited". It's what gets the robot out of the corner at the door wall: going back along the wall towards the gap costs 0.71 of goal bias but gains 1.5 of novelty.
   - **Time-spent penalty:** −0.1 for each recorded position within 0.25 m of that point. Positions are recorded every 0.25 s, so this grows the longer the robot lingers. It gets the robot out of dead-end pockets (P5), where a simple been-here-or-not bonus isn't enough.
5. **Lining up with gaps** (`best_lane`): for each direction, also try lanes shifted sideways up to 12 cm in 2 cm steps, if there's room to slide there first. The robot moves along the direction with a sideways speed of 2 × the shift.
   - **Why:** with only 16 directions, the robot couldn't line up with a gap slightly off its path (P3). The 2 cm steps fix narrow gaps with noise (P11).
6. **Commitment and hysteresis:** keep a direction for 0.15 m, or until it's blocked, before deciding again. Only switch if another direction scores at least 0.2 higher.
   - **Why:** without them, the robot dithered between two directions whenever one kept flipping between blocked and free (P1).
7. **Speed:** the speed along the direction is never more than the unshifted free distance, so it slows down near obstacles and can't run into anything while lining up. The total speed is capped at 0.3 m/s, the real robot's limit.
8. **Seeing movement** (`seen_past` / `changed_points`): compare each new scan with the previous one. A point counts as moved if the new scan, looking towards where that point was (the ray pointing at it and its two neighbours, taking the shortest), sees more than 10 cm past it. The reverse is checked too: a new point where the old scan saw more than 10 cm past it means something arrived.
   - **Why it beats comparing grid cells (V2-M1):** a wall can never become see-through, however the robot moves. The 10 cm margin is about 10 times the noise sources (LiDAR noise about 1 cm, pose error, and up to 1.5 cm of robot movement between scan and tick). The neighbouring rays stop a ray that slips past an obstacle's edge from counting.
   - **It never compares the same ray number in the two scans.** It recomputes each point's angle and distance from the new LiDAR position.
   - **Measured (2D, with noise):** standing still in front of a wall, 0–1 points, against 23–98 changed grid cells. Driving the static course, up to 5–15 points per scan pair, against up to 37–78 cells.
9. **Finding pendulum lines** (`update_lines`): each pendulum swings across the course at a fixed distance ahead, so its moving points keep the same position along the robot's forward axis.
   - A line is confirmed once there are 4 or more moved points within 0.12 m on 5 scan pairs within 2 s.
   - The line's position is the 10th percentile of its points, and only ever moves closer to the robot.
   - **Why:** a thick pendulum's sides add moving points further back, which made the robot stop too close (P7). Stopping early is the safe mistake. The 4-point, 5-scan rule stops noise from creating lines that don't exist (P11).
10. **Stop short of lines:** the robot's front stops 0.08 m before any confirmed line, and before any line with movement in the last 2 s that isn't confirmed yet.
    - **Why:** to wait outside the pendulum's sweep. Stopping at unconfirmed lines too stops the robot driving into a pendulum that hasn't been confirmed yet (P12).
11. **Watching the pendulum while waiting** (`line_sides`): take the scan points along the line, from 6 cm in front of it to 20 cm behind it, to allow for pendulum thickness. Find the nearest point on each side of the robot's lane (its width plus 5 cm each side). If that gap shrinks faster than 0.05 m/s the pendulum is approaching; if it grows, it's leaving.
    - **Why nearest points rather than outlines:** from close up, the pendulum is seen almost edge-on, so its outline is unreliable (P7). Walls never move, so they never look like they're approaching.
12. **When to cross:**
    - **Go if** nothing is in the lane, nothing is approaching, and either:
      - the pendulum has just left the lane (within 0.4 s), moving towards the side with more room; or
      - nothing on the line has moved for 1 s, by both the side gaps and the motion test.
    - **Why the side with more room:** the time until a pendulum returns runs from about half its period (lane in the middle of the swing) to a full period (lane near one end of the swing). Leaving towards the far side gives the longest gap.
    - If something static sits in the lane for 2 s, the line is ignored, because it isn't a pendulum.
13. **Crossing:** drive straight forward at 0.3 m/s until the robot's rear is 0.20 m past the line. Never abandon a crossing halfway; pause only if physically blocked, and give up after 3 s.
    - **Why:** stopping inside a sweep is the worst place to be (P10).
14. **Final approach and goal:** once a line has been crossed and no line is ahead, drive straight forward. The goal is reached when the robot can't move forward (free distance under 0.10 m) and there is a wall at ±45° (free distance under 0.25 m both ways), for 1 s.
    - **Why:** this matches the confirmed goal definition: past the pendulums, the robot can no longer move forward along its own x-axis. Driving straight stops the robot wandering back behind a line (P9). The wall check stops a pendulum in front from counting as the goal (P12).
15. **Template rules:** only the two placeholder lines inside `INSERT CODE HERE` were replaced. Helper functions are added outside the class, and new attributes are added at the end of `__init__`. No template line was changed.

### Problems found during development, and fixes (in order)
- **P1: dithering at the door corner.** The first version used the goal bias plus a novelty bonus, 16 directions and a 0.15 m free threshold. At the corner, "right" (towards the goal) flipped between blocked and free every 1.5 cm, so the robot rocked left and right.
  - **Fix:** commit to a direction for 0.15 m; look only as far as the free distance for novelty, so a short dead end counts as visited; novelty weight 2.0.
- **P2: waiting never fired, and the goal fired early or never.** Testing only for points disappearing found too few: a pendulum sliding along its own length uncovers only a few points per scan. Separately, the goal ("pendulums seen and can't move forward for 1 s") fired when a pendulum blocked the way, or never fired when the pendulums weren't detected.
  - **Fix:** the motion test also counts points appearing (P3). The goal rules were redone later (P4, P9, P12).
- **P3: stuck at gaps slightly off the robot's path.** For example, a 0.35 m gap between an obstacle and a wall, or a passage just beyond an obstacle's corner. With only 16 directions the robot couldn't line up, and rocked back and forth.
  - **Fix:** sideways-shifted lanes (`best_lane`) and a sideways speed towards the best lane.
  - **Result:** 6 of 12 layouts reached the end wall.
- **P4: false goal at the door wall.** The Gazebo-style pendulums are always moving, so they were seen through the gap. The robot then couldn't move forward at the door wall, so "pendulums seen and can't move forward" counted as the goal.
  - **Fix (temporary):** the goal needs pendulums seen moving behind the robot. Later replaced by the line logic (P9).
- **P5: dead-end pockets, then wandering away.**
  - The goal bias kept pulling the robot back into a pocket in the goal direction, next to the door. A been-here-or-not memory couldn't get it out.
  - The first fix, a time-spent penalty of 0.5 per recorded position, was too strong: a single pass leaves about 7 positions (−3.5), more than the goal bias can ever add. The robot slid past the door gap, then treated the whole area as visited and wandered back the way it came.
  - **Final fix:** penalty 0.1 per position, with the capped novelty bonus (1.5) back, plus the room bonus.
  - **Result:** navigation only, through the door in 12/12 layouts, 12/12 with noise, and 14/14 new layouts, with no wall or obstacle contacts.
- **P6: the simple "wait if something moves in my path" rule stopped the robot inside a pendulum's sweep.** With real-style pendulums (held, then released when the robot nears the door), there were contacts in 12/12 runs (1–14 each), and the goal was called 0.5–1.4 m short in 8/12.
  - **Fix:** pendulum line tracking, stopping short of lines, and timed crossing.
- **P7: unsafe crossings ("nothing moved for 1 s" while the pendulum was moving).**
  - The real-style pendulums are 6–16 cm thick, and moving points on their sides placed the line about 5 cm too far ahead. The robot waited 2–3 cm from the pendulum's face and was brushed while waiting.
  - From that close, the face is seen almost edge-on, so the first watching method (groups of points longer than 10 cm) found nothing.
  - **Fixes:** the line is placed at its near end and only moves closer; the watched strip runs from 6 cm in front to 20 cm behind the line; the nearest point on each side is tracked instead of groups; "nothing moved" also needs no motion hits on the line for 1 s; standoff 0.10 m (later 0.08).
- **P8: waiting forever.** The stricter "nothing moved" rule from P7 never let the robot go when pendulum tracking failed. Fixed by the nearest-point tracking in P7.
  - **Result:** real-style pendulums, 10/12 reached the end wall.
- **P9: wandering back after the pendulums.** After crossing both lines, the robot wandered back behind line 1 (lines stay marked as crossed) and declared the goal there.
  - **Fix:** after the last line, drive straight forward to the wall.
  - **Result:** real-style 12/12; Gazebo-style 11/12.
- **P10: remaining contacts while crossing.** Three causes:
  - A crossing takes about 1.4–1.5 s (robot length 0.22 m + standoff + pendulum thickness, at 0.3 m/s), and fast pendulums (2.6 s period) can return in about 1.3 s.
  - The crossing was declared done when the rear was 7 cm past the line, but pendulums can be 16 cm thick.
  - A crossing was abandoned if something was right in front, leaving the robot inside the sweep.
  - **Fixes:** crossing done only when the rear is 0.20 m past the line; never abandon a crossing (pause if blocked, give up after 3 s); standoff 0.08 m.
  - **Result:** real-style 12/12, with 6/12 runs contact-free.
- **P11: noise.**
  - With 1 cm noise, up to 9 false pendulum lines were found, and the robot waited about 2 minutes at lines that didn't exist. **Fix:** a line needs 4 points per scan pair on 5 scan pairs (was 2 points on 4).
  - Navigation got stuck at very tight gaps with noise, such as 0.34 m between an obstacle and a wall, which left a 4 cm window when the lanes stepped in 4 cm. **Fix:** lanes in 2 cm steps.
- **P12: the stricter line test confirmed the second line too late.** The robot was already at that pendulum during the final approach, and the goal was called behind it.
  - **Fixes:** also stop short of unconfirmed lines with movement in the last 2 s; the goal needs a wall at ±45°.
- **P13: Gazebo test by the user, robot didn't take a visible gap** (user's screenshot; obstacles moved in the Gazebo window).
  - Passing sideways, the robot's length is across the gap, so the code needed 0.22 + 2 × 0.03 = 0.28 m. The gaps were 0.26 m (litter bin to wall) and 0.28 m (can to wall).
  - The layout was rebuilt in 2D from Gazebo's live obstacle poses, the mesh sizes and the world file:
    - `side_margin` 0.03: never got through in 90 s;
    - 0.02: through in 24 s;
    - 0.01: through in 20 s.
  - With 0.02 on the usual tests: Gazebo-style pendulums 12/12 (was 10/12); navigation 12/12 and 14/14; no wall or obstacle contacts; with noise 11/12.
  - **Conclusion:** a tuning issue (the side margin). It's a trade-off on the real robot, where a smaller margin leaves less room for noise.

### Final test results (2D simulator, `side_margin` 0.03)
| Test | Reached the goal / got through | Runs with no pendulum contact |
|---|---|---|
| Navigation only: 12 layouts / with noise / 14 new layouts | 12/12, 12/12, 14/14 | – |
| Real-style pendulums, 12 layouts | 12/12 | 6/12 (7 contacts) |
| Real-style pendulums, 14 new layouts | 14/14 | 3/14 (15 contacts) |
| Real-style pendulums, with noise (1 cm LiDAR, 3 mm pose) | 11/12 (one 0.34 m gap failed) | 8/12 |
| Gazebo-style pendulums | 10/12 (12/12 with `side_margin` 0.02) | 10/12 |

Real-robot mode (720-reading scans, maze-frame poses, start headings 96.3° / 0° / −45°): no errors, about 2 ms per tick. The grid version took 240–310 ms.

- **Observed (Gazebo, user, 2026-11-05):** the user ran this code as their local `obstacle_course2.py`: "works pretty well now on gazebo".
- **Still open:**
  - With real-style pendulums, there is still about one contact per run.
  - The real pendulums' speed is unknown. The test pendulums swing with a 2.6–4.5 s period; slower real ones would make the crossing much safer.
- **Planned, to be implemented later:** pendulum period detection. While waiting at a line, measure each pendulum's period from its passes through the lane. Predict when it will next reach the lane, and only cross if the predicted clear time is longer than the crossing time (about 1.4 s).

### Real-robot version (2026-11-06, still Claude's reference code)
The user asked for heading control and whatever else the reference needed to run on the real robot.
1. **Heading hold:** every movement goes through `drive()`. It turns at 1.5 × the heading error (in radians), up to 0.5 rad/s, and only moves when within 10° of the target heading.
   - **Why:** the robot must never rotate. All the logic assumes the robot's frame stays lined up with the course, and on the real robot the heading drifts.
2. **Start-up checks** (real robot only):
   - **Spin check:** turn left at 0.4 rad/s for up to 2 s; the heading must rise by 3°. **Why:** if the heading's sign were reversed, heading hold would spin the robot.
   - **Square up with the walls** (`wall_alignment`): try turns up to ±25° and pick the one where the LiDAR points line up most sharply with the robot's axes. **Why:** the robot is never placed exactly straight.
   - **Heading check:** slide 10 cm in the direction with the most room, and compare the way the robot actually moved with the Optitrack heading. If they differ by more than 15°, use the measured heading. **Why:** a wrong `heading_offset_deg` would corrupt all the world-frame maths (LiDAR points, visited positions, pendulum lines). The 15° threshold is there because over 10 cm the measurement scatters by several degrees: in the 2D test, an 8° error was measured as anything from 4.5° to 12°.
3. **Stop if the pose or LiDAR stops updating** for 0.5 s, and carry on when the data comes back.
4. **Motion test:** an `inf` reading counts as "can't tell". **Why:** real LiDARs report missed returns as `inf`, which would otherwise look like something moving.
5. **The course rotation doesn't matter.** No absolute direction is used anywhere. The real course's forward direction is about +96° in the maze frame, which is a +96° rotation from the Gazebo axes.

### More problems and fixes
- **P14: the spin check failed on the real robot** with "-0.0 deg": the heading didn't change during a 0.5 s turn at 0.25 rad/s. Either the base didn't respond to that short, slow turn, or the pose didn't follow.
  - **Fix:** turn at 0.4 rad/s for up to 2 s, pass at +3°, and say which failure it was: "didn't turn" or "heading went the wrong way".
  - **Observed (user):** after the update, the robot started and ran on the real robot, including the pendulum part.
- **P15: pendulums with small swings (user, real robot).** After swinging out of the robot's way, a pendulum came back very soon and hit the robot while it was crossing; or its swing was so small that it never fully left the robot's lane. The user's observation for the fix: "if the amplitude of the swing is small, there would be lots of free space on the left/right of the swinging obstacle that never gets hit by the obstacle."
  - **Why:** a pendulum's period hardly depends on its amplitude. A small swing comes back just as soon, but spends more of each period in front of the robot.
  - **Fix 1, go around the swing:** while waiting, record the pendulum's sideways range from its moving points over the last 5 s. After watching for 3 s, if there's room beside that range for the robot plus 10 cm on each side, slide sideways (still short of the line) and cross there. If the swing grows into that lane, give it up.
  - **Fix 2, only time a crossing when the gap is long enough:** measure how long the lane stays clear on each swing, separately for each side the pendulum leaves to. Only take "the pendulum just left the lane" if that clear time, minus the time since it left, is at least the crossing time (about 1.7 s) plus 0.3 s.
- **P16: small swings went undetected in the 2D test.** The robot drove straight through without finding a line. Slow swings uncover too few points between scans 0.1 s apart.
  - **Fix:** find lines by comparing each scan with the one from 0.3 s earlier. Walls still never become see-through, so this doesn't add false movement: with no pendulums and with noise, there were no false crossings.
- **P17: watching kept restarting.** The robot started watching a line from scratch every time the line's estimated position moved slightly closer.
  - **Fix:** track the line itself rather than its position.
- **P18: with wide, fast swings, fix 2 can wait forever.** If the swing covers the whole corridor, there's no room beside it, and the lane may never stay clear long enough.
  - **Switch:** `self.max_wait` = seconds to wait at one line before taking the old gamble. The default `None` waits as long as it takes.

**Results** (2D, real-robot mode, 12 layouts each; no wall or obstacle contacts in any run):

| Pendulums | `max_wait` | Reached the goal | Runs with no pendulum contact |
|---|---|---|---|
| small swings (half-width 0.10–0.20 m) | None | 12/12 (all crossed beside the swing) | 12/12 |
| medium swings (0.20–0.35 m) | None | 11/12 (1 still waiting) | 12/12 |
| small to medium, with off-angle start, wrong heading, drift and noise | None / 20 s | 11/12 / 12/12 | 12/12 / 12/12 |
| wide swings | None | 4/12 (the rest still waiting) | 12/12 |
| wide swings | 20 s | 12/12 | 7/12 |

- **P19: after a false pendulum line was ignored, the robot declared the goal (user, real robot, `line_depth` 0.05).** A static obstacle was taken for a pendulum. The robot waited, then correctly decided it wasn't one ("not a pendulum line: ignoring it"), then said "goal reached" and stopped for good.
  - **Why (a real logic bug, reproduced in 2D):** ignoring a line was done by marking it as crossed. But "a line has been crossed and no line is ahead" is exactly the condition for "past the pendulums: drive straight to the goal". The robot drove forward until blocked, and something wide in front counted as the end wall.
  - **Fix:** a separate "ignored" mark. Ignored lines are left out of stopping, waiting, crossing and the past-the-pendulums check. After the fix, the same reproduction carries on navigating.
  - **The false detection itself** is occasional: the motion test isn't perfect on real LiDAR data. The 2 s rule caught it correctly.
- **P20: waiting at a full-width swing with no movement (user, real robot).** The robot crossed the first pendulum beside its small swing, then stopped in front of the second, which "swings almost the full width of the corridor", and didn't move for a long time.
  - **Why:** no room beside the swing, and the lane never stayed clear long enough. A full-width swing passes the middle of the corridor twice per period, so a lane in the middle is only clear for about a third of a period at a time. The crossing needed about 1.7 s, plus 0.3 s margin.
  - **Fixes:**
    - `line_depth` set to the real pendulum's thickness plus a margin: 3 cm + 5 cm = 0.08 (was 0.20, sized for the simulator's 6–16 cm pendulums). The crossing now takes about 1.3 s.
    - With no room beside the swing, wait near the nearer wall, with the robot's side 8 cm from it. There the pendulum only reaches the lane once per swing (at the end of it), so the lane is clear for about 60% of a period. Gap timing restarts at the new spot.
    - A status line every 2 s while waiting: the swing range, the last measured clear time, and the time needed.
- **P21: a wall counted as "in the lane".** When the robot waited within 5 cm of a wall, the lane check (robot width + 5 cm each side) included the wall, so the lane never looked clear.
  - **Fix:** walls are left out of the lane check. A wall also has points at the same sideways position well before and well after the line; a thin pendulum doesn't.
- **P22: a second pendulum found too late.** A small, slow swing hidden behind the first pendulum was only noticed when the robot was 11 cm away, and it dropped off again after 2 s. The robot drove into its swing area, then froze there, because new possible lines kept appearing right in front of it.
  - **Fixes:** stop for a possible line only while there's still room to stop short of it, and keep stopping for up to 4 s after the last movement seen there. If a line is confirmed when the robot is already at it, finish crossing it straight away instead of waiting inside it.

**Results after P19–P22** (2D, real-robot mode, 3 cm pendulums; no wall or obstacle contacts, and no pendulum contacts, in any of these runs):

| Pendulums | Reached the goal |
|---|---|
| small swings, 14 new layouts | 13/14 |
| small to medium, with off-angle start, wrong heading, drift and noise | 11/12 |
| wide swings, 12 layouts / 14 new layouts | 6/12 / 5/14 |
| no pendulums, with noise | no false crossings |

With wide swings, the rest were still waiting when the 4-minute test ended (some crossed after about 2 minutes), because `max_wait` is None.

- **Reverted (2026-11-06):** after trying the P19–P22 version on the real robot, the user said: "this newest version of code is no good. it has a lot of problems. we should revert back to the last version", and that the previous version "was working pretty well until your last edit". The reference was reverted to `d17d32a`, the version before P19–P22. P19–P22 above are kept for the record; none of them is in the current reference, so the false-goal bug (P19) is present again.
- **Reverted again (2026-11-06):** after trying `d17d32a`, the user said: "this version doesnt even work properly in simulation... hmm, can we revert back one more commit?" The reference was reverted to `1b8e7c1` (the real-robot version with the spin-check fix). So the P15 changes (crossing beside small swings, timing only long-enough gaps, comparing with the scan 0.3 s back, the `max_wait` switch) are not in the current reference either.
- **Fixed from `1b8e7c1` (2026-11-09, commit `5f1dfb7`).** User: "can you fix obstacle_course2.py based on the problems i reported in a robust, as simple as possible way? ... simple is almost always better than complex."
  1. **P19, false goal:** an ignored static line is now flagged `ignored` and no longer counts as a crossed pendulum for the final approach (2 lines). Repro (`harness/false_line_repro.py`): the old version said "goal reached" right after ignoring the line; the new one carries on.
  2. **P15, small swings:** fix 1 of `d17d32a` is back (cross beside a small swing, scans compared 0.3 s apart, keep watching the same line). Fix 2 (gap timing, `max_wait`) is left out, because it caused P18/P20 (waiting forever at a full-width swing). Two changes:
     - **One side-lane try per line.** Slow Gazebo swings fooled the 5 s swing estimate, so `d17d32a` slid back and forth between side lanes forever. That is likely why "d17d32a doesn't work properly in simulation". If the swing reaches the chosen lane, it falls back to the old timing.
     - **A 5 s watch before choosing** (was 3 s). With 3 s, one run judged the swing too early, slid into it and got stuck there being hit.
  - **Result (2D, `reactive_study.py`, 21 layouts each):**
    - Gazebo-style full-width swings: 19/21 → 21/21 at the goal; runs with a pendulum contact 3 → 1.
    - Small hand-released swings (half-width 0.15–0.3 m): 19/21 → 21/21; contact runs 7 → 2.

## V2-D2: Write my own reactive controller instead of using Claude's reference (2026-11-06)
- **Decision:** the user will write their own reactive controller from the original template (`obstacle_course_template.py`, commit `88d3f81`), rather than use Claude's reference code (V2-REF).
- **Why (in the user's words):** "i dont want to use the one that you have just coded without being able to do it myself."
- **Planned:** pendulum period detection, to be implemented later (see V2-REF, "Planned").

## V2-R1: Real-robot runs of Claude's reactive reference code (2026-11-06)
- **Code:** Claude's reference code (V2-REF), copied into the user's local `obstacle_course2.py`.
- **Observed:**
  1. **First start:** "START-UP CHECK FAILED: turning left changed the heading by -0.0 deg". Fixed by P14.
  2. **After that:** the robot starts and runs on the real robot, including the pendulum part. But it doesn't always clear pendulums with small swings (P15).
  3. **With fixes 1 and 2 (P15–P18, `max_wait` None):** "it looks good now". It crossed the first pendulum, which had "a pretty small amplitude, so there was a big clearing on the right that it can cross through". Then it detected and stopped in front of the second pendulum, "which swings almost the full width of the corridor, then it just stopped there without moving at all for a long time". The terminal did not say it had reached the goal, so it hadn't wrongly decided it was finished.
  4. **With `line_depth` = 0.05:** a static obstacle was mistaken for a pendulum. The robot stopped and waited, realised it wasn't a pendulum, then decided the goal was reached and stopped for good (P19).
- **Fix:**

## V2-M2: The robot stops for good in a corner, though its back and left are open (Gazebo, own code) (2026-11-06)
- **Setup (commit `8fbfe1b`):** the feasibility check `blocked()` slides the robot's rectangle along each direction in 1 cm steps, growing every step's rectangle by 1 cm on **all four sides**, so the area between steps is covered.
- **Observed (Gazebo):** the robot sat in the corner between the right wall and the gate wall, not touching either, with its back and left wide open. It printed `goal score:None,proximity score:None` and `nowehre to go` and never moved again.
- **Why:**
  - No direction was feasible. The growth on the sides facing *away* from the motion still reaches back over walls the robot is moving away from. Moving back (180°), the first step's rectangle still sticks out 1 cm to the right, onto the right wall; moving left (90°), it still sticks out 1 cm ahead, onto the gate wall.
  - **2D test, both walls at the same distance from the robot:**

    | distance | feasible directions |
    |---|---|
    | 12 mm | 90°–180° |
    | 8 mm | 112.5°–157.5° |
    | 3.5–5 mm | 135° only |
    | 2.5 mm or less | none |

  - It got that close probably because the scan is older than the robot's pose: scans arrive slower than the 20 Hz loop, while the robot moves 1.5 cm per tick. Not measured in Gazebo. In the 2D harness, "nothing feasible" happened in 6 of 189 runs, each time pressed against a wall.
- **Options considered:**
  - **Remove the growth:** fixes the deadlock, but loses the 1 cm buffer. In 21 2D layouts, wall contact went from 2 runs (2 ticks) to 6 runs (55 ticks) with an exact LiDAR.
  - **Grow only on the side(s) the robot is moving towards:** the user's earlier idea of growing only by the shift. Keeps the buffer in the direction of travel.
- **Fix (commit `ea7efd4`):** grow only towards the motion: `x_lo, x_hi = cx - L - (step if c < 0), cx + L + (step if c > 0)`, the same for y, with a 1e-9 tolerance for cos/sin ≈ 0.
  - **Corner test:** with both walls 2–5 mm away, all of 90°–180° are feasible.
  - **2D harness, 21 layouts, 60 s:** 21/21 through with an exact LiDAR and with 1 cm noise. No "nothing feasible" stops. Contacts: 2 runs (5 ticks) exact, 1 run (10 ticks) noisy.

## V2-D3: Room bonus: look past the feasibility distance, up to 1 m (own code, commit `b6ac3e0`) (2026-11-07)
- **Decision:** add `room_score = w_free * free_distance(...)` to each direction's score, with `w_free` = 1.0. `free_distance` uses the same stepped rectangles as `blocked()`, but keeps stepping out to 1 m and returns the last clear distance. Feasibility is unchanged: `blocked()` with `test_distance`.
- **Why (user's words):** "i noticed that the robot takes very long to 'bounce out' of deadends/corners, so adding this bonus room score would help to tip the balance towards unfavourable directions such as left and backwards. it would also help the robot go through the gap towards dynamic obstacles i believe." Claude had suggested it as the highest-value step, because cutting Claude's reference down to a 15 cm look-ahead took it from 21/21 to 13/21. The user put lane shifts on hold ("im not very sure how much i really need this lane shift thing").
- **Same commit:** `test_distance` 0.10 is used for feasibility, the commitment, the path check and the proximity spot; separate sim and real attribute blocks, with the sim robot at its Gazebo size (half 0.1075 × 0.095 m, LiDAR 0.041 m ahead of the centre).
- **Observed (Gazebo):** "the robot still bounces for a very long time in top right corners and deadends before exiting".
- **2D harness** (Gazebo-size robot, 21 layouts, no pendulums, 90 s): 20/21, mean 28.4 s, slowest 57.3 s, 3.8 turn-backs per run. A turn-back is a decision that reverses the direction by 135° or more.

## V2-D4: Hysteresis on the chosen direction (own code, commits `4d3f96f`, `8fbfe1b`, `78f31fe`) (2026-11-06 to 07)
- **Decision:** keep the current direction while it's feasible, unless the new best score beats it by more than 0.2. The user put the room bonus on hold at first: "i think my code could work if i just add a hysteresis control."
- **Bug (`4d3f96f`):** `best_thetas, best_score, goal_score, proximity_score, best_score = theta, score, ..., best_score` lists `best_score` twice, so it was set back to −inf. Every direction then passed `score > best_score`, and the last feasible one in the list always won, so the robot drove backwards. The user fixed it themselves, then used Claude's shorter version of the block (`8fbfe1b`).
- **First version (`8fbfe1b`):** compared the new best with the score the old direction had *when it was chosen*. Harness: 20/21 with it against 21/21 without, mean 30.1 s both.
- **Change (`78f31fe`):** compare with the old direction's score *now*. `best_direction` returns every feasible direction's score in a dict, and the check is `new_score <= scores[self.last_best_theta] + 0.2`.
  - **Why:** the room score shrinks as the robot nears a wall, so the stored score stays too high. The robot kept going until the direction was blocked, then rebounded.
  - The first push used `scores[self.last_best_score]` and crashed with a `KeyError` (the dict is keyed by direction). Fixed in `e390b2d`.
  - The same commit raised the sim `visited_weight` from 0.1 to 0.2.
  - **Harness, with the fix:** 21/21; mean 26.2 s exact LiDAR, 26.8 s with 1 cm noise; slowest 41.6 s and 46.8 s; 0.4 and 0.7 turn-backs per run. Before (`b6ac3e0`): 28.4 s, 57.3 s, 3.8.

## V2-D5: Novelty bonus as well as the proximity penalty (own code, commit `e390b2d`) (2026-11-07)
- **Decision:** add `novelty_score = w_new * min(1, nearest visited point / probe)`, judged at the point `min(probe, free)` along each direction, with `w_new` = 1.5 and `probe` = 0.4 m. Visited points include the current position, so a direction blocked soon gets low novelty.
- **Why:**
  - **Observed:** still bouncing "in the top right corner and in tight pockets".
  - **Claude's explanation:** the goal (front-right) points into a top-right corner, a 2.0 goal gap against going back-left. The room score closes only about 0.7–0.9 of it, and one pass adds only about 1 of proximity penalty, so the robot has to re-enter the corner once or twice before leaving. Novelty takes away up to 1.5 from every direction into an already-visited corner after a single visit.
- **User's question 1:** "why dont we just increase the weight of the visited proximity score?" Harness, proximity weight only, no novelty:

  | `visited_weight` | through (exact / noisy) | mean time | slowest | turn-backs |
  |---|---|---|---|---|
  | 0.1 | 20 / 21 | 28.4 / 30.2 s | 57 / 55 s | 3.8 / 3.6 |
  | 0.3 | 21 / 21 | 29.8 / 31.3 s | 59 / 82 s | 0.3 / 0.8 |
  | 1.0 | 20 / 20 | 31.4 / 29.9 s | 61 / 60 s | 0.4 / 0.5 |
  | 3.0 | 15 / 20 | 33.6 / 38.9 s | 66 / 78 s | 0.8 / 1.2 |
  | 0.1 + novelty | 21 / 21 | 26.8 / 27.6 s | 39 / 41 s | 0.3 / 0.8 |

  A heavier penalty stops the turning back but is slower. It has no upper limit, so it also blocks the way back out of a dead end, and it only sees the next 10 cm. Novelty is capped at 1.5 and judged up to 40 cm out.
- **User's question 2:** "why dont we just have a novelty score then, and dont use the proximity score anymore?" Harness, novelty with proximity 0.2 against novelty only: 21/21 (24.8 / 25.1 s) against **17/21 and 19/21**. The failed runs rocked between two spots for the full 90 s (170–240 turn-backs). Novelty bottoms out at 0 once everything nearby is visited, so it can't tell "once" from "100 times". The growing proximity penalty is what guarantees the robot eventually leaves. Claude's reference uses both for the same reason (V2-REF, P5).
- **Result (`e390b2d`, harness):** 21/21; mean 24.8 s exact, 25.1 s noisy; slowest 38.0 s and 49.0 s; 0.3 and 0.7 turn-backs per run; no contacts. Claude's reference in the same test: 24.1 s mean, 37.7 s slowest.

## V2-D6: Goal direction changed from 315° to 337.5° (own code, local change after `e390b2d`) (2026-11-07)
- **Decision:** `self.goal_direction = np.deg2rad(337.5)`, i.e. forward and slightly right, instead of 315° (front-right).
- **Why:** the user found that "the door/gap detection is not always very smooth". Claude's 2D measurement, over 21 layouts, from 0.6 m before the door to 0.3 m past it (a straight run takes about 3 s):
  - **Times:** 3–7 s in 16 layouts, 15–23 s in 5, mean 7.9 s. Claude's reference: mean 5.9 s, 2 slow.
  - **Cause 1:** the robot reached the door wall 2–5 mm off the gap's edge, so going straight clipped the edge and was blocked. In seed 19, the gap edge was at y = −2.435 and the robot's right side at about −2.437 to −2.44. With the goal at 315°, sliding right along the door wall (270°) scored the same goal as going forward (0°), since both are 45° away. With full room and novelty too, the robot slid away from the gap into the corner.
  - **Cause 2:** the goal pulled the robot past the gap into the corner between the door wall and the right wall, which is exactly front-right. In seed 2, going straight through scored 2.98 against 2.82 for carrying on diagonally, within the 0.2 hysteresis margin, so it kept going diagonally.
  - Claude pointed to lane shifts (V2-REF, P3) as the fix for cause 1.
- **Observed (user, Gazebo):** "i just changed the general goal to 337.5 degrees and now it is a lot better."
  - With 337.5°, going forward (22.5° off the goal, score 0.92) beats sliding right along the door wall (67.5° off, score 0.38).
  - Not yet measured in the 2D harness. Not pushed at the time of logging (GitHub `main` still has 315°).

## V2-D7: Lane shifts: line up with gaps slightly off the robot's path (own code, commit `38be8f8`) (2026-11-08)
- **Decision:** for each of the 16 directions, also try starting from positions shifted sideways by up to ±12 cm, in 2 cm steps. Use the shift that gives the most room, and slide sideways onto it while driving. Ported from Claude's reference (`best_lane`, V2-REF P3).
  - A shift only counts if the robot can first slide that far sideways (free distance at θ ± 90°).
  - Each shift is valued as `min(free, 0.6) − |shift|`, so it has to buy more room than it costs. If the straight lane already has 0.6 m or more, no shifts are tried.
  - **Feasible** = the best lane's free distance ≥ `test_distance`. With no shift this is identical to the old `blocked()` test.
  - **Path check:** uses the lane's free distance.
  - **Room score:** uses the lane's free distance.
  - **Movement:** sideways speed is 2 × shift (capped at 0.15 m/s). While shifting, forward speed is at most the unshifted free distance, so the robot creeps past an edge instead of hitting it. Total speed is capped at `self.speed`. The lane is recalculated every tick, so the shift shrinks to 0 as the robot lines up.
- **Why:**
  - **The narrow-gap map.** In a Gazebo map with a narrow gap (about 0.28 m) between a litter bin and the bottom wall of the second corridor, the user observed that the robot "keeps circulating around the left, near-top-wall side of the course without being able to go down low enough to find the gap ... eventually it found the gap but it took really long". In a 2D rebuild of that map (walls from the world file, obstacles at the poses Gazebo reported), the user's code took 62 s to get past the bin.
  - **Two causes found:**
    1. The goal (337.5°) pulls towards the top wall, away from the gap.
    2. The gap only fits the 0.19 m robot if its centre is inside a band about 9 cm wide.
  - **Claude's reference on that map.** The user tested it in Gazebo with `side_margin` 0.01: "it finds the narrow gap and crosses it successfully really fast. Is that because of its lane shift?" In the 2D rebuild, the reference got past the bin in 22 s, never without lane shifts, and in 59 s with the goal at 337.5°. It assumes a 0.24 m robot, so with 1 cm margins its lane had only about 2 cm of slack.
  - The door analysis (V2-D6, cause 1) had also pointed to lane shifts.
- **Result (2D harness, same code as pushed):**
  - **Narrow-gap map:** past the bin at 21.2 s (was 62.4 s) with an exact LiDAR. With 1 cm noise, 5 of 6 runs at about 21 s (was 65–234 s); 1 run didn't get past in 240 s.
  - **21 layouts:** 21/21, mean 23.1 s (23.0 before), slowest 38 s (37 s before), no contacts. With 1 cm noise: 21/21, mean 25.8 s.
- **Cost:** `all_lanes` runs every tick: 7–21 ms on the development PC, against a 50 ms tick. Not yet timed on the robot. `blocked()` and the old `feasible_directions()` are no longer used.

## V2-D8: Minimum commanded speed (own code, commit `a1ca979`) (2026-11-08)
- **Decision:** if the total commanded speed is above 0 but below `self.min_speed`, scale forward and sideways up to it, keeping the direction. The value is 0.075 m/s in both the sim and real blocks.
- **Why (user's words):** "it seems to move really slowly at times ... im worried that if the commanded speed is too low, the real robot wont be able to move at all in real life."
  - **Where the slow commands come from (2D harness, 21 layouts):** only lane shifting. When the straight path is blocked right ahead, forward speed drops to about 0 and the robot slides at 2 × shift (0.04 m/s for a 2 cm shift). 2.4% of moving ticks were below 0.10 m/s, all while shifting; the lowest was 0.04 m/s.
  - Real motors usually have a deadband. Gazebo's VelocityControl moves at any speed, so the sim can't show it. The robot's actual threshold is not yet measured, so 0.075 is a placeholder.
  - Scaling keeps the direction, so a robot creeping past an edge (forward 0) still doesn't drive into it.
- **Result (with 0.08):** 21/21 exact and noisy, no contacts; narrow-gap map 20.5 s, and all 6 noisy runs about 21 s.

## V2-D9: Faster `free_distance`: all steps checked at once (own code, commit `a1ca979`) (2026-11-08)
- **Decision:** `free_distance` checks all 1 cm steps at once (a table of steps × points, then `np.any(axis=1)` and `np.argmax` for the first hit) instead of a Python loop. It also skips LiDAR points too far from the path to be hit. Same rectangles, same answers: identical in 144,000 random cases, and identical robot paths in the harness.
- **Why:** the user asked whether Claude's reference running fine on the real robot meant their code would too ("would that translate to my code?").
  - **Measured on the development PC (Core Ultra 9):** the user's loop took 14–23 ms median per tick (worst 44 ms, one 319 ms spike), against 2.0–2.5 ms for Claude's reference. With lane shifts, `all_lanes` stepped 16 directions × up to 13 lanes every tick.
  - The code runs on the robot's own computer, which is likely several times slower, against a 50 ms tick.
  - User's commit message: "realised my code was doing lots of calculations and iterations, so used some numpy magic to make things faster preemptively jic the real robot cannot process as fast".
- **Result:** about 5 ms median per tick, worst 17–19 ms, on the same PC. Not yet measured on the robot (`ros2 topic hz /cmd_vel` should show about 20 Hz).
- **Same commit:** the real robot's `W` changed from 0.11 to 0.10, the recorded real half-width (0.20 m wide). With `W` larger than the robot, a wall touching the robot falls inside the ignored "own body" rectangle. That was the Gazebo stuck-against-wall failure, where `W` was 0.11 against the Gazebo robot's 0.095.

## V2-D10: 1 cm side clearance in `free_distance` (own code, commit `e228519`) (2026-11-09)
- **Decision:** grow the robot's rectangle by `m` = 1 cm on every side except the side it moves away from; the existing 1 cm step growth on the leading side is unchanged:
  `x_lo = cx - L - (0 if c > 1e-9 else m) - (step if c < -1e-9 else 0)`, and likewise for `x_hi`, `y_lo` and `y_hi`.
- **Why:** after the faster `free_distance`, one harness run (seed 5) touched a thin can for 14 ticks. The robot slid sideways (270°) with the can's edge exactly level with its front edge, a 0.5 mm graze. Growing only the leading side leaves 0 clearance on sides parallel to the motion.
- **Options discussed:**
  - **Enlarge `L` and `W`:** gives the same results, but re-creates the ignored "own body" band (the Gazebo stuck-against-wall failure).
  - **Grow all four sides (the user's idea):** tested. It brings back the V2-M2 corner trap (no feasible direction with both walls 2 mm away), and more contacts: 7 runs / 90 ticks.
  - **Not growing the side being left** avoids both. User: "isnt the simplest way to add buffer just to alter the line in free_distance() ...?" The final four lines are that idea, with the side being left excluded.
- **Result (2D harness):**
  - **21 layouts:** 21/21 exact (24.4 s mean) and noisy (25.1 s), no contacts.
  - **Narrow-gap map (0.28 m gap):** 21.2 s exact, but 0/6 with 1 cm noise (6/6 with no clearance).
  - **Clearance sweep:** 0.5 cm gave 4/6; 2 cm failed exact too.
  - **Trade-off:** clearance against squeezing through very tight gaps when the LiDAR is noisy.

## V2-D11: Pendulum handling: hug the right wall, wait, dash (own code, commit `1cf8705`) (2026-11-09)
- **First version (not kept):**
  - At the user's request ("what are your recommendations ... simple is robust"), Claude recommended detect → stop short → watch → cross. The user specified: slide to the wall with more room, about 3 cm from it, and cross when the pendulum leaves the front.
  - Claude wrote and tested it (Claude's p12). Changes found by testing: compare scans 0.5 s apart in both directions; a whole-corridor window; the line is fixed once found; a wider lane on the open side; false-alarm exits.
  - Result: 0 hits in 84 2D runs. But on the user's first random Gazebo map "it failed on the second obstacle".
- **New plan (user's words):** "once the robot detects a pendulum, it chooses an arbitrary side of the wall to slide towards (lets say default right side). then once it is only 3cm away from the wall, it make sure it is only a short distance from the line of the pendulum (by moving forward towards the pendulum line until maybe it is only 5cm away) and then waits for the dynamic obstacle or at least a part of it to appear once in front of itself. once the dynamic obstacle leaves the front of the robot, the robot will dash across ... to just before the second pendulum and wait again". A timeout rule: if the obstacle doesn't appear in front within 10 s (Gazebo) / 4 s (real), cross anyway.
  - **Confirmed details:** trigger on the first detection, but only once the straight path to the stopping point is clear (so the door isn't in the way); 'in front' = from the robot's front to 10 cm past the line; stay on the same wall for the second pendulum; the dash ends with the rear 10 cm past the line; the timeout counts from being in position; after the second pendulum, drive straight on until the robot can't go further (the goal, as defined in the real-trial facts).
- **Changes Claude made, found by testing in 2D:**
  1. Creep forward first, then slide right. Sliding first hit the door posts: 30 obstacle hits.
  2. 'In front' is widened 10 cm on the open side. The user had approved robot-width only; flagged to them. Near-miss swings had hit the robot during the timeout dash.
  3. Dash only if the whole dash path is clear.
  4. Count a pendulum as crossed only if 5 or more moving points were seen at its line while waiting. Noise-triggered false lines had sent the robot to the "goal" early.
  5. Something static in front for `t_wait` → not a pendulum, back to navigating.
  6. Back up if already closer than 5 cm.
- **Result (2D, Gazebo-size robot, 21 layouts per condition, 252 runs: gazebo/real style × normal/small/full-width/noisy, plus real-style with `t_wait` 4 s):**
  - all reached the goal at the end wall, 0 obstacle hits;
  - 1 pendulum hit, a sim artefact (a full-width test pendulum swinging through the side wall);
  - mean 40-59 s.
  - No pendulums: unchanged with an exact LiDAR (23.1 s); 36 s against 24 s with 1 cm noise (false alarms).

## V2-M3: Static obstacles detected as pendulums in Gazebo (own code, `1cf8705`) (2026-11-09)
- **Observed (user):** "it falsely detects quite some static obstacles as dynamic obstacles, especially those green cans on gazebo, perhaps due to their cylindrical and irregular shape."
- **Investigation:**
  - The green bin's outline at the LiDAR's height is ribbed (radius 0.200-0.226 m, from the mesh). Modelled exactly in 2D, it caused no false detections.
  - A 15 s recording of the user's running Gazebo showed none, but the robot wasn't near the bins; Gazebo then closed.
  - **Reproduced by scan lag.** Gazebo's LiDAR is 10 Hz and the loop 20 Hz, and each scan is paired with the robot's pose at the tick it's first seen, so a scan can be up to about 100 ms older than its pose (about 3 cm at 0.3 m/s). While the robot moves, points on small or thin obstacles land a few cm off and look moved.
  - **2D, no pendulums, 21 layouts:** false pendulum procedures 0 / 7 / 32 with 0 / 50 / 100 ms lag; got through 21 / 18 / 17.
  - Not confirmed as the cause in the user's Gazebo.
- **Proposed fix (Claude's q8):** after the door check, stand still 2 s and count moving points at the line from 1 s on. Standing still, the pose doesn't change, so lag can't fake motion. Fewer than 5 means not a pendulum: ignore that spot and carry on.
  - With 100 ms lag: no pendulums 21/21 (was 17/21); pendulum runs reached the goal 20-21/21 (was 16-20/21).
  - Without lag: unchanged. Cost: 2 s per check.
- **User's questions:**
  - "would a simpler fix be just changing the frequency of the timercallback to match that of the lidar scan?" Answer, from reasoning only: no. The timer and the LiDAR aren't synchronised, so the lag stays 0-100 ms. A constant lag only cancels when the robot's velocity is the same for both scans, and it changes direction every 10 cm. And every tick-counting value (clock, visited points, check rate) would change.
  - "standing still for the check means my robot would literally pause, seemingly for no reason?" Answer: only when a detection passes the door check. A real pendulum stops there anyway, and a false alarm costs 2 s. Alternative: creep slowly during the check (error = speed × lag).
  - "is there a good simple way where we can just eliminate scan lag?" Answer: only partly. A second `scan` subscription can save the pose at the moment a scan arrives (up to 50 ms better). Transport delay, unmatched clocks (Optitrack against the robot) and the spinning real LiDAR (one scan collected over about 0.1 s) remain.
- **Fix (adopted, in commit `852ded4`):** the user adopted Claude's stand-still check as proposed above. After the door check, a new `check` mode stands still for 2 s and counts moving points at the line from 1 s on. Fewer than 5 means not a pendulum: the spot goes into `false_lines` and the robot carries on (`navigate`, or `find` after the first crossing). Otherwise it goes on to `position`. The user chose it after the questions above (pausing only happens when a detection passes the door check; the lag can't be removed simply).

## V2-D12: Heading drift fix: hold the heading, line up with the wall at each pendulum (own code, commit `852ded4`) (2026-11-09)
- **Problem (Gazebo):** after the first half of the course, the robot's heading had turned −5.5° (nose into the right wall), probably from a contact. At the first pendulum it slid to the wall and then stood in `wait` forever: the dash check (`free_distance` ahead ≥ about 0.365 m) never passed, because the tilted path ran into the wall (free ahead measured live: 0.13–0.30 m). The swinging pendulum kept resetting the static timeout.
  - **Same cause, found in 2D:** `goal` stops at the side wall ("goal reached" 0.2–1.2 m short), and the in-front zone can catch the wall, so a real pendulum is dismissed.
  - **Tolerance:** 2° into the wall already stuck every run; 1° was fine.
- **User remarks:** "something like this might likely happen on the real robot too, since the real robot would inevitably drift or be placed down at the start not facing exactly forward." And: "the course itself is not going to be perfect ... the corridor may not point exactly straight ahead, parallel walls may not be exactly parallel".
- **Options considered:**
  - **Slide away from the wall when the dash is blocked:** unsticks it, but `goal` and the in-front zone still fail.
  - **Turn once to a fixed straight-ahead heading:** fails once the robot drifts again, and a fixed course heading is fragile against a crooked wall.
  - **Chosen, the user's idea:** "before the robot crosses the pendulum, we make it align itself parallel to the wall that it is closest to first, instead of making its heading exactly align straight ahead".
- **Decision:**
  1. Hold the start heading for the whole run: if more than 1° off, stop and turn back at 0.3 rad/s. Never during the dash.
  2. At each pendulum, once at the right wall (end of `position`), fit a line to the wall points alongside the robot (`np.polyfit`) and hold that wall's direction from then on. Fewer than 5 points: keep the old heading.
- **Result (2D, 21 layouts each):** start or course off by −5° to +3° gave 21/21 at the end wall with 0 hits, also with ±3°/m drift and 5 mm LiDAR noise. The weak case is starting 5° or more with the nose away from the wall (8 pendulum hits): the slide to the wall carries the robot into its 5 cm gap before it lines up. Mitigation: place the robot square at the start.
- **To check on the real robot:** that 0.3 rad/s actually turns it, and that the first correction turns the right way.

## V2-D13: Debug prints, tuning and real-robot refinements before the first real test (own code, commits `6ac5ab8`, `1ddbf14`) (2026-11-09)
- **Prints.** The user asked for "print debug statements that reflects on the terminal what state the robot is in ... and others that provide valuable real time info of what the robot is experiencing".
  - They chose plain `print()` with no timestamps: "im using my own eyes to match what prints on terminal and where the robot is on the gazebo map".
  - **What prints:** one line at each pendulum event:
    - pendulum detected (with the line's distance);
    - stopping 2 s to check; false alarm (with the moving-point count); pendulum confirmed;
    - at the wall, lining up (with the turn angle); pendulum in front; something static in front (not a pendulum);
    - GO (pendulum has passed / timeout); crossed pendulum N; crossed but nothing moved;
    - pendulum line already behind (forgotten); no second pendulum found, so driving to the goal.
  - Removed the per-decision score print, which buried everything else.
- **Tuning (the user's; "this code works pretty well n im happy with it"):**
  - `goal_direction`: 315° → 337.5°, shared by sim and real.
  - Sim block only: `w_new` 1.5 → 1.8 and `min_speed` 0.075 → 0.2.
  - Real `min_speed` kept at 0.075. Claude: at 0.2 m/s, the creep to the line and the end-wall approach would overshoot with real delays.
- **Real-robot refinements.** The user had asked "what would LIKELY go wrong on the real robot ... how can we thus improve our code?":
  - **Wall gap:** sliding stops at free right ≤ 0.04, about 6 cm from the wall (was about 3.5 cm). The tilt the dash, goal and in-front checks tolerate rises from about 1.5° to about 8°. It also helps if the LiDAR can't see very close.
  - **Heading hold with hysteresis:** start turning at 3° off and stop within 1° (was a single 1° band). With a real delay of about 0.1–0.2 s, a 0.3 rad/s turn overshoots 2–3°, so a single 1° band could wobble forever.
  - **After a wall fit, `node.turning = True`:** it then lines up to within 1° even when the wall is only 1–3° off.
- **Still to check on the real robot:**
  - LiDAR `range_min` (if above about 0.10, the goal stop must move further from the wall);
  - loop rate (`ros2 topic hz /cmd_vel`);
  - the first turn's direction and that 0.3 rad/s turns it;
  - where the pendulums hang and how far they swing (the off-centre case is on hold).

## V2-M4: Optitrack heading 175° off on robot 4 (real robot, own code) (2026-11-10)
- **Observed (real robot, `bingda_004`):**
  - many false pendulum detections, even after raising the trigger to 15 points;
  - the robot oscillated around the same spots, especially with the front and right blocked, and didn't move on to the door.
  - `obstacle_course2.py` crossed the same area fine.
- **Attempts before the cause was known** (each kept, because each also fixes a real hardware issue; see "Decision"):
  - a tighter detection window (only in front of the robot);
  - `wall_gap` 0.08 for the LiDAR's 15 cm blind zone;
  - a slower, wider heading hold (6°/2°, 0.2 rad/s).
  - Claude's 2D test of measuring the visited penalty further ahead made sim worse (414 reversals against 3), so it wasn't suggested.
- **User's diagnosis:** "its almost like the visited points are not doing anything to push the robot away from the visited places. could it be a optitrack global pose error such that the novelty score or the proximity score are not calculated correctly?"
- **Cause:** the pose heading did not match the robot's real forward direction.
  - `obstacle_course2.py`'s start-up check on the same robot printed: "WARNING: the robot's true heading is +175 deg from the pose heading. Using the measured heading instead".
  - The config has measured heading offsets only for robots 7 and 12. Robot 4 used the "auto" offset, which assumes its Optitrack rigid body was created facing maze +y; it was created facing roughly the opposite way.
  - The rotated course axes (Gazebo +x = real −y) were not the problem. A pure rotation cancels out, because all world maths uses the pose's own heading.
  - **Effects of the wrong heading:**
    - the visited and novelty points were evaluated almost behind the robot, so it was drawn back to where it had been;
    - the pendulum detector corrected for the robot's motion in nearly the opposite direction, so static things looked as if they moved.
  - `obstacle_course2.py` was immune only because its start-up check measures the true heading and uses it instead.
- **Fix:** `4 = 7.1` under `[heading_offsets]` in `optitrack_variables.config` (the section is meant for this: "a rigid body that was NOT created facing maze +y"). Reported heading = Optitrack heading − 87.86 − offset, so the offset drops by 175° from auto's −177.9°, giving −352.9 ≡ +7.1°. The check then read "true heading −2.3 deg from the pose heading", within the 10 cm check's accuracy.
- **Result:** the user reported "it works way better now".
- **Remark for the report:** a frame error in the global pose can masquerade as tuning or sensor problems (oscillation, false detections). Several symptom fixes were tried before the root cause was found; checking the pose heading against the actual direction of travel should come first on a new robot.

## V2-M5: Optitrack stops tracking past the second pendulum (real robot) (2026-11-10)
- **Observed:** after crossing the second pendulum, the robot never stopped at the goal and kept ramming the cardboard end wall. The last print was "GO: pendulum has passed", so it was still in `cross`, which ends only once the distance travelled (from the Optitrack pose) reaches the dash length.
- **Diagnosis (user's call to find the cause before adding logic: "shouldnt we try to figure out why the robot never stops even after crossing the seconod pendulum, instead of adding new logic?"):** debug prints at GO and every `cross` tick showed:
  - GO with dash 0.37 m and free ahead 0.71 m, so the LiDAR saw the end wall and there was room;
  - the pose then froze at (0.77, 0.19, −98°) and `travelled` stuck at 0.06 m;
  - meanwhile `free ahead` kept falling from 0.31 to 0.00 m: the robot was still moving, but its pose wasn't updating.
- **Cause:** Optitrack loses the robot past the second pendulum. The user confirmed it: "optitrack stops working past the second pendulum."
- **Decision (user):** no code change. "we can just remember that in our log but we dont have to change our code to accomodate that, because the optitrack should not stop updating."
  - Claude had offered a time limit on the dash (cross_length / speed + 0.3 s) so `cross` would end even with a frozen pose. Not adopted.
- **Remark for the report:** with a frozen pose, everything that uses it misbehaves (the dash distance, the heading hold, pendulum detection), while `goal` (LiDAR only) would still work.

## V2-D14: Wall alignment at the pendulums removed; line up with the start heading instead (own code, commit `06e55ef`) (2026-11-12)
- **Problem (real robot):** at the second pendulum the right wall has a slight convexity. The user: "the robot never fully aligns, and it ends up keep trying to align with the wall when it is there ... it just keeps rotating, and never stops rotating such that it never crosses the second pendulum." The wall fit (V2-D12) trusts only about 23 cm of wall beside the robot, so a bump there gives a wrong heading.
- **User's idea:** "instead of aligning the robot with the wall before crossing the obstacles, i jsut make the robot align itself with the correct global heading(using optitrack's publised pose)."
- **On the way:** the robot had been recalibrated in Optitrack, so the user removed the `4 = 7.1` override from V2-M4 ("the heading is fixed already"). Lined up with the corridor, the pose now read −168.7°, against about −97° before the recalibration. A hard-coded global heading would silently break after any recalibration.
- **Decision:** don't hard-code a heading. Hold the heading the robot starts with (it is placed square to the course, and `course_heading` stays `None` until the first tick). At the end of `position`, the wall fit is replaced by `node.turning = True`, so the robot turns back to within 2° of the start heading before waiting.
- **Why it's enough:** nothing depends on the local wall any more, and a few degrees of placement error are covered by the 6 cm wall gap and the slide-away fallback in `wait`.
- **Watch for:** if it ever keeps rotating at the second pendulum again, the only remaining cause is a frozen pose (Optitrack drops out near there, V2-M5).

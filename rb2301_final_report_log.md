# RB2301 Final Project: decisions and mistakes log

One entry per decision or mistake. Fill in **Observed** and **Fix** when you find out what actually happened.

---

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

---

### Path-following setup (for reference)
Pure pursuit with the robot treated as a point that can move in any direction: each tick it drives at a constant speed straight towards the path point `lookahead_distance` ahead of it, and it stops within `goal_tolerance` of the last path point. Starting values: lookahead 0.3 m, speed 0.3 m/s, tolerance 0.05 m. Current values: lookahead 0.05 m (see D6), speed 0.3 m/s, tolerance 0.05 m.

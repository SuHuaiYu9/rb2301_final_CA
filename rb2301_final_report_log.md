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

## E1: `select_target` crashes when there are no frontier cells
- **Observed (2D test, 1 of 11 layouts):** `ValueError: attempt to get argmin of an empty sequence` from `np.argmin(d)` in `select_target`, raised inside `timer_callback`.
- **Fix:**

---

### Path-following setup (for reference)
Pure pursuit with the robot treated as a point that can move in any direction: each tick it drives at a constant speed straight towards the path point `lookahead_distance` ahead of it, and it stops within `goal_tolerance` of the last path point. Starting values: lookahead 0.3 m, speed 0.3 m/s, tolerance 0.05 m.

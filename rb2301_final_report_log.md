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
- **Observed:**
- **Fix:**

## D2: World-frame velocity sent straight to `move_2D` (2026-11-03)
- **Decision:** pure pursuit computes the velocity `(vx, vy)` in the world frame and passes it directly to `move_2D`. `move_2D` actually expects velocities in the robot's own frame.
- **Why:** Gazebo spawns the robot at heading 0°, where the world frame and the robot's frame are identical.
- **Potential errors:**
  - Whenever the heading isn't 0°, the robot moves in a direction rotated by its heading. For example, at 90° a +x command moves it along +y.
  - On the real robot the pose is in the CA2 maze frame, and the starting heading is whatever direction the robot was placed in, so it's unlikely to be 0°.
  - Heading drift (D1) makes this worse during a run, even in Gazebo.
- **Observed:**
- **Fix:**

---

### Path-following setup (for reference)
Pure pursuit with the robot treated as a point that can move in any direction: each tick it drives at a constant speed straight towards the path point `lookahead_distance` ahead of it, and it stops within `goal_tolerance` of the last path point. Starting values: lookahead 0.3 m, speed 0.3 m/s, tolerance 0.05 m.

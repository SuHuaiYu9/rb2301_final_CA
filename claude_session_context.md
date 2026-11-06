# Claude session context: RB2301 final project (read this first in a new chat or after a context reset)

Saved 2026-11-03, last updated 2026-11-05. It lets work continue seamlessly after context compaction **or in a new chat**.
**This copy is for version 2 (the reactive code).** It lives on branch `claude/reactive-version` (with the report log and `claude_tools/`). Version 1 has its own copy on `claude/final-project-review-3m8r3v`, kept by a different chat (section 00). The user's own code is on `main`, which Claude must never change.

## 00. Current task: version 2, the reactive code (from 2026-11-05, a new chat started for it)
- **Goal:** help the user create a **new version of the code**, based on the current codebase (`main` at `676268b`, which ran the full course in Gazebo: S1 in the log), using a different approach: **no hard-coded door x-value and no hard-coded global goal.**
- **Why, in the user's words:** "the prof just said that he would prefer if the whole project is reactive instead of harcoded."
- **The user designs the new approach; Claude translates it into code.** All the ground rules in section 1 still apply.
  - The user hasn't described the new approach yet, so **don't propose one. Ask for or wait for their plan.**
  - Critique their logic only if they explicitly ask.
- **Where the new version lives (the user's decision, 2026-11-05):** a new file, `src/rb2301_fp/rb2301_fp/obstacle_course2.py`, on `main`. Version 1 stays in `obstacle_course.py`. Claude never edits `main` or the user's files.
- **Hard-coded values in the current code, for reference only** (a list of facts, not advice; the user named only the door x and the goal):
  - `self.global_goal`: used for the distance in `select_target` / `select_target2`, and for the stop at the goal after the door;
  - `self.door_x` = 3.4 (Gazebo), and `cx >= 2.9` in the real branch: switch to the after-door logic;
  - `self.origin` with `map_size_x` / `map_size_y` (6.02 × 4.5 m): where the grid sits;
  - LiDAR offsets 0.114 (Gazebo) and 0.045 (real); centre offset 0.0725;
  - after the door: the 0.25 m clearance, the front quarters `last_scan[:91]` and `last_scan[270:]`, and `move_2D(0.5, 0, 0)`.
- **Two chats, two branches (set up 2026-11-05):**
  - the old chat (session `01VT1WaLnxVP53q3RnFi24Uu`) is **still in use, for version 1 only** (`obstacle_course.py`). It logs on `claude/final-project-review-3m8r3v`;
  - this chat handles version 2 and logs **only** on `claude/reactive-version`. Never push to the other branch.
  - The version 2 log is a continuation of the version 1 log: it starts with a copy of the version 1 entries (as of commit `329a919`), and version 2 entries go under the heading "Version 2 (reactive) starts here" at the end.

## 0. First steps in a new chat or after a context reset
1. **Read this whole file, then `rb2301_final_report_log.md` and `claude_tools/README.md`.**
2. **Make sure your branch has these files.**
   - **On the user's own machine** (Claude desktop app, working directory `~/Downloads/rb2301_final_project-main`): that folder is the user's, and it must stay on `main`. The user works only on `main` and wants no other local branches. **Never `git checkout` or create branches in it.** Instead, clone the Claude branch into the scratchpad and do all log, context and tools work there:
     ```bash
     git clone -q --branch claude/reactive-version git@github.com:SuHuaiYu9/rb2301_final_CA.git "$SP/claude_branch"
     ```
     (2026-11-05: a session checked out the Claude branch in the user's folder, which confused them; it was switched back to `main` and the local branch copy deleted.)
   - **In a cloud session,** your designated branch may differ from `claude/reactive-version`. If it does, start it from this branch so the log, this file and the tools carry over:
   ```bash
   git fetch origin claude/reactive-version && git checkout -B <your-designated-branch> origin/claude/reactive-version
   ```
   Commit and push the log there. Tell the user which branch now holds the log.
3. **Get the user's current code** into your scratchpad (`$SP` = the scratchpad directory from your system prompt):
   ```bash
   git clone --depth 10 https://github.com/SuHuaiYu9/rb2301_final_CA.git "$SP/user_repo"     # first time
   cd "$SP/user_repo" && git fetch -q --depth 10 origin main && git checkout -q FETCH_HEAD && git log --oneline -3   # each "check it"
   ```
   The user may also create their own branch for the new version. If they say "check it", ask which branch if it isn't `main`.
4. **Test tools:** `claude_tools/` (2D simulator and test scripts; see its README). Previous sessions' scratchpad copies are gone; don't refer to them. **The scratchpad can be wiped in the middle of a session** (it happened on 2026-11-05): if a clone is missing, clone it again; push anything worth keeping.
5. **Re-read section 1 (ground rules)** before answering. The most common way to upset this user is giving advice they didn't ask for.
6. **Continue from section 00 (current task),** using section 2 for the state of things.

## 0b. How to update the report log (whenever the user says "log this")
- **File:** `rb2301_final_report_log.md` in your clone of branch `claude/reactive-version` (`$SP/claude_branch/` on the user's machine).
- **Where:** add version 2 entries under the heading "Version 2 (reactive) starts here" at the end of the file. Don't add or change version 1 entries here; the old chat logs those on its own branch.
- **Entry types:**
  - `D#` = a decision the user made;
  - `M#` = a problem or mistake observed;
  - `R#` = a real-robot run;
  - `E#` = an error or crash;
  - `S#` = a milestone or success.
  Version 2 entries use a **`V2-` prefix** so they never clash with version 1 numbers. Next free numbers: **V2-D3, V2-M2, V2-R2, V2-E1, V2-S1**. V2-REF problems now run up to P18. (V2-REF = Claude's reference code, a one-off entry type.)
- **Fields:** Decision/Observed, Why, Potential errors (only if they ask), Observed, Fix. Leave Fix blank unless they say what they did. Add a "Later:" line to an older entry when a value changes, as with D5.
- **Record the user's own remarks for the report faithfully,** as with R1's "Remark for the report".
- **Commit and push:** `git add rb2301_final_report_log.md && git commit -m "Log ..."` with the trailer lines **your own session's** system prompt gives (Co-Authored-By, plus your own Claude-Session URL, not the old one). Then push to your branch (section 0, step 2). Push access works.
- **Also offer to log** things that are clearly report-worthy, but only offer; don't log without being asked.

---

## 1. Ground rules from the user (follow strictly)
1. **The user writes the project themselves.** Claude's only job is to translate the user's ideas and algorithms into correct, efficient Python when asked.
2. **Do NOT correct their high-level logic, algorithm choices, edge cases or drift** unless they explicitly ask (for example "why does X happen?", "how should I fix X?", "investigate X"). They want to make mistakes and find them, so they have material for their report.
3. **DO flag code-level problems:** syntax errors, crashes, and code that doesn't do what they described. These count as translation errors.
4. **One exception:** if something on the real robot could hurt someone or damage hardware, say so once.
5. **Never suggest** heading hold, or rotating world-frame velocity into the robot frame. D1 and D2 are deliberate learning choices.
6. **When the user is just "updating" you on a plan,** acknowledge briefly and don't advise. They got angry about unsolicited advice.
7. **Never modify their codebase or `main`.** Claude maintains only `rb2301_final_report_log.md`, this file and `claude_tools/`, on the Claude branch. Log only what they ask to log, and record their remarks faithfully.
8. **They prefer short, minimal answers** when they ask for "only the necessary things".
9. **To check their code:** they push to GitHub `main`. Fetch it into the scratchpad clone (section 0, step 3), run `py_compile`, run the harness in `claude_tools/`, and report only code-level issues unless asked for more.
10. **Answer "why" questions with evidence:** use the 2D simulator harness and real measurements, not guesses. Say when something is unverified.
11. **Git:** commit messages end with your own session's Co-Authored-By and Claude-Session lines. Push to your Claude branch (section 0, step 2), never to `main`. A stop hook may demand that uncommitted or untracked files in the working directory be committed and pushed.
12. **Code suggestions must be minimal,** and must NOT modify any given template code unless a comment in it explicitly says it can be modified (for example the `###### INSERT CODE HERE ######` block). New helper functions can go outside that block.

**The user's own words** (verbatim, from the first chat):
- "your only job is to help me occasionally translate my ideas/high-level logic into python code. you are not to correct my higher level logic and algorithmic-level thinking without explicit instructions, even if you notice clear errors, or loopholes, or edge cases, or possible mistakes/drifts. i need to be able to discover those myself so i can learn from them... so your only job is to translate my ideas/algorithms correctly and efficiently into code."
- "do NOT change my codebase... you are only allowed to make suggestions to me, i will do any changes there personally myself."
- "we want to make MINIMAL code changes... we also should NOT modify ANY given code unless the code has a comment that explicitly says it can be modified."
- "dont include heading hold(stop suggesting potential real-world fixes for non-ideal scenarios like drift, this is part of my learning opportunity...)"
- "dont rotate world-frame velocity even when the heading isnt 0"
- "no dont tell me what to change now!! ... i am just telling you what i want to do, updating you."

## 2. Status right now (updated 2026-11-05)
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
- **Critique permission:** the user allowed high-level critique for ONE question ("you are allowed to give high-level logic critique now"). The default rule (section 1) still applies. **2026-11-05 (version 2 chat):** the user asked for algorithmic and high-level help again (see the version 2 progress notes).
- **Last thing discussed (2026-11-05):** D9–D11 and S1 were logged and pushed. Then the user started a new chat to build the reactive version (section 00). The goal-stop bug and E1 are still open in the current code.
- **In this chat (2026-11-05):** the version 1 / version 2 branch split was set up (section 00). Version 2 progress on `main`:
  - `d5accb8` / `c6662ef`: `obstacle_course2.py` = `676268b` plus the user's plan comments. Run it with an entry point `obstacle_course2` in `setup.py` (user's call) or `/usr/bin/python3 src/rb2301_fp/rb2301_fp/obstacle_course2.py`.
  - `745bf6f`: `select_target3` (Claude wrote it to the user's spec): score = A·right + B·front + C·unknown-neighbours, signed distances along the heading from the robot centre, metres, A = B = 1, C = 0.01 placeholder, argmax over seen free cells outside the buffer. Used in both Gazebo calls; the real branch still uses `select_target2` / `select_target`. User: Gazebo run "everything looks good".
  - Next (code handed over, not yet pushed when this was written): dynamic-state detection. Window fixed in the world = cells 0–0.2 m ahead of the centre along the heading, 1 m wide (`window_cells` helper); every `compare_every` = 4 callbacks, count saved window cells that went 99 → 0; ≥ `dynamic_threshold` → `self.dynamic_state = True` for good. In that state: stop if any `last_scan[:91]` / `last_scan[270:]` reading ≤ 0.22, else `move_2D(self.pursuit_speed, 0, 0)`. Replaces the `passed_door` block and the `cx >= door_x` branch (no goal stop any more).
  - Threshold placeholder 25, from `dyn_freed_study.py` (2D, 7 layouts): max freed per comparison with no swinging obstacles 9–75; near swinging obstacles 24–111. Threshold 25 triggers in 3/7 no-swinger runs and 6/7 swinger runs; no value separates them in that test.
  - **Why static cells trip it** (user asked "why?" after Gazebo triggered the dynamic state early and often; `dyn_why_study.py`, 2D, 7 layouts, no swinging obstacles): standing still frees 0 cells; moving, 85–100% of freed cells are within 1.5 cm of a real wall or obstacle surface. Surface cells flip 99 → 0 because rays from a new viewpoint cross a cell that an earlier ray marked as the hit cell (D8 lets scans clear 99), and because each scan is reapplied on the next tick from a pose ~1.5 cm further on (scans every 2nd tick). With a fresh scan every tick the counts were about the same (max 11–74), so the viewpoint effect alone is enough. Gazebo-specific extras (2 cm panel gaps, pose vs scan timing) unverified. No fix proposed (not asked).
  - `af71c1b` (user): false alarm. In the dynamic state, on every blocked tick: stop, count freed cells in `prev_window`, ≤ 5 → `dynamic_state = False`, then re-save the window. Also `window_width` 0.5, `compare_every` 2, `dynamic_threshold` 15; the real branch's pure pursuit is now inside the `else:` (indentation bug fixed). Code check: it decides on the FIRST blocked tick, comparing against a window saved one tick earlier or at switch-on, and can switch off in the same tick it switched on. The user wanted a wait while still. Reported; waiting for the length of "a while".
  - User: "a while" = 4 timer callbacks. Claude handed over (not yet pushed when this was written): `self.still_ticks = 0`, `self.wait_ticks = 4` in `__init__`. In the dynamic state: front clear → move and reset `still_ticks`; blocked → stop, and on the first blocked tick (`still_ticks == 0`) save the window. Increment `still_ticks`; when `still_ticks > wait_ticks` (4 callbacks after the save), count freed; ≤ 5 → false alarm; either way reset `still_ticks`, so a real obstacle starts a new 4-tick wait. Harness trace: every check ran 4 ticks after the stop, never switched on and off in the same tick, one command per tick, both outcomes occurred. Gazebo branch only.
  - `b66d56e` (user): that code, plus `compare_every` 8. User: false alarms now and then, but the dynamic state FAILS to switch on in front of the swinging obstacles.
  - **2026-11-05: the user explicitly allowed algorithmic / high-level critique** ("now it is time for you to help me on the algorithmic level and higher-level logic too") and asked what Claude thinks about robustness, including the real robot. Evidence (`dyn_robust_study.py`, 2D; noise 1 cm LiDAR σ, 3 mm and 0.3° pose jitter):
    - standing still with a wall in the window: grid count median 23–26 per comparison at 1 cm cells, 98 at 0.5 cm (0 without noise); seen-through 0–1;
    - moving, static course: grid up to 37–78 per comparison; seen-through up to 5–15 per scan pair;
    - rule "seen-through ≥ 5 on 2 consecutive scan pairs": 0/7 static runs fire, 7/7 door runs fire with a swinger ahead, 0/7 fire with none ahead. The current grid rule (≥ 15) fired in 7/7 static runs.
    - geometry: the window reaches 0.2 m from the centre (9 cm past the front), but the stop is at 0.22 m from the LiDAR (≈ 0.26 m from the centre), so the obstacle that caused a stop straight ahead is never in the false-alarm window.
    - risk flagged: a 1 m lookahead may switch on before the door (swingers seen through the gap), and the dynamic mode drives straight forward.
  - **User's next direction (2026-11-05):** a prof said the course can be solved with purely reactive control and no occupancy grid ("nested if...else", a bias towards the goal's general direction, a bias towards unvisited places). Fixed course shape (user's sketch): straight corridor → right turn into a second straight corridor → left turn through the door into the dynamic region → goal wall at the back. The robot starts near the back of corridor 1, facing its far end. Optitrack gives the pose. The user asked for the easiest solution.
    - Claude proposed (untested): **right-hand wall following in the robot frame** (no rotation, so F/L/B/R = LiDAR 0/90/180/270°), using a `clear(d)` robot-width-band helper. Rules: right side open (> 0.45) → turn right; wall ahead (< 0.3) → turn left; else move along T and keep the right wall at 0.25 m. Then V1's crossing rule after the door (forward; stop if front quadrants < 0.25; stop at the goal wall when ±45° is all < 0.45). The right-hand outer wall runs unbroken from the start to the goal, through the door gap (the door wall is attached to it).
    - Weak points stated: door detection (both sides < 0.3 m could be an obstacle slot; the alternative is a seen-through motion trigger); Gazebo swingers have no collision, so the V1 crossing rule was never tested against real hits; it relies on the start heading (D1); the path is longer. Offered to prototype it in the 2D harness before the user writes anything.
    - User: **the prof does not want wall following** ("too simplistic"). Asked for a feasible, robust approach without it.
    - Claude proposed (untested): **direction scoring**. Every tick, for 16 robot-frame directions: `free(θ)` = robot-width swept-band clearance; skip if < 0.15 m. score = cos(θ − goal direction, 45° right of forward) + 1.5 × novelty (distance from the point 0.4 m along θ to the nearest visited position, /0.4, capped at 1; needs the pose). Hysteresis 0.2. Top of the if/else: wait if the seen-through test finds motion ahead within about 0.6 m of the path; goal = pendulums seen moving AND can't move forward (memory fact: goal = can't move forward along body x, past the pendulums). Novelty 1.5 > 0.71 is what gets it out of the door corner. Offered a 2D harness test (the `real` swing model releases the pendulums at the door).
    - User: "write the whole code for this now. i will read through it then implement it myself." Claude wrote `claude_tools/reference/obstacle_course_reactive.py` (template + new code), and tuned it in the 2D harness (`reactive_study.py`). Final scoring per direction (16 directions, robot frame):
      `free` = best of 7 sideways-shifted lanes (`best_lane`, shifts up to ±0.12 m, needs side room); blocked if < 0.10. score = cos(θ − (−45°)) + 1.0·min(free, 1) + 1.5·novelty(probe at min(0.4, free)) − 0.1·(positions recorded every 0.25 s within 0.25 m of the probe). Commit 0.15 m or until blocked; hysteresis 0.2. Command = along speed (≤ unshifted free) + side speed 2·shift (±0.15), total capped at 0.3.
      Pendulums: `changed_points` (seen-past in both directions); wait if ≥ 3 in the path region (0.5 m ahead, strip + 0.3), hold 0.5 s; `pendulums_behind` (≥ 5 behind on 2 scans) + forward blocked for 1 s = goal.
      Results: navigation only (no pendulums) gets through the door in 12/12 layouts, 12/12 with 1 cm noise + 3 mm jitter, and 14/14 new layouts, with 0 static contacts, in 16–42 s. Real mode: no exceptions at headings 96.3 / 0 / −45°, ticks 1–2 ms.
      **Pendulum section NOT solved:** with the `real` model, pendulum contacts in 12/12 runs (1–14 each) and false goals 0.5–1.4 m short in 8/12; with `gazebo`, contacts in 11/12 and false goals in 4/12. The simple wait stops the robot inside a sweep line. Next idea (not built): timing-based crossing (stop before the line; cross only right after the pendulum leaves the lane moving away, towards the side with more room; never stop inside the line).
      **2026-11-05 later: pendulum crossing built (user: "yes build and test the pendulum crossing").** Lines = s positions (along the robot's forward axis) where moving points (`changed_points`) pile up; confirmed with ≥ 4 points on 5 scan pairs within 2 s; s = min over time of the 10th percentile (near face). Stop 0.08 m short of any confirmed line, or of any line with movement in the last 2 s. At a confirmed line, `line_sides` gives the nearest scan point on each side of the robot's lane within s−0.06..s+0.20; the speed of the gap growing or shrinking gives approaching/leaving. GO when the pendulum has just left the lane (≤ 0.4 s ago) moving towards the roomier side (free at ±90°), or when nothing on the line has moved for 1 s (side speeds and motion hits). Ignore the line if something static sits in the lane for 2 s. Crossing: straight forward at 0.3 until the rear is 0.20 past the line; pause if physically blocked, give up after 3 s. After the last line: straight forward; goal = forward < 0.10 and ±45° < 0.25 (a wall) for 1 s. Lane shifts in 2 cm steps.
      Final results (2D): navigation 12/12, 12/12 with noise, 14/14 new layouts, 0 static contacts. Real-style pendulums: success 12/12 (contact-free 6/12), new layouts 14/14 (contact-free 3/14), with noise 11/12 (seed 12: 0.34 m gap). Gazebo-style: 10/12 (seeds 9 and 11 stuck), contact-free 10/12. Real mode: no exceptions, 2 ms ticks. **Still open:** about one pendulum contact per run with real-style pendulums (the crossing takes about 1.4 s against a 2.6–4.5 s period). Next idea: measure each pendulum's period and predict its return before going.
      **Gap question (user's Gazebo screenshot, 2026-11-05):** the user's local `obstacle_course2.py` (unpushed) = the reference file exactly, plus a `setup.py` entry point `obstacle_course2`. Live `gz model -p` poses (the user had apparently moved the obstacles in the GUI): litter bin (octagon, radius 0.226) at (2.170, −1.020), can at (2.569, −0.730), can at (2.552, 0.121), drawer (0.41 × 0.50) at (1.035, 0.313); walls from the 21:34 world (x_off −0.12; second-corridor walls at x 1.68 and 2.88; saved in `harness/walls_2134.txt`). Gaps: bin to back wall 0.264, can to far wall 0.281. Passing sideways needs 2·(0.11 + side_margin) = 0.28 at 0.03. 2D replay (`harness/replay_layout.py`): margin 0.03 never gets through (90 s); 0.02 through in 24 s; 0.01 in 20 s. Regression at 0.02: navigation 12/12 and 14/14, Gazebo-style pendulums 12/12 (was 10/12), 0 static contacts; with noise 11/12 (seed 3). Answer: a tuning issue (side margin). The world file is re-randomised on every launch (it changed twice during the check).
      **2026-11-06: the user will write their OWN reactive controller** from `obstacle_course_template.py` (pushed at `88d3f81`, identical to the original template). They don't want to use Claude's reference without being able to write it themselves. A part-by-part teaching plan (Part 1 given: `scan_to_points` + attributes + centre/points in the INSERT block) was set aside; they will design it themselves. Ground rules apply again: translate their ideas, and don't redesign unless asked. Logged as V2-D1, V2-REF and V2-D2. Pendulum period detection is planned for later.
      **User's own plan (2026-11-06, design only, no code yet):** 12 directions "each 22.5 degrees apart" (asked: 12 × 22.5° = 270°, which do they mean?). For each direction, check whether moving 15 cm that way hits an obstacle; the ones that don't are feasible. Score each feasible one on the general goal direction plus a proximity-to-visited-points score (linear, negative, closer = more negative), evaluated where the robot would be after 15 cm. Record the robot centre in a visited array every 10 timer callbacks (0.5 s = 15 cm at 0.3 m/s). Pick the highest score. Dynamic obstacles: something that was hit before is no longer hit in a later scan, with geometry to find the new ray that points at the same spot after robot movement.
      **2026-11-06: real-robot version of the reference** (user asked Claude to add heading control etc. to "the obstacle_course2.py code" for a real test soon; Claude updated its own reference file, not the user's). Added:
        - `drive()`: heading hold, P control 1.5/rad, max 0.5 rad/s; translates only within 10° of the target;
        - start-up (real only): spin check (turn left 0.5 s, heading must rise, else stop); square up with the walls (`wall_alignment`, ±25° search for the sharpest x/y point histograms); heading check (slide 10 cm in the roomiest direction, measured vs pose heading, > 15° → `heading_fix` + WARNING; no room → stop);
        - stop if pose or scan objects don't change for 0.5 s;
        - `seen_past`: inf = can't tell (real LiDAR dropouts).
      R1 data suggest the Optitrack heading is about 180° off (start heading +96.3° but the goal is behind-left; at −90° the goal is forward-right). Tests (`real_reactive_study.py`): 8° off + heading 180° wrong 12/12; −10° + drift +2°/s + noise 12/12; 5° + heading 20° wrong + drift −2°/s + noise 12/12; flip → aborted; frozen pose → aborted. Gazebo mode unchanged (gazebo-style 10/12, real-style 12/12). Pendulum contacts still in about half the runs.
      **2026-11-06, real-robot test feedback:** the start-up spin check failed on the real robot ("-0.0 deg"). It now turns at 0.4 rad/s for up to 2 s, passes at +3°, and says whether the robot didn't turn or the sign is reversed. After that it ran on the real robot, including the pendulums. User: small swings come back too soon or never leave the lane, "but lots of free space on the left/right". Built and tested:
        - fix 1: go around the swing (swing range from moving points on the line over 5 s; after 3 s watching, `side_lane` with 0.10 clearance, slide sideways, then cross; give up the lane if the swing grows);
        - fix 2: per-side clear-time measurement; a timed crossing only if the remaining clear time is at least crossing time + 0.3 s;
        - lines found by comparing with the scan 0.3 s back (`compare_back` 3), because small swings went undetected;
        - the watched line is tracked by object, not by `s`;
        - `self.max_wait` (None = wait; a number = take the old gamble after that many seconds).
      2D real-mode results: small swings 12/12 with 0 contacts (all beside); medium 11/12 with 0 contacts; wide swings with no max_wait 4/12 and 0 contacts (waits), with max_wait 20 s 12/12 but contacts in 5/12 runs; no pendulums with noise, no false crossings. Gazebo-mode Gazebo-style pendulums with no max_wait: 4/12 (they never slow down).
      **P19–P22 (2026-11-06):** ignored lines get an 'ignored' flag, not 'crossed' (the old way triggered a false goal; `false_line_repro.py` reproduces it). `line_depth` 0.08 (real pendulum 3 cm thick). Wait near the nearer wall (side 8 cm from it) when there's no lane beside the swing; status print every 2 s. Walls left out of `line_sides`. Possible lines only stop the robot while it can still stop short (window 80 ticks); a line confirmed when the robot is already at it is crossed straight away. Results (3 cm pendulums, HX=0.015): small swings 13/14, small-medium awkward 11/12, wide 6/12 and 5/14 (the rest waiting), 0 contacts. `real_reactive_study.py`'s STUCK label can fire when a long wait ends (it measures 30 s without moving).
      Earlier iterations that failed (for the report if asked): plain novelty → dithering at blocked-direction boundaries; no lane shift → stuck at offset gaps; strong time-spent penalty (0.5 per sample) → a single pass made places repellent, so it slid past the gap and wandered away.
- **About the user:** casual and blunt (swears), learns by doing. Sometimes sends code as screenshots, sometimes pushes to `main` and says "check it". If they say "check" without attaching code, look on GitHub `main`.

## 3. Version 1 algorithm, the starting point for version 2 (`src/rb2301_fp/rb2301_fp/obstacle_course.py` on `main`, commit `676268b`)
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
- **`claude_tools/`** (on the Claude branch): `sim/sim2d.py` (the 2D course) and `harness/*.py` (scripts that run the user's node offline). See `claude_tools/README.md` for arguments and caveats. They were smoke-tested on `676268b` from their place in the repo.
  - The harness takes `<code_dir>` = `<clone>/src/rb2301_fp/rb2301_fp` and `<sim_dir>` = `claude_tools/sim`.
  - Scripts that depend on the map override the node's grid so it covers each 2D layout (test setup only).
  - **For the new reactive version,** `target_study.py`'s goal override and `DOOR_X` patch may no longer apply, so adapt the scripts rather than assuming they fit.
  - **The scripts do `import obstacle_course as oc`.** To run version 2, copy `obstacle_course2.py` into a scratch folder as `obstacle_course.py` and pass that folder as `<code_dir>` (copy `optitrack_variables.config` too for real mode), or adapt the import.
- **Claude's own navigator** (`course_navigator.py` / `nav.py`): **the user rejected it. Never suggest it.** It's not in the repo.
- **Don't use `pkill -f <name>`** where the pattern matches this shell's own command line, or the shell kills itself (exit 144). Use the `[x]` bracket trick instead.
- **When deleting:** use `"${VAR:?}"` paths. A safety check blocks `rm -rf $VAR`.
- **Harness method that worked well for "why" questions:** wrap or monkeypatch the user's functions on the imported module (for example `oc.A_star`, `oc.select_target`, `oc.frontier_cells`) to record or swap behaviour, and compare variants over several seeds. Say when results come from the 2D test rather than Gazebo.

## 7. Report log (`rb2301_final_report_log.md`, this branch)
Entries:
- **D1** no heading control; **D2** world-frame velocity sent straight to `move_2D`; **M1** brushing obstacles; **M2** no path because the start cell is in the buffer;
- **D3** fallback `select_target2`; **D4** replanning every other tick; **D5** buffer from `robot_length` (plus later notes); **D6** lookahead 0.05;
- **D7** planning from the robot's centre; **D8** clearing occupied cells; **M3** wide-enough gap rejected (plus a later note about 1 cm cells);
- **R1** first real-robot runs, plus the user's report remark linking D1/D2 and M2 with R1; **E1** empty-target crash;
- **D9** every target from `select_target2`, with the 2D frontier-vs-free test; **D10** stop and wait after the door, including the dropped grid-differencing idea and the goal-stop bug from the code check;
- **D11** separate buffer sizes along x and y; **S1** first full run of the course in Gazebo (commit `676268b`), with the user's remark.

Most "Fix" fields are blank for the user to fill in.

**Version 2 entries** (under "Version 2 (reactive) starts here"): **V2-D1** switch to purely reactive control (wall following rejected by the prof); **V2-REF** Claude's reference code with design rationale, problems P1–P13 and fixes, final results, and the planned period detection; **V2-D2** user writes their own reactive controller from `obstacle_course_template.py` (88d3f81). **V2-M1** the dynamic state switching on early (why: static surface cells flip while moving), with the user's false-alarm attempt (`af71c1b`) and its code check. Offered and unanswered earlier: V2-D entries for `select_target3` and the grid-window detection design.

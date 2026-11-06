# Claude's development helpers for the reactive reference code

- `reactive_parts_d17d32a.py`: the reference code split into three blocks (`###HELPERS###`, `###INIT###`, `###TIMER###`). This is the CURRENT version (`d17d32a`), and the one to edit.
- `reactive_parts_3d4b88f.py`: the reverted P19–P22 version, kept for reference only.
- `template.py`: the original course template (same as `obstacle_course_template.py`).
- `assemble.py <dir>`: builds the full node from `<dir>/template.py` and `<dir>/reactive_parts.py`, writing `<dir>/claude_branch/claude_tools/reference/obstacle_course_reactive.py` and `<dir>/rx/obstacle_course.py`. It expects the scratchpad layout used before (`$SP/reactive_parts.py`, `$SP/template.py`, `$SP/claude_branch`, `$SP/rx`). Recreate that layout, or adapt the paths.
- `summ_real.py <label>`: reads `real_reactive_study.py` output on stdin and prints goal / contacts / crossing-type tallies.

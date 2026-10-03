# SemNav rules
Source of truth: docs/architecture.txt (PDF text). Implement it; do not redesign it.
If you find a conflict, missing requirement, or feasibility problem: STOP, record it in
docs/DECISIONS.md with options, and ask me before continuing.

## Stack
ROS 2 Humble, Ubuntu 22.04, Gazebo Classic, C++17, ament_cmake.
Python only for launch files and scripts.
Build:  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
Test:   colcon test --event-handlers console_direct+ && colcon test-result --verbose
Format: clang-format -i on changed files. Warnings: -Wall -Wextra -Wpedantic, no warnings allowed.
Never run sudo. If something needs sudo, tell me the exact command and I will run it.

## Protocol for every task
1. Inspect first: list the repo, read files you will touch, read the relevant PDF section.
2. Work in one small vertical slice. Keep the workspace buildable at every step.
3. Pure logic goes in header-only classes with no ROS dependency, so it is unit-testable.
4. All tunables are ROS parameters. No magic numbers.
5. Add no dependency, topic, or message field not in the PDF without asking.
6. Do not modify files outside the task scope. Do not delete or overwrite existing code
   without explaining what it does first.
7. Build and run tests yourself. Show real command output. Fix failures before finishing.
8. Update docs/CHECKLIST.md. Log deviations in docs/DECISIONS.md.
9. Commit with a clear message when the phase's "done when" conditions pass.

## Git rules
- Single branch: main. Do not create other branches.
- Commit to main after every working slice (builds + tests pass), with a specific message.
- Never commit a failing build, secrets, models (*.onnx), or build/install/log folders.
- Before any large or risky change, make sure the current working state is committed.
- Do not push, rebase, amend, or force anything. I will push and tag myself.

## Completion report (end every task with this)
Files changed | Commands run and results | Tests added/passed | Failures or unresolved issues |
Deviations from architecture | What I need to approve or verify manually.

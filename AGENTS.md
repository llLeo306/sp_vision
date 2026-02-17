# Repository Guidelines

## Project Structure & Module Organization
This repository is a C++17 XRobot-based vision stack.
- `Modules/`: core modules (`SpVisionCamera`, `SpVisionDetector`, `SpVisionTracker`, `SpVisionAimer`, `SpVisionState`, plus SharedTopic modules).
- `User/`: application entry and runtime configs (`main.cpp`, `xrobot.yaml`, `xrobot_sharetopic.yaml`, generated `xrobot_main.hpp`).
- `tool/`: offline analysis/plot scripts (for logs and debug CSVs).
- `libxr/`: framework dependency (git submodule).
- `build/`: local build output (do not commit).
- `logs/`: runtime logs and analysis artifacts.

## Build, Test, and Development Commands
Use repository root as working directory.
- `cmake -S . -B build`: configure project.
- `cmake --build build -j$(nproc) --target sp_vision_xrobot`: build main executable.
- `./build/sp_vision_xrobot`: run with current `User/xrobot_main.hpp` wiring.
- `xrobot_gen_main -c User/xrobot.yaml -o User/xrobot_main.hpp`: regenerate module entry after YAML changes.
- `python3 tool/plot_aim_log.py`: render latest aim trace plot from `logs/analysis`.

## Coding Style & Naming Conventions
- Follow `.clang-format` (Google base, custom brace wrapping, `ColumnLimit: 100`, pointer alignment `Type * ptr`).
- Prefer 2-space indentation and keep includes grouped/clean.
- Naming follows existing code:
  - Module/class names: `SpVisionTracker`, `SharedTopicClient` (PascalCase).
  - File/function/variable names: `snake_case` (e.g., `plot_aim_log.py`, `max_temp_lost_count`).
- Keep configuration keys stable and explicit in `User/*.yaml`.

## Testing Guidelines
There is no root-level `ctest` suite for vision modules yet; use repeatable smoke checks:
- Build and run `sp_vision_xrobot` with demo video (`User/xrobot.yaml`).
- Verify key windows/topics/logs update correctly.
- For `libxr` changes, enable and build its union tests with `-DLIBXR_TEST_BUILD=ON`, then run `./build/test`.
- Name new tests by behavior (`test_<feature>.cpp`) consistent with `libxr/test/`.

## Commit & Pull Request Guidelines
Recent history uses short imperative subjects (`Fix ...`, `Update ...`, `add ...`). Keep this style, but make scope explicit.
- Commit title format: `<verb> <scope>: <what changed>` (example: `Fix tracker: clamp outpost relock threshold`).
- PRs should include: purpose, affected modules/configs, validation steps, and screenshots/log snippets for detection/tracking behavior changes.
- Link related issues and call out hardware-dependent changes (camera SDK, serial/CAN, OpenVINO device settings).

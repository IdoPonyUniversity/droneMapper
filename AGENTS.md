# Repository Guidelines

## Project Structure & Module Organization

This repository is a C++20 CMake project for a mock LiDAR sensor over 3D voxel maps.

- `src/app/`: executable entry point (`main.cpp`) and future simulator runner.
- `src/core/`: strong units and future shared geometry helpers.
- `src/config/`: Assignment 1 configuration domain models and validation.
- `src/io/`: fixed input parsers and future map text I/O.
- `src/map/`: map interfaces and ground-truth map implementations.
- `src/sensors/`: sensor interfaces plus sensor mocks.
- `src/movement/`, `src/simulation/`, and `src/drone/`: future movement interface, simulator truth state/drivers, and drone algorithm code.
- `tests/`: GoogleTest unit tests and shared helpers.
- `data_maps/`: sample `.npy` voxel maps used by tests.
- `sample_inputs/`: small Assignment 1 input fixtures for future manual `drone_mapper` runs.
- `.devcontainer/` and `CMakeLists.txt`: container and FetchContent-based dependency setup.

Keep public API changes in `src/` paired with focused tests in `tests/`. `src/config/Config.h` and `src/config/Config.cpp` hold the Assignment 1 `DroneConfig`, `MissionConfig`, `LidarConfig`, boundary, resolution, and validation domain models; `DroneConfig::drone_radius` is the Exercise 1 sphere source of truth for clearance and collision checks. `src/io/InputParsers.h` and `src/io/InputParsers.cpp` parse the Assignment 1 fixed input files with documented defaults for missing config keys. Malformed lines, unknown keys, duplicate scalar keys, invalid present values, and invalid map contents throw `std::runtime_error` so callers can catch and finish `main` normally.

## Build, Test, and Development Commands

Run commands from the repository root inside the provided dev container:

```bash
cmake -B build
```

Configures `build/` using FetchContent for dependencies.

```bash
cmake --build build
```

Builds the single `drone_mapper` executable and `drone_mapper_tests` (tests compile the production sources directly; there is no project library target).

```bash
ctest --test-dir build --output-on-failure
```

Runs the registered GoogleTest suite and prints failing test output.


## Coding Style & Naming Conventions

Use modern C++20 and follow the existing style. Use 4-space indentation, same-line braces for functions/classes, `PascalCase` for class names, and `snake_case` for functions, variables, and private helpers. Keep code inside the `drone_mapper` namespace. Prefer strongly typed units from `Units.h` over raw numeric distances or angles. Mark query methods `[[nodiscard]]` where ignored results would be suspicious, and use `const`/`noexcept` when the surrounding pattern supports it.

## Testing Guidelines

Tests use GoogleTest and are built into `drone_mapper_tests`. Name files after the component under test, such as `VoxelGridTests.cpp`, and name cases descriptively with `TEST(ComponentTests, Behavior)`. Use `tests/TestHelpers.h` for positions, orientations, beam reconstruction, and unit conversions. Prefer small fake maps/sensors for behavior tests; use `data_maps/` only when validating `.npy` loading or real map coordinates.

## Commit & Pull Request Guidelines

Recent commits use short, imperative summaries, for example `Remove positionAfter* helpers; fix devcontainer IntelliSense`. Keep commits focused and avoid committing generated directories such as `build/` or `vcpkg_installed/`. Pull requests should include a concise description, affected components, test commands run, and any map/data assumptions. Link related issues when available and include screenshots only for tooling or documentation UI changes.

## Agent-Specific Instructions

Do not rewrite unrelated files or generated outputs. Preserve sample map files unless the task explicitly requires changing test fixtures. When adding dependencies, update `CMakeLists.txt` with appropriate `FetchContent_Declare` and link targets.
Keep `tasks.md` updated when assignment implementation status or task breakdown changes.
ALWAYS READ assignment.md - these are the instructions for this project.

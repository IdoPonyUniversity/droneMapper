# Mock LiDAR Sensor

An implementation for a mock LiDAR sensor scanning a 3D voxel occupancy map. The code is intended as a starting point for your Mapping Drone project in the 2026 "Advanced Topics in Programming" course.

Note: If you wish to use this implementation, you are responsible for adapting it to your project.

The attached container includes all the requirements you will need to build and run this project.

## What Is Included

This project includes:

- A mock LiDAR sensor implementation.
- Interfaces for a 3D map, LiDAR sensor, and position sensor.
- A voxel-grid map loader for `.npy` files.
- Strongly typed position, distance, and angle units.
- Example maps under `data_maps/`.
- A small runnable example.
- Unit tests using GoogleTest.

Note - The implementation for PositionSensor is ad-hoc. In your project these will be different!
## Project Structure

```text
src/                     Public interfaces, type definitions, and implementation files
tests/                   Unit tests
data_maps/               Example voxel maps
.devcontainer/           Development container setup
CMakeLists.txt           CMake build configuration (uses FetchContent for dependencies)
```

## Building the Project

Open the project inside the provided container, then run:

```bash
cmake -B build
cmake --build build
```

## Running the Example

After building, run:

```bash
./build/mock_lidar_test
```

You can also run the example with a specific map:

```bash
./build/mock_lidar_test data_maps/five_voxels_y4_pattern.npy
```

## Running the Tests

Run all tests with:

```bash
ctest --test-dir build --output-on-failure
```

Or run the test executable directly:

```bash
./build/drone_mapper_tests
```

## Main Components

`IMap3D` is the interface used by the LiDAR sensor to query whether a position in space is occupied.

`VoxelGrid` is an implementation of `IMap3D` that loads a 3D NumPy `.npy` file. The map is interpreted as a voxel grid, where each voxel represents 1 cm.

`IPositionSensor` provides the current position and heading of the drone.

`MockLidarSensor` scans the map by tracing beams from the drone position. It returns the distance and relative angle for each hit.

`Units.h` defines the physical units used by the project, including X/Y/Z lengths and horizontal/altitude angles.

## Assignment File Formats

The assignment fixes the file names but lets us choose the formats. This project uses:

- simple line-oriented `key=value` text for `drone_config.txt` and `mission_config.txt`.
- binary NumPy `.npy` array data for `map_input.txt` and `map_output.txt`.

Configuration text file rules:

- One entry per line.
- Leading/trailing whitespace around the key and value is ignored.
- Empty lines are ignored.
- Lines starting with `#` are comments.
- Some keys, such as `recharge`, may appear multiple times and are collected as lists.
- Missing keys use the defaults documented below.
- Duplicate scalar keys are invalid input and make parsing fail.
- Unknown keys, malformed non-comment lines, and invalid present values are invalid input and make parsing fail.
- Numeric distances are in centimeters.
- Numeric angles are in degrees.
- Coordinates are written as `x,y,z` in centimeters.

A small manual-run fixture is available under [`sample_inputs/basic/`](sample_inputs/basic/). The parsers in `src/InputParsers.h` implement the concrete config keys and map file format documented here, returning parsed config/map objects and throwing `std::runtime_error` for invalid input.

### `drone_config.txt`

Drone capability configuration. Distances are in centimeters and angles are in degrees.

| Key | Meaning | Default if missing |
| --- | --- | --- |
| `drone_radius_cm` | Exercise 1 drone sphere radius, used directly by movement/collision checks for clearance from obstacles and boundaries. | `1` |
| `max_rotate_deg` | Maximum absolute rotation per movement request. | `90` |
| `max_advance_cm` | Maximum absolute horizontal advance per movement request. | `1` |
| `max_elevate_cm` | Maximum absolute vertical movement per movement request. | `1` |
| `lidar_z_min_cm` | Minimum accurate LiDAR distance. Hits closer than this are returned with distance `0`. | `1` |
| `lidar_z_max_cm` | Maximum LiDAR detection distance. | `10` |
| `lidar_circle_spacing_cm` | LiDAR beam-circle spacing `D` measured at `Z-min`. Circle `n` radius is `n * D`. | `1` |
| `lidar_circle_count` | Number of LiDAR circles, including circle `0`. | `1` |

Validation rules:

- `drone_radius_cm` and movement limit values must be positive; `lidar_z_min_cm` may be `0`.
- `lidar_z_max_cm` must be greater than `lidar_z_min_cm`.
- `lidar_circle_count` must be at least `1`.

Example:

```text
drone_radius_cm=1
max_rotate_deg=90
max_advance_cm=1
max_elevate_cm=1
lidar_z_min_cm=1
lidar_z_max_cm=5
lidar_circle_spacing_cm=1
lidar_circle_count=2
```

### `mission_config.txt`

Mission-specific configuration. Boundaries are inclusive centimeter coordinates in world space.

| Key | Meaning | Default if missing |
| --- | --- | --- |
| `boundary_min_x_cm` | Minimum mapped X coordinate. | `0` |
| `boundary_max_x_cm` | Maximum mapped X coordinate. | map input `size_x_cm - 1` if available, else `0` |
| `boundary_min_y_cm` | Minimum mapped Y coordinate. | `0` |
| `boundary_max_y_cm` | Maximum mapped Y coordinate. | map input `size_y_cm - 1` if available, else `0` |
| `boundary_min_z_cm` | Minimum mapped height coordinate. | `0` |
| `boundary_max_z_cm` | Maximum mapped height coordinate. | map input `size_z_cm - 1` if available, else `0` |
| `initial_x_cm` | Initial drone center X coordinate. | `boundary_min_x_cm` |
| `initial_y_cm` | Initial drone center Y coordinate. | `boundary_min_y_cm` |
| `initial_z_cm` | Initial drone center height coordinate. | `boundary_min_z_cm` |
| `initial_heading_deg` | Initial XY heading. `0` is east, `90` is south, `180` is west, `270` is north. | `0` |
| `resolution_xy_decimals` | Required decimal places after the dot for X/Y coordinates. Exercise 1 supports only `0` for 1 cm cells. | `0` |
| `resolution_z_decimals` | Required decimal places after the dot for height coordinates. Exercise 1 supports only `0` for 1 cm cells. | `0` |

Optional repeated key:

| Key | Meaning |
| --- | --- |
| `recharge` | Placeholder for future recharge positions, as `x,y,z`. Exercise 1 doesn't use this key because the battery is infinite. |

Validation rules:

- Each min boundary must be less than or equal to its matching max boundary.
- The initial position must be inside the configured boundaries.
- `initial_heading_deg` must be in the range `[0, 360]`.
- Exercise 1 accepts only `resolution_xy_decimals=0` and `resolution_z_decimals=0`; other values are invalid input.
- Bad `recharge` lines are invalid input.

Example:

```text
boundary_min_x_cm=0
boundary_max_x_cm=4
boundary_min_y_cm=0
boundary_max_y_cm=4
boundary_min_z_cm=0
boundary_max_z_cm=2
initial_x_cm=1
initial_y_cm=1
initial_z_cm=1
initial_heading_deg=0
resolution_xy_decimals=0
resolution_z_decimals=0
```

### `map_input.txt` and `map_output.txt`

These files use the binary NumPy `.npy` format.

`map_input.txt` is the simulator ground-truth building map:

- Row-major 3D NumPy array with shape `[X, Y, Z]`.
- One voxel represents `1 cm` in each dimension.
- Use signed 32-bit integers (`int32`) so `map_output.txt` can use the same container format for negative values.
- Allowed input values are:
  - `0`: empty
  - `1`: occupied

`map_output.txt` is the drone-produced map and uses the same `.npy` container format:

- Shape should match the mapped grid used for `map_input.txt`.
- Allowed output values are:
  - `0`: mapped empty
  - `1`: mapped occupied
  - `-1`: not mapped, inside mission mapping boundaries
  - `-2`: not mapped because outside mission mapping boundaries
- If the array covers a larger area than the mission boundaries, every in-array cell outside the mission boundaries must contain `-2`.
- Coordinates outside the declared array shape are also interpreted as `-2` by the drone-visible map API.

## Current `.npy` Map Support

The starter LiDAR example still supports the original `.npy` maps stored in `data_maps/`.

Each `.npy` map is expected to be a row-major 3D array with shape `[X, Y, Z]`. A value of `0` means empty space. A non-zero value means occupied space.

Coordinates are interpreted in centimeters. For example, the position `(2 cm, 4 cm, 2 cm)` maps to voxel index `(2, 4, 2)`.

Positions outside the map are treated as empty space.

## Adapting This Code

This code is intentionally written as a starting point, not as a complete Mapping Drone solution.

Before using it in your project, consider:

- Whether the map format matches your project.
- Whether the LiDAR scan pattern matches your needs.
- How your drone should convert relative LiDAR hits into world-space coordinates.
- Whether the sensor interfaces should be extended.
- What additional tests are required for your final implementation.

Generated files such as `build/` and `vcpkg_installed/` should not be committed to your repository.

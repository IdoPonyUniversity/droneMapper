cmake_minimum_required(VERSION 3.14)
project(drone_mapper)

include(FetchContent)

# --- fmt (needed by mp-units)
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG        10.2.1
)
FetchContent_MakeAvailable(fmt)

# --- mp-units library
set(MP_UNITS_API_CONTRACTS NONE CACHE STRING "" FORCE)
FetchContent_Declare(
  mp-units
  GIT_REPOSITORY https://github.com/mpusz/mp-units.git
  GIT_TAG        v2.5.0
  SOURCE_SUBDIR  src
)
FetchContent_MakeAvailable(mp-units)

# --- TinyNPY
FetchContent_Declare(
  TinyNPY
  GIT_REPOSITORY https://github.com/cdcseacave/TinyNPY.git
  GIT_TAG        v1.1
)
FetchContent_MakeAvailable(TinyNPY)

# --- Core library: everything except main() lives here so tests can link it.
add_library(drone_mapper_lib
  src/Config.cpp
  src/InputParsers.cpp
  src/MockLidarSensor.cpp
  src/SimulationContext.cpp
  src/SparseTextMap.cpp
  src/VoxelGrid.cpp
)
target_include_directories(drone_mapper_lib PUBLIC src)
target_compile_features(drone_mapper_lib PUBLIC cxx_std_20)
target_link_libraries(drone_mapper_lib PUBLIC mp-units::mp-units TinyNPY)

# --- Assignment executable
add_executable(drone_mapper
  src/main.cpp
)
target_link_libraries(drone_mapper PRIVATE drone_mapper_lib)

# --- Tests
enable_testing()
option(DRONE_MAPPER_BUILD_TESTS "Build unit tests" ON)

if(DRONE_MAPPER_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.17.0
  )
  FetchContent_MakeAvailable(googletest)
  add_subdirectory(tests)
endif()

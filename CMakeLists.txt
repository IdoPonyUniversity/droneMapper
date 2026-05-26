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

# --- Shared production sources. The app is intentionally a single executable
# target; tests compile these same sources directly instead of linking a
# repository library target.
set(DRONE_MAPPER_SOURCE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/app
  ${CMAKE_CURRENT_SOURCE_DIR}/src/config
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core
  ${CMAKE_CURRENT_SOURCE_DIR}/src/io
  ${CMAKE_CURRENT_SOURCE_DIR}/src/map
  ${CMAKE_CURRENT_SOURCE_DIR}/src/movement
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors
  ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation
  ${CMAKE_CURRENT_SOURCE_DIR}/src/drone
  ${CMAKE_CURRENT_SOURCE_DIR}/src/drone/algorithms
)

set(DRONE_MAPPER_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/config/Config.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/io/InputParsers.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/MockLidarSensor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/simulation/SimulationContext.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/map/SparseTextMap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/map/VoxelGrid.cpp
)

# --- Assignment executable
add_executable(drone_mapper
  ${DRONE_MAPPER_SOURCES}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/app/main.cpp
)
target_include_directories(drone_mapper PRIVATE ${DRONE_MAPPER_SOURCE_DIRS})
target_compile_features(drone_mapper PRIVATE cxx_std_20)
target_link_libraries(drone_mapper PRIVATE mp-units::mp-units TinyNPY)

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

include(GoogleTest)

add_executable(drone_mapper_tests
  ConfigTests.cpp
  InputParsersTests.cpp
  MockLidarSensorTests.cpp
  SimulationContextTests.cpp
  SparseTextMapTests.cpp
  VoxelGridTests.cpp
)
target_link_libraries(drone_mapper_tests
  PRIVATE
    drone_mapper_lib
    GTest::gtest_main
)
target_compile_definitions(drone_mapper_tests
  PRIVATE
    DRONE_MAPPER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/.."
)

gtest_discover_tests(drone_mapper_tests)

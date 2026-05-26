include(GoogleTest)

add_executable(drone_mapper_tests
  ${DRONE_MAPPER_SOURCES}
  ConfigTests.cpp
  InputParsersTests.cpp
  MockLidarSensorTests.cpp
  SimulationContextTests.cpp
  SparseTextMapTests.cpp
  VoxelGridTests.cpp
)
target_include_directories(drone_mapper_tests
  PRIVATE
    ${DRONE_MAPPER_INCLUDE_DIR}
)
target_link_libraries(drone_mapper_tests
  PRIVATE
    mp-units::mp-units
    TinyNPY
    GTest::gtest_main
)
target_compile_features(drone_mapper_tests PRIVATE cxx_std_20)
target_compile_definitions(drone_mapper_tests
  PRIVATE
    DRONE_MAPPER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/.."
)

gtest_discover_tests(drone_mapper_tests)

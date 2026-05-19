#include "TestHelpers.h"

#include <cpp_course/InputParsers.h>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

using namespace cpp_course;

namespace {

[[nodiscard]] std::filesystem::path sample_input_path(const char* filename) {
    return std::filesystem::path(CPP_COURSE_SOURCE_DIR) / "sample_inputs" / "basic" / filename;
}

[[nodiscard]] std::filesystem::path unique_temp_dir(const std::string& test_name) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    std::filesystem::path dir = std::filesystem::temp_directory_path() / ("cpp_course_" + test_name + "_" + std::to_string(stamp));
    std::filesystem::create_directories(dir);
    return dir;
}

void write_text_file(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream output(path, std::ios::trunc);
    ASSERT_TRUE(output) << "failed to create " << path;
    output << contents;
}

template <typename Callable>
void expect_runtime_error_contains(Callable&& callable, const std::string& needle) {
    try {
        callable();
        FAIL() << "expected std::runtime_error containing: " << needle;
    } catch (const std::runtime_error& error) {
        EXPECT_NE(std::string(error.what()).find(needle), std::string::npos) << error.what();
    }
}

[[nodiscard]] std::string valid_drone_config_text() {
    return "drone_radius_cm=1\n"
           "max_rotate_deg=90\n"
           "max_advance_cm=1\n"
           "max_elevate_cm=1\n"
           "lidar_z_min_cm=1\n"
           "lidar_z_max_cm=5\n"
           "lidar_circle_spacing_cm=1\n"
           "lidar_circle_count=2\n";
}

[[nodiscard]] std::string valid_mission_config_text() {
    return "boundary_min_x_cm=0\n"
           "boundary_max_x_cm=4\n"
           "boundary_min_y_cm=0\n"
           "boundary_max_y_cm=4\n"
           "boundary_min_z_cm=0\n"
           "boundary_max_z_cm=2\n"
           "initial_x_cm=1\n"
           "initial_y_cm=1\n"
           "initial_z_cm=1\n"
           "initial_heading_deg=0\n"
           "resolution_xy_decimals=0\n"
           "resolution_z_decimals=0\n";
}

} // namespace

TEST(InputParsersTests, ParsesValidDroneConfig) {
    const DroneConfig config = parse_drone_config(sample_input_path("drone_config.txt"));

    EXPECT_EQ(config.drone_radius, 1.0 * cm);
    EXPECT_EQ(config.max_rotate, 90.0 * horizontal_angle[deg]);
    EXPECT_EQ(config.max_advance, 1.0 * cm);
    EXPECT_EQ(config.max_elevate, 1.0 * cm);
    EXPECT_EQ(config.lidar.beam_length_min, 1.0 * cm);
    EXPECT_EQ(config.lidar.beam_length_max, 5.0 * cm);
    EXPECT_EQ(config.lidar.circle_spacing, 1.0 * cm);
    EXPECT_EQ(config.lidar.fov_circles, 2U);
}

TEST(InputParsersTests, ThrowsForMalformedConfigLine) {
    const std::filesystem::path dir = unique_temp_dir("malformed_line");
    const std::filesystem::path path = dir / "drone_config.txt";
    write_text_file(path, "drone_radius_cm 1\n");

    expect_runtime_error_contains([&] {
        (void)parse_drone_config(path);
    }, "expected key=value");

    std::filesystem::remove_all(dir);
}

TEST(InputParsersTests, ThrowsForUnknownConfigKey) {
    const std::filesystem::path dir = unique_temp_dir("unknown_key");
    const std::filesystem::path path = dir / "drone_config.txt";
    write_text_file(path, valid_drone_config_text() + "unknown_key=123\n");

    expect_runtime_error_contains([&] {
        (void)parse_drone_config(path);
    }, "unknown key 'unknown_key'");

    std::filesystem::remove_all(dir);
}

TEST(InputParsersTests, ThrowsForDuplicateScalarConfigKey) {
    const std::filesystem::path dir = unique_temp_dir("duplicate_key");
    const std::filesystem::path path = dir / "drone_config.txt";
    write_text_file(path, valid_drone_config_text() + "max_rotate_deg=45\n");

    expect_runtime_error_contains([&] {
        (void)parse_drone_config(path);
    }, "duplicate key 'max_rotate_deg'");

    std::filesystem::remove_all(dir);
}

TEST(InputParsersTests, ThrowsForInvalidDroneConfigValue) {
    const std::filesystem::path dir = unique_temp_dir("invalid_drone_value");
    const std::filesystem::path path = dir / "drone_config.txt";
    write_text_file(path,
                    "drone_radius_cm=-1\n"
                    "max_rotate_deg=90\n"
                    "max_advance_cm=1\n"
                    "max_elevate_cm=1\n"
                    "lidar_z_min_cm=1\n"
                    "lidar_z_max_cm=5\n"
                    "lidar_circle_spacing_cm=1\n"
                    "lidar_circle_count=2\n");

    expect_runtime_error_contains([&] {
        (void)parse_drone_config(path);
    }, "drone_radius_cm");

    std::filesystem::remove_all(dir);
}

TEST(InputParsersTests, UsesDroneConfigDefaultsForMissingKeys) {
    const std::filesystem::path dir = unique_temp_dir("missing_drone_key");
    const std::filesystem::path path = dir / "drone_config.txt";
    write_text_file(path,
                    "max_rotate_deg=45\n"
                    "lidar_z_max_cm=5\n");

    const DroneConfig config = parse_drone_config(path);

    EXPECT_EQ(config.drone_radius, 1.0 * cm);
    EXPECT_EQ(config.max_rotate, 45.0 * horizontal_angle[deg]);
    EXPECT_EQ(config.max_advance, 1.0 * cm);
    EXPECT_EQ(config.max_elevate, 1.0 * cm);
    EXPECT_EQ(config.lidar.beam_length_min, 1.0 * cm);
    EXPECT_EQ(config.lidar.beam_length_max, 5.0 * cm);
    EXPECT_EQ(config.lidar.circle_spacing, 1.0 * cm);
    EXPECT_EQ(config.lidar.fov_circles, 1U);

    std::filesystem::remove_all(dir);
}

TEST(InputParsersTests, ParsesMapInputIntoSparseGroundTruthMap) {
    const SparseTextMap map = parse_map_input(sample_input_path("map_input.txt"));

    EXPECT_EQ(map.x_size(), 5U);
    EXPECT_EQ(map.y_size(), 5U);
    EXPECT_EQ(map.z_size(), 3U);
    EXPECT_EQ(map.occupied_count(), 4U);
    EXPECT_EQ(map.get(test::make_position(2.0, 2.0, 1.0)), 1);
    EXPECT_EQ(map.get(test::make_position(0.0, 0.0, 0.0)), 0);
}

TEST(InputParsersTests, ParsesMissionConfigWithRechargePositions) {
    const std::filesystem::path dir = unique_temp_dir("mission_success");
    const std::filesystem::path path = dir / "mission_config.txt";
    write_text_file(path, valid_mission_config_text() + "recharge=2,3,1\n");

    const MissionConfig config = parse_mission_config(path, MapDimensions{5, 5, 3});

    EXPECT_EQ(config.boundaries.min_x, 0.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_x, 4.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_y, 4.0 * y_extent[cm]);
    EXPECT_EQ(config.boundaries.max_z, 2.0 * z_extent[cm]);
    EXPECT_EQ(config.initial_position.x, 1.0 * x_extent[cm]);
    EXPECT_EQ(config.initial_position.y, 1.0 * y_extent[cm]);
    EXPECT_EQ(config.initial_position.z, 1.0 * z_extent[cm]);
    EXPECT_EQ(test::degrees(config.initial_heading.horizontal), 0.0);
    ASSERT_EQ(config.recharge_positions.size(), 1U);
    EXPECT_EQ(config.recharge_positions.front().z, 1.0 * z_extent[cm]);

    std::filesystem::remove_all(dir);
}

TEST(InputParsersTests, UsesMissionConfigDefaultsForMissingKeys) {
    const std::filesystem::path dir = unique_temp_dir("mission_defaults");
    const std::filesystem::path path = dir / "mission_config.txt";
    write_text_file(path,
                    "boundary_min_x_cm=1\n"
                    "initial_y_cm=2\n");

    const MissionConfig config = parse_mission_config(path, MapDimensions{5, 6, 7});

    EXPECT_EQ(config.boundaries.min_x, 1.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_x, 4.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_y, 5.0 * y_extent[cm]);
    EXPECT_EQ(config.boundaries.max_z, 6.0 * z_extent[cm]);
    EXPECT_EQ(config.initial_position.x, 1.0 * x_extent[cm]);
    EXPECT_EQ(config.initial_position.y, 2.0 * y_extent[cm]);
    EXPECT_EQ(config.initial_position.z, 0.0 * z_extent[cm]);
    EXPECT_EQ(test::degrees(config.initial_heading.horizontal), 0.0);
    EXPECT_EQ(config.resolution.xy_decimal_places, 0);
    EXPECT_EQ(config.resolution.z_decimal_places, 0);

    std::filesystem::remove_all(dir);
}

TEST(InputParsersTests, ThrowsForBadRechargePosition) {
    const std::filesystem::path dir = unique_temp_dir("bad_recharge");
    const std::filesystem::path path = dir / "mission_config.txt";
    write_text_file(path, valid_mission_config_text() + "recharge=1,2\n");

    expect_runtime_error_contains([&] {
        (void)parse_mission_config(path, MapDimensions{5, 5, 3});
    }, "invalid coordinate triplet");

    std::filesystem::remove_all(dir);
}

TEST(InputParsersTests, ThrowsForMissionBoundariesOutsideMapDimensions) {
    const std::filesystem::path dir = unique_temp_dir("mission_map_dimensions");
    const std::filesystem::path path = dir / "mission_config.txt";
    write_text_file(path, valid_mission_config_text());

    expect_runtime_error_contains([&] {
        (void)parse_mission_config(path, MapDimensions{4, 5, 3});
    }, "X boundaries must fit");

    std::filesystem::remove_all(dir);
}

TEST(InputParsersTests, ThrowsForUnsupportedMissionResolution) {
    const std::filesystem::path dir = unique_temp_dir("mission_invalid_resolution");
    const std::filesystem::path path = dir / "mission_config.txt";
    write_text_file(path,
                    "boundary_min_x_cm=0\n"
                    "boundary_max_x_cm=4\n"
                    "boundary_min_y_cm=0\n"
                    "boundary_max_y_cm=4\n"
                    "boundary_min_z_cm=0\n"
                    "boundary_max_z_cm=2\n"
                    "initial_x_cm=1\n"
                    "initial_y_cm=1\n"
                    "initial_z_cm=1\n"
                    "initial_heading_deg=0\n"
                    "resolution_xy_decimals=1\n"
                    "resolution_z_decimals=0\n");

    expect_runtime_error_contains([&] {
        (void)parse_mission_config(path, MapDimensions{5, 5, 3});
    }, "only 0 decimal-place XY/Z resolution is supported");

    std::filesystem::remove_all(dir);
}

TEST(InputParsersTests, ThrowsForMissingMapInput) {
    EXPECT_THROW({
        (void)parse_map_input("/tmp/does-not-exist-map-input.txt");
    }, std::runtime_error);
}

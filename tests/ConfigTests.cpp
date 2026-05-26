#include "TestHelpers.h"

#include "config/Config.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

using namespace drone_mapper;

namespace {

[[nodiscard]] bool contains_error(const std::vector<std::string>& errors, const std::string& expected) {
    return std::find(errors.begin(), errors.end(), expected) != errors.end();
}

} // namespace

TEST(ConfigTests, DroneConfigDefaultsMatchDocumentedCapabilities) {
    const DroneConfig config;

    EXPECT_EQ(config.drone_radius, 1.0 * cm);
    EXPECT_EQ(config.max_rotate, 90.0 * horizontal_angle[deg]);
    EXPECT_EQ(config.max_advance, 1.0 * cm);
    EXPECT_EQ(config.max_elevate, 1.0 * cm);
    EXPECT_EQ(config.lidar.beam_length_min, 1.0 * cm);
    EXPECT_EQ(config.lidar.beam_length_max, 10.0 * cm);
    EXPECT_EQ(config.lidar.circle_spacing, 1.0 * cm);
    EXPECT_EQ(config.lidar.fov_circles, 1U);
    EXPECT_TRUE(config.is_valid());
}

TEST(ConfigTests, DroneConfigStoresCustomSphereRadius) {
    DroneConfig config;
    config.drone_radius = 3.0 * cm;

    EXPECT_EQ(config.drone_radius, 3.0 * cm);
}

TEST(ConfigTests, DroneConfigValidationRejectsInvalidPhysicalCapabilities) {
    DroneConfig config;
    config.drone_radius = 0.0 * cm;
    config.max_rotate = 0.0 * horizontal_angle[deg];
    config.max_advance = -1.0 * cm;
    config.max_elevate = 0.0 * cm;

    const std::vector<std::string> errors = config.validate();

    EXPECT_FALSE(errors.empty());
    EXPECT_FALSE(config.is_valid());
    EXPECT_TRUE(contains_error(errors, "drone_radius must be positive"));
    EXPECT_TRUE(contains_error(errors, "max_rotate must be positive"));
    EXPECT_TRUE(contains_error(errors, "max_advance must be positive"));
    EXPECT_TRUE(contains_error(errors, "max_elevate must be positive"));
}

TEST(ConfigTests, DroneConfigValidationRejectsInvalidLidarCapabilities) {
    DroneConfig config;
    config.lidar.beam_length_min = -1.0 * cm;
    config.lidar.beam_length_max = -2.0 * cm;
    config.lidar.circle_spacing = 0.0 * cm;
    config.lidar.fov_circles = 0;

    const std::vector<std::string> errors = config.validate();

    EXPECT_FALSE(config.is_valid());
    EXPECT_TRUE(contains_error(errors, "lidar.beam_length_min must be non-negative"));
    EXPECT_TRUE(contains_error(errors, "lidar.beam_length_max must be greater than lidar.beam_length_min"));
    EXPECT_TRUE(contains_error(errors, "lidar.circle_spacing must be positive"));
    EXPECT_TRUE(contains_error(errors, "lidar.fov_circles must be at least 1"));
}

TEST(ConfigTests, MissionConfigDefaultsRepresentSingleCellMission) {
    const MissionConfig config;

    EXPECT_EQ(config.boundaries.min_x, 0.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.max_x, 0.0 * x_extent[cm]);
    EXPECT_EQ(config.boundaries.min_y, 0.0 * y_extent[cm]);
    EXPECT_EQ(config.boundaries.max_y, 0.0 * y_extent[cm]);
    EXPECT_EQ(config.boundaries.min_z, 0.0 * z_extent[cm]);
    EXPECT_EQ(config.boundaries.max_z, 0.0 * z_extent[cm]);
    EXPECT_EQ(config.initial_position.x, 0.0 * x_extent[cm]);
    EXPECT_EQ(config.initial_position.y, 0.0 * y_extent[cm]);
    EXPECT_EQ(config.initial_position.z, 0.0 * z_extent[cm]);
    EXPECT_EQ(config.initial_heading.horizontal, 0.0 * horizontal_angle[deg]);
    EXPECT_EQ(config.initial_heading.altitude, 0.0 * altitude_angle[deg]);
    EXPECT_EQ(config.resolution.xy_decimal_places, 0);
    EXPECT_EQ(config.resolution.z_decimal_places, 0);
    EXPECT_TRUE(config.recharge_positions.empty());
    EXPECT_TRUE(config.is_valid());
}

TEST(ConfigTests, MappingBoundariesCheckInclusiveContainment) {
    MappingBoundaries boundaries;
    boundaries.min_x = 1.0 * x_extent[cm];
    boundaries.max_x = 2.0 * x_extent[cm];
    boundaries.min_y = 3.0 * y_extent[cm];
    boundaries.max_y = 4.0 * y_extent[cm];
    boundaries.min_z = 5.0 * z_extent[cm];
    boundaries.max_z = 6.0 * z_extent[cm];

    EXPECT_TRUE(boundaries.contains(test::make_position(1.0, 3.0, 5.0)));
    EXPECT_TRUE(boundaries.contains(test::make_position(2.0, 4.0, 6.0)));
    EXPECT_FALSE(boundaries.contains(test::make_position(0.99, 3.0, 5.0)));
    EXPECT_FALSE(boundaries.contains(test::make_position(2.01, 4.0, 6.0)));
}

TEST(ConfigTests, MissionConfigValidationRejectsInvalidBoundariesAndUnsupportedResolution) {
    MissionConfig config;
    config.boundaries.min_x = 2.0 * x_extent[cm];
    config.boundaries.max_x = 1.0 * x_extent[cm];
    config.resolution.xy_decimal_places = 1;

    const std::vector<std::string> errors = config.validate();

    EXPECT_FALSE(config.is_valid());
    EXPECT_TRUE(contains_error(errors, "boundary min_x must be less than or equal to max_x"));
    EXPECT_TRUE(contains_error(errors, "only 0 decimal-place XY/Z resolution is supported"));
}

TEST(ConfigTests, MissionConfigValidationRejectsInitialPositionOutsideBoundaries) {
    MissionConfig config;
    config.boundaries.max_x = 2.0 * x_extent[cm];
    config.boundaries.max_y = 2.0 * y_extent[cm];
    config.boundaries.max_z = 2.0 * z_extent[cm];
    config.initial_position = test::make_position(3.0, 1.0, 1.0);

    const std::vector<std::string> errors = config.validate();

    EXPECT_FALSE(config.is_valid());
    EXPECT_TRUE(contains_error(errors, "initial_position must be inside mission boundaries"));
}

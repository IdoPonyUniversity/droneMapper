#include "TestHelpers.h"

#include "map/SparseTextMap.h"

#include <gtest/gtest.h>

using namespace drone_mapper;

TEST(SparseTextMapTests, EmptyMapReturnsZeroEverywhere) {
    const SparseTextMap map(5, 5, 5);
    EXPECT_EQ(map.x_size(), 5U);
    EXPECT_EQ(map.y_size(), 5U);
    EXPECT_EQ(map.z_size(), 5U);
    EXPECT_EQ(map.occupied_count(), 0U);
    EXPECT_EQ(map.get(test::make_position(0.0, 0.0, 0.0)), 0);
    EXPECT_EQ(map.get(test::make_position(2.5, 2.5, 2.5)), 0);
}

TEST(SparseTextMapTests, OccupiedVoxelReportsAsOccupiedAtAnyPointInsideIt) {
    SparseTextMap map(5, 5, 5);
    map.setOccupied(2, 3, 4);

    EXPECT_EQ(map.occupied_count(), 1U);
    EXPECT_EQ(map.get(test::make_position(2.0, 3.0, 4.0)), 1);
    EXPECT_EQ(map.get(test::make_position(2.5, 3.5, 4.5)), 1);
    EXPECT_EQ(map.get(test::make_position(2.99, 3.99, 4.99)), 1);
    // adjacent voxels remain empty
    EXPECT_EQ(map.get(test::make_position(1.5, 3.5, 4.5)), 0);
    EXPECT_EQ(map.get(test::make_position(3.0, 3.5, 4.5)), 0);
}

TEST(SparseTextMapTests, OutOfBoundsCoordinatesReturnZero) {
    SparseTextMap map(5, 5, 5);
    map.setOccupied(0, 0, 0);
    map.setOccupied(4, 4, 4);

    // Negative
    EXPECT_EQ(map.get(test::make_position(-0.5, 0.0, 0.0)), 0);
    EXPECT_EQ(map.get(test::make_position(0.0, -0.5, 0.0)), 0);
    EXPECT_EQ(map.get(test::make_position(0.0, 0.0, -0.5)), 0);
    // Past upper bound
    EXPECT_EQ(map.get(test::make_position(5.0, 4.0, 4.0)), 0);
    EXPECT_EQ(map.get(test::make_position(4.0, 5.0, 4.0)), 0);
    EXPECT_EQ(map.get(test::make_position(4.0, 4.0, 5.0)), 0);
}

TEST(SparseTextMapTests, SetOccupiedSilentlyIgnoresOutOfBounds) {
    SparseTextMap map(5, 5, 5);
    map.setOccupied(10, 10, 10);
    EXPECT_EQ(map.occupied_count(), 0U);
}

TEST(SparseTextMapTests, SetDimensionsReplacesShape) {
    SparseTextMap map;
    map.setDimensions(10, 20, 30);
    EXPECT_EQ(map.x_size(), 10U);
    EXPECT_EQ(map.y_size(), 20U);
    EXPECT_EQ(map.z_size(), 30U);
    map.setOccupied(9, 19, 29);
    EXPECT_EQ(map.get(test::make_position(9.5, 19.5, 29.5)), 1);
}

TEST(SparseTextMapTests, DuplicateSetsAreIdempotent) {
    SparseTextMap map(3, 3, 3);
    map.setOccupied(1, 1, 1);
    map.setOccupied(1, 1, 1);
    EXPECT_EQ(map.occupied_count(), 1U);
}

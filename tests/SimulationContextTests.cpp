#include "TestHelpers.h"

#include "IMap3D.h"
#include "SimulationContext.h"
#include "SparseTextMap.h"

#include <gtest/gtest.h>

using namespace drone_mapper;

namespace {

class EmptyMap final : public IMap3D {
public:
    [[nodiscard]] int get(const Position3D&) const override { return 0; }
};

class AlwaysOccupiedMap final : public IMap3D {
public:
    [[nodiscard]] int get(const Position3D&) const override { return 1; }
};

} // namespace

TEST(SimulationContextTests, ConstructorStoresInitialPose) {
    const EmptyMap map;
    SimulationContext ctx(map,
                          test::make_position(1.0, 2.0, 3.0),
                          test::make_orientation(45.0, 0.0));
    EXPECT_EQ(ctx.position().x, 1.0 * x_extent[cm]);
    EXPECT_EQ(ctx.position().y, 2.0 * y_extent[cm]);
    EXPECT_EQ(ctx.position().z, 3.0 * z_extent[cm]);
    EXPECT_NEAR(test::degrees(ctx.heading().horizontal), 45.0, 1e-9);
}

TEST(SimulationContextTests, ApplyAdvanceMovesAlongHorizontalHeading) {
    const EmptyMap map;
    SimulationContext ctx(map,
                          test::make_position(0.0, 0.0, 0.0),
                          test::make_orientation(0.0, 0.0));
    ctx.applyAdvance(5.0 * cm);

    EXPECT_NEAR(ctx.position().x.force_numerical_value_in(cm), 5.0, 1e-9);
    EXPECT_NEAR(ctx.position().y.force_numerical_value_in(cm), 0.0, 1e-9);
    EXPECT_NEAR(ctx.position().z.force_numerical_value_in(cm), 0.0, 1e-9);
}

TEST(SimulationContextTests, ApplyAdvanceAtNinetyDegreesMovesPositiveY) {
    const EmptyMap map;
    SimulationContext ctx(map,
                          test::make_position(0.0, 0.0, 0.0),
                          test::make_orientation(90.0, 0.0));
    ctx.applyAdvance(7.0 * cm);

    EXPECT_NEAR(ctx.position().x.force_numerical_value_in(cm), 0.0, 1e-9);
    EXPECT_NEAR(ctx.position().y.force_numerical_value_in(cm), 7.0, 1e-9);
}

TEST(SimulationContextTests, ApplyAdvanceWithNegativeDistanceMovesBackward) {
    const EmptyMap map;
    SimulationContext ctx(map,
                          test::make_position(10.0, 10.0, 0.0),
                          test::make_orientation(0.0, 0.0));
    ctx.applyAdvance(-3.0 * cm);
    EXPECT_NEAR(ctx.position().x.force_numerical_value_in(cm), 7.0, 1e-9);
    EXPECT_NEAR(ctx.position().y.force_numerical_value_in(cm), 10.0, 1e-9);
}

TEST(SimulationContextTests, ApplyElevateChangesOnlyZ) {
    const EmptyMap map;
    SimulationContext ctx(map,
                          test::make_position(1.0, 2.0, 3.0),
                          test::make_orientation(45.0, 0.0));
    ctx.applyElevate(4.5 * cm);
    EXPECT_NEAR(ctx.position().x.force_numerical_value_in(cm), 1.0, 1e-9);
    EXPECT_NEAR(ctx.position().y.force_numerical_value_in(cm), 2.0, 1e-9);
    EXPECT_NEAR(ctx.position().z.force_numerical_value_in(cm), 7.5, 1e-9);
}

TEST(SimulationContextTests, ApplyRotateAccumulatesHorizontalAngle) {
    const EmptyMap map;
    SimulationContext ctx(map,
                          test::make_position(0.0, 0.0, 0.0),
                          test::make_orientation(10.0, 0.0));
    ctx.applyRotate(30.0 * horizontal_angle[deg]);
    EXPECT_NEAR(test::degrees(ctx.heading().horizontal), 40.0, 1e-9);
    ctx.applyRotate(-15.0 * horizontal_angle[deg]);
    EXPECT_NEAR(test::degrees(ctx.heading().horizontal), 25.0, 1e-9);
}

TEST(SimulationContextTests, WouldCollideTrueWhenSphereOverlapsOccupiedVoxel) {
    SparseTextMap map(10, 10, 10);
    map.setOccupied(5, 5, 5);
    SimulationContext ctx(map,
                          test::make_position(0.0, 0.0, 0.0),
                          test::make_orientation(0.0, 0.0));
    // Sphere center inside the occupied voxel (5..6 on each axis), radius small.
    EXPECT_TRUE(ctx.wouldCollide(test::make_position(5.5, 5.5, 5.5), 0.1 * cm));
}

TEST(SimulationContextTests, WouldCollideFalseWhenSphereDoesNotReachOccupiedVoxel) {
    SparseTextMap map(10, 10, 10);
    map.setOccupied(5, 5, 5);
    SimulationContext ctx(map,
                          test::make_position(0.0, 0.0, 0.0),
                          test::make_orientation(0.0, 0.0));
    // Sphere centered far from the only occupied voxel.
    EXPECT_FALSE(ctx.wouldCollide(test::make_position(0.5, 0.5, 0.5), 1.0 * cm));
}

TEST(SimulationContextTests, WouldCollideTrueWhenSphereJustReachesAdjacentVoxel) {
    SparseTextMap map(10, 10, 10);
    map.setOccupied(5, 5, 5);
    SimulationContext ctx(map,
                          test::make_position(0.0, 0.0, 0.0),
                          test::make_orientation(0.0, 0.0));
    // Sphere center at (4.0, 5.5, 5.5). Closest face of voxel [5..6]x[5..6]x[5..6]
    // along x is at x=5.0, distance 1.0. radius 1.5 -> collides.
    EXPECT_TRUE(ctx.wouldCollide(test::make_position(4.0, 5.5, 5.5), 1.5 * cm));
    EXPECT_FALSE(ctx.wouldCollide(test::make_position(4.0, 5.5, 5.5), 0.5 * cm));
}

TEST(SimulationContextTests, WouldCollideFalseOnEmptyMapEvenForLargeSphere) {
    const EmptyMap map;
    SimulationContext ctx(map,
                          test::make_position(0.0, 0.0, 0.0),
                          test::make_orientation(0.0, 0.0));
    EXPECT_FALSE(ctx.wouldCollide(test::make_position(0.0, 0.0, 0.0), 50.0 * cm));
}

TEST(SimulationContextTests, WouldCollideTrueOnAlwaysOccupiedMapForAnyRadius) {
    const AlwaysOccupiedMap map;
    SimulationContext ctx(map,
                          test::make_position(0.0, 0.0, 0.0),
                          test::make_orientation(0.0, 0.0));
    EXPECT_TRUE(ctx.wouldCollide(test::make_position(5.0, 5.0, 5.0), 0.1 * cm));
}

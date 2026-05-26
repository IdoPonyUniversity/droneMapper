#pragma once

#include "map/IMap3D.h"
#include "core/Units.h"

namespace drone_mapper {

// Owns the simulator's ground-truth state for one drone: its true pose
// (Position3D + Orientation) and a reference to the ground-truth IMap3D.
// Only the mocks (position sensor, movement driver, lidar) touch this; the
// drone itself only sees the interfaces in Layer 2.
//
// All movement is signed: positive Advance moves along the current XY heading;
// positive Elevate moves +Z; positive Rotate increases the horizontal angle
// (driver layer maps Left/Right to the sign convention).
class SimulationContext {
public:
    SimulationContext(const IMap3D& truth_map,
                      Position3D initial_position,
                      Orientation initial_heading) noexcept;

    [[nodiscard]] Position3D position() const noexcept;
    [[nodiscard]] Orientation heading() const noexcept;

    void applyRotate(HorizontalAngle delta) noexcept;
    void applyAdvance(PhysicalLength distance) noexcept;
    void applyElevate(PhysicalLength distance) noexcept;

    // True if a sphere centered at `candidate` with the given radius overlaps
    // any occupied voxel of the ground-truth map. Uses sphere-AABB tests
    // against 1-cm voxel cubes.
    [[nodiscard]] bool wouldCollide(Position3D candidate, PhysicalLength radius) const;

private:
    const IMap3D& truth_;
    Position3D position_;
    Orientation heading_;
};

} // namespace drone_mapper

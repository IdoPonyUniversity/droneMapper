#include "SimulationContext.h"

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>

namespace drone_mapper {

namespace {

[[nodiscard]] Position3D advanced_position(const Position3D& origin,
                                           const Orientation& heading,
                                           PhysicalLength distance) noexcept {
    const auto cos_h = si::cos(heading.horizontal);
    const auto sin_h = si::sin(heading.horizontal);
    const double d_cm = distance.force_numerical_value_in(cm);
    return {
        origin.x + cos_h.force_numerical_value_in(mp::one) * d_cm * x_extent[cm],
        origin.y + sin_h.force_numerical_value_in(mp::one) * d_cm * y_extent[cm],
        origin.z,
    };
}

[[nodiscard]] Position3D elevated_position(const Position3D& origin,
                                           PhysicalLength distance) noexcept {
    return {
        origin.x,
        origin.y,
        origin.z + distance.force_numerical_value_in(cm) * z_extent[cm],
    };
}

} // namespace

SimulationContext::SimulationContext(const IMap3D& truth_map,
                                     Position3D initial_position,
                                     Orientation initial_heading) noexcept
    : truth_(truth_map),
      position_(initial_position),
      heading_(initial_heading) {}

Position3D SimulationContext::position() const noexcept { return position_; }

Orientation SimulationContext::heading() const noexcept { return heading_; }

void SimulationContext::applyRotate(HorizontalAngle delta) noexcept {
    heading_.horizontal = heading_.horizontal + delta;
}

void SimulationContext::applyAdvance(PhysicalLength distance) noexcept {
    position_ = advanced_position(position_, heading_, distance);
}

void SimulationContext::applyElevate(PhysicalLength distance) noexcept {
    position_ = elevated_position(position_, distance);
}

bool SimulationContext::wouldCollide(Position3D candidate, PhysicalLength radius) const {
    const double cx = candidate.x.force_numerical_value_in(cm);
    const double cy = candidate.y.force_numerical_value_in(cm);
    const double cz = candidate.z.force_numerical_value_in(cm);
    const double r = radius.force_numerical_value_in(cm);
    if (r < 0.0) {
        return false;
    }
    const double r2 = r * r;

    // Each voxel (i,j,k) covers the 1 cm cube [i, i+1) per axis.
    const int x_lo = static_cast<int>(std::floor(cx - r));
    const int x_hi = static_cast<int>(std::floor(cx + r));
    const int y_lo = static_cast<int>(std::floor(cy - r));
    const int y_hi = static_cast<int>(std::floor(cy + r));
    const int z_lo = static_cast<int>(std::floor(cz - r));
    const int z_hi = static_cast<int>(std::floor(cz + r));

    for (int i = x_lo; i <= x_hi; ++i) {
        const double x_min = static_cast<double>(i);
        const double x_max = x_min + 1.0;
        const double dx = std::clamp(cx, x_min, x_max) - cx;
        for (int j = y_lo; j <= y_hi; ++j) {
            const double y_min = static_cast<double>(j);
            const double y_max = y_min + 1.0;
            const double dy = std::clamp(cy, y_min, y_max) - cy;
            for (int k = z_lo; k <= z_hi; ++k) {
                const double z_min = static_cast<double>(k);
                const double z_max = z_min + 1.0;
                const double dz = std::clamp(cz, z_min, z_max) - cz;
                if (dx * dx + dy * dy + dz * dz > r2) {
                    continue;
                }
                const Position3D probe{
                    (static_cast<double>(i) + 0.5) * x_extent[cm],
                    (static_cast<double>(j) + 0.5) * y_extent[cm],
                    (static_cast<double>(k) + 0.5) * z_extent[cm],
                };
                if (truth_.get(probe) != 0) {
                    return true;
                }
            }
        }
    }
    return false;
}

} // namespace drone_mapper

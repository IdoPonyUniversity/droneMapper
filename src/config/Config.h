#pragma once

#include "core/Units.h"

#include <cstddef>
#include <string>
#include <vector>

namespace drone_mapper {

// Static scan settings for a LiDAR-like sensor.
struct LidarConfig {
    PhysicalLength beam_length_min{};
    PhysicalLength beam_length_max{};
    PhysicalLength circle_spacing{};
    std::size_t fov_circles{};
};

struct DroneConfig {
    PhysicalLength drone_radius{1.0 * cm};
    HorizontalAngle max_rotate{90.0 * horizontal_angle[deg]};
    PhysicalLength max_advance{1.0 * cm};
    PhysicalLength max_elevate{1.0 * cm};
    LidarConfig lidar{
        1.0 * cm,
        10.0 * cm,
        1.0 * cm,
        1,
    };

    [[nodiscard]] std::vector<std::string> validate() const;
    [[nodiscard]] bool is_valid() const;
};

struct MappingBoundaries {
    XLength min_x{0.0 * x_extent[cm]};
    XLength max_x{0.0 * x_extent[cm]};
    YLength min_y{0.0 * y_extent[cm]};
    YLength max_y{0.0 * y_extent[cm]};
    ZLength min_z{0.0 * z_extent[cm]};
    ZLength max_z{0.0 * z_extent[cm]};

    [[nodiscard]] bool contains(const Position3D& position) const noexcept;
    [[nodiscard]] std::vector<std::string> validate() const;
};

struct MappingResolution {
    int xy_decimal_places{0};
    int z_decimal_places{0};

    [[nodiscard]] bool is_supported() const noexcept;
};

struct MissionConfig {
    MappingBoundaries boundaries{};
    Position3D initial_position{};
    Orientation initial_heading{};
    MappingResolution resolution{};
    std::vector<Position3D> recharge_positions{};

    [[nodiscard]] std::vector<std::string> validate() const;
    [[nodiscard]] bool is_valid() const;
};

} // namespace drone_mapper

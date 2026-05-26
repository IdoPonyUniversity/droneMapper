#include "config/Config.h"

#include <stdexcept>
#include <string>

namespace drone_mapper {

namespace {

[[nodiscard]] double max_index_coordinate(std::size_t size) noexcept {
    return static_cast<double>(size - 1U);
}

[[noreturn]] void throw_validation_error(const char* prefix, const char* message) {
    throw std::runtime_error(std::string(prefix) + message);
}

void validate_boundaries(const MappingBoundaries& boundaries, const char* prefix) {
    if (boundaries.min_x > boundaries.max_x) {
        throw_validation_error(prefix, "boundary min_x must be less than or equal to max_x");
    }
    if (boundaries.min_y > boundaries.max_y) {
        throw_validation_error(prefix, "boundary min_y must be less than or equal to max_y");
    }
    if (boundaries.min_z > boundaries.max_z) {
        throw_validation_error(prefix, "boundary min_z must be less than or equal to max_z");
    }
}

} // namespace

void DroneConfig::validate() const {
    constexpr const char* prefix = "invalid drone configuration: ";

    if (drone_radius <= 0.0 * cm) {
        throw_validation_error(prefix, "drone_radius must be positive");
    }
    if (max_rotate <= 0.0 * horizontal_angle[deg]) {
        throw_validation_error(prefix, "max_rotate must be positive");
    }
    if (max_advance <= 0.0 * cm) {
        throw_validation_error(prefix, "max_advance must be positive");
    }
    if (max_elevate <= 0.0 * cm) {
        throw_validation_error(prefix, "max_elevate must be positive");
    }
    if (lidar.beam_length_min < 0.0 * cm) {
        throw_validation_error(prefix, "lidar.beam_length_min must be non-negative");
    }
    if (lidar.beam_length_max <= lidar.beam_length_min) {
        throw_validation_error(prefix, "lidar.beam_length_max must be greater than lidar.beam_length_min");
    }
    if (lidar.circle_spacing <= 0.0 * cm) {
        throw_validation_error(prefix, "lidar.circle_spacing must be positive");
    }
    if (lidar.fov_circles < 1U) {
        throw_validation_error(prefix, "lidar.fov_circles must be at least 1");
    }
}

bool MappingBoundaries::contains(const Position3D& position) const noexcept {
    return position.x >= min_x && position.x <= max_x &&
           position.y >= min_y && position.y <= max_y &&
           position.z >= min_z && position.z <= max_z;
}

void MappingBoundaries::validate() const {
    validate_boundaries(*this, "invalid mission boundaries: ");
}

bool MappingResolution::is_supported() const noexcept {
    return xy_decimal_places == 0 && z_decimal_places == 0;
}

void MissionConfig::validate(std::optional<MapDimensions> map_dimensions) const {
    constexpr const char* prefix = "invalid mission configuration: ";

    validate_boundaries(boundaries, prefix);
    if (map_dimensions) {
        if (map_dimensions->x_size == 0U || map_dimensions->y_size == 0U || map_dimensions->z_size == 0U) {
            throw_validation_error(prefix, "map dimensions must be non-empty");
        }
        if (boundaries.min_x.force_numerical_value_in(cm) < 0.0 ||
            boundaries.max_x.force_numerical_value_in(cm) > max_index_coordinate(map_dimensions->x_size)) {
            throw_validation_error(prefix, "X boundaries must fit within provided map dimensions");
        }
        if (boundaries.min_y.force_numerical_value_in(cm) < 0.0 ||
            boundaries.max_y.force_numerical_value_in(cm) > max_index_coordinate(map_dimensions->y_size)) {
            throw_validation_error(prefix, "Y boundaries must fit within provided map dimensions");
        }
        if (boundaries.min_z.force_numerical_value_in(cm) < 0.0 ||
            boundaries.max_z.force_numerical_value_in(cm) > max_index_coordinate(map_dimensions->z_size)) {
            throw_validation_error(prefix, "Z boundaries must fit within provided map dimensions");
        }
    }
    if (!boundaries.contains(initial_position)) {
        throw_validation_error(prefix, "initial_position must be inside mission boundaries");
    }
    const double heading_degrees = initial_heading.horizontal.force_numerical_value_in(deg);
    if (heading_degrees < 0.0 || heading_degrees > 360.0) {
        throw_validation_error(prefix, "initial_heading must be in the range [0, 360]");
    }
    if (!resolution.is_supported()) {
        throw_validation_error(prefix, "only 0 decimal-place XY/Z resolution is supported");
    }
}

} // namespace drone_mapper

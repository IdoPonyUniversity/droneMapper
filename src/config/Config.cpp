#include "config/Config.h"

namespace drone_mapper {

namespace {

[[nodiscard]] double centimeters(PhysicalLength length) noexcept {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double degrees_value(HorizontalAngle angle) noexcept {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] bool positive(PhysicalLength length) noexcept {
    return centimeters(length) > 0.0;
}

[[nodiscard]] bool non_negative(PhysicalLength length) noexcept {
    return centimeters(length) >= 0.0;
}

[[nodiscard]] bool positive(HorizontalAngle angle) noexcept {
    return degrees_value(angle) > 0.0;
}

} // namespace

std::vector<std::string> DroneConfig::validate() const {
    std::vector<std::string> errors;

    if (!positive(drone_radius)) {
        errors.emplace_back("drone_radius must be positive");
    }
    if (!positive(max_rotate)) {
        errors.emplace_back("max_rotate must be positive");
    }
    if (!positive(max_advance)) {
        errors.emplace_back("max_advance must be positive");
    }
    if (!positive(max_elevate)) {
        errors.emplace_back("max_elevate must be positive");
    }
    if (!non_negative(lidar.beam_length_min)) {
        errors.emplace_back("lidar.beam_length_min must be non-negative");
    }
    if (!(centimeters(lidar.beam_length_max) > centimeters(lidar.beam_length_min))) {
        errors.emplace_back("lidar.beam_length_max must be greater than lidar.beam_length_min");
    }
    if (!positive(lidar.circle_spacing)) {
        errors.emplace_back("lidar.circle_spacing must be positive");
    }
    if (lidar.fov_circles < 1U) {
        errors.emplace_back("lidar.fov_circles must be at least 1");
    }

    return errors;
}

bool DroneConfig::is_valid() const {
    return validate().empty();
}

bool MappingBoundaries::contains(const Position3D& position) const noexcept {
    return position.x >= min_x && position.x <= max_x &&
           position.y >= min_y && position.y <= max_y &&
           position.z >= min_z && position.z <= max_z;
}

std::vector<std::string> MappingBoundaries::validate() const {
    std::vector<std::string> errors;

    if (min_x > max_x) {
        errors.emplace_back("boundary min_x must be less than or equal to max_x");
    }
    if (min_y > max_y) {
        errors.emplace_back("boundary min_y must be less than or equal to max_y");
    }
    if (min_z > max_z) {
        errors.emplace_back("boundary min_z must be less than or equal to max_z");
    }

    return errors;
}

bool MappingResolution::is_supported() const noexcept {
    return xy_decimal_places == 0 && z_decimal_places == 0;
}

std::vector<std::string> MissionConfig::validate() const {
    std::vector<std::string> errors = boundaries.validate();

    if (errors.empty() && !boundaries.contains(initial_position)) {
        errors.emplace_back("initial_position must be inside mission boundaries");
    }
    if (!resolution.is_supported()) {
        errors.emplace_back("only 0 decimal-place XY/Z resolution is supported");
    }

    return errors;
}

bool MissionConfig::is_valid() const {
    return validate().empty();
}

} // namespace drone_mapper

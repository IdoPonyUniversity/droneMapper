#pragma once

#include <cpp_course/Config.h>
#include <cpp_course/SparseTextMap.h>

#include <cstddef>
#include <filesystem>
#include <optional>

namespace cpp_course {

struct MapDimensions {
    std::size_t x_size{};
    std::size_t y_size{};
    std::size_t z_size{};
};

[[nodiscard]] DroneConfig parse_drone_config(const std::filesystem::path& path);

[[nodiscard]] MissionConfig parse_mission_config(const std::filesystem::path& path, std::optional<MapDimensions> map_dimensions = std::nullopt);

[[nodiscard]] SparseTextMap parse_map_input(const std::filesystem::path& path);
} // namespace cpp_course

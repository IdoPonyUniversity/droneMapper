#include "io/InputParsers.h"

#include "map/VoxelGrid.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_mapper {

namespace {

struct KeyValueLine {
    std::string key{};
    std::string value{};
    std::size_t line_number{};
};

using ScalarMap = std::unordered_map<std::string, KeyValueLine>;

[[nodiscard]] std::string trim(std::string_view text) {
    const auto begin = std::find_if_not(text.begin(), text.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    const auto end = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();

    if (begin >= end) {
        return {};
    }
    return std::string(begin, end);
}

[[nodiscard]] std::string location(const std::filesystem::path& path, std::size_t line_number = 0U) {
    std::ostringstream stream;
    stream << path.string();
    if (line_number != 0U) {
        stream << ':' << line_number;
    }
    return stream.str();
}

[[noreturn]] void throw_input_error(const std::filesystem::path& path, std::size_t line_number, const std::string& message) {
    throw std::runtime_error(location(path, line_number) + ": " + message);
}

[[nodiscard]] std::vector<KeyValueLine> read_key_value_lines(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw_input_error(path, 0U, "could not be opened");
    }

    std::vector<KeyValueLine> lines;
    std::string line;
    std::size_t line_number = 0U;
    while (std::getline(input, line)) {
        ++line_number;
        const std::string stripped = trim(line);
        if (stripped.empty() || stripped.front() == '#') {
            continue;
        }

        const std::size_t separator = stripped.find('=');
        if (separator == std::string::npos) {
            throw_input_error(path, line_number, "malformed line; expected key=value");
        }

        const std::string key = trim(std::string_view(stripped).substr(0, separator));
        const std::string value = trim(std::string_view(stripped).substr(separator + 1));
        if (key.empty()) {
            throw_input_error(path, line_number, "malformed line; key must not be empty");
        }

        lines.push_back({key, value, line_number});
    }

    return lines;
}

[[nodiscard]] std::optional<double> parse_double(std::string_view text) {
    const std::string value = trim(text);
    if (value.empty()) {
        return std::nullopt;
    }

    try {
        std::size_t index = 0U;
        const double parsed = std::stod(value, &index);
        if (index != value.size() || !std::isfinite(parsed)) {
            return std::nullopt;
        }
        return parsed;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

[[nodiscard]] std::optional<int> parse_int(std::string_view text) {
    const std::string value = trim(text);
    if (value.empty()) {
        return std::nullopt;
    }

    try {
        std::size_t index = 0U;
        const long parsed = std::stol(value, &index, 10);
        if (index != value.size() || parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
            return std::nullopt;
        }
        return static_cast<int>(parsed);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

[[nodiscard]] std::string join_errors(const std::vector<std::string>& errors) {
    std::ostringstream stream;
    for (std::size_t i = 0U; i < errors.size(); ++i) {
        if (i != 0U) {
            stream << "; ";
        }
        stream << errors[i];
    }
    return stream.str();
}

[[nodiscard]] ScalarMap collect_scalars(const std::filesystem::path& path,
                                        const std::vector<KeyValueLine>& lines,
                                        const std::unordered_set<std::string>& scalar_keys,
                                        const std::unordered_set<std::string>& repeatable_keys,
                                        std::vector<KeyValueLine>* repeatable_lines = nullptr) {
    ScalarMap scalars;

    for (const KeyValueLine& line : lines) {
        if (repeatable_keys.count(line.key) != 0U) {
            if (repeatable_lines != nullptr) {
                repeatable_lines->push_back(line);
            }
            continue;
        }

        if (scalar_keys.count(line.key) == 0U) {
            throw_input_error(path, line.line_number, "unknown key '" + line.key + "'");
        }

        if (scalars.count(line.key) != 0U) {
            throw_input_error(path, line.line_number, "duplicate key '" + line.key + "'");
        }
        scalars.emplace(line.key, line);
    }

    return scalars;
}

[[nodiscard]] const KeyValueLine* find_scalar(const ScalarMap& scalars, const std::string& key) noexcept {
    const auto found = scalars.find(key);
    if (found == scalars.end()) {
        return nullptr;
    }
    return &found->second;
}

[[nodiscard]] double required_double(const std::filesystem::path& path, const KeyValueLine& line) {
    const std::optional<double> value = parse_double(line.value);
    if (!value) {
        throw_input_error(path, line.line_number, "invalid number for '" + line.key + "'");
    }
    return *value;
}

[[nodiscard]] int required_int(const std::filesystem::path& path, const KeyValueLine& line) {
    const std::optional<int> value = parse_int(line.value);
    if (!value) {
        throw_input_error(path, line.line_number, "invalid integer for '" + line.key + "'");
    }
    return *value;
}

[[nodiscard]] double required_positive_double(const std::filesystem::path& path, const KeyValueLine& line, std::string_view unit_name) {
    const double value = required_double(path, line);
    if (value <= 0.0) {
        throw_input_error(path, line.line_number, "'" + line.key + "' must be a positive " + std::string(unit_name));
    }
    return value;
}

[[nodiscard]] double required_non_negative_double(const std::filesystem::path& path, const KeyValueLine& line, std::string_view unit_name) {
    const double value = required_double(path, line);
    if (value < 0.0) {
        throw_input_error(path, line.line_number, "'" + line.key + "' must be a non-negative " + std::string(unit_name));
    }
    return value;
}

[[nodiscard]] std::size_t required_positive_size(const std::filesystem::path& path, const KeyValueLine& line) {
    const int value = required_int(path, line);
    if (value < 1) {
        throw_input_error(path, line.line_number, "'" + line.key + "' must be a positive integer");
    }
    return static_cast<std::size_t>(value);
}

[[nodiscard]] std::vector<double> required_coordinate_triplet(const std::filesystem::path& path, const KeyValueLine& line) {
    std::vector<double> values;
    std::string remaining{line.value};
    while (true) {
        const std::size_t comma = remaining.find(',');
        const std::string token = comma == std::string::npos ? remaining : remaining.substr(0, comma);
        const std::optional<double> value = parse_double(token);
        if (!value) {
            throw_input_error(path, line.line_number, "invalid coordinate triplet for '" + line.key + "'; expected x,y,z centimeters");
        }
        values.push_back(*value);

        if (comma == std::string::npos) {
            break;
        }
        remaining = remaining.substr(comma + 1U);
    }

    if (values.size() != 3U) {
        throw_input_error(path, line.line_number, "invalid coordinate triplet for '" + line.key + "'; expected x,y,z centimeters");
    }
    return values;
}

[[nodiscard]] Position3D make_position(double x_cm, double y_cm, double z_cm) {
    return {
        x_cm * x_extent[cm],
        y_cm * y_extent[cm],
        z_cm * z_extent[cm],
    };
}

[[nodiscard]] double x_centimeters(XLength length) noexcept {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double y_centimeters(YLength length) noexcept {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double z_centimeters(ZLength length) noexcept {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double max_index_coordinate(std::size_t size) noexcept {
    return size == 0U ? 0.0 : static_cast<double>(size - 1U);
}

void validate_boundaries_against_map_dimensions(const std::filesystem::path& path,
                                                const MissionConfig& config,
                                                const MapDimensions& dimensions) {
    if (x_centimeters(config.boundaries.min_x) < 0.0 || x_centimeters(config.boundaries.max_x) > max_index_coordinate(dimensions.x_size)) {
        throw_input_error(path, 0U, "X boundaries must fit within provided map dimensions");
    }
    if (y_centimeters(config.boundaries.min_y) < 0.0 || y_centimeters(config.boundaries.max_y) > max_index_coordinate(dimensions.y_size)) {
        throw_input_error(path, 0U, "Y boundaries must fit within provided map dimensions");
    }
    if (z_centimeters(config.boundaries.min_z) < 0.0 || z_centimeters(config.boundaries.max_z) > max_index_coordinate(dimensions.z_size)) {
        throw_input_error(path, 0U, "Z boundaries must fit within provided map dimensions");
    }
}

} // namespace

DroneConfig parse_drone_config(const std::filesystem::path& path) {
    const std::vector<std::string> known_keys{
        "drone_radius_cm",
        "max_rotate_deg",
        "max_advance_cm",
        "max_elevate_cm",
        "lidar_z_min_cm",
        "lidar_z_max_cm",
        "lidar_circle_spacing_cm",
        "lidar_circle_count",
    };
    const std::unordered_set<std::string> scalar_keys(known_keys.begin(), known_keys.end());

    const std::vector<KeyValueLine> lines = read_key_value_lines(path);
    const ScalarMap scalars = collect_scalars(path, lines, scalar_keys, {});

    DroneConfig config;
    if (const KeyValueLine* line = find_scalar(scalars, "drone_radius_cm")) {
        config.drone_radius = required_positive_double(path, *line, "distance") * cm;
    }
    if (const KeyValueLine* line = find_scalar(scalars, "max_rotate_deg")) {
        config.max_rotate = required_positive_double(path, *line, "angle") * horizontal_angle[deg];
    }
    if (const KeyValueLine* line = find_scalar(scalars, "max_advance_cm")) {
        config.max_advance = required_positive_double(path, *line, "distance") * cm;
    }
    if (const KeyValueLine* line = find_scalar(scalars, "max_elevate_cm")) {
        config.max_elevate = required_positive_double(path, *line, "distance") * cm;
    }
    if (const KeyValueLine* line = find_scalar(scalars, "lidar_z_min_cm")) {
        config.lidar.beam_length_min = required_non_negative_double(path, *line, "distance") * cm;
    }
    if (const KeyValueLine* line = find_scalar(scalars, "lidar_z_max_cm")) {
        config.lidar.beam_length_max = required_positive_double(path, *line, "distance") * cm;
    }
    if (const KeyValueLine* line = find_scalar(scalars, "lidar_circle_spacing_cm")) {
        config.lidar.circle_spacing = required_positive_double(path, *line, "distance") * cm;
    }
    if (const KeyValueLine* line = find_scalar(scalars, "lidar_circle_count")) {
        config.lidar.fov_circles = required_positive_size(path, *line);
    }

    const std::vector<std::string> validation_errors = config.validate();
    if (!validation_errors.empty()) {
        throw_input_error(path, 0U, "invalid drone configuration: " + join_errors(validation_errors));
    }

    return config;
}

MissionConfig parse_mission_config(const std::filesystem::path& path, std::optional<MapDimensions> map_dimensions) {
    const std::vector<std::string> known_scalar_keys{
        "boundary_min_x_cm",
        "boundary_max_x_cm",
        "boundary_min_y_cm",
        "boundary_max_y_cm",
        "boundary_min_z_cm",
        "boundary_max_z_cm",
        "initial_x_cm",
        "initial_y_cm",
        "initial_z_cm",
        "initial_heading_deg",
        "resolution_xy_decimals",
        "resolution_z_decimals",
    };
    const std::unordered_set<std::string> scalar_keys(known_scalar_keys.begin(), known_scalar_keys.end());
    const std::unordered_set<std::string> repeatable_keys{"recharge"};

    std::vector<KeyValueLine> recharge_lines;
    const std::vector<KeyValueLine> lines = read_key_value_lines(path);
    const ScalarMap scalars = collect_scalars(path, lines, scalar_keys, repeatable_keys, &recharge_lines);

    MissionConfig config;
    if (map_dimensions) {
        config.boundaries.max_x = max_index_coordinate(map_dimensions->x_size) * x_extent[cm];
        config.boundaries.max_y = max_index_coordinate(map_dimensions->y_size) * y_extent[cm];
        config.boundaries.max_z = max_index_coordinate(map_dimensions->z_size) * z_extent[cm];
    }

    if (const KeyValueLine* line = find_scalar(scalars, "boundary_min_x_cm")) {
        config.boundaries.min_x = required_double(path, *line) * x_extent[cm];
    }
    if (const KeyValueLine* line = find_scalar(scalars, "boundary_max_x_cm")) {
        config.boundaries.max_x = required_double(path, *line) * x_extent[cm];
    }
    if (const KeyValueLine* line = find_scalar(scalars, "boundary_min_y_cm")) {
        config.boundaries.min_y = required_double(path, *line) * y_extent[cm];
    }
    if (const KeyValueLine* line = find_scalar(scalars, "boundary_max_y_cm")) {
        config.boundaries.max_y = required_double(path, *line) * y_extent[cm];
    }
    if (const KeyValueLine* line = find_scalar(scalars, "boundary_min_z_cm")) {
        config.boundaries.min_z = required_double(path, *line) * z_extent[cm];
    }
    if (const KeyValueLine* line = find_scalar(scalars, "boundary_max_z_cm")) {
        config.boundaries.max_z = required_double(path, *line) * z_extent[cm];
    }

    config.initial_position = {
        config.boundaries.min_x,
        config.boundaries.min_y,
        config.boundaries.min_z,
    };
    if (const KeyValueLine* line = find_scalar(scalars, "initial_x_cm")) {
        config.initial_position.x = required_double(path, *line) * x_extent[cm];
    }
    if (const KeyValueLine* line = find_scalar(scalars, "initial_y_cm")) {
        config.initial_position.y = required_double(path, *line) * y_extent[cm];
    }
    if (const KeyValueLine* line = find_scalar(scalars, "initial_z_cm")) {
        config.initial_position.z = required_double(path, *line) * z_extent[cm];
    }

    if (const KeyValueLine* line = find_scalar(scalars, "initial_heading_deg")) {
        const double initial_heading = required_double(path, *line);
        if (initial_heading < 0.0 || initial_heading > 360.0) {
            throw_input_error(path, line->line_number, "'initial_heading_deg' must be in the range [0, 360]");
        }
        config.initial_heading.horizontal = initial_heading * horizontal_angle[deg];
    }

    if (const KeyValueLine* line = find_scalar(scalars, "resolution_xy_decimals")) {
        config.resolution.xy_decimal_places = required_int(path, *line);
    }
    if (const KeyValueLine* line = find_scalar(scalars, "resolution_z_decimals")) {
        config.resolution.z_decimal_places = required_int(path, *line);
    }

    const std::vector<std::string> boundary_errors = config.boundaries.validate();
    if (!boundary_errors.empty()) {
        throw_input_error(path, 0U, "invalid mission boundaries: " + join_errors(boundary_errors));
    }

    if (map_dimensions) {
        validate_boundaries_against_map_dimensions(path, config, *map_dimensions);
    }

    if (!config.boundaries.contains(config.initial_position)) {
        throw_input_error(path, 0U, "initial position must be inside mission boundaries");
    }

    for (const KeyValueLine& line : recharge_lines) {
        const std::vector<double> coordinates = required_coordinate_triplet(path, line);
        config.recharge_positions.push_back(make_position(coordinates[0], coordinates[1], coordinates[2]));
    }

    const std::vector<std::string> validation_errors = config.validate();
    if (!validation_errors.empty()) {
        throw_input_error(path, 0U, "invalid mission configuration: " + join_errors(validation_errors));
    }

    return config;
}

SparseTextMap parse_map_input(const std::filesystem::path& path) {
    VoxelGrid grid;

    try {
        grid.load(path.string());
    } catch (const std::exception& error) {
        throw std::runtime_error("map_input.txt could not be loaded: " + std::string(error.what()));
    }

    const MapDimensions dimensions{grid.x_size(), grid.y_size(), grid.z_size()};
    if (dimensions.x_size == 0U || dimensions.y_size == 0U || dimensions.z_size == 0U) {
        throw std::runtime_error("map_input.txt must be a non-empty 3D map");
    }

    SparseTextMap map(dimensions.x_size, dimensions.y_size, dimensions.z_size);
    for (std::size_t x = 0U; x < dimensions.x_size; ++x) {
        for (std::size_t y = 0U; y < dimensions.y_size; ++y) {
            for (std::size_t z = 0U; z < dimensions.z_size; ++z) {
                const Position3D position = make_position(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
                const int value = grid.get(position);
                if (value == 0) {
                    continue;
                }
                if (value == 1) {
                    map.setOccupied(x, y, z);
                    continue;
                }

                std::ostringstream stream;
                stream << "map_input.txt contains unsupported value " << value
                       << " at voxel (" << x << ',' << y << ',' << z << "); expected only 0 or 1";
                throw std::runtime_error(stream.str());
            }
        }
    }

    return map;
}

} // namespace drone_mapper

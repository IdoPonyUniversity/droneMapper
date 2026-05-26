#include "map/SparseTextMap.h"

#include <cmath>
#include <optional>

namespace drone_mapper {

namespace {

[[nodiscard]] std::optional<std::size_t> centimeters_to_index(double centimeters) {
    if (centimeters < 0.0) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(std::floor(centimeters));
}

} // namespace

SparseTextMap::SparseTextMap(std::size_t x_size, std::size_t y_size, std::size_t z_size)
    : x_size_(x_size), y_size_(y_size), z_size_(z_size) {}

void SparseTextMap::setDimensions(std::size_t x_size, std::size_t y_size, std::size_t z_size) {
    x_size_ = x_size;
    y_size_ = y_size;
    z_size_ = z_size;
}

void SparseTextMap::setOccupied(std::size_t x, std::size_t y, std::size_t z) {
    if (x >= x_size_ || y >= y_size_ || z >= z_size_) {
        return;
    }
    occupied_.insert(pack(x, y, z));
}

int SparseTextMap::get(const Position3D& pos) const {
    const auto x = centimeters_to_index(pos.x.force_numerical_value_in(cm));
    const auto y = centimeters_to_index(pos.y.force_numerical_value_in(cm));
    const auto z = centimeters_to_index(pos.z.force_numerical_value_in(cm));
    if (!x || !y || !z) {
        return 0;
    }
    if (*x >= x_size_ || *y >= y_size_ || *z >= z_size_) {
        return 0;
    }
    return occupied_.count(pack(*x, *y, *z)) != 0 ? 1 : 0;
}

std::uint64_t SparseTextMap::pack(std::size_t x, std::size_t y, std::size_t z) noexcept {
    // 21 bits per axis -> up to 2,097,152 cm (~20 km) per dimension, plenty for any realistic building.
    constexpr std::uint64_t kMask = (1ULL << 21) - 1ULL;
    return (static_cast<std::uint64_t>(x) & kMask)
         | ((static_cast<std::uint64_t>(y) & kMask) << 21)
         | ((static_cast<std::uint64_t>(z) & kMask) << 42);
}

} // namespace drone_mapper

#pragma once

#include "IMap3D.h"

#include <cstddef>
#include <cstdint>
#include <unordered_set>

namespace drone_mapper {

// In-memory IMap3D backed by a sparse set of occupied 1-cm voxels and
// fixed [0..x_size) x [0..y_size) x [0..z_size) bounds. Used by the
// simulator as the ground-truth map (not visible to the drone).
class SparseTextMap final : public IMap3D {
public:
    SparseTextMap() = default;
    SparseTextMap(std::size_t x_size, std::size_t y_size, std::size_t z_size);

    void setDimensions(std::size_t x_size, std::size_t y_size, std::size_t z_size);

    // Mark voxel (x,y,z) as occupied. Out-of-bounds calls are ignored.
    void setOccupied(std::size_t x, std::size_t y, std::size_t z);

    [[nodiscard]] std::size_t x_size() const noexcept { return x_size_; }
    [[nodiscard]] std::size_t y_size() const noexcept { return y_size_; }
    [[nodiscard]] std::size_t z_size() const noexcept { return z_size_; }

    [[nodiscard]] std::size_t occupied_count() const noexcept { return occupied_.size(); }

    // IMap3D
    [[nodiscard]] int get(const Position3D& pos) const override;

private:
    [[nodiscard]] static std::uint64_t pack(std::size_t x, std::size_t y, std::size_t z) noexcept;

    std::size_t x_size_ = 0;
    std::size_t y_size_ = 0;
    std::size_t z_size_ = 0;
    std::unordered_set<std::uint64_t> occupied_{};
};

} // namespace drone_mapper

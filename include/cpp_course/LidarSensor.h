#pragma once

#include <vector>

#include <cpp_course/Units.h>

namespace cpp_course {

// A single LiDAR return. The angle is relative to the requested scan orientation.
struct LidarHit {
    PhysicalLength distance{};
    Orientation angle{};
};

typedef std::vector<LidarHit> ScanResults;

class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;

    // Scans around the requested relative orientation and returns all beam hits.
    [[nodiscard]] virtual ScanResults scan(Orientation scan_orientation) const = 0;
};

} // namespace cpp_course

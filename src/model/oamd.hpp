#pragma once

#include "eac3/emdf.hpp"

#include <cstdint>
#include <vector>

namespace atmos::model {

// A static object placement in normalised Atmos coordinates (x right, y front,
// z up, all in [-1, 1] except y in [0, 1]).
struct ObjectPlacement final {
    float x = 0.0F;
    float y = 0.5F;
    float z = 0.0F;
};

struct AtmosConfig final {
    std::uint16_t bed_channels = 6; // 5.1 bed
    std::vector<ObjectPlacement> objects;
};

[[nodiscard]] eac3::OamdFrame make_oamd(const AtmosConfig& config);

} // namespace atmos::model

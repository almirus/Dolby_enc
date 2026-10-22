#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace atmos::eac3 {

class Imdct256 final {
public:
    Imdct256();
    void reset() noexcept;
    void synthesize(std::span<const std::int32_t, 256> coefficients,
        std::span<float, 256> output, bool short_blocks = false);

private:
    std::array<double, 256> window_{};
    std::array<double, 128> delay_{};
};

} // namespace atmos::eac3

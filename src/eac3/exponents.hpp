#pragma once

#include "audio_frame.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace atmos {
class BitReader;
}

namespace atmos::eac3 {

[[nodiscard]] std::size_t decoded_exponent_count(
    ExponentStrategy strategy, std::size_t group_count);

void decode_exponents(BitReader& bits, ExponentStrategy strategy,
    std::size_t group_count, std::uint8_t absolute_exponent,
    std::span<std::int8_t> output);

} // namespace atmos::eac3

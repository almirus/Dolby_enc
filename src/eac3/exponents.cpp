#include "exponents.hpp"

#include "common/bit_reader.hpp"

#include <array>
#include <string>

namespace atmos::eac3 {

std::size_t decoded_exponent_count(
    ExponentStrategy strategy, std::size_t group_count) {
    const unsigned value = static_cast<unsigned>(strategy);
    if (value == 0U || value > 3U) throw ParseError("invalid exponent strategy");
    const unsigned group_size = value + (strategy == ExponentStrategy::D45 ? 1U : 0U);
    return group_count * 3U * group_size;
}

void decode_exponents(BitReader& bits, ExponentStrategy strategy,
    std::size_t group_count, std::uint8_t absolute_exponent,
    std::span<std::int8_t> output) {
    const std::size_t required = decoded_exponent_count(strategy, group_count);
    if (output.size() < required) {
        throw ParseError("invalid exponent output or initial value");
    }
    const unsigned value = static_cast<unsigned>(strategy);
    const unsigned group_size = value + (strategy == ExponentStrategy::D45 ? 1U : 0U);
    std::size_t cursor = 0;
    int previous = absolute_exponent;
    for (std::size_t group = 0; group < group_count; ++group) {
        const unsigned code = bits.read(7U);
        if (code >= 125U) throw ParseError("exponent group code out of range");
        const std::array<unsigned, 3> deltas = {
            code / 25U, (code % 25U) / 5U, code % 5U
        };
        for (const unsigned delta : deltas) {
            previous += static_cast<int>(delta) - 2;
            if (previous < 0 || previous > 24) {
                throw ParseError("decoded exponent out of range: " +
                    std::to_string(previous));
            }
            for (unsigned repeat = 0; repeat < group_size; ++repeat) {
                output[cursor++] = static_cast<std::int8_t>(previous);
            }
        }
    }
}

} // namespace atmos::eac3

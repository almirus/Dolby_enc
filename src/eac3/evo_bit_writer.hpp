#pragma once

#include "common/bit_writer.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>

namespace atmos::eac3 {

// Writes the non-extended frame configuration accepted by
// ddp_udc_int_evo_parse_frame_config. Configurations 1 and 2 end after the
// first two-bit field; configuration 0 has a second two-bit field.
inline bool write_evo_frame_config(BitWriter& bits, unsigned config,
    unsigned subconfig = 0U) {
    if (config > 2U || subconfig > 3U || (config != 0U && subconfig != 0U)) {
        return false;
    }
    bits.write(config, 2U);
    if (config == 0U) bits.write(subconfig, 2U);
    return true;
}

// Payload IDs 0..30 are encoded directly as the five-bit ID. ID 31 is the
// escape value and uses a separate extended coding path in the native
// encoder; reject it until that path is ported.
inline bool write_evo_payload_id(BitWriter& bits, unsigned payload_id) {
    if (payload_id > 30U) return false;
    bits.write(payload_id, 5U);
    return true;
}

// Inverse of the variable_bits() reader used by the Dolby EVO parser:
// each continued group contributes (digit + 1) before the next base shift.
inline bool write_variable_bits(BitWriter& bits, std::uint32_t value,
    unsigned width) {
    if (width == 0U || width > 31U) return false;
    const std::uint32_t base = std::uint32_t{1U} << width;
    std::uint32_t power = base;
    std::uint32_t next_group_minimum = base;
    unsigned groups = 1U;
    while (value >= next_group_minimum && next_group_minimum <=
        (std::numeric_limits<std::uint32_t>::max() - 1U) / base) {
        next_group_minimum = (next_group_minimum + 1U) * base;
        ++groups;
    }
    for (unsigned group = 2U; group < groups; ++group) power *= base;

    std::uint32_t remaining = value;
    for (unsigned group = groups; group > 1U; --group) {
        const std::uint32_t place = power;
        const std::uint32_t quotient = remaining / place;
        if (quotient == 0U) return false;
        bits.write(quotient - 1U, width);
        bits.write_bit(true);
        remaining -= quotient * place;
        power /= base;
    }
    bits.write(remaining, width);
    bits.write_bit(false);
    return true;
}

} // namespace atmos::eac3

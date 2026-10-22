#include "restart_header.hpp"

#include "common/bit_reader.hpp"

namespace atmos::truehd {

std::optional<RestartHeader> parse_restart_header(
    std::span<const std::uint8_t> substream_data) {
    try {
        BitReader bits(substream_data);
        if (!bits.read_bit() || !bits.read_bit()) return std::nullopt;
        RestartHeader result;
        result.sync_word = static_cast<std::uint16_t>(bits.read(14U));
        if (result.sync_word < 0x31EAU || result.sync_word > 0x31ECU) return std::nullopt;
        result.noise_type = result.sync_word == 0x31EBU;
        result.object_substream = result.sync_word == 0x31ECU;
        result.output_timestamp = static_cast<std::uint16_t>(bits.read(16U));
        result.min_channel = static_cast<std::uint8_t>(bits.read(4U));
        result.max_channel = static_cast<std::uint8_t>(bits.read(4U));
        result.max_matrix_channel = static_cast<std::uint8_t>(bits.read(4U));
        if (result.max_channel < result.min_channel || result.max_channel >= 16U ||
            result.max_matrix_channel >= 16U) return std::nullopt;
        result.noise_shift = static_cast<std::uint8_t>(bits.read(4U));
        result.noise_seed = bits.read(23U);
        result.max_shift = static_cast<std::uint8_t>(bits.read(4U));
        result.max_lsbs = static_cast<std::uint8_t>(bits.read(5U));
        result.max_bits = static_cast<std::uint8_t>(bits.read(5U));
        if (bits.read(5U) != result.max_bits) return std::nullopt;
        result.data_check_present = bits.read_bit();
        result.lossless_check = static_cast<std::uint8_t>(bits.read(8U));
        result.unknown = static_cast<std::uint16_t>(bits.read(16U));
        result.channel_assignment.reserve(result.max_matrix_channel + 1U);
        for (unsigned channel = 0; channel <= result.max_matrix_channel; ++channel) {
            result.channel_assignment.push_back(static_cast<std::uint8_t>(bits.read(6U)));
        }
        result.checksum = static_cast<std::uint8_t>(bits.read(8U));
        result.end_bit = bits.position();
        return result;
    } catch (const ParseError&) {
        return std::nullopt;
    }
}

} // namespace atmos::truehd

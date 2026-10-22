#include "major_sync.hpp"

#include "common/bit_reader.hpp"

#include <array>

namespace atmos::truehd {

std::optional<MajorSync> parse_major_sync(
    std::span<const std::uint8_t> data, std::size_t sync_offset) {
    if (sync_offset + 28U > data.size()) {
        return std::nullopt;
    }
    std::size_t header_size = 28U;
    if ((data[sync_offset + 25U] & 1U) != 0U) {
        header_size += 2U + static_cast<std::size_t>(data[sync_offset + 26U] >> 4U) * 2U;
    }
    if (sync_offset + header_size > data.size()) return std::nullopt;
    try {
        BitReader bits(data.subspan(sync_offset, header_size - 2U));
        if (bits.read(32U) != 0xF8726FBAU) {
            return std::nullopt;
        }
        MajorSync result;
        result.header_size = static_cast<std::uint16_t>(header_size);
        const std::uint8_t rate_code = static_cast<std::uint8_t>(bits.read(4U));
        static constexpr std::array<std::uint32_t, 3> rate_48 = {
            48000U, 96000U, 192000U
        };
        static constexpr std::array<std::uint32_t, 3> rate_44 = {
            44100U, 88200U, 176400U
        };
        const unsigned shift = rate_code & 7U;
        if (shift > 2U) {
            return std::nullopt;
        }
        result.sample_rate = (rate_code & 8U) != 0U ? rate_44[shift] : rate_48[shift];
        result.samples_per_access_unit = static_cast<std::uint16_t>(40U << shift);
        bits.skip(4U);
        bits.skip(4U);
        const std::uint16_t assignment_6 = static_cast<std::uint16_t>(bits.read(5U));
        bits.skip(2U);
        const std::uint16_t assignment_8 = static_cast<std::uint16_t>(bits.read(13U));
        result.channel_assignment = assignment_8 != 0U ? assignment_8 : assignment_6;
        result.channels = truehd_channels(result.channel_assignment);
        if (bits.read(16U) != 0xB752U) {
            return std::nullopt;
        }
        result.flags = static_cast<std::uint16_t>(bits.read(16U));
        bits.skip(16U);
        result.variable_bitrate = bits.read_bit();
        const std::uint32_t peak_rate_code = bits.read(15U);
        result.peak_bitrate = static_cast<std::uint32_t>(
            (static_cast<std::uint64_t>(peak_rate_code) * result.sample_rate + 8U) >> 4U);
        result.substream_count = static_cast<std::uint8_t>(bits.read(4U));
        bits.skip(2U);
        result.extended_substream_info = static_cast<std::uint8_t>(bits.read(2U));
        result.substream_info = static_cast<std::uint8_t>(bits.read(8U));
        bits.skip(17U);
        result.dialogue_normalization[0] = static_cast<std::uint8_t>(bits.read(6U));
        bits.skip(6U);
        result.dialogue_normalization[1] = static_cast<std::uint8_t>(bits.read(5U));
        bits.skip(11U);
        result.dialogue_normalization[2] = static_cast<std::uint8_t>(bits.read(5U));
        bits.skip(13U);
        if (bits.read_bit()) {
            const std::size_t extra_bits = (static_cast<std::size_t>(bits.read(4U)) + 1U) * 16U - 4U;
            const std::size_t extra_start = bits.position();
            if ((result.substream_info & 0x80U) != 0U) {
                if (extra_bits < 17U) return std::nullopt;
                result.dialogue_normalization[3] = static_cast<std::uint8_t>(bits.read(5U));
                bits.skip(6U);
                result.tracks_in_16ch = static_cast<std::uint8_t>(bits.read(5U) + 1U);
                result.objects_only = bits.read_bit();
                if (!result.objects_only) {
                    const auto description = bits.read(4U);
                    if (description == 1U) {
                        const bool distributed = bits.read_bit();
                        bits.skip(1U);
                        const bool lfe_only = bits.read_bit();
                        if (lfe_only) result.channel_based_bed_ids = {3};
                        else {
                            const bool standard_assignment = bits.read_bit();
                            const auto assignment = bits.read(standard_assignment ? 10U : 17U);
                            constexpr std::array<std::array<std::uint8_t, 2>, 10> groups{{
                                {{0,1}}, {{2,255}}, {{3,255}}, {{4,5}}, {{6,7}},
                                {{8,9}}, {{10,11}}, {{12,13}}, {{14,15}}, {{16,255}}
                            }};
                            if (standard_assignment) {
                                for (unsigned group = 0; group < groups.size(); ++group)
                                    if (assignment & (1U << group))
                                        for (const auto id : groups[group])
                                            if (id != 255) result.channel_based_bed_ids.push_back(id);
                            } else {
                                for (unsigned id = 0; id < 17U; ++id)
                                    if (assignment & (1U << id))
                                        result.channel_based_bed_ids.push_back(static_cast<std::uint8_t>(id));
                            }
                        }
                        if (distributed || result.channel_based_bed_ids.size() != result.tracks_in_16ch)
                            result.channel_based_bed_ids.clear();
                    }
                }
                if (bits.position() > extra_start + extra_bits) return std::nullopt;
            }
            if (bits.position() < extra_start + extra_bits) {
                bits.skip(extra_start + extra_bits - bits.position());
            }
        }
        result.atmosphere_hint = result.tracks_in_16ch != 0U || result.objects_only;
        return result;
    } catch (const ParseError&) {
        return std::nullopt;
    }
}

} // namespace atmos::truehd

#include "bitstream_info.hpp"

#include "common/bit_reader.hpp"

#include <algorithm>
#include <array>

namespace atmos::eac3 {
namespace {

void parse_volume(BitReader& bits, BitstreamInfo& info) {
    const unsigned programs = info.audio_coding_mode == 0U ? 2U : 1U;
    for (unsigned program = 0; program < programs; ++program) {
        info.dialog_normalization_bits[program] = bits.position();
        ++info.dialog_normalization_count;
        info.dialog_normalization[program] =
            static_cast<std::int8_t>(-static_cast<int>(bits.read(5U)));
        if (bits.read_bit()) {
            info.compression_word[program] = static_cast<std::uint8_t>(bits.read(8U));
        }
    }
}

void parse_mixing_metadata(BitReader& bits, BitstreamInfo& info) {
    info.mixing_metadata_present = bits.read_bit();
    if (!info.mixing_metadata_present) return;

    if (info.audio_coding_mode > 2U) {
        info.preferred_downmix = static_cast<std::uint8_t>(bits.read(2U));
        if ((info.audio_coding_mode & 1U) != 0U) {
            info.center_mix_level_ltrt = static_cast<std::uint8_t>(bits.read(3U));
            info.center_mix_level = static_cast<std::uint8_t>(bits.read(3U));
        }
        if ((info.audio_coding_mode & 4U) != 0U) {
            info.surround_mix_level_ltrt = static_cast<std::uint8_t>(bits.read(3U));
            info.surround_mix_level = static_cast<std::uint8_t>(bits.read(3U));
        }
    }
    if (info.lfe && (info.lfe_mix_level_present = bits.read_bit())) {
        info.lfe_mix_level = static_cast<std::uint8_t>(bits.read(5U));
    }

    if (info.stream_type != 0U) return;
    const unsigned programs = info.audio_coding_mode == 0U ? 2U : 1U;
    for (unsigned program = 0; program < programs; ++program) {
        if (bits.read_bit()) bits.skip(6U);
    }
    if (bits.read_bit()) bits.skip(6U);
    switch (bits.read(2U)) {
    case 1U: bits.skip(5U); break;
    case 2U: bits.skip(12U); break;
    case 3U: bits.skip((static_cast<std::size_t>(bits.read(5U)) + 2U) * 8U); break;
    default: break;
    }
    if (info.audio_coding_mode < 2U) {
        for (unsigned program = 0; program < programs; ++program) {
            if (bits.read_bit()) bits.skip(14U);
        }
    }
    if (bits.read_bit()) {
        for (unsigned block = 0; block < info.blocks; ++block) {
            if (info.blocks == 1U || bits.read_bit()) bits.skip(5U);
        }
    }
}

void parse_informational_metadata(BitReader& bits, BitstreamInfo& info,
    std::uint8_t sample_rate_code) {
    info.informational_metadata_present = bits.read_bit();
    if (!info.informational_metadata_present) return;
    info.bitstream_mode = static_cast<std::uint8_t>(bits.read(3U));
    bits.skip(2U);
    if (info.audio_coding_mode == 2U) bits.skip(4U);
    if (info.audio_coding_mode >= 6U) bits.skip(2U);
    const unsigned programs = info.audio_coding_mode == 0U ? 2U : 1U;
    for (unsigned program = 0; program < programs; ++program) {
        if (bits.read_bit()) bits.skip(8U);
    }
    if (sample_rate_code != 3U) bits.skip(1U);
}

void parse_additional_bsi(BitReader& bits, BitstreamInfo& info) {
    if (!bits.read_bit()) return;
    const unsigned byte_count = bits.read(6U) + 1U;
    for (unsigned byte = 0; byte < byte_count; ++byte) {
        if (byte == 0U) {
            bits.skip(7U);
            info.extension_type_a = bits.read_bit();
            if (info.extension_type_a) {
                if (++byte >= byte_count) {
                    throw ParseError("missing E-AC-3 complexity index");
                }
                info.complexity_index_type_a = static_cast<std::uint8_t>(bits.read(8U));
            }
        } else {
            bits.skip(8U);
        }
    }
}

} // namespace

std::optional<BitstreamInfo> parse_bitstream_info(
    std::span<const std::uint8_t> data, std::size_t offset) {
    if (offset + 7U > data.size()) return std::nullopt;
    try {
        BitReader bits(data.subspan(offset));
        if (bits.read(16U) != 0x0B77U) return std::nullopt;

        BitstreamInfo info;
        info.offset = offset;
        info.stream_type = static_cast<std::uint8_t>(bits.read(2U));
        if (info.stream_type == 3U) return std::nullopt;
        info.substream_id = static_cast<std::uint8_t>(bits.read(3U));
        info.frame_size = (static_cast<std::size_t>(bits.read(11U)) + 1U) * 2U;
        if (info.frame_size < 8U || offset + info.frame_size > data.size()) {
            return std::nullopt;
        }

        static constexpr std::array<std::uint32_t, 3> full_rates = {
            48000U, 44100U, 32000U
        };
        static constexpr std::array<std::uint32_t, 3> half_rates = {
            24000U, 22050U, 16000U
        };
        static constexpr std::array<std::uint8_t, 4> block_counts = {
            1U, 2U, 3U, 6U
        };
        const std::uint8_t sample_rate_code = static_cast<std::uint8_t>(bits.read(2U));
        if (sample_rate_code == 3U) {
            const std::uint8_t reduced_code = static_cast<std::uint8_t>(bits.read(2U));
            if (reduced_code == 3U) return std::nullopt;
            info.sample_rate = half_rates[reduced_code];
            info.blocks = 6U;
        } else {
            info.sample_rate = full_rates[sample_rate_code];
            info.blocks = block_counts[bits.read(2U)];
        }
        info.audio_coding_mode = static_cast<std::uint8_t>(bits.read(3U));
        info.lfe = bits.read_bit();
        info.bitstream_id = static_cast<std::uint8_t>(bits.read(5U));
        if (info.bitstream_id < 11U || info.bitstream_id > 16U) return std::nullopt;
        static constexpr std::array<std::uint8_t, 8> channel_counts = {
            2U, 1U, 2U, 3U, 3U, 4U, 4U, 5U
        };
        info.channel_count = static_cast<std::uint8_t>(
            channel_counts[info.audio_coding_mode] + (info.lfe ? 1U : 0U));
        info.samples = static_cast<std::uint16_t>(info.blocks * 256U);

        parse_volume(bits, info);
        if (info.stream_type == 1U) {
            info.channel_map_present = bits.read_bit();
            if (info.channel_map_present) info.channel_map =
                static_cast<std::uint16_t>(bits.read(16U));
        }
        parse_mixing_metadata(bits, info);
        info.informational_start_bit = bits.position();
        parse_informational_metadata(bits, info, sample_rate_code);
        info.informational_end_bit = bits.position();
        if (info.stream_type == 0U && info.blocks != 6U) bits.skip(1U);
        if (info.stream_type == 2U && (info.blocks == 6U || bits.read_bit())) {
            bits.skip(6U);
        }
        info.additional_bsi_start_bit = bits.position();
        parse_additional_bsi(bits, info);
        info.additional_bsi_end_bit = bits.position();
        // The audio frame header itself starts here; `audio_frame_start_bit` is
        // advanced past it by parse_audio_frame_syntax.
        info.audio_frame_header_bit = bits.position();
        info.audio_frame_start_bit = bits.position();
        if (info.audio_frame_start_bit >= info.frame_size * 8U) return std::nullopt;
        return info;
    } catch (const ParseError&) {
        return std::nullopt;
    }
}

} // namespace atmos::eac3

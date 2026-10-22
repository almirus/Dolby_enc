#include "audio_frame.hpp"

#include "common/bit_reader.hpp"

#include <array>

namespace atmos::eac3 {
namespace {

using StrategyRow = std::array<ExponentStrategy, 6>;
constexpr ExponentStrategy R = ExponentStrategy::Reuse;
constexpr ExponentStrategy A = ExponentStrategy::D15;
constexpr ExponentStrategy B = ExponentStrategy::D25;
constexpr ExponentStrategy C = ExponentStrategy::D45;

// ETSI TS 102 366, table E2.14.
constexpr std::array<StrategyRow, 32> frame_exponent_strategies{{
    {A,R,R,R,R,R}, {A,R,R,R,R,C}, {A,R,R,R,B,R}, {A,R,R,R,C,C},
    {B,R,R,B,R,R}, {B,R,R,B,R,C}, {B,R,R,C,B,R}, {B,R,R,C,C,C},
    {B,R,A,R,R,R}, {B,R,B,R,R,C}, {B,R,B,R,B,R}, {B,R,B,R,C,C},
    {B,R,C,B,R,R}, {B,R,C,B,R,C}, {B,R,C,C,B,R}, {B,R,C,C,C,C},
    {C,A,R,R,R,R}, {C,A,R,R,R,C}, {C,B,R,R,B,R}, {C,B,R,R,C,C},
    {C,B,R,B,R,R}, {C,B,R,B,R,C}, {C,B,R,C,B,R}, {C,B,R,C,C,C},
    {C,C,A,R,R,R}, {C,C,B,R,R,C}, {C,C,B,R,B,R}, {C,C,B,R,C,C},
    {C,C,C,B,R,R}, {C,C,C,B,R,C}, {C,C,C,C,B,R}, {C,C,C,C,C,C}
}};

unsigned floor_log2(std::size_t value) {
    unsigned result = 0;
    while (value > 1U) {
        value >>= 1U;
        ++result;
    }
    return result;
}

} // namespace

std::optional<AudioFrameSyntax> parse_audio_frame_syntax(
    std::span<const std::uint8_t> frame, const BitstreamInfo& info) {
    if (frame.size() < info.frame_size || info.blocks == 0U ||
        info.blocks > AudioFrameSyntax::max_blocks) {
        return std::nullopt;
    }
    try {
        BitReader bits(frame.first(info.frame_size));
        bits.skip(info.audio_frame_start_bit);
        AudioFrameSyntax syntax;
        static constexpr std::array<std::uint8_t, 8> fbw_counts = {
            2U, 1U, 2U, 3U, 3U, 4U, 4U, 5U
        };
        syntax.full_bandwidth_channels = fbw_counts[info.audio_coding_mode];
        syntax.lfe_channel = info.lfe ?
            static_cast<std::uint8_t>(syntax.full_bandwidth_channels + 1U) : 0U;

        if (info.blocks == 6U) {
            syntax.ac3_exponent_strategy = bits.read_bit();
            syntax.parse_aht = bits.read_bit();
        } else {
            syntax.ac3_exponent_strategy = true;
        }
        syntax.snr_offset_strategy = static_cast<std::uint8_t>(bits.read(2U));
        syntax.transient_processing = bits.read_bit();
        syntax.block_switch_syntax = bits.read_bit();
        syntax.dither_flag_syntax = bits.read_bit();
        syntax.bit_allocation_syntax = bits.read_bit();
        syntax.fast_gain_syntax = bits.read_bit();
        syntax.delta_bit_allocation_syntax = bits.read_bit();
        syntax.skip_syntax_bit = bits.position();
        syntax.skip_syntax = bits.read_bit();
        // This one flag is both "spectral extension off" and "no SPX attenuation
        // data follows"; it is transmitted once.
        const bool parse_spx_attenuation = bits.read_bit();

        unsigned coupling_blocks = 0;
        if (info.audio_coding_mode > 1U) {
            bool in_use = false;
            for (unsigned block = 0; block < info.blocks; ++block) {
                const bool present = block == 0U || bits.read_bit();
                syntax.coupling_strategy_present[block] = present;
                if (present) in_use = bits.read_bit();
                syntax.coupling[block][0] = in_use;
                coupling_blocks += in_use ? 1U : 0U;
            }
        }

        if (syntax.ac3_exponent_strategy) {
            for (unsigned block = 0; block < info.blocks; ++block) {
                const unsigned first = syntax.coupling[block][0] ? 0U : 1U;
                for (unsigned channel = first;
                    channel <= syntax.full_bandwidth_channels; ++channel) {
                    syntax.exponent_strategy[block][channel] =
                        static_cast<ExponentStrategy>(bits.read(2U));
                }
            }
        } else {
            const unsigned first = info.audio_coding_mode > 1U && coupling_blocks != 0U ? 0U : 1U;
            for (unsigned channel = first;
                channel <= syntax.full_bandwidth_channels; ++channel) {
                const auto& row = frame_exponent_strategies[bits.read(5U)];
                for (unsigned block = 0; block < info.blocks; ++block) {
                    syntax.exponent_strategy[block][channel] = row[block];
                }
            }
        }
        if (info.lfe) {
            for (unsigned block = 0; block < info.blocks; ++block) {
                syntax.exponent_strategy[block][syntax.lfe_channel] = bits.read_bit() ? A : R;
            }
        }
        if (info.stream_type == 0U && (info.blocks == 6U || bits.read_bit())) {
            bits.skip(5U * syntax.full_bandwidth_channels);
        }

        const unsigned channels = syntax.full_bandwidth_channels + (info.lfe ? 1U : 0U);
        if (syntax.parse_aht) {
            const unsigned first = coupling_blocks == 6U ? 0U : 1U;
            for (unsigned channel = first; channel <= channels; ++channel) {
                bool eligible = true;
                for (unsigned block = 1; block < 6U; ++block) {
                    if (syntax.exponent_strategy[block][channel] != R ||
                        (channel == 0U && syntax.coupling_strategy_present[block])) {
                        eligible = false;
                    }
                }
                syntax.adaptive_hybrid_transform[channel] = eligible && bits.read_bit();
            }
        }
        if (syntax.snr_offset_strategy == 0U) {
            const int coarse = (static_cast<int>(bits.read(6U)) - 15) * 16;
            const auto offset = static_cast<std::int16_t>(
                (coarse + static_cast<int>(bits.read(4U))) * 4);
            syntax.snr_offset.fill(offset);
        }
        if (syntax.transient_processing) {
            for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels; ++channel) {
                if (bits.read_bit()) bits.skip(18U);
            }
        }
        syntax.spectral_extension_attenuation.fill(-1);
        for (unsigned channel = 1; channel <= syntax.full_bandwidth_channels; ++channel) {
            if (parse_spx_attenuation && bits.read_bit()) {
                syntax.spectral_extension_attenuation[channel] =
                    static_cast<std::int8_t>(bits.read(5U));
            }
        }
        if (info.blocks > 1U && bits.read_bit()) {
            const std::size_t words = info.frame_size / 2U;
            bits.skip((info.blocks - 1U) * (4U + floor_log2(words - 1U)));
        }
        syntax.audio_blocks_start_bit = bits.position();
        return syntax;
    } catch (const ParseError&) {
        return std::nullopt;
    }
}

} // namespace atmos::eac3

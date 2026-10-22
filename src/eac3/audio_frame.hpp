#pragma once

#include "bitstream_info.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace atmos::eac3 {

enum class ExponentStrategy : std::uint8_t {
    Reuse = 0,
    D15 = 1,
    D25 = 2,
    D45 = 3
};

struct AudioFrameSyntax final {
    static constexpr std::size_t max_blocks = 6U;
    static constexpr std::size_t max_channels = 7U;

    std::array<std::array<ExponentStrategy, max_channels>, max_blocks>
        exponent_strategy{};
    std::array<std::array<bool, max_channels>, max_blocks> coupling{};
    std::array<bool, max_blocks> coupling_strategy_present{};
    std::array<bool, max_channels> adaptive_hybrid_transform{};
    std::array<std::int8_t, max_channels> spectral_extension_attenuation{};
    std::array<std::int16_t, max_channels> snr_offset{};
    std::size_t audio_blocks_start_bit = 0;
    std::size_t skip_syntax_bit = 0;
    std::uint8_t full_bandwidth_channels = 0;
    std::uint8_t lfe_channel = 0;
    std::uint8_t snr_offset_strategy = 0;
    bool ac3_exponent_strategy = false;
    bool parse_aht = false;
    bool transient_processing = false;
    bool block_switch_syntax = false;
    bool dither_flag_syntax = false;
    bool bit_allocation_syntax = false;
    bool fast_gain_syntax = false;
    bool delta_bit_allocation_syntax = false;
    bool skip_syntax = false;
};

[[nodiscard]] std::optional<AudioFrameSyntax> parse_audio_frame_syntax(
    std::span<const std::uint8_t> frame, const BitstreamInfo& info);

} // namespace atmos::eac3

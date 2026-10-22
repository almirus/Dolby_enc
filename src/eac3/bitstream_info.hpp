#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace atmos::eac3 {

struct BitstreamInfo final {
    std::size_t offset = 0;
    std::size_t audio_frame_header_bit = 0;
    std::size_t frame_size = 0;
    std::size_t audio_frame_start_bit = 0;
    std::size_t informational_start_bit = 0;
    std::size_t informational_end_bit = 0;
    std::size_t additional_bsi_start_bit = 0;
    std::size_t additional_bsi_end_bit = 0;
    std::uint32_t sample_rate = 0;
    std::uint16_t samples = 0;
    std::uint16_t channel_map = 0;
    std::array<std::int8_t, 2> dialog_normalization{};
    std::array<std::size_t, 2> dialog_normalization_bits{};
    std::uint8_t dialog_normalization_count = 0;
    std::array<std::uint8_t, 2> compression_word{};
    std::uint8_t stream_type = 0;
    std::uint8_t substream_id = 0;
    std::uint8_t blocks = 0;
    std::uint8_t audio_coding_mode = 0;
    std::uint8_t channel_count = 0;
    std::uint8_t bitstream_id = 0;
    std::uint8_t bitstream_mode = 0;
    std::uint8_t preferred_downmix = 0;
    std::uint8_t center_mix_level_ltrt = 0;
    std::uint8_t center_mix_level = 0;
    std::uint8_t surround_mix_level_ltrt = 0;
    std::uint8_t surround_mix_level = 0;
    std::uint8_t lfe_mix_level = 0;
    std::uint8_t complexity_index_type_a = 0;
    bool lfe = false;
    bool channel_map_present = false;
    bool mixing_metadata_present = false;
    bool informational_metadata_present = false;
    bool lfe_mix_level_present = false;
    bool extension_type_a = false;
};

[[nodiscard]] std::optional<BitstreamInfo> parse_bitstream_info(
    std::span<const std::uint8_t> data, std::size_t offset);

} // namespace atmos::eac3

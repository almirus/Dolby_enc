#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace atmos::eac3 {

struct SyncFrame final {
    std::size_t offset = 0;
    std::size_t size = 0;
    std::uint32_t sample_rate = 0;
    std::uint16_t samples = 0;
    std::uint8_t stream_type = 0;
    std::uint8_t substream_id = 0;
    std::uint8_t blocks = 0;
    std::uint8_t audio_coding_mode = 0;
    std::uint8_t channel_count = 0;
    std::uint8_t bitstream_id = 0;
    bool lfe = false;
    bool emdf_sync = false;
    bool emdf_valid = false;
    bool joc_payload = false;
    bool oamd_payload = false;
    std::uint8_t joc_channels = 0;
    std::uint8_t joc_objects = 0;
    std::uint8_t joc_active_objects = 0;
    std::uint8_t joc_sparse_objects = 0;
    std::uint8_t joc_fine_objects = 0;
    std::array<std::uint8_t, 64> joc_object_flags{};
    std::uint16_t oamd_objects = 0;
    std::uint8_t oamd_bed_channels = 0;
    std::uint16_t oamd_position_updates = 0;
    bool oamd_first_position_valid = false;
    float oamd_first_position_x = 0.0F;
    float oamd_first_position_y = 0.0F;
    float oamd_first_position_z = 0.0F;
    std::string emdf_error;
    bool extension_type_a = false;
    std::uint8_t complexity_index_type_a = 0;
    std::size_t audio_frame_start_bit = 0;
    std::size_t audio_blocks_start_bit = 0;
    bool audio_frame_syntax_valid = false;
    bool adaptive_hybrid_transform_syntax = false;
    std::uint8_t coupling_blocks = 0;
    std::uint8_t spectral_blocks_decoded = 0;
    bool spectral_extension_used = false;
    bool enhanced_coupling_used = false;
    std::size_t nonzero_spectral_coefficients = 0;
    std::string spectral_error;
    std::uint8_t snr_offset_strategy = 0;
    std::array<std::uint8_t, 7> first_block_exponent_strategy{};
    bool block_switch_syntax = false;
    bool dither_flag_syntax = false;
    bool bit_allocation_syntax = false;
    bool fast_gain_syntax = false;
    bool delta_bit_allocation_syntax = false;
    bool skip_syntax = false;
};

[[nodiscard]] std::optional<SyncFrame> parse_sync_frame(
    std::span<const std::uint8_t> data, std::size_t offset);
[[nodiscard]] bool contains_emdf_sync(std::span<const std::uint8_t> frame);

} // namespace atmos::eac3

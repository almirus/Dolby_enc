#pragma once

#include "dolby/channel_layout.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace atmos::truehd {

struct MajorSync final {
    std::uint16_t header_size = 0;
    std::uint16_t flags = 0;
    std::uint32_t sample_rate = 0;
    std::uint32_t peak_bitrate = 0;
    std::uint16_t samples_per_access_unit = 0;
    std::uint16_t channel_assignment = 0;
    std::uint8_t substream_count = 0;
    std::uint8_t substream_info = 0;
    std::uint8_t extended_substream_info = 0;
    std::uint8_t tracks_in_16ch = 0;
    std::array<std::optional<std::uint8_t>, 4> dialogue_normalization; // 2ch, 6ch, 8ch, 16ch
    bool objects_only = false;
    std::vector<std::uint8_t> channel_based_bed_ids; // Fixed 16ch presentation, in decoded signal order.
    bool variable_bitrate = false;
    bool atmosphere_hint = false;
    std::vector<ChannelId> channels;
};

[[nodiscard]] std::optional<MajorSync> parse_major_sync(
    std::span<const std::uint8_t> data, std::size_t sync_offset);

} // namespace atmos::truehd

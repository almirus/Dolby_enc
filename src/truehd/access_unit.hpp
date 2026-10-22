#pragma once

#include "major_sync.hpp"
#include "restart_header.hpp"
#include "eac3/emdf.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace atmos::truehd {

struct SubstreamEntry final {
    std::size_t data_offset = 0;
    std::size_t data_size = 0;
    bool extra_word_present = false;
    bool nonrestart = false;
    bool check_data_present = false;
    std::uint16_t extra_word = 0;
    std::optional<RestartHeader> restart_header;
};

struct AccessUnit final {
    std::size_t offset = 0;
    std::size_t size = 0;
    std::uint16_t input_timing = 0;
    bool has_major_sync = false;
    std::optional<MajorSync> major_sync;
    std::vector<SubstreamEntry> substreams;
    std::size_t extra_data_offset = 0;
    std::size_t extra_data_size = 0;
    std::vector<eac3::EmdfMetadata> atmos_metadata;
};

[[nodiscard]] std::optional<AccessUnit> parse_access_unit(
    std::span<const std::uint8_t> data, std::size_t offset,
    const MajorSync* stream_info = nullptr);

// All OAMD payloads for presentation 3; malformed EVO returns one valid=false record.
[[nodiscard]] std::vector<eac3::EmdfMetadata> parse_evolution_frame(
    std::span<const std::uint8_t> data);

} // namespace atmos::truehd

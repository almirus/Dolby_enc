#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace atmos::truehd {

struct RestartHeader final {
    std::uint16_t sync_word = 0;
    std::uint16_t output_timestamp = 0;
    std::uint8_t min_channel = 0;
    std::uint8_t max_channel = 0;
    std::uint8_t max_matrix_channel = 0;
    std::uint8_t noise_shift = 0;
    std::uint32_t noise_seed = 0;
    std::uint8_t max_shift = 0;
    std::uint8_t max_lsbs = 0;
    std::uint8_t max_bits = 0;
    std::uint8_t lossless_check = 0;
    std::uint16_t unknown = 0;
    bool noise_type = false;
    bool object_substream = false;
    bool data_check_present = false;
    std::vector<std::uint8_t> channel_assignment;
    std::uint8_t checksum = 0;
    std::size_t end_bit = 0;
};

[[nodiscard]] std::optional<RestartHeader> parse_restart_header(
    std::span<const std::uint8_t> substream_data);

} // namespace atmos::truehd

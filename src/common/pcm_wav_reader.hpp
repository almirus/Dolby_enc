#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string_view>
#include <vector>

namespace atmos {

// PCM is returned as signed 24-bit integers in ascending Dolby speaker-ID order.
// No-mask layouts use L,R,C,LFE,Ls,Rs,Lb,Rb followed by height pairs, as applicable.
class PcmWavReader final {
public:
    explicit PcmWavReader(const std::filesystem::path& path, std::string_view layout = {});
    [[nodiscard]] unsigned sample_rate() const noexcept { return sample_rate_; }
    [[nodiscard]] unsigned bit_depth() const noexcept { return valid_bits_; }
    [[nodiscard]] std::uint64_t frames() const noexcept { return frames_; }
    [[nodiscard]] std::span<const std::uint8_t> channel_ids() const noexcept { return ids_; }
    // max_frames must be nonzero. An empty result means EOF; storage is reused.
    [[nodiscard]] std::span<const std::int32_t> read(std::size_t max_frames);

private:
    std::ifstream file_;
    unsigned sample_rate_ = 0, valid_bits_ = 0, container_bits_ = 0, channels_ = 0;
    std::uint64_t frames_ = 0, remaining_ = 0;
    std::vector<std::uint8_t> ids_, raw_;
    std::vector<unsigned> order_;
    std::vector<std::int32_t> pcm_;
};

} // namespace atmos

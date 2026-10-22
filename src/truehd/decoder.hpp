#pragma once

#include "major_sync.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace atmos::truehd {

struct PcmFrame final {
    std::vector<float> interleaved;
    std::uint32_t sample_rate = 0;
    std::uint8_t channels = 0;
    std::uint16_t samples_per_channel = 0;
};

class BedDecoder final {
public:
    struct State;
    explicit BedDecoder(unsigned presentation = 2U);
    ~BedDecoder();
    BedDecoder(const BedDecoder&) = delete;
    BedDecoder& operator=(const BedDecoder&) = delete;
    void reset() noexcept;
    [[nodiscard]] bool decode(std::span<const std::uint8_t> access_unit,
        PcmFrame& output, std::string& error);
    [[nodiscard]] bool decode_with_presentation2(
        std::span<const std::uint8_t> access_unit, PcmFrame& output,
        PcmFrame& presentation2, std::string& error);

private:
    [[nodiscard]] bool decode_internal(std::span<const std::uint8_t> access_unit,
        PcmFrame& output, PcmFrame* presentation2, std::string& error);
    std::array<State*, 4> states_{};
    std::optional<MajorSync> stream_info_;
    unsigned presentation_ = 2U;
};

} // namespace atmos::truehd

#pragma once

#include "audio_block.hpp"
#include "emdf.hpp"
#include "imdct.hpp"
#include "mantissa.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace atmos::eac3 {

struct PcmFrame final {
    std::vector<float> interleaved;
    std::uint32_t sample_rate = 0;
    std::uint8_t channels = 0;
    std::uint16_t samples_per_channel = 0;
    std::uint16_t short_transforms = 0;
    EmdfMetadata metadata;
};

class BedDecoder final {
public:
    void reset() noexcept;
    [[nodiscard]] bool decode(std::span<const std::uint8_t> frame,
        PcmFrame& output, std::string& error);

private:
    std::array<Imdct256, 7> transforms_{};
    DitherGenerator dither_{};
    EcplDecodeState ecpl_state_{};
};

} // namespace atmos::eac3

#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace atmos::truehd {

// Experimental CBI writer with static OAMD. Input is signed PCM24 in ascending Dolby speaker-ID
// order, 48 kHz. Packets contain 40 samples; the final packet carries an EOS trim.
class Encoder final {
public:
    explicit Encoder(std::span<const std::uint8_t> speaker_ids);
    [[nodiscard]] std::vector<std::uint8_t> encode(
        std::span<const std::int32_t> interleaved, bool final);

private:
    std::vector<std::uint8_t> ids_;
    std::uint16_t assignment_ = 0;
    std::uint64_t units_ = 0;
    std::array<std::uint8_t,4> lossless_{};
    std::vector<std::uint8_t> evolution_;
    std::size_t protection_bit_ = 0;
    bool finished_ = false;
};

} // namespace atmos::truehd

#pragma once

#include "emdf.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace atmos::eac3 {

class JocDecoder final {
public:
    JocDecoder();
    ~JocDecoder();
    JocDecoder(const JocDecoder&) = delete;
    JocDecoder& operator=(const JocDecoder&) = delete;

    void reset() noexcept;
    // One 1536-sample frame; full-bandwidth objects have 1217 samples of filter delay.
    [[nodiscard]] bool render(const JocFrame& joc, std::span<const float> bed,
        std::uint8_t bed_channels, std::uint16_t samples_per_channel,
        std::vector<float>& objects, std::string& error);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace atmos::eac3

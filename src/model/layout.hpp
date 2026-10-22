#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace atmos::model {

// A bed layout in the classic AC-3 channel order the core encoder consumes
// (L C R Ls Rs [Lrs Rrs] [LFE]).
struct BedLayout final {
    std::string name;
    std::vector<std::string> channels;
    std::uint8_t audio_coding_mode = 7; // 3/2
    bool lfe = true;
};

[[nodiscard]] bool parse_bed_layout(const std::string& name, BedLayout& layout);
[[nodiscard]] std::string supported_bed_layouts();

} // namespace atmos::model

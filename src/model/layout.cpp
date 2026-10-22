#include "model/layout.hpp"

#include <array>

namespace atmos::model {

bool parse_bed_layout(const std::string& name, BedLayout& layout) {
    // Encoder (AC-3) channel order for a 3/2+LFE bed.
    static const std::array<std::string, 6> channels = {"L", "C", "R", "Ls", "Rs", "LFE"};
    if (name != "5.1" && name != "5.1.0") return false;
    layout.name = "5.1";
    layout.channels.assign(channels.begin(), channels.end());
    layout.audio_coding_mode = 7;
    layout.lfe = true;
    return true;
}

std::string supported_bed_layouts() { return "5.1"; }

} // namespace atmos::model

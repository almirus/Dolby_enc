#include "channel_layout.hpp"

#include <array>

namespace atmos {

std::string channel_name(ChannelId channel) {
    switch (channel) {
    case ChannelId::FrontLeft: return "FL";
    case ChannelId::FrontRight: return "FR";
    case ChannelId::FrontCenter: return "C";
    case ChannelId::Lfe: return "LFE";
    case ChannelId::SideLeft: return "SL";
    case ChannelId::SideRight: return "SR";
    case ChannelId::RearLeft: return "RL";
    case ChannelId::RearRight: return "RR";
    case ChannelId::TopFrontLeft: return "TFL";
    case ChannelId::TopFrontRight: return "TFR";
    case ChannelId::FrontLeftCenter: return "FLC";
    case ChannelId::FrontRightCenter: return "FRC";
    case ChannelId::RearCenter: return "RC";
    case ChannelId::TopCenter: return "TC";
    case ChannelId::WideLeft: return "WL";
    case ChannelId::WideRight: return "WR";
    case ChannelId::TopFrontCenter: return "TFC";
    }
    return "unknown";
}

std::vector<ChannelId> truehd_channels(std::uint16_t assignment) {
    static const std::array<std::vector<ChannelId>, 13> groups = {{
        {ChannelId::FrontLeft, ChannelId::FrontRight},
        {ChannelId::FrontCenter},
        {ChannelId::Lfe},
        {ChannelId::SideLeft, ChannelId::SideRight},
        {ChannelId::TopFrontLeft, ChannelId::TopFrontRight},
        {ChannelId::FrontLeftCenter, ChannelId::FrontRightCenter},
        {ChannelId::RearLeft, ChannelId::RearRight},
        {ChannelId::RearCenter},
        {ChannelId::TopCenter},
        {ChannelId::SideLeft, ChannelId::SideRight},
        {ChannelId::WideLeft, ChannelId::WideRight},
        {ChannelId::TopFrontCenter},
        {ChannelId::Lfe},
    }};
    std::vector<ChannelId> result;
    for (unsigned bit = 0; bit < groups.size(); ++bit) {
        if ((assignment & (1U << bit)) != 0U) {
            result.insert(result.end(), groups[bit].begin(), groups[bit].end());
        }
    }
    return result;
}

} // namespace atmos

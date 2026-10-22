#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace atmos {

enum class ChannelId {
    FrontLeft,
    FrontRight,
    FrontCenter,
    Lfe,
    SideLeft,
    SideRight,
    RearLeft,
    RearRight,
    TopFrontLeft,
    TopFrontRight,
    FrontLeftCenter,
    FrontRightCenter,
    RearCenter,
    TopCenter,
    WideLeft,
    WideRight,
    TopFrontCenter,
};

[[nodiscard]] std::string channel_name(ChannelId channel);
[[nodiscard]] std::vector<ChannelId> truehd_channels(std::uint16_t assignment);

} // namespace atmos

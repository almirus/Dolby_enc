#include "model/oamd.hpp"

#include <cmath>

namespace atmos::model {

eac3::OamdFrame make_oamd(const AtmosConfig& config) {
    eac3::OamdFrame oamd;
    // OAMD object count covers the bed's channels plus the dynamic objects; the
    // dynamic objects follow the bed so their render info is transmitted.
    oamd.object_count = static_cast<std::uint16_t>(config.bed_channels +
        config.objects.size());
    oamd.bed_channels = static_cast<std::uint8_t>(config.bed_channels);
    for (std::size_t index = 0; index < config.objects.size(); ++index) {
        const ObjectPlacement& placement = config.objects[index];
        eac3::OamdPositionUpdate update;
        update.object = static_cast<std::uint16_t>(config.bed_channels + index);
        // OAMD positions are x,y in [0,1] and z in [-1,1].
        update.x = (placement.x + 1.0F) * 0.5F;
        update.y = placement.y;
        update.z = placement.z;
        oamd.position_updates.push_back(update);
    }
    return oamd;
}

} // namespace atmos::model

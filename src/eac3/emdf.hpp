#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <span>
#include <string>
#include <vector>

namespace atmos::eac3 {

struct JocObject final {
    bool active = false;
    bool sparse = false;
    bool fine_quantization = false;
    bool steep_slope = false;
    std::uint8_t bands_index = 0;
    std::uint8_t bands = 0;
    std::uint8_t data_points = 0;
    std::array<std::uint8_t, 2> timeslot_offsets{};
    std::array<std::array<std::uint8_t, 23>, 2> sparse_channel{};
    std::array<std::array<std::uint8_t, 23>, 2> sparse_vector{};
    std::array<std::array<std::array<std::uint8_t, 23>, 7>, 2> matrix{};
};

struct JocFrame final {
    std::uint16_t sequence_count = 0;
    std::uint8_t channels = 0;
    std::uint8_t object_count = 0;
    float gain = 1.0F;
    // 0xff selects the encoder's default (0 for five carriers, 2 for seven).
    std::uint8_t downmix_config = 0xff;
    std::array<JocObject, 64> objects{};
};

struct OamdPositionUpdate final {
    // Parsed wire deltas are resolved before rendering; original flags remain in object_records.
    std::uint16_t object = 0;
    std::uint16_t sample_offset = 0;
    std::uint16_t ramp_duration = 0;
    std::uint8_t block = 0;
    bool differential = false;
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    bool distance_specified = false;
    bool object_at_infinity = false;
    float distance = 1.0F;
    bool snap = false;
};

struct OamdGainUpdate final {
    std::uint16_t object = 0;
    std::uint16_t sample_offset = 0;
    std::uint8_t block = 0;
    float gain = 1.0F;
    std::uint16_t ramp_duration = 0;
};

struct OamdSpreadUpdate final {
    std::uint16_t object = 0;
    std::uint16_t sample_offset = 0;
    std::uint16_t ramp_duration = 0;
    std::uint8_t block = 0;
    float size = -1.0F;
    float divergence = -1.0F;
};

struct OamdZoneUpdate final {
    std::uint16_t object = 0;
    std::uint16_t sample_offset = 0;
    std::uint8_t block = 0;
    std::uint8_t horizontal = 0;
    bool elevation = true;
    std::uint16_t ramp_duration = 0;
};

struct OamdTrimConfig final {
    std::uint8_t mode = 0; // 0 auto, 1 disabled, 2 explicit
    std::array<std::uint8_t, 3> levels{16,16,16}; // centre/surround/height: wire0..15,16 absent
    std::array<std::int8_t, 2> balance{}; // overhead/floor and listener: signed sixteenths
    bool operator==(const OamdTrimConfig&) const = default;
};

struct OamdFrame final {
    std::uint16_t object_count = 0;
    std::uint8_t bed_channels = 0;
    std::uint8_t isf_channels = 0; // Intermediate spatial format signals are not dynamic objects.
    std::vector<std::uint8_t> bed_channel_ids;
    std::vector<OamdPositionUpdate> position_updates;
    std::vector<OamdGainUpdate> gain_updates;
    std::vector<OamdSpreadUpdate> spread_updates;
    std::vector<OamdZoneUpdate> zone_updates;
    bool trim_present = false;
    std::uint8_t warp_mode = 1, trim_reserved = 0, global_trim_mode = 0;
    std::array<OamdTrimConfig, 9> trims{};
    std::vector<bool> trim_bypass; // empty when per-object bypass is absent
};

struct OamdBasicInfo final {
    std::uint8_t gain_index = 1; // mute; code3 is resolved to the original explicit code
    std::uint8_t gain_bits = 0;
    std::uint8_t priority = 0; // 0..31 explicit, 32 default
};

struct OamdRenderInfo final {
    std::uint8_t x = 31, y = 31, z = 0;
    bool positive_z = true;
    bool absolute_position = true;
    bool distance_specified = false, infinity = false;
    std::uint8_t distance = 0, zone = 0;
    bool elevation = true;
    std::uint8_t size_index = 0;
    std::array<std::uint8_t, 3> size{};
    bool screen_reference = false;
    std::uint8_t screen_factor = 0, depth_factor = 0;
    bool snap = false;
};

// Bit ranges within the original element body, retained without requantization.
struct OamdObjectRecord final {
    std::size_t begin = 0;
    std::size_t render_begin = 0;
    std::size_t render_end = 0;
    std::size_t end = 0;
    bool inactive = false;
    bool full_basic = false;
    bool full_render = false;
    bool differential = false;
    bool dynamic = false;
    OamdBasicInfo basic;
    OamdRenderInfo render;
};

struct OamdElement final {
    std::uint8_t id = 0;
    std::vector<std::uint8_t> body;
    std::size_t object_header_begin = 0;
    std::size_t object_data_begin = 0;
    std::size_t object_data_end = 0;
    std::vector<std::vector<OamdObjectRecord>> object_records;
};

// Opaque wire fields retained for reframing without requantizing object metadata.
struct OamdPayload final {
    std::uint32_t sample_offset = 0; // outer source EVO offset, relative to its access unit
    std::vector<std::uint8_t> header;
    std::size_t header_bits = 0; // through alternate_object, before element count
    bool alternate_object = false;
    std::vector<OamdElement> elements;
};

struct EmdfMetadata final {
    std::size_t primary_protection_bit = 0; // relative to the EMDF content, excluding sync/size
    std::vector<std::uint8_t> primary_protection;
    std::vector<std::uint8_t> secondary_protection;
    bool valid = false;
    bool has_joc = false;
    bool has_oamd = false;
    std::uint8_t joc_channels = 0;
    std::uint8_t joc_objects = 0;
    std::uint8_t joc_active_objects = 0;
    std::uint8_t joc_sparse_objects = 0;
    std::uint8_t joc_fine_objects = 0;
    float joc_gain = 1.0F;
    JocFrame joc;
    std::uint16_t oamd_objects = 0;
    std::uint8_t oamd_bed_channels = 0;
    std::uint16_t oamd_position_updates = 0;
    float first_position_x = 0.0F;
    float first_position_y = 0.0F;
    float first_position_z = 0.0F;
    bool first_position_valid = false;
    OamdFrame oamd;
    OamdPayload oamd_payload;
    std::string error;
};

[[nodiscard]] EmdfMetadata parse_emdf(std::span<const std::uint8_t> data);
[[nodiscard]] EmdfMetadata parse_emdf_frame(std::span<const std::uint8_t> data);
[[nodiscard]] EmdfMetadata parse_oamd_payload(std::span<const std::uint8_t> data);

} // namespace atmos::eac3

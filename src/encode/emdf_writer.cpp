#include "encode/emdf_writer.hpp"

#include "common/bit_writer.hpp"
#include "eac3/emdf.hpp"
#include "eac3/joc_tables.hpp"
#include "crypto/mat_dec_hmac.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <span>
#include <stdexcept>
#include <vector>

namespace atmos::encode {
namespace {

// Big-endian variable-bit integer, the inverse of the reader in emdf.cpp.
void write_variable(BitWriter& writer, std::uint32_t value, unsigned width) {
    std::vector<std::uint32_t> groups;
    std::uint32_t remaining = value;
    const std::uint32_t mask = (1U << width) - 1U;
    for (;;) {
        groups.push_back(remaining & mask);
        remaining >>= width;
        if (remaining == 0U) break;
        remaining -= 1U;
    }
    for (std::size_t index = groups.size(); index-- > 0U;) {
        writer.write(groups[index], width);
        writer.write_bit(index != 0U);
    }
}

void write_bytes(BitWriter& writer, std::span<const std::uint8_t> bytes) {
    for (const std::uint8_t byte : bytes) writer.write(byte, 8U);
}

std::vector<std::uint8_t> take_aligned(BitWriter& writer) {
    writer.align();
    std::vector<std::uint8_t> bytes = writer.bytes();
    return bytes;
}

// Symbol -> (bit count, code) for one of the JOC Huffman tables.
struct HuffmanEncoder final {
    std::map<int, std::pair<unsigned, std::uint32_t>> codes;

    explicit HuffmanEncoder(std::span<const std::array<int, 2>> table) {
        for (std::size_t leaf = 0; leaf < table.size(); ++leaf) {
            if (table[leaf][0] >= 0 || table[leaf][1] >= 0) continue;
            const int symbol = ~table[leaf][0];
            if (table[leaf][1] != table[leaf][0]) continue;
            codes.emplace(symbol, std::pair<unsigned, std::uint32_t>{0U, 0U});
        }
        // Walk the tree from the root, recording each leaf's path.
        std::vector<std::pair<int, std::pair<unsigned, std::uint32_t>>> stack = {{0, {0U, 0U}}};
        while (!stack.empty()) {
            const auto [node, state] = stack.back();
            stack.pop_back();
            if (node < 0) continue;
            for (unsigned bit = 0; bit < 2U; ++bit) {
                const int child = table[static_cast<std::size_t>(node)][bit];
                const unsigned bits = state.first + 1U;
                const std::uint32_t code = (state.second << 1U) | bit;
                if (child < 0) {
                    codes[~child] = {bits, code};
                } else {
                    stack.push_back({child, {bits, code}});
                }
            }
        }
    }

    void write(BitWriter& writer, int symbol) const {
        const auto found = codes.find(symbol);
        if (found == codes.end()) throw std::runtime_error("invalid JOC Huffman symbol");
        writer.write(found->second.second, found->second.first);
    }
};

const HuffmanEncoder& matrix_table(bool fine) {
    static const HuffmanEncoder coarse(eac3::joc_tables::joc_huff_code_coarse_generic);
    static const HuffmanEncoder precise(eac3::joc_tables::joc_huff_code_fine_generic);
    return fine ? precise : coarse;
}

const HuffmanEncoder& sparse_vector_table(bool fine) {
    static const HuffmanEncoder coarse(eac3::joc_tables::joc_huff_code_coarse_coeff_sparse);
    static const HuffmanEncoder precise(eac3::joc_tables::joc_huff_code_fine_coeff_sparse);
    return fine ? precise : coarse;
}

const HuffmanEncoder& sparse_channel_table(unsigned channels) {
    static const HuffmanEncoder five(eac3::joc_tables::joc_huff_code_5ch_pos_index_sparse);
    static const HuffmanEncoder seven(eac3::joc_tables::joc_huff_code_7ch_pos_index_sparse);
    return channels == 7 ? seven : five;
}

std::vector<std::uint8_t> build_joc(const eac3::JocFrame& joc, std::size_t* bit_count = nullptr) {
    if ((joc.channels != 5 && joc.channels != 7) ||
        joc.object_count == 0 || joc.object_count > 16 || !std::isfinite(joc.gain) ||
        joc.gain < .0625F || joc.gain > 15.75F)
        throw std::runtime_error("unsupported JOC header");
    const unsigned downmix = joc.downmix_config == 0xff ? (joc.channels == 7 ? 2U : 0U) : joc.downmix_config;
    if (downmix > 4 || (downmix == 0 || downmix == 3 ? 5U : 7U) != joc.channels)
        throw std::runtime_error("JOC downmix selector disagrees with channel count");
    int gain_power = 0;
    const float gain_fraction = (std::frexp(joc.gain, &gain_power) * 2 - 1) * 32;
    const auto gain_mantissa = static_cast<unsigned>(gain_fraction);
    if (gain_fraction != static_cast<float>(gain_mantissa))
        throw std::runtime_error("JOC gain is not exactly representable");
    BitWriter writer;
    writer.write(downmix, 3);
    writer.write(joc.object_count - 1U, 6);
    writer.write(0, 3);
    writer.write(static_cast<unsigned>(gain_power + 3), 3);
    writer.write(gain_mantissa, 5);
    if (joc.sequence_count > 1023) throw std::runtime_error("invalid JOC sequence count");
    writer.write(joc.sequence_count, 10);
    for (unsigned object = 0; object < joc.object_count; ++object) {
        const auto& info = joc.objects[object];
        writer.write_bit(info.active);
        if (!info.active) continue;
        if (info.bands_index > 7 || info.data_points < 1 || info.data_points > 2)
            throw std::runtime_error("unsupported JOC object coding");
        writer.write(info.bands_index, 3);
        writer.write_bit(info.sparse);
        writer.write_bit(info.fine_quantization);
        writer.write_bit(info.steep_slope);
        writer.write(info.data_points - 1U, 1);
        if (info.steep_slope)
            for (unsigned point = 0; point < info.data_points; ++point) {
                if (info.timeslot_offsets[point] < 1 || info.timeslot_offsets[point] > 24)
                    throw std::runtime_error("invalid JOC timeslot offset");
                writer.write(info.timeslot_offsets[point] - 1U, 5);
            }
    }
    for (unsigned object = 0; object < joc.object_count; ++object) {
        const auto& info = joc.objects[object];
        if (!info.active) continue;
        for (unsigned point = 0; point < info.data_points; ++point) {
            if (info.sparse) {
                if (info.sparse_channel[point][0] >= joc.channels) throw std::runtime_error("invalid JOC sparse channel");
                writer.write(info.sparse_channel[point][0], 3);
                for (unsigned band = 1; band < eac3::joc_tables::band_count[info.bands_index]; ++band)
                    sparse_channel_table(joc.channels).write(writer, info.sparse_channel[point][band]);
                for (unsigned band = 0; band < eac3::joc_tables::band_count[info.bands_index]; ++band)
                    sparse_vector_table(info.fine_quantization).write(writer, info.sparse_vector[point][band]);
            } else for (unsigned channel = 0; channel < joc.channels; ++channel)
                for (unsigned band = 0; band < eac3::joc_tables::band_count[info.bands_index]; ++band)
                    matrix_table(info.fine_quantization).write(writer, info.matrix[point][channel][band]);
        }
    }
    if (bit_count) *bit_count = writer.position();
    return take_aligned(writer);
}

std::vector<std::uint8_t> build_trim(const eac3::OamdFrame& oamd) {
    if (oamd.warp_mode > 3 || oamd.trim_reserved > 3 || oamd.global_trim_mode > 2 ||
        (!oamd.trim_bypass.empty() && oamd.trim_bypass.size() != oamd.object_count))
        throw std::runtime_error("invalid OAMD trim header or bypass count");
    BitWriter writer;
    writer.write_bit(false); // element prefix
    writer.write(oamd.warp_mode, 2); writer.write(oamd.trim_reserved, 2);
    writer.write(oamd.global_trim_mode, 2);
    for (const auto& trim : oamd.trims) {
        if (trim.mode > 2 || std::any_of(trim.levels.begin(), trim.levels.end(), [](auto v) { return v > 16; }) ||
            std::any_of(trim.balance.begin(), trim.balance.end(), [](int v) { return v < -16 || v > 16; }))
            throw std::runtime_error("invalid OAMD trim values");
        eac3::OamdTrimConfig empty; empty.mode = trim.mode;
        if ((trim.mode != 2 && trim != empty) ||
            (oamd.global_trim_mode != 2 && (trim.mode != oamd.global_trim_mode || trim != empty)))
            throw std::runtime_error("OAMD trim values require explicit per-config mode");
        if (oamd.global_trim_mode != 2) continue;
        writer.write_bit(trim.mode == 0);
        if (trim.mode == 0) continue;
        writer.write_bit(trim.mode == 1);
        if (trim.mode == 1) continue;
        unsigned presence = 0;
        for (unsigned field = 0; field < 3; ++field) if (trim.levels[field] != 16) presence |= 1U << field;
        for (unsigned field = 0; field < 2; ++field) if (trim.balance[field]) presence |= 8U << field;
        writer.write(presence, 5);
        for (unsigned field = 0; field < 3; ++field)
            if (presence & (1U << field)) writer.write(trim.levels[field], 4);
        for (unsigned field = 0; field < 2; ++field) if (presence & (8U << field)) {
            writer.write_bit(trim.balance[field] > 0);
            writer.write(static_cast<unsigned>(std::abs(static_cast<int>(trim.balance[field])) - 1), 4);
        }
    }
    writer.write_bit(!oamd.trim_bypass.empty());
    for (const bool bypass : oamd.trim_bypass) writer.write_bit(bypass);
    return take_aligned(writer);
}

std::vector<std::uint8_t> build_oamd(const eac3::OamdFrame& oamd) {
    if (oamd.isf_channels) throw std::runtime_error("writing OAMD ISF signals is unsupported");
    BitWriter writer;
    writer.write(0U, 2U);                            // OAMD version 0
    const unsigned objects = oamd.object_count;
    const unsigned bed_channels = oamd.bed_channels;
    if (objects == 0 || objects > 159 || bed_channels > objects || bed_channels > 17)
        throw std::runtime_error("invalid OAMD object/bed count");
    writer.write(std::min(objects - 1U, 31U), 5);
    if (objects >= 32) writer.write(objects - 32U, 7);
    const bool dynamic_only = bed_channels == 1 && oamd.bed_channel_ids == std::vector<std::uint8_t>{3};
    writer.write_bit(dynamic_only);
    if (dynamic_only) {
        writer.write_bit(true); // LFE bed followed by dynamic objects
    } else {
        writer.write((bed_channels ? 1U : 0U) | (objects > bed_channels ? 4U : 0U), 4);
        if (bed_channels) {
            writer.write_bit(false);
            writer.write_bit(false);
            writer.write_bit(false);
            writer.write_bit(false); // explicit 17-bit bed channel mask
            const auto& ids = oamd.bed_channel_ids;
            if ((!ids.empty() && ids.size() != bed_channels) || (ids.empty() && bed_channels != 6))
                throw std::runtime_error("OAMD bed requires explicit channel IDs");
            std::array<bool, 17> present{};
            if (ids.empty()) for (unsigned c = 0; c < 6; ++c) present[c] = true;
            for (const auto id : ids) {
                if (id >= 17 || present[id]) throw std::runtime_error("invalid OAMD bed channel ID");
                present[id] = true;
            }
            unsigned assignment = 0;
            for (unsigned id = 0; id < present.size(); ++id) if (present[id]) assignment |= 1U << id;
            writer.write(assignment, 17);
        }
        if (objects > bed_channels) {
            const auto dynamic = objects - bed_channels;
            writer.write(std::min(dynamic - 1U, 31U), 5);
            if (dynamic >= 32) writer.write(dynamic - 32U, 7);
        }
    }
    const auto check_update = [&](const auto& update) {
        if (update.object >= objects || update.sample_offset != 0 || update.block != 0)
            throw std::runtime_error("unsupported OAMD update timing or object ID");
    };
    for (const auto& update : oamd.position_updates) {
        check_update(update);
        if (update.differential || update.ramp_duration != 0 || update.distance_specified ||
            !std::isfinite(update.x) || !std::isfinite(update.y) || !std::isfinite(update.z) ||
            update.x < 0 || update.x > 1 || update.y < 0 || update.y > 1 || std::abs(update.z) > 1)
            throw std::runtime_error("unsupported OAMD position");
    }
    for (const auto& update : oamd.gain_updates) {
        check_update(update);
        if (update.ramp_duration || !std::isfinite(update.gain) || update.gain < 0 || update.gain > 5.623414F)
            throw std::runtime_error("unsupported OAMD gain");
    }
    for (const auto& update : oamd.spread_updates) {
        check_update(update);
        if (update.ramp_duration || update.divergence > 0 || !std::isfinite(update.size) || update.size > 1)
            throw std::runtime_error("unsupported OAMD spread");
    }
    for (const auto& update : oamd.zone_updates) {
        check_update(update);
        if (update.object < bed_channels || update.horizontal > 7 || update.ramp_duration)
            throw std::runtime_error("invalid OAMD zone update");
    }
    const unsigned bed_objects = bed_channels;
    writer.write_bit(false);                         // alternate object flag
    // A bed-only program still needs initial active/gain records.
    writer.write(oamd.trim_present ? 2U : 1U, 4U);
    BitWriter element;
    element.write_bit(false);                    // element skip bit
    element.write(0U, 2U);                       // offset mode 0
    element.write(0U, 3U);                       // one block
    element.write(0U, 6U);                       // block offset 0
    element.write(0U, 2U);                       // ramp duration code 0
    element.write_bit(true);                     // no 5-bit skip
    for (unsigned object = 0; object < objects; ++object) {
        element.write_bit(false);                // active
        float gain = 1;
        for (const auto& update : oamd.gain_updates) if (update.object == object) gain = update.gain;
        element.write(gain == 1 ? 0U : gain == 0 ? 1U : 2U, 2);
        if (gain != 0 && gain != 1) {
            const int db = static_cast<int>(std::lround(20 * std::log10(gain)));
            const int code = db > 0 ? 15 - db : 14 - db;
            if (code < 0 || code > 63) throw std::runtime_error("OAMD gain is out of range");
            element.write(static_cast<unsigned>(code), 6);
        }
        element.write_bit(true);                 // no 5-bit skip
        if (object >= bed_objects) {
            const eac3::OamdPositionUpdate* update = nullptr;
            for (const auto& candidate : oamd.position_updates)
                if (candidate.object == object) { update = &candidate; break; }
            const float x = update ? std::clamp(update->x, 0.0F, 1.0F) : 0.5F;
            const float y = update ? std::clamp(update->y, 0.0F, 1.0F) : 0.5F;
            const float z = update ? std::clamp(update->z, -1.0F, 1.0F) : 0.0F;
            element.write(static_cast<std::uint32_t>(std::lround(x * 62.0F)), 6U);
            element.write(static_cast<std::uint32_t>(std::lround(y * 62.0F)), 6U);
            element.write_bit(z >= 0.0F);
            element.write(static_cast<std::uint32_t>(std::lround(
                std::abs(z) * 15.0F)), 4U);
            element.write_bit(false);            // no distance
            std::uint8_t horizontal = 0;
            bool elevation = true;
            for (const auto& zone : oamd.zone_updates) if (zone.object == object) {
                horizontal = zone.horizontal;
                elevation = zone.elevation;
            }
            element.write(horizontal, 3U);
            element.write_bit(elevation);        // zero excludes the entire height layer
            float size = 0;
            for (const auto& spread : oamd.spread_updates)
                if (spread.object == object && spread.size >= 0) size = spread.size;
            element.write(size == 0 ? 0U : 1U, 2);
            if (size != 0) element.write(static_cast<unsigned>(std::lround(size * 31)), 5);
            element.write_bit(false);            // no fields present (bit 8)
            element.write_bit(update && update->snap); // snap to a nearby speaker
        }
        element.write_bit(false);                // no object element tail
    }
    const std::vector<std::uint8_t> element_bytes = take_aligned(element);
    writer.write(1U, 4U);                        // element index
    write_variable(writer, static_cast<std::uint32_t>(element_bytes.size() - 1U), 4U);
    write_bytes(writer, element_bytes);
    if (oamd.trim_present) {
        const auto trim = build_trim(oamd);
        writer.write(2, 4);
        write_variable(writer, static_cast<std::uint32_t>(trim.size() - 1), 4);
        write_bytes(writer, trim);
    }
    return take_aligned(writer);
}

} // namespace

std::vector<std::uint8_t> write_oamd_payload(const eac3::OamdFrame& oamd) {
    return build_oamd(oamd);
}

std::vector<std::uint8_t> write_joc_payload(const eac3::JocFrame& joc) {
    return build_joc(joc);
}

std::size_t joc_payload_bit_count(const eac3::JocFrame& joc) {
    std::size_t bits = 0;
    build_joc(joc, &bits);
    return bits;
}

std::vector<std::uint8_t> build_emdf(const eac3::OamdFrame& oamd,
    const eac3::JocFrame& joc, bool with_oamd, bool with_joc,
    std::span<const std::uint8_t> raw_oamd) {
    BitWriter writer;
    writer.write(0U, 2U);                            // version
    writer.write(0U, 3U);                            // key
    const auto payload = [&](unsigned id, std::span<const std::uint8_t> bytes) {
        writer.write(id, 5);
        writer.write_bit(false); // no sample offset
        writer.write_bit(false); // no duration
        writer.write_bit(true);  // group present
        write_variable(writer, 0, 2);
        writer.write_bit(false); // Temporary MediaInfo compatibility trial: codecdatae=0.
        writer.write_bit(false); // retain known payload
        writer.write_bit(true);  // frame aligned
        writer.write_bit(false); // no create duplicate
        writer.write_bit(false); // no remove duplicate
        writer.write(0, 5);      // priority
        writer.write(0, 2);      // processing allowed
        write_variable(writer, static_cast<std::uint32_t>(bytes.size()), 8U);
        write_bytes(writer, bytes);
    };
    if (with_oamd) {
        if (raw_oamd.empty()) {
            const auto bytes = build_oamd(oamd);
            payload(11U, bytes);
        } else {
            const auto parsed = eac3::parse_oamd_payload(raw_oamd);
            if (!parsed.valid || parsed.oamd.object_count != oamd.object_count ||
                parsed.oamd.bed_channels != oamd.bed_channels)
                throw std::runtime_error("invalid reframed OAMD payload");
            payload(11U, raw_oamd);
        }
    }
    if (with_joc) {
        const auto bytes = build_joc(joc);
        payload(14U, bytes);
    }
    writer.write(0U, 5U);                            // end of payloads
    writer.write(2U, 2U);                            // four primary protection bytes
    writer.write(0U, 2U);                            // no secondary protection
    writer.write(0U, 32U);                           // filled after encoding the audio
    std::vector<std::uint8_t> content = take_aligned(writer);
    std::vector<std::uint8_t> container;
    container.push_back(0x58U);
    container.push_back(0x38U);
    container.push_back(static_cast<std::uint8_t>((content.size() >> 8U) & 0xFFU));
    container.push_back(static_cast<std::uint8_t>(content.size() & 0xFFU));
    container.insert(container.end(), content.begin(), content.end());
    return container;
}

namespace {

std::array<std::uint8_t, 32> emdf_digest(std::span<const std::uint8_t> emdf,
    std::span<const std::uint8_t> audio, eac3::EmdfMetadata& metadata) {
    if (emdf.size() < 4 || emdf[0] != 0x58 || emdf[1] != 0x38 ||
        emdf.size() != 4U + (static_cast<unsigned>(emdf[2]) << 8U) + emdf[3])
        throw std::runtime_error("invalid EMDF protection input");
    metadata = eac3::parse_emdf_frame(emdf.subspan(4));
    if (!metadata.valid || metadata.primary_protection.empty())
        throw std::runtime_error("malformed EMDF protection");
    std::vector<std::uint8_t> content(emdf.begin() + 4, emdf.end());
    const auto first = metadata.primary_protection_bit;
    const auto count = (metadata.primary_protection.size() + metadata.secondary_protection.size()) * 8U;
    for (std::size_t bit = first; bit < first + count; ++bit)
        content[bit / 8] &= static_cast<std::uint8_t>(~(1U << (7U - bit % 8)));
    crypto::MatDecHmacSha256Context context;
    static_cast<void>(crypto::DDP_udc_hmac_obf_sha256_init(&context));
    crypto::mat_dec_hmac_sha256_update(&context, audio.data(), audio.size());
    crypto::mat_dec_hmac_sha256_update(&context, content.data(), content.size());
    std::array<std::uint8_t, 32> digest{};
    crypto::mat_dec_hmac_sha256_final(&context, digest);
    return digest;
}

} // namespace

void sign_emdf(std::vector<std::uint8_t>& emdf, std::span<const std::uint8_t> audio) {
    eac3::EmdfMetadata metadata;
    const auto digest = emdf_digest(emdf, audio, metadata);
    if (!metadata.secondary_protection.empty())
        throw std::runtime_error("signing rolling secondary protection is unsupported");
    const auto start = 32U + metadata.primary_protection_bit;
    for (std::size_t bit = 0; bit < metadata.primary_protection.size() * 8U; ++bit) {
        auto& byte = emdf[(start + bit) / 8];
        const auto mask = static_cast<std::uint8_t>(1U << (7U - (start + bit) % 8));
        byte = static_cast<std::uint8_t>((byte & ~mask) |
            (((digest[bit / 8] >> (7U - bit % 8)) & 1U) ? mask : 0U));
    }
}

bool verify_emdf_primary(std::span<const std::uint8_t> emdf,
    std::span<const std::uint8_t> audio) {
    eac3::EmdfMetadata metadata;
    const auto digest = emdf_digest(emdf, audio, metadata);
    unsigned difference = 0;
    for (std::size_t index = 0; index < metadata.primary_protection.size(); ++index)
        difference |= digest[index] ^ metadata.primary_protection[index];
    return difference == 0;
}

} // namespace atmos::encode


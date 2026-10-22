#include "emdf.hpp"

#include "common/bit_reader.hpp"
#include "common/bit_writer.hpp"
#include "joc_tables.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>

namespace atmos::eac3 {
namespace {

std::uint32_t variable_bits(BitReader& bits, unsigned width) {
    std::uint32_t value = 0;
    bool more = false;
    do {
        value += bits.read(width);
        more = bits.read_bit();
        if (more) value = (value + 1U) << width;
    } while (more);
    return value;
}

std::uint32_t variable_bits(BitReader& bits, unsigned width, unsigned limit) {
    std::uint32_t value = 0;
    bool more = false;
    do {
        value += bits.read(width);
        more = bits.read_bit();
        if (more) value = (value + 1U) << width;
    } while (more && limit-- != 0U);
    return value;
}

void parse_basic_object_info(BitReader& bits, bool all, const OamdBasicInfo& previous_object,
    OamdBasicInfo& info) {
    const unsigned blocks = all ? 3U : bits.read(2U);
    if ((blocks & 1U) != 0U) {
        info.gain_index = static_cast<std::uint8_t>(bits.read(2U));
        info.gain_bits = info.gain_index == 2U ? static_cast<std::uint8_t>(bits.read(6U)) : 0;
        if (info.gain_index == 3U) {
            info.gain_index = previous_object.gain_index;
            info.gain_bits = previous_object.gain_bits;
        }
    }
    if ((blocks & 2U) != 0U)
        info.priority = bits.read_bit() ? 32 : static_cast<std::uint8_t>(bits.read(5U));
}

float basic_gain(const OamdBasicInfo& info) {
    if (info.gain_index == 0) return 1.0F;
    if (info.gain_index == 1) return 0.0F;
    const int decibels = info.gain_bits < 15 ? 15 - static_cast<int>(info.gain_bits) :
        14 - static_cast<int>(info.gain_bits);
    const float level = std::pow(10.0F, static_cast<float>(decibels) * 0.05F);
    return static_cast<float>(static_cast<unsigned>(level*4096.0F))/4096.0F;
}

void parse_render_object_info(BitReader& bits, unsigned block, bool all,
    OamdRenderInfo& info, bool& differential) {
    const unsigned fields = all ? 15U : bits.read(4U);
    if ((fields & 1U) != 0U) {
        differential = block != 0U && bits.read_bit();
        if (differential) {
            if (!info.absolute_position) throw ParseError("unresolved OAMD differential position");
            info.x = static_cast<std::uint8_t>(std::clamp(static_cast<int>(info.x) + bits.read_signed(3), 0, 62));
            info.y = static_cast<std::uint8_t>(std::clamp(static_cast<int>(info.y) + bits.read_signed(3), 0, 62));
            const int previous_z = (info.positive_z ? 1 : -1) * info.z;
            const int resolved_z = std::clamp(previous_z + bits.read_signed(3), -15, 15);
            info.positive_z = resolved_z >= 0;
            info.z = static_cast<std::uint8_t>(std::abs(resolved_z));
        } else {
            info.x = static_cast<std::uint8_t>(std::min(bits.read(6U), 62U));
            info.y = static_cast<std::uint8_t>(std::min(bits.read(6U), 62U));
            info.positive_z = bits.read_bit();
            info.z = static_cast<std::uint8_t>(bits.read(4U));
        }
        info.distance_specified = bits.read_bit();
        info.infinity = false;
        info.distance = 0;
        if (info.distance_specified) {
            info.infinity = bits.read_bit();
            if (!info.infinity) info.distance = static_cast<std::uint8_t>(bits.read(4U));
        }
    }
    if ((fields & 2U) != 0U) {
        const auto horizontal = static_cast<std::uint8_t>(bits.read(3U));
        const bool elevation = bits.read_bit();
        info.zone = horizontal;
        info.elevation = elevation;
    }
    if ((fields & 4U) != 0U) {
        const unsigned size_code = bits.read(2U);
        if (size_code == 3) throw ParseError("reserved OAMD size index");
        info.size_index = static_cast<std::uint8_t>(size_code);
        info.size = {};
        if (size_code == 1U) {
            info.size[0] = static_cast<std::uint8_t>(bits.read(5U));
        } else if (size_code == 2U) {
            for (auto& code : info.size) code = static_cast<std::uint8_t>(bits.read(5U));
        }
    }
    if ((fields & 8U) != 0U) {
        info.screen_reference = bits.read_bit();
        info.screen_factor = info.screen_reference ? static_cast<std::uint8_t>(bits.read(3U)) : 0;
        info.depth_factor = info.screen_reference ? static_cast<std::uint8_t>(bits.read(2U)) : 0;
    }
    info.snap = bits.read_bit();
}

void append_render_state(EmdfMetadata& metadata, const OamdRenderInfo& info,
    unsigned object, unsigned block, unsigned offset, unsigned ramp) {
    constexpr std::array<float, 16> distance_factors = {
        1.1F,1.3F,1.6F,2.0F,2.5F,3.2F,4.0F,5.0F,
        6.3F,7.9F,10.0F,12.6F,15.8F,20.0F,25.1F,50.1F
    };
    const auto id = static_cast<std::uint16_t>(object);
    const auto time = static_cast<std::uint16_t>(offset);
    const auto duration = static_cast<std::uint16_t>(ramp);
    const auto index = static_cast<std::uint8_t>(block);
    const float x = static_cast<float>(info.x) / 62.0F;
    const float y = static_cast<float>(info.y) / 62.0F;
    const float z = static_cast<float>((info.positive_z ? 1 : -1) * info.z) / 15.0F;
    const float distance = !info.distance_specified ? 1.0F :
        info.infinity ? distance_factors.back() : distance_factors[info.distance];
    // Resolve metadata targets before the renderer applies time-domain ramps.
    ++metadata.oamd_position_updates;
    metadata.oamd.position_updates.push_back({id,time,duration,index,false,x,y,z,
        info.distance_specified,info.infinity,distance,info.snap});
    metadata.oamd.zone_updates.push_back({id,time,index,info.zone,info.elevation,duration});
    const float size = static_cast<float>(*std::max_element(info.size.begin(), info.size.end())) / 31.0F;
    metadata.oamd.spread_updates.push_back({id,time,duration,index,size,-1.0F});
    if (!metadata.first_position_valid) {
        metadata.first_position_valid = true;
        metadata.first_position_x = x; metadata.first_position_y = y; metadata.first_position_z = z;
    }
}

struct ObjectElementContext final {
    unsigned blocks = 0;
    unsigned bed_objects = 0;
    std::array<unsigned, 8> block_offsets{};
    std::array<unsigned, 8> ramp_durations{};
    std::vector<std::array<bool, 8>> active;
};

void parse_object_element(BitReader& bits, unsigned object_count,
    unsigned bed_objects, EmdfMetadata& metadata, ObjectElementContext& context,
    OamdElement& wire, std::size_t body_start) {
    const unsigned offset_mode = bits.read(2U);
    unsigned sample_offset = 0U;
    if (offset_mode == 1U) {
        static constexpr std::array<unsigned, 4> offsets = {8U,16U,18U,24U};
        sample_offset = offsets[bits.read(2U)];
    } else if (offset_mode == 2U) {
        sample_offset = bits.read(5U);
    }
    else if (offset_mode == 3U) throw ParseError("unsupported OAMD offset");
    const unsigned blocks = bits.read(3U) + 1U;
    std::array<unsigned, 8> block_offsets{};
    std::array<unsigned, 8> ramp_durations{};
    static constexpr std::array<unsigned, 16> extended_ramps = {
        32U,64U,128U,256U,320U,480U,1000U,1001U,
        1024U,1600U,1601U,1602U,1920U,2000U,2002U,2048U
    };
    for (unsigned block = 0; block < blocks; ++block) {
        block_offsets[block] = 32U * bits.read(6U) + sample_offset;
        const unsigned ramp = bits.read(2U);
        if (ramp == 3U) {
            ramp_durations[block] = bits.read_bit() ? extended_ramps[bits.read(4U)] :
                bits.read(11U);
        } else {
            static constexpr std::array<unsigned, 3> ramps = {0U,512U,1536U};
            ramp_durations[block] = ramps[ramp];
        }
    }
    context.blocks = blocks;
    context.bed_objects = bed_objects;
    context.block_offsets = block_offsets;
    context.ramp_durations = ramp_durations;
    context.active.assign(object_count, {});
    wire.object_header_begin = bits.position() - body_start;
    if (!bits.read_bit()) bits.skip(5U);
    wire.object_data_begin = bits.position() - body_start;
    wire.object_records.resize(object_count);
    std::array<OamdBasicInfo, 8> previous_object;
    for (auto& info : previous_object) info.gain_index = 0; // first-object code3 means 0dB
    for (unsigned object = 0; object < object_count; ++object) {
        OamdBasicInfo previous_block;
        OamdRenderInfo previous_render;
        for (unsigned block = 0; block < blocks; ++block) {
            OamdObjectRecord record;
            record.begin = bits.position() - body_start;
            record.dynamic = object >= bed_objects;
            const bool inactive = bits.read_bit();
            record.inactive = inactive;
            context.active[object][block] = !inactive;
            const unsigned basic = inactive ? 0U : (block == 0U ? 1U : bits.read(2U));
            record.full_basic = basic == 1U;
            record.basic = basic == 0U ? OamdBasicInfo{} : previous_block;
            if ((basic & 1U) != 0U) parse_basic_object_info(bits, basic == 1U,
                previous_object[block], record.basic);
            previous_block = previous_object[block] = record.basic;
            // Native OAMDI supplies a resolved gain target at every block, including reuse.
            metadata.oamd.gain_updates.push_back({
                static_cast<std::uint16_t>(object),
                static_cast<std::uint16_t>(block_offsets[block]),
                static_cast<std::uint8_t>(block), basic_gain(record.basic),
                static_cast<std::uint16_t>(ramp_durations[block])});
            record.render_begin = bits.position() - body_start;
            unsigned render = 0U;
            if (!inactive && object >= bed_objects) {
                render = block == 0U ? 1U : bits.read(2U);
            }
            record.full_render = render == 1U;
            record.render = render == 0U ? OamdRenderInfo{} : previous_render;
            if ((render & 1U) != 0U) {
                parse_render_object_info(bits, block, render == 1U, record.render, record.differential);
            }
            if (record.dynamic && !inactive)
                append_render_state(metadata, record.render, object, block, block_offsets[block], ramp_durations[block]);
            previous_render = record.render;
            record.render_end = bits.position() - body_start;
            if (bits.read_bit()) bits.skip((bits.read(4U) + 1U) * 8U);
            record.end = bits.position() - body_start;
            wire.object_records[object].push_back(record);
        }
    }
    wire.object_data_end = bits.position() - body_start;
}

float divergence_from_code(unsigned code) {
    static constexpr std::array<float, 64> values = {
        -1.0F,0.0F,.004026F,.007160F,.012731F,.020173F,.028485F,.040210F,
        .050582F,.063601F,.079914F,.100299F,.125666F,.140532F,.157027F,.175282F,
        .195417F,.217536F,.241718F,.268002F,.296377F,.326766F,.359017F,.392895F,
        .428081F,.464184F,.500755F,.537316F,.573389F,.608529F,.642346F,.674524F,
        .704833F,.733123F,.759320F,.783416F,.805451F,.825506F,.843686F,.860112F,
        .874914F,.888222F,.900168F,.910875F,.920461F,.929035F,.936698F,.943544F,
        .949656F,.955112F,.959980F,.964322F,.968195F,.974729F,.979923F,.984050F,
        .987330F,.989935F,.992874F,.994955F,.996817F,.998210F,.998993F,1.0F
    };
    return values[code & 63U];
}

void parse_extended_object_element(BitReader& bits, const ObjectElementContext& context,
    EmdfMetadata& metadata) {
    const bool divergence_present = bits.read_bit();
    if (divergence_present && context.blocks != 0U) {
        static constexpr std::array<float, 4> compact = {
            .500755F,.608529F,.704833F,1.0F
        };
        for (unsigned object = 0; object < context.active.size(); ++object) {
            float previous = 0.0F;
            for (unsigned block = 0; block < context.blocks; ++block) {
                if (!context.active[object][block] || object < context.bed_objects) {
                    previous = 0.0F;
                    continue;
                }
                float value = 0.0F;
                if (bits.read_bit()) {
                    const unsigned mode = bits.read(2U);
                    if (mode == 0U) value = compact[bits.read(2U)];
                    else if (mode == 1U) value = previous;
                    else value = divergence_from_code(bits.read(6U));
                }
                previous = std::max(value, 0.0F);
                metadata.oamd.spread_updates.push_back({
                    static_cast<std::uint16_t>(object),
                    static_cast<std::uint16_t>(context.block_offsets[block]),
                    static_cast<std::uint16_t>(context.ramp_durations[block]),
                    static_cast<std::uint8_t>(block), -1.0F, previous});
            }
        }
    }
    const bool extended_position_present = bits.read_bit();
    if (!extended_position_present || context.blocks == 0U) return;
    for (unsigned object = 0; object < context.active.size(); ++object) {
        for (unsigned block = 0; block < context.blocks; ++block) {
            if (!context.active[object][block] || object < context.bed_objects) continue;
            if (!bits.read_bit()) continue;
            const unsigned presence = bits.read(3U);
            static constexpr std::array<float, 4> precision = {
                1.0F,2.0F,-1.0F,-2.0F
            };
            float x = 0.0F;
            float y = 0.0F;
            float z = 0.0F;
            if ((presence & 1U) != 0U) x = precision[bits.read(2U)] / 310.0F;
            if ((presence & 2U) != 0U) y = precision[bits.read(2U)] / 310.0F;
            if ((presence & 4U) != 0U) z = precision[bits.read(2U)] / 75.0F;
            const auto match = [object, block](const OamdPositionUpdate& update) {
                return update.object == object && update.block == block;
            };
            const auto found = std::find_if(metadata.oamd.position_updates.rbegin(),
                metadata.oamd.position_updates.rend(), match);
            if (found != metadata.oamd.position_updates.rend()) {
                found->x += x;
                found->y += y;
                found->z += z;
            }
        }
    }
}

int huffman_decode(BitReader& bits, std::span<const std::array<int, 2>> table) {
    int node = 0;
    do {
        if (node < 0 || static_cast<std::size_t>(node) >= table.size()) {
            throw ParseError("invalid JOC Huffman node");
        }
        node = table[static_cast<std::size_t>(node)][bits.read_bit() ? 1U : 0U];
    } while (node > 0);
    return ~node;
}

std::span<const std::array<int, 2>> matrix_table(bool fine) {
    if (fine) return joc_tables::joc_huff_code_fine_generic;
    return joc_tables::joc_huff_code_coarse_generic;
}

std::span<const std::array<int, 2>> vector_table(bool fine) {
    if (fine) return joc_tables::joc_huff_code_fine_coeff_sparse;
    return joc_tables::joc_huff_code_coarse_coeff_sparse;
}

std::span<const std::array<int, 2>> channel_table(unsigned channels) {
    if (channels == 7U) return joc_tables::joc_huff_code_7ch_pos_index_sparse;
    return joc_tables::joc_huff_code_5ch_pos_index_sparse;
}

void parse_joc(BitReader& bits, EmdfMetadata& metadata) {
    const unsigned downmix = bits.read(3U);
    if (downmix > 4U) throw ParseError("unsupported JOC downmix");
    metadata.joc.downmix_config = static_cast<std::uint8_t>(downmix);
    metadata.joc_channels = static_cast<std::uint8_t>(
        downmix == 0U || downmix == 3U ? 5U : 7U);
    metadata.joc_objects = static_cast<std::uint8_t>(bits.read(6U) + 1U);
    if (bits.read(3U) != 0U) throw ParseError("unsupported JOC extension");
    const int gain_power = static_cast<int>(bits.read(3U)) - 4;
    metadata.joc_gain = (1.0F + static_cast<float>(bits.read(5U)) / 32.0F) *
        std::pow(2.0F, static_cast<float>(gain_power));
    metadata.joc.channels = metadata.joc_channels;
    metadata.joc.object_count = metadata.joc_objects;
    metadata.joc.gain = metadata.joc_gain;
    metadata.joc.sequence_count = static_cast<std::uint16_t>(bits.read(10U));
    for (unsigned object = 0; object < metadata.joc_objects; ++object) {
        auto& info = metadata.joc.objects[object];
        info.active = bits.read_bit();
        if (!info.active) continue;
        ++metadata.joc_active_objects;
        info.bands_index = static_cast<std::uint8_t>(bits.read(3U));
        info.bands = joc_tables::band_count[info.bands_index];
        info.sparse = bits.read_bit();
        info.fine_quantization = bits.read_bit();
        if (info.sparse) ++metadata.joc_sparse_objects;
        if (info.fine_quantization) ++metadata.joc_fine_objects;
        info.steep_slope = bits.read_bit();
        info.data_points = static_cast<std::uint8_t>(bits.read(1U) + 1U);
        if (info.steep_slope) {
            for (unsigned point = 0; point < info.data_points; ++point) {
                info.timeslot_offsets[point] = static_cast<std::uint8_t>(bits.read(5U) + 1U);
            }
        }
    }
    for (unsigned object = 0; object < metadata.joc_objects; ++object) {
        auto& info = metadata.joc.objects[object];
        if (!info.active) continue;
        if (info.sparse) {
            const auto channels = channel_table(metadata.joc_channels);
            const auto vectors = vector_table(info.fine_quantization);
            for (unsigned point = 0; point < info.data_points; ++point) {
                info.sparse_channel[point][0] = static_cast<std::uint8_t>(bits.read(3U));
                if (info.sparse_channel[point][0] >= metadata.joc_channels) {
                    throw ParseError("invalid JOC sparse channel");
                }
                for (unsigned band = 1; band < info.bands; ++band) {
                    info.sparse_channel[point][band] = static_cast<std::uint8_t>(
                        huffman_decode(bits, channels));
                }
                for (unsigned band = 0; band < info.bands; ++band) {
                    info.sparse_vector[point][band] = static_cast<std::uint8_t>(
                        huffman_decode(bits, vectors));
                }
            }
        } else {
            const auto table = matrix_table(info.fine_quantization);
            for (unsigned point = 0; point < info.data_points; ++point) {
                for (unsigned channel = 0; channel < metadata.joc_channels; ++channel) {
                    for (unsigned band = 0; band < info.bands; ++band) {
                        info.matrix[point][channel][band] = static_cast<std::uint8_t>(
                            huffman_decode(bits, table));
                    }
                }
            }
        }
    }
    metadata.has_joc = true;
}

void parse_trim_element(BitReader& bits, unsigned objects, OamdFrame& oamd) {
    oamd.trim_present = true;
    oamd.warp_mode = static_cast<std::uint8_t>(bits.read(2));
    oamd.trim_reserved = static_cast<std::uint8_t>(bits.read(2));
    oamd.global_trim_mode = static_cast<std::uint8_t>(bits.read(2));
    if (oamd.global_trim_mode > 2) throw ParseError("reserved OAMD global trim mode");
    oamd.trims = {};
    oamd.trim_bypass.clear();
    for (auto& trim : oamd.trims) {
        trim.mode = oamd.global_trim_mode;
        if (oamd.global_trim_mode != 2) continue;
        if (bits.read_bit()) { trim.mode = 0; continue; }
        if (bits.read_bit()) { trim.mode = 1; continue; }
        const unsigned presence = bits.read(5);
        for (unsigned field = 0; field < 3; ++field)
            if (presence & (1U << field)) trim.levels[field] = static_cast<std::uint8_t>(bits.read(4));
        for (unsigned field = 0; field < 2; ++field) if (presence & (8U << field)) {
            const int sign = bits.read_bit() ? 1 : -1;
            trim.balance[field] = static_cast<std::int8_t>(sign * static_cast<int>(bits.read(4) + 1));
        }
    }
    if (bits.read_bit())
        for (unsigned object = 0; object < objects; ++object) oamd.trim_bypass.push_back(bits.read_bit());
}

void parse_oamd(BitReader& bits, EmdfMetadata& metadata) {
    auto header_reader = bits;
    const auto header_start = bits.position();
    unsigned version = bits.read(2U);
    if (version == 3U) version += bits.read(3U);
    if (version != 0U) throw ParseError("unsupported OAMD version");
    metadata.oamd_objects = static_cast<std::uint16_t>(bits.read(5U) + 1U);
    if (metadata.oamd_objects == 32U) {
        metadata.oamd_objects = static_cast<std::uint16_t>(
            metadata.oamd_objects + bits.read(7U));
    }
    unsigned isf_objects = 0;
    if (bits.read_bit()) {
        metadata.oamd_bed_channels = bits.read_bit() ? 1U : 0U;
        if (metadata.oamd_bed_channels != 0U) metadata.oamd.bed_channel_ids.push_back(3U);
    } else {
        const unsigned description = bits.read(4U);
        if ((description & 1U) != 0U) {
            bits.skip(1U);
            const unsigned beds = bits.read_bit() ? bits.read(3U) + 2U : 1U;
            static constexpr std::array<unsigned, 10> standard_width = {
                2U,1U,1U,2U,2U,2U,2U,2U,2U,1U
            };
            static constexpr std::array<std::array<unsigned, 2>, 10> standard_ids = {{
                {{0U,1U}}, {{2U,2U}}, {{3U,3U}}, {{4U,5U}}, {{6U,7U}},
                {{8U,9U}}, {{10U,11U}}, {{12U,13U}}, {{14U,15U}}, {{16U,16U}}
            }};
            for (unsigned bed = 0; bed < beds; ++bed) {
                if (bits.read_bit()) {
                    ++metadata.oamd_bed_channels;
                    metadata.oamd.bed_channel_ids.push_back(3U);
                } else if (bits.read_bit()) {
                    const unsigned assignment = bits.read(10U);
                    for (unsigned group = 0; group < standard_width.size(); ++group) {
                        if ((assignment & (1U << group)) != 0U) {
                            metadata.oamd_bed_channels = static_cast<std::uint8_t>(
                                metadata.oamd_bed_channels + standard_width[group]);
                            metadata.oamd.bed_channel_ids.push_back(
                                static_cast<std::uint8_t>(standard_ids[group][0]));
                            if (standard_width[group] == 2U) metadata.oamd.bed_channel_ids.push_back(
                                static_cast<std::uint8_t>(standard_ids[group][1]));
                        }
                    }
                } else {
                    const unsigned assignment = bits.read(17U);
                    for (unsigned channel = 0; channel < 17U; ++channel) {
                        if ((assignment & (1U << channel)) != 0U) {
                            ++metadata.oamd_bed_channels;
                            metadata.oamd.bed_channel_ids.push_back(
                                static_cast<std::uint8_t>(channel));
                        }
                    }
                }
            }
        }
        if ((description & 2U) != 0U) {
            static constexpr std::array<unsigned, 6> isf_counts = {4U,8U,10U,14U,15U,30U};
            const unsigned index = bits.read(3U);
            if (index >= isf_counts.size()) throw ParseError("unsupported OAMD ISF");
            isf_objects = isf_counts[index];
            metadata.oamd.isf_channels = static_cast<std::uint8_t>(isf_objects);
        }
        if ((description & 4U) != 0U && bits.read(5U) == 31U) bits.skip(7U);
        if ((description & 8U) != 0U) bits.skip((bits.read(4U) + 1U) * 8U);
    }
    const bool alternate_object = bits.read_bit();
    metadata.oamd_payload.alternate_object = alternate_object;
    metadata.oamd_payload.header_bits = bits.position() - header_start;
    BitWriter header;
    for (std::size_t bit = 0; bit < metadata.oamd_payload.header_bits; ++bit)
        header.write_bit(header_reader.read_bit());
    metadata.oamd_payload.header = header.bytes();
    unsigned element_count = bits.read(4U);
    if (element_count == 15U) element_count += bits.read(5U);
    const unsigned bed_objects = metadata.oamd_bed_channels + isf_objects;
    ObjectElementContext object_context;
    for (unsigned element = 0; element < element_count; ++element) {
        const unsigned element_index = bits.read(4U);
        const std::size_t element_start = bits.position();
        const std::size_t element_size = variable_bits(bits, 4U, 4U) + 1U;
        const std::size_t element_end = bits.position() + element_size * 8U;
        auto body_reader = bits;
        const auto body_start = bits.position();
        OamdElement wire_element;
        wire_element.id = static_cast<std::uint8_t>(element_index);
        bits.skip(alternate_object ? 5U : 1U);
        if (element_index == 1U) {
            parse_object_element(bits, metadata.oamd_objects, bed_objects, metadata,
                object_context, wire_element, body_start);
        } else if (element_index == 2U) {
            parse_trim_element(bits, metadata.oamd_objects, metadata.oamd);
        } else if (element_index == 5U && object_context.blocks != 0U) {
            parse_extended_object_element(bits, object_context, metadata);
        }
        // Native oamdi 0x8730 ignores the declared size of the final known element.
        // Lucy carries complete object data with an understated final element length.
        const bool recover_final = element_index == 1U && element + 1U == element_count &&
            (bits.position() > element_end || element_size > body_reader.remaining() / 8U);
        if (bits.position() > element_end && !recover_final) {
            throw ParseError("invalid OAMD element length: read " +
                std::to_string(bits.position() - element_end) + " excess bits (start " +
                std::to_string(element_start) + ", size " + std::to_string(element_size) +
                ", end " + std::to_string(element_end) + ", position " +
                std::to_string(bits.position()) + ")");
        }
        const auto stored_end = recover_final ? bits.position() : element_end;
        BitWriter body;
        for (std::size_t bit = body_start; bit < stored_end; ++bit)
            body.write_bit(body_reader.read_bit());
        body.align();
        wire_element.body = body.bytes();
        if (!recover_final) bits.skip(element_end - bits.position());
        metadata.oamd_payload.elements.push_back(std::move(wire_element));
    }
    metadata.has_oamd = true;
    metadata.oamd.object_count = metadata.oamd_objects;
    metadata.oamd.bed_channels = metadata.oamd_bed_channels;
}

bool parse_content(BitReader& bits, std::size_t end, EmdfMetadata& metadata) {
    unsigned version = bits.read(2U);
    if (version == 3U) version += variable_bits(bits, 2U);
    unsigned key = bits.read(3U);
    if (key == 7U) key += variable_bits(bits, 3U);
    if (version != 0U || key != 0U) return false;
    bool terminated = false;
    while (bits.position() < end) {
        unsigned payload = bits.read(5U);
        if (payload == 0U) { terminated = true; break; }
        if (payload == 31U) payload += variable_bits(bits, 5U);
        const bool sample_offset = bits.read_bit();
        if (sample_offset) bits.skip(12U);
        if (bits.read_bit()) static_cast<void>(variable_bits(bits, 11U));
        if (bits.read_bit()) static_cast<void>(variable_bits(bits, 2U));
        if (bits.read_bit()) bits.skip(8U);
        if (!bits.read_bit()) {
            bool aligned = false;
            if (!sample_offset) {
                aligned = bits.read_bit();
                if (aligned) bits.skip(2U);
            }
            if (sample_offset || aligned) bits.skip(7U);
        }
        const std::size_t payload_size = static_cast<std::size_t>(variable_bits(bits, 8U));
        const std::size_t payload_end = bits.position() + payload_size * 8U;
        if (payload_end > end) return false;
        if (payload == 14U) parse_joc(bits, metadata);
        else if (payload == 11U) parse_oamd(bits, metadata);
        if (bits.position() > payload_end) return false;
        bits.skip(payload_end - bits.position());
    }
    if (!terminated) return false;
    const unsigned primary = bits.read(2);
    const unsigned secondary = bits.read(2);
    if (primary == 0) return false; // Annex H reserves primary length code zero.
    constexpr unsigned protection_bytes[] = {0,1,4,16};
    metadata.primary_protection_bit = bits.position();
    for (unsigned index = 0; index < protection_bytes[primary]; ++index)
        metadata.primary_protection.push_back(static_cast<std::uint8_t>(bits.read(8)));
    for (unsigned index = 0; index < protection_bytes[secondary]; ++index)
        metadata.secondary_protection.push_back(static_cast<std::uint8_t>(bits.read(8)));
    if (bits.position() > end || end - bits.position() > 7U) return false;
    while (bits.position() < end) if (bits.read_bit()) return false;
    metadata.valid = true;
    return true;
}

bool parse_block(std::span<const std::uint8_t> data, std::size_t offset,
    EmdfMetadata& metadata) {
    if (offset + 4U > data.size()) return false;
    const std::size_t length = static_cast<std::size_t>(data[offset + 2U] << 8U) |
        data[offset + 3U];
    if (offset + 4U + length > data.size()) return false;
    BitReader bits(data.subspan(offset + 4U, length));
    return parse_content(bits, length * 8U, metadata);
}

} // namespace

EmdfMetadata parse_emdf(std::span<const std::uint8_t> data) {
    EmdfMetadata result;
    try {
        for (std::size_t offset = 0; offset + 4U <= data.size(); ++offset) {
            if (data[offset] == 0x58U && data[offset + 1U] == 0x38U &&
                parse_block(data, offset, result)) break;
        }
    } catch (const ParseError& error) {
        result.valid = false;
        result.error = error.what();
    }
    return result;
}

EmdfMetadata parse_emdf_frame(std::span<const std::uint8_t> data) {
    EmdfMetadata result;
    try {
        BitReader bits(data);
        static_cast<void>(parse_content(bits, data.size() * 8U, result));
    } catch (const ParseError& error) {
        result.valid = false;
        result.error = error.what();
    }
    return result;
}

EmdfMetadata parse_oamd_payload(std::span<const std::uint8_t> data) {
    EmdfMetadata result;
    try {
        BitReader bits(data);
        parse_oamd(bits, result);
        result.valid = true;
    } catch (const ParseError& error) {
        result.valid = false;
        result.error = error.what();
    }
    return result;
}

} // namespace atmos::eac3

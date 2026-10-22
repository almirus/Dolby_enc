#include "sync_frame.hpp"

#include "bitstream_info.hpp"
#include "audio_frame.hpp"
#include "audio_block.hpp"
#include "emdf.hpp"

namespace atmos::eac3 {

bool contains_emdf_sync(std::span<const std::uint8_t> frame) {
    if (frame.size() < 2U) {
        return false;
    }
    const std::size_t bits = frame.size() * 8U;
    for (std::size_t start = 0; start + 16U <= bits; ++start) {
        std::uint16_t value = 0;
        for (unsigned bit = 0; bit < 16U; ++bit) {
            const std::size_t position = start + bit;
            value = static_cast<std::uint16_t>((value << 1U) |
                ((frame[position >> 3U] >> (7U - (position & 7U))) & 1U));
        }
        if (value == 0x5838U) {
            return true;
        }
    }
    return false;
}

std::optional<SyncFrame> parse_sync_frame(
    std::span<const std::uint8_t> data, std::size_t offset) {
    if (offset + 7U > data.size() || data[offset] != 0x0BU ||
        data[offset + 1U] != 0x77U) {
        return std::nullopt;
    }
    const auto info = parse_bitstream_info(data, offset);
    if (!info) return std::nullopt;
    SyncFrame frame;
        frame.offset = offset;
        frame.stream_type = info->stream_type;
        frame.substream_id = info->substream_id;
        frame.size = info->frame_size;
        frame.sample_rate = info->sample_rate;
        frame.blocks = info->blocks;
        frame.audio_coding_mode = info->audio_coding_mode;
        frame.lfe = info->lfe;
        frame.bitstream_id = info->bitstream_id;
        frame.channel_count = info->channel_count;
        frame.samples = info->samples;
        frame.extension_type_a = info->extension_type_a;
        frame.complexity_index_type_a = info->complexity_index_type_a;
        frame.audio_frame_start_bit = info->audio_frame_start_bit;
        const auto syntax = parse_audio_frame_syntax(
            data.subspan(offset, frame.size), *info);
        frame.audio_frame_syntax_valid = syntax.has_value();
        if (syntax) {
            frame.audio_blocks_start_bit = syntax->audio_blocks_start_bit;
            frame.adaptive_hybrid_transform_syntax = syntax->parse_aht;
            frame.snr_offset_strategy = syntax->snr_offset_strategy;
            frame.block_switch_syntax = syntax->block_switch_syntax;
            frame.dither_flag_syntax = syntax->dither_flag_syntax;
            frame.bit_allocation_syntax = syntax->bit_allocation_syntax;
            frame.fast_gain_syntax = syntax->fast_gain_syntax;
            frame.delta_bit_allocation_syntax = syntax->delta_bit_allocation_syntax;
            frame.skip_syntax = syntax->skip_syntax;
            for (unsigned channel = 0; channel < frame.first_block_exponent_strategy.size();
                ++channel) {
                frame.first_block_exponent_strategy[channel] = static_cast<std::uint8_t>(
                    syntax->exponent_strategy[0][channel]);
            }
            for (unsigned block = 0; block < frame.blocks; ++block) {
                if (syntax->coupling[block][0]) ++frame.coupling_blocks;
            }
            DitherGenerator dither;
            const auto spectral = decode_spectral_frame(
                data.subspan(offset, frame.size), *info, *syntax,
                dither, &frame.spectral_error);
            if (spectral) {
                frame.spectral_blocks_decoded = spectral->decoded_blocks;
                frame.spectral_extension_used = spectral->spectral_extension_used;
                frame.enhanced_coupling_used = spectral->enhanced_coupling_used;
                frame.nonzero_spectral_coefficients = spectral->nonzero_coefficients;
                const auto emdf = parse_emdf(spectral->auxiliary_data);
                frame.emdf_valid = emdf.valid;
                frame.joc_payload = emdf.has_joc;
                frame.oamd_payload = emdf.has_oamd;
                frame.joc_channels = emdf.joc_channels;
                frame.joc_objects = emdf.joc_objects;
                frame.joc_active_objects = emdf.joc_active_objects;
                frame.joc_sparse_objects = emdf.joc_sparse_objects;
                frame.joc_fine_objects = emdf.joc_fine_objects;
                for (unsigned object = 0; object < emdf.joc.object_count; ++object) {
                    const auto& joc_object = emdf.joc.objects[object];
                    frame.joc_object_flags[object] = static_cast<std::uint8_t>(
                        (joc_object.active ? 1U : 0U) |
                        (joc_object.sparse ? 2U : 0U) |
                        (joc_object.fine_quantization ? 4U : 0U) |
                        (joc_object.steep_slope ? 8U : 0U) |
                        ((joc_object.data_points - 1U) << 4U) |
                        (joc_object.bands_index << 5U));
                }
                frame.oamd_objects = emdf.oamd_objects;
                frame.oamd_bed_channels = emdf.oamd_bed_channels;
                frame.oamd_position_updates = emdf.oamd_position_updates;
                frame.oamd_first_position_valid = emdf.first_position_valid;
                frame.oamd_first_position_x = emdf.first_position_x;
                frame.oamd_first_position_y = emdf.first_position_y;
                frame.oamd_first_position_z = emdf.first_position_z;
                frame.emdf_error = emdf.error;
            }
        }
    frame.emdf_sync = contains_emdf_sync(data.subspan(offset, frame.size));
    return frame;
}

} // namespace atmos::eac3

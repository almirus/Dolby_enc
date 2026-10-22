#include "eac3/audio_block.hpp"
#include "eac3/audio_frame.hpp"
#include "eac3/bitstream_info.hpp"
#include "eac3/emdf.hpp"
#include "eac3/sync_frame.hpp"
#include "encode/frame_muxer.hpp"
#include "encode/emdf_writer.hpp"
#include "common/crc.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

std::vector<std::uint8_t> read_file(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("cannot open " + path.string());
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

std::string hex(std::span<const std::uint8_t> bytes, std::size_t limit) {
    static constexpr char digits[] = "0123456789ABCDEF";
    std::string out;
    const std::size_t count = std::min(bytes.size(), limit);
    for (std::size_t i = 0; i < count; ++i) {
        if (i != 0U) out.push_back(' ');
        out.push_back(digits[bytes[i] >> 4U]);
        out.push_back(digits[bytes[i] & 15U]);
    }
    if (bytes.size() > count) out += " ...";
    return out;
}

int verify_primary(std::span<const std::uint8_t> data) {
    std::size_t offset = 0, frames = 0;
    unsigned expected_sequence = 0;
    while (offset < data.size()) {
        const auto info = atmos::eac3::parse_bitstream_info(data, offset);
        if (!info || info->stream_type != 0 || info->blocks != 6)
            throw std::runtime_error("expected a complete independent six-block E-AC-3 frame at byte " + std::to_string(offset));
        const auto frame = data.subspan(offset, info->frame_size);
        if (atmos::crc16_ansi(frame.subspan(2)) != 0)
            throw std::runtime_error("CRC mismatch at byte " + std::to_string(offset));
        const auto syntax = atmos::eac3::parse_audio_frame_syntax(frame, *info);
        if (!syntax) throw std::runtime_error("invalid protected audio header");
        atmos::eac3::DitherGenerator dither;
        dither.reset();
        const auto spectral = atmos::eac3::decode_spectral_frame(frame, *info, *syntax, dither);
        if (!spectral) throw std::runtime_error("invalid protected spectral frame");
        const auto metadata = atmos::eac3::parse_emdf(spectral->auxiliary_data);
        if (!metadata.valid || !metadata.has_joc || !metadata.has_oamd ||
            metadata.joc.sequence_count != expected_sequence ||
            !atmos::encode::verify_emdf_primary(spectral->auxiliary_data,
                atmos::encode::protection_audio(frame)))
            throw std::runtime_error("Atmos metadata/authentication mismatch at byte " + std::to_string(offset));
        expected_sequence = expected_sequence == 1023 ? 1 : expected_sequence + 1;
        offset += frame.size();
        ++frames;
    }
    if (!frames) throw std::runtime_error("empty protected stream");
    std::cout << "verified primary authentication: " << frames << " frames\n";
    return 0;
}

void dump_frame(std::span<const std::uint8_t> data, std::size_t offset,
    std::size_t index) {
    const auto frame = atmos::eac3::parse_sync_frame(data, offset);
    if (!frame) {
        std::cout << "frame " << index << " @" << offset << "  <unparsable>\n";
        return;
    }
    std::cout << "frame " << index << " @" << offset << " size=" << frame->size
              << " strmtyp=" << unsigned(frame->stream_type)
              << " sub=" << unsigned(frame->substream_id)
              << " acmod=" << unsigned(frame->audio_coding_mode)
              << " lfe=" << frame->lfe
              << " bsid=" << unsigned(frame->bitstream_id)
              << " fs=" << frame->sample_rate
              << " blocks=" << unsigned(frame->blocks)
              << " channels=" << unsigned(frame->channel_count)
              << " samples=" << frame->samples
              << " afs_bit=" << frame->audio_frame_start_bit
              << " abs_bit=" << frame->audio_blocks_start_bit
              << " extA=" << frame->extension_type_a
              << " cmplx=" << unsigned(frame->complexity_index_type_a) << "\n";
    std::cout << "   emdf_sync=" << frame->emdf_sync << " emdf_valid=" << frame->emdf_valid
              << " joc=" << frame->joc_payload << " oamd=" << frame->oamd_payload
              << " joc_ch=" << unsigned(frame->joc_channels)
              << " joc_obj=" << unsigned(frame->joc_objects)
              << " [" << unsigned(frame->joc_active_objects) << " act,"
              << unsigned(frame->joc_sparse_objects) << " sparse,"
              << unsigned(frame->joc_fine_objects) << " fine]"
              << " oamd_obj=" << frame->oamd_objects
              << " bed_ch=" << unsigned(frame->oamd_bed_channels)
              << " pos_upd=" << frame->oamd_position_updates;
    if (frame->oamd_first_position_valid) {
        std::cout << " firstPos=(" << frame->oamd_first_position_x << ","
                  << frame->oamd_first_position_y << "," << frame->oamd_first_position_z << ")";
    }
    std::cout << "\n";
    if (!frame->emdf_error.empty()) std::cout << "   emdf_error=" << frame->emdf_error << "\n";
    if (!frame->spectral_error.empty())
        std::cout << "   spectral_error=" << frame->spectral_error << "\n";
    std::cout << "   snr_strategy=" << unsigned(frame->snr_offset_strategy)
              << " aht=" << frame->adaptive_hybrid_transform_syntax
              << " spx=" << frame->spectral_extension_used
              << " ecpl=" << frame->enhanced_coupling_used
              << " nonzero=" << frame->nonzero_spectral_coefficients
              << " bs=" << frame->block_switch_syntax
              << " dith=" << frame->dither_flag_syntax
              << " ba=" << frame->bit_allocation_syntax
              << " fg=" << frame->fast_gain_syntax
              << " dba=" << frame->delta_bit_allocation_syntax
              << " skip=" << frame->skip_syntax << "\n";

    const std::span<const std::uint8_t> view(data.data() + offset, frame->size);
    {
        const auto info = atmos::eac3::parse_bitstream_info(view, 0U);
        if (info) {
            const auto syntax = atmos::eac3::parse_audio_frame_syntax(view, *info);
            if (syntax) {
                atmos::eac3::DitherGenerator dither;
                const auto spectral = atmos::eac3::decode_spectral_frame(
                    view, *info, *syntax, dither);
                if (spectral) {
                    std::cout << "   block_end_bits=";
                    for (unsigned block = 0; block < info->blocks; ++block)
                        std::cout << (block ? "," : "") << spectral->blocks[block].end_bit;
                    std::cout << "\n";
                }
                if (spectral && !spectral->auxiliary_data.empty()) {
                    const auto metadata = atmos::eac3::parse_emdf(spectral->auxiliary_data);
                    if (metadata.valid && metadata.has_joc) std::cout << "   joc_gain=" << metadata.joc.gain << "\n";
                    std::cout << "   skip_bytes[" << spectral->auxiliary_data.size()
                              << "]=" << hex(spectral->auxiliary_data, 96U) << "\n";
                    try {
                        const auto audio = atmos::encode::protection_audio(view);
                        std::cout << "   protected_audio_bytes=" << audio.size()
                            << " tail=" << hex(std::span<const std::uint8_t>(audio).last(std::min<std::size_t>(8, audio.size())), 8)
                            << " primary_valid=" << atmos::encode::verify_emdf_primary(spectral->auxiliary_data, audio) << "\n";
                    } catch (const std::exception& error) {
                        std::cout << "   protection_error=" << error.what() << "\n";
                    }
                }
            }
        }
    }
    for (std::size_t i = 0; i + 3U < view.size(); ++i) {
        if (view[i] == 0x58U && view[i + 1U] == 0x38U) {
            const std::size_t length =
                (static_cast<std::size_t>(view[i + 2U]) << 8U) | view[i + 3U];
            if (length > view.size() - i - 4U) continue;
            std::cout << "   emdf @" << i << " len=" << length
                      << " hdr=" << hex(view.subspan(i, 8U), 8U) << "\n";
            std::cout << "     payload=" << hex(view.subspan(i + 4U, length), 64U) << "\n";
            break;
        }
    }
    // Jump to the next readable frame rather than trusting frmsiz on its own.
    const std::size_t next = offset + frame->size;
    if (next + 1U < data.size() && (data[next] != 0x0BU || data[next + 1U] != 0x77U)) {
        const std::size_t found = [&] {
            for (std::size_t i = next; i + 1U < data.size(); ++i)
                if (data[i] == 0x0BU && data[i + 1U] == 0x77U) return i;
            return data.size();
        }();
        std::cout << "   [gap] next sync at " << found << ", frmsiz predicted " << next << "\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
    if (argc < 2) {
        std::cerr << "usage: dump-eac3 <file.eac3> [max_frames] [offset]\n"
            "       dump-eac3 <generated.ec3> --verify-primary\n";
        return 2;
    }
    const auto data = read_file(argv[1]);
    if (argc == 3 && std::string(argv[2]) == "--verify-primary") return verify_primary(data);
    const std::size_t max_frames = argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 4U;
    std::size_t offset = argc > 3 ? std::strtoul(argv[3], nullptr, 10) : 0U;
    std::cout << "file=" << argv[1] << " size=" << data.size() << " start=" << offset << "\n";

    // Skip leading garbage: advance to the first offset the frame parser accepts.
    while (offset + 8U <= data.size() && !atmos::eac3::parse_sync_frame(data, offset)) ++offset;
    std::cout << "first valid frame at " << offset << "\n";

    std::size_t index = 0;
    while (offset + 8U <= data.size() && index < max_frames) {
        dump_frame(data, offset, index);
        const auto frame = atmos::eac3::parse_sync_frame(data, offset);
        if (!frame) break;
        offset += frame->size;
        ++index;
    }
    std::cout << "dumped " << index << " frames, ended at " << offset << "/" << data.size() << "\n";
    return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
}

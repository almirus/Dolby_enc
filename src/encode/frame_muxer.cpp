#include "encode/frame_muxer.hpp"

#include "encode/emdf_writer.hpp"
#include "common/bit_writer.hpp"
#include "common/crc.hpp"
#include "eac3/audio_block.hpp"
#include <algorithm>
#include <stdexcept>

namespace atmos::encode {

std::size_t joc_payload_budget(std::size_t oamd_emdf_bytes, unsigned objects, std::uint32_t bitrate) {
    if (!objects || objects > 16) throw std::runtime_error("invalid JOC budget object count");
    // 30 header bits; each object needs at most13 header+3 channel+16 Huffman bits.
    const std::size_t minimum = 4 + 4 * objects;
    constexpr std::size_t overhead = 8, capacity = 2047, historical = 511;
    if (oamd_emdf_bytes > capacity - overhead - minimum)
        throw std::runtime_error("OAMD and minimum JOC exceed the EMDF skip-field capacity");
    const auto previous = oamd_emdf_bytes < historical - overhead ?
        historical - overhead - oamd_emdf_bytes : 0;
    if (previous >= minimum) return previous;
    // Dense OAMD keeps the native372-byte fit above448k; coarser low-rate fits protect core bits.
    const auto reserve = bitrate > 448000 ? std::size_t{372} : minimum;
    return std::min(reserve, capacity - overhead - oamd_emdf_bytes);
}

std::vector<std::uint8_t> protection_audio(std::span<const std::uint8_t> frame) {
    const auto info = eac3::parse_bitstream_info(frame, 0);
    if (!info) throw std::runtime_error("invalid E-AC-3 protection frame");
    const auto syntax = eac3::parse_audio_frame_syntax(frame, *info);
    if (!syntax) throw std::runtime_error("invalid protected audio syntax");
    eac3::DitherGenerator dither;
    dither.reset();
    std::string error;
    const auto spectral = eac3::decode_spectral_frame(frame, *info, *syntax, dither, &error);
    if (!spectral) throw std::runtime_error("cannot canonicalize protected audio: " + error);
    const auto end = spectral->blocks[info->blocks - 1].end_bit;
    auto excluded = spectral->skip_ranges;
    excluded.emplace_back(info->informational_start_bit, info->informational_end_bit);
    excluded.emplace_back(info->additional_bsi_start_bit, info->additional_bsi_end_bit);
    excluded.emplace_back(syntax->skip_syntax_bit, syntax->skip_syntax_bit + 1);
    std::sort(excluded.begin(), excluded.end());
    BitWriter writer;
    std::size_t cursor = 32; // syncword, stream/substream ID and frame size are not hashed
    const auto copy = [&](std::size_t first, std::size_t last) {
        for (auto bit = first; bit < last; ++bit)
            writer.write((frame[bit / 8] >> (7U - bit % 8)) & 1U, 1);
    };
    for (const auto& [first, last] : excluded) {
        if (first < cursor || last < first || last > end)
            throw std::runtime_error("invalid protection exclusion range");
        copy(cursor, first);
        cursor = last;
    }
    copy(cursor, end);
    const auto bits = writer.position();
    writer.align();
    auto audio = writer.bytes();
    // Dolby hashes whole 16-bit words of the byte-aligned audio: a complete
    // trailing byte is zero-padded up to the next word, an incomplete one is
    // excluded. Confirmed on 1534 frames of a native DEE stream; dropping the
    // complete byte instead rejects 93 of them.
    if ((audio.size() & 1U) != 0U) {
        if ((bits & 7U) == 0U) audio.push_back(0);
        else audio.pop_back();
    }
    return audio;
}

std::vector<std::uint8_t> iec61937_burst(std::span<const std::uint8_t> frame) {
    // One E-AC-3 syncframe occupies one S/PDIF slot: 1536 samples at 128 bits
    // per sample frame is 196608 bits, i.e. 24576 bytes with the 8-byte burst
    // header. Padding the slot is what keeps the bursts at the frame rate.
    constexpr std::size_t slot_bytes = 24576;
    if ((frame.size() & 1U) != 0U || frame.size() + 8U > slot_bytes) {
        throw std::runtime_error("E-AC-3 frame cannot be framed as one IEC 61937 burst");
    }
    std::vector<std::uint8_t> burst(slot_bytes, 0);
    const auto put16 = [&](std::size_t at, unsigned value) {
        burst[at] = static_cast<std::uint8_t>(value & 0xFFU);
        burst[at + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    };
    put16(0U, 0xF872U);                       // Pa
    put16(2U, 0x4E1FU);                       // Pb
    put16(4U, 0x0015U);                       // Pc: data type 0x15, E-AC-3
    put16(6U, static_cast<unsigned>(frame.size())); // Pd: payload size in bytes
    // S/PDIF carries each 16-bit word LSB first.
    for (std::size_t index = 0; index < frame.size(); index += 2U) {
        burst[8U + index] = frame[index + 1U];
        burst[9U + index] = frame[index];
    }
    return burst;
}

void encode_atmos_frame(CoreEncoder& encoder, std::span<const float> interleaved,
    const eac3::OamdFrame& oamd, const eac3::JocFrame& joc, bool with_oamd,
    bool with_joc, std::vector<std::uint8_t>& frame,
    std::span<const std::uint8_t> raw_oamd) {
    auto emdf = (with_oamd || with_joc) ?
        build_emdf(oamd, joc, with_oamd, with_joc, raw_oamd) : std::vector<std::uint8_t>{};
    encoder.set_skip_payload(emdf);
    encoder.encode(interleaved, frame);
    if (!emdf.empty()) {
        sign_emdf(emdf, protection_audio(frame));
        std::size_t copied = 0;
        const auto patch = [&](std::size_t first, std::size_t last) {
            if (last < first || last > frame.size() * 8U || last - first > emdf.size() * 8U - copied)
                throw std::runtime_error("invalid EMDF fragment range");
            for (auto bit = first; bit < last; ++bit, ++copied) {
                const auto mask = static_cast<std::uint8_t>(1U << (7U - bit % 8));
                auto& byte = frame[bit / 8];
                byte = static_cast<std::uint8_t>((byte & ~mask) |
                    (((emdf[copied / 8] >> (7U - copied % 8)) & 1U) ? mask : 0U));
            }
        };
        if (emdf.size() <= 511U) {
            const auto start = encoder.payload_start_bit();
            patch(start, start + emdf.size() * 8U);
        } else {
            const auto info = eac3::parse_bitstream_info(frame, 0);
            if (!info) throw std::runtime_error("invalid fragmented E-AC-3 frame");
            const auto syntax = eac3::parse_audio_frame_syntax(frame, *info);
            if (!syntax) throw std::runtime_error("invalid fragmented audio syntax");
            eac3::DitherGenerator dither;
            dither.reset();
            const auto spectral = eac3::decode_spectral_frame(frame, *info, *syntax, dither);
            if (!spectral) throw std::runtime_error("invalid fragmented audio");
            for (const auto& [first, last] : spectral->skip_ranges) {
                if (last < first) throw std::runtime_error("invalid skip-field fragment");
                if (last - first == 1U) continue;
                if (last - first < 10U || (last - first - 10U) % 8U != 0)
                    throw std::runtime_error("invalid skip-field fragment");
                patch(first + 10U, last);
            }
        }
        if (copied != emdf.size() * 8U) throw std::runtime_error("truncated EMDF fragments");
        const auto crc = atmos::crc16_ansi(std::span<const std::uint8_t>(frame).subspan(2, frame.size() - 4));
        frame[frame.size() - 2] = static_cast<std::uint8_t>(crc >> 8U);
        frame.back() = static_cast<std::uint8_t>(crc);
    }
}

} // namespace atmos::encode

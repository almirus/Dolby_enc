#include "encode/mp4_writer.hpp"

#include <cstdint>
#include <fstream>
#include <vector>

namespace atmos::encode {
namespace {

constexpr std::uint32_t timescale = 48000U;
constexpr std::uint32_t samples_per_frame = 1536U;

void put8(std::vector<std::uint8_t>& out, unsigned value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

void put16(std::vector<std::uint8_t>& out, unsigned value) {
    put8(out, value >> 8U);
    put8(out, value);
}

void put32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    put8(out, value >> 24U);
    put8(out, value >> 16U);
    put8(out, value >> 8U);
    put8(out, value);
}

void put_type(std::vector<std::uint8_t>& out, const char (&type)[5]) {
    for (unsigned index = 0; index < 4U; ++index) put8(out, static_cast<unsigned char>(type[index]));
}

// size + type + payload
void box(std::vector<std::uint8_t>& out, const char (&type)[5], const std::vector<std::uint8_t>& payload) {
    std::vector<std::uint8_t> body;
    put_type(body, type);
    body.insert(body.end(), payload.begin(), payload.end());
    put32(out, static_cast<std::uint32_t>(body.size() + 4U));
    out.insert(out.end(), body.begin(), body.end());
}

std::vector<std::uint8_t> full_box(const char (&type)[5], unsigned version, unsigned flags,
    const std::vector<std::uint8_t>& payload) {
    std::vector<std::uint8_t> body;
    put8(body, version);
    put8(body, flags >> 16U);
    put8(body, flags >> 8U);
    put8(body, flags);
    body.insert(body.end(), payload.begin(), payload.end());
    std::vector<std::uint8_t> out;
    box(out, type, body);
    return out;
}

void put_matrix(std::vector<std::uint8_t>& out) {
    put32(out, 0x00010000U); put32(out, 0U); put32(out, 0U);
    put32(out, 0U); put32(out, 0x00010000U); put32(out, 0U);
    put32(out, 0U); put32(out, 0U); put32(out, 0x40000000U);
}

} // namespace

bool write_mp4(const std::filesystem::path& ec3_path,
    const std::filesystem::path& mp4_path, std::uint32_t bitrate, std::string& error) {
    std::ifstream input(ec3_path, std::ios::binary);
    if (!input) {
        error = "cannot read " + ec3_path.string();
        return false;
    }

    // Pass 1: frame sizes and total mdat payload.
    std::vector<std::uint32_t> sizes;
    std::uint64_t payload_bytes = 0;
    std::vector<std::uint8_t> header(4);
    while (input.read(reinterpret_cast<char*>(header.data()), 4)) {
        if (header[0] != 0x0BU || header[1] != 0x77U) {
            error = "not an E-AC-3 frame in " + ec3_path.string();
            return false;
        }
        const auto size = ((static_cast<unsigned>(header[2] & 0x07U) << 8U) | header[3]) * 2U + 2U;
        sizes.push_back(size);
        payload_bytes += size;
        input.seekg(static_cast<std::streamoff>(size - 4U), std::ios::cur);
        if (!input) {
            error = "truncated E-AC-3 frame in " + ec3_path.string();
            return false;
        }
    }
    if (sizes.empty()) {
        error = "no E-AC-3 frames in " + ec3_path.string();
        return false;
    }

    // ftyp
    std::vector<std::uint8_t> ftyp_payload;
    put_type(ftyp_payload, "isom");
    put32(ftyp_payload, 512U);
    put_type(ftyp_payload, "iso2");
    put_type(ftyp_payload, "mp41");
    std::vector<std::uint8_t> ftyp;
    box(ftyp, "ftyp", ftyp_payload);

    const std::uint32_t mdat_offset = static_cast<std::uint32_t>(ftyp.size() + 8U);
    const std::uint64_t duration = static_cast<std::uint64_t>(sizes.size()) * samples_per_frame;

    std::ofstream output(mp4_path, std::ios::binary);
    if (!output) {
        error = "cannot write " + mp4_path.string();
        return false;
    }
    output.write(reinterpret_cast<const char*>(ftyp.data()), static_cast<std::streamsize>(ftyp.size()));
    std::vector<std::uint8_t> mdat_header;
    put32(mdat_header, static_cast<std::uint32_t>(8U + payload_bytes));
    put_type(mdat_header, "mdat");
    output.write(reinterpret_cast<const char*>(mdat_header.data()),
        static_cast<std::streamsize>(mdat_header.size()));

    input.clear();
    input.seekg(0);
    std::vector<std::uint8_t> frame;
    for (std::size_t index = 0; index < sizes.size(); ++index) {
        frame.resize(sizes[index]);
        if (!input.read(reinterpret_cast<char*>(frame.data()), static_cast<std::streamsize>(frame.size()))) {
            error = "truncated E-AC-3 frame in " + ec3_path.string();
            return false;
        }
        output.write(reinterpret_cast<const char*>(frame.data()),
            static_cast<std::streamsize>(frame.size()));
    }

    // moov
    std::vector<std::uint8_t> mvhd_payload;
    put32(mvhd_payload, 0U);                       // creation time
    put32(mvhd_payload, 0U);                       // modification time
    put32(mvhd_payload, 1000U);                    // timescale: ms
    put32(mvhd_payload, static_cast<std::uint32_t>(duration * 1000U / timescale));
    put32(mvhd_payload, 0x00010000U);              // rate 1.0
    put16(mvhd_payload, 0x0100U);                  // volume 1.0
    put16(mvhd_payload, 0U);
    put32(mvhd_payload, 0U);
    put32(mvhd_payload, 0U);
    put_matrix(mvhd_payload);
    for (int index = 0; index < 6; ++index) put32(mvhd_payload, 0U); // pre_defined
    put32(mvhd_payload, 2U);                       // next track ID
    const auto mvhd = full_box("mvhd", 0, 0, mvhd_payload);

    std::vector<std::uint8_t> tkhd_payload;
    put32(tkhd_payload, 0U);
    put32(tkhd_payload, 0U);
    put32(tkhd_payload, 1U);                       // track ID
    put32(tkhd_payload, 0U);
    put32(tkhd_payload, static_cast<std::uint32_t>(duration * 1000U / timescale));
    put32(tkhd_payload, 0U);
    put32(tkhd_payload, 0U);
    put16(tkhd_payload, 0U);                       // layer
    put16(tkhd_payload, 0U);                       // alternate group
    put16(tkhd_payload, 0x0100U);                  // volume
    put16(tkhd_payload, 0U);
    put_matrix(tkhd_payload);
    put32(tkhd_payload, 0U);                       // width
    put32(tkhd_payload, 0U);                       // height
    const auto tkhd = full_box("tkhd", 0, 3, tkhd_payload); // flags 3: enabled, in movie

    std::vector<std::uint8_t> mdhd_payload;
    put32(mdhd_payload, 0U);
    put32(mdhd_payload, 0U);
    put32(mdhd_payload, timescale);
    put32(mdhd_payload, static_cast<std::uint32_t>(duration));
    put16(mdhd_payload, 0x55C4U);                  // language: und
    put16(mdhd_payload, 0U);
    const auto mdhd = full_box("mdhd", 0, 0, mdhd_payload);

    std::vector<std::uint8_t> hdlr_payload;
    put32(hdlr_payload, 0U);
    put_type(hdlr_payload, "soun");
    put32(hdlr_payload, 0U); put32(hdlr_payload, 0U); put32(hdlr_payload, 0U);
    const char handler_name[] = "SoundHandler";
    for (const char* text = handler_name; *text; ++text) put8(hdlr_payload, static_cast<unsigned char>(*text));
    put8(hdlr_payload, 0U);
    const auto hdlr = full_box("hdlr", 0, 0, hdlr_payload);

    std::vector<std::uint8_t> smhd_payload;
    put16(smhd_payload, 0U);
    put16(smhd_payload, 0U);
    const auto smhd = full_box("smhd", 0, 0, smhd_payload);

    std::vector<std::uint8_t> url_payload;
    url_payload.push_back(0U); // flags 1: media data in the same file
    const auto url = full_box("url ", 0, 1, url_payload);
    std::vector<std::uint8_t> dref_payload;
    put32(dref_payload, 1U);
    dref_payload.insert(dref_payload.end(), url.begin(), url.end());
    const auto dref = full_box("dref", 0, 0, dref_payload);
    std::vector<std::uint8_t> dinf_payload;
    dinf_payload.insert(dinf_payload.end(), dref.begin(), dref.end());
    std::vector<std::uint8_t> dinf;
    box(dinf, "dinf", dinf_payload);

    // dec3: stream data rate, one independent substream, 48 kHz, bsid 16,
    // 3/2 with LFE (5.1), no dependent substreams.
    std::vector<std::uint8_t> dec3_payload;
    const std::uint32_t data_rate = bitrate / 1000U;
    dec3_payload.push_back(static_cast<std::uint8_t>(data_rate >> 5U));
    dec3_payload.push_back(static_cast<std::uint8_t>(((data_rate & 0x1FU) << 3U) | 0U)); // num_ind_sub-1 = 0
    // fscod(2)=0 bsid(5)=16 reserved(1)=0 asvc(1)=0 bsmod(3)=0 acmod(3)=7 lfeon(1)=1
    // reserved(3)=0 num_dep_sub(4)=0 reserved(1)=0
    const std::uint32_t sub = (0U << 30U) | (16U << 25U) | (0U << 24U) | (0U << 23U) |
        (0U << 20U) | (7U << 17U) | (1U << 16U) | (0U << 13U) | (0U << 9U) | 0U;
    dec3_payload.push_back(static_cast<std::uint8_t>(sub >> 16U));
    dec3_payload.push_back(static_cast<std::uint8_t>(sub >> 8U));
    dec3_payload.push_back(static_cast<std::uint8_t>(sub));
    const auto dec3 = full_box("dec3", 0, 0, dec3_payload);

    std::vector<std::uint8_t> sample_entry;
    put32(sample_entry, 0U); put16(sample_entry, 0U); put16(sample_entry, 0U); // reserved
    put16(sample_entry, 1U);                       // data reference index
    put32(sample_entry, 0U); put32(sample_entry, 0U); // version/revision/vendor
    put16(sample_entry, 6U);                       // channel count
    put16(sample_entry, 16U);                      // sample size
    put16(sample_entry, 0U);                       // pre-defined
    put16(sample_entry, 0U);                       // reserved
    put32(sample_entry, timescale << 16U);         // sample rate 16.16
    sample_entry.insert(sample_entry.end(), dec3.begin(), dec3.end());
    std::vector<std::uint8_t> stsd_payload;
    put32(stsd_payload, 1U);
    box(stsd_payload, "ec-3", sample_entry);
    const auto stsd = full_box("stsd", 0, 0, stsd_payload);

    std::vector<std::uint8_t> stts_payload;
    put32(stts_payload, 1U);
    put32(stts_payload, static_cast<std::uint32_t>(sizes.size()));
    put32(stts_payload, samples_per_frame);
    const auto stts = full_box("stts", 0, 0, stts_payload);

    std::vector<std::uint8_t> stsc_payload;
    put32(stsc_payload, 1U);
    put32(stsc_payload, 1U);                       // first chunk
    put32(stsc_payload, static_cast<std::uint32_t>(sizes.size())); // samples per chunk
    put32(stsc_payload, 1U);                       // sample description index
    const auto stsc = full_box("stsc", 0, 0, stsc_payload);

    std::vector<std::uint8_t> stsz_payload;
    put32(stsz_payload, 0U);                       // varying sizes
    put32(stsz_payload, static_cast<std::uint32_t>(sizes.size()));
    for (const auto size : sizes) put32(stsz_payload, size);
    const auto stsz = full_box("stsz", 0, 0, stsz_payload);

    std::vector<std::uint8_t> stco_payload;
    put32(stco_payload, 1U);
    put32(stco_payload, mdat_offset);
    const auto stco = full_box("stco", 0, 0, stco_payload);

    std::vector<std::uint8_t> stbl_payload;
    for (const auto* part : {&stsd, &stts, &stsc, &stsz, &stco})
        stbl_payload.insert(stbl_payload.end(), part->begin(), part->end());
    std::vector<std::uint8_t> stbl;
    box(stbl, "stbl", stbl_payload);

    std::vector<std::uint8_t> minf_payload;
    minf_payload.insert(minf_payload.end(), smhd.begin(), smhd.end());
    minf_payload.insert(minf_payload.end(), dinf.begin(), dinf.end());
    minf_payload.insert(minf_payload.end(), stbl.begin(), stbl.end());
    std::vector<std::uint8_t> minf;
    box(minf, "minf", minf_payload);

    std::vector<std::uint8_t> mdia_payload;
    mdia_payload.insert(mdia_payload.end(), mdhd.begin(), mdhd.end());
    mdia_payload.insert(mdia_payload.end(), hdlr.begin(), hdlr.end());
    mdia_payload.insert(mdia_payload.end(), minf.begin(), minf.end());
    std::vector<std::uint8_t> mdia;
    box(mdia, "mdia", mdia_payload);

    std::vector<std::uint8_t> trak_payload;
    trak_payload.insert(trak_payload.end(), tkhd.begin(), tkhd.end());
    trak_payload.insert(trak_payload.end(), mdia.begin(), mdia.end());
    std::vector<std::uint8_t> trak;
    box(trak, "trak", trak_payload);

    std::vector<std::uint8_t> moov_payload;
    moov_payload.insert(moov_payload.end(), mvhd.begin(), mvhd.end());
    moov_payload.insert(moov_payload.end(), trak.begin(), trak.end());
    std::vector<std::uint8_t> moov;
    box(moov, "moov", moov_payload);

    output.write(reinterpret_cast<const char*>(moov.data()), static_cast<std::streamsize>(moov.size()));
    output.close();
    if (!output) {
        error = "failed while writing " + mp4_path.string();
        return false;
    }
    return true;
}

} // namespace atmos::encode

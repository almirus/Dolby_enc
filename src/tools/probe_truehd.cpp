// Walks a demuxed TrueHD/MLP elementary stream: reports the major-sync stream
// info, the Atmos (OAMD) metadata it carries, and how many access units the
// in-tree decoder reconstructs. Plan M1 acceptance aid.
#include "truehd/access_unit.hpp"
#include "truehd/decoder.hpp"

#include <cstdint>
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

std::string channel_list(const std::vector<atmos::ChannelId>& channels) {
    std::string out;
    for (std::size_t index = 0; index < channels.size(); ++index) {
        if (index != 0U) out += " ";
        out += atmos::channel_name(channels[index]);
    }
    return out;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: probe-truehd <stream.thd> [max_au]\n";
        return 2;
    }
    const auto data = read_file(argv[1]);
    const std::size_t limit = argc > 2 ? std::strtoull(argv[2], nullptr, 10)
        : (data.size() / 4U);
    std::cout << "file=" << argv[1] << " size=" << data.size() << "\n";

    atmos::truehd::BedDecoder decoder(2U);
    atmos::truehd::BedDecoder object_decoder(3U);
    atmos::truehd::MajorSync info{};
    bool have_info = false;
    std::size_t offset = 0;
    std::size_t access_units = 0;
    std::size_t decoded = 0;
    std::size_t atmos_units = 0;
    std::size_t audio_samples = 0;
    std::string last_error;
    std::size_t audio_objects = 0;
    std::size_t audio_object_count = 0;
    double object_peak = 0.0;
    double bed_peak = 0.0;

    while (offset + 8U <= data.size() && access_units < limit) {
        const auto unit = atmos::truehd::parse_access_unit(data,
            offset, have_info ? &info : nullptr);
        if (!unit) { offset += 2U; continue; }
        if (unit->major_sync && !have_info) {
            info = *unit->major_sync;
            have_info = true;
            std::cout << "major sync: rate=" << info.sample_rate
                      << " channels=" << info.channels.size()
                      << " (" << channel_list(info.channels) << ")"
                      << " substreams=" << unsigned(info.substream_count)
                      << " objects_only=" << info.objects_only
                      << " atmos_hint=" << info.atmosphere_hint << "\n";
            for (std::size_t index = 0; index < unit->substreams.size(); ++index) {
                const auto& entry = unit->substreams[index];
                std::cout << "  substream " << index << ": size=" << entry.data_size
                          << " restart=" << (entry.restart_header ? "yes" : "no")
                          << " object=" << (entry.restart_header ?
                              entry.restart_header->object_substream : false) << "\n";
            }
        }
        bool has_oamd = false;
        for (const auto& metadata : unit->atmos_metadata) {
            if (!metadata.has_oamd) continue;
            has_oamd = true;
            audio_object_count = metadata.oamd_objects;
            audio_objects += metadata.oamd.position_updates.size();
        }
        if (has_oamd) ++atmos_units;
        atmos::truehd::PcmFrame pcm;
        std::string error;
        if (decoder.decode(std::span<const std::uint8_t>(data).subspan(
                offset, unit->size), pcm, error)) {
            ++decoded;
            audio_samples += pcm.samples_per_channel;
        } else {
            last_error = error;
        }
        if (access_units < 2000U) {
            atmos::truehd::PcmFrame objects;
            atmos::truehd::PcmFrame bed;
            std::string object_error;
            if (object_decoder.decode_with_presentation2(
                    std::span<const std::uint8_t>(data).subspan(offset, unit->size),
                    objects, bed, object_error)) {
                if (access_units == 0U)
                    std::cout << "presentation 3: objects=" << unsigned(objects.channels)
                              << " bed=" << unsigned(bed.channels)
                              << " samples=" << objects.samples_per_channel << "\n";
                for (float value : objects.interleaved)
                    object_peak = std::max(object_peak, static_cast<double>(std::abs(value)));
                for (float value : bed.interleaved)
                    bed_peak = std::max(bed_peak, static_cast<double>(std::abs(value)));
            } else if (access_units == 0U) {
                std::cout << "presentation 3 decode failed: " << object_error << "\n";
            }
        }
        offset += unit->size;
        ++access_units;
    }

    std::cout << "access units=" << access_units << " decoded=" << decoded
              << " atmos_units=" << atmos_units << "\n";
    if (audio_object_count != 0U) {
        std::cout << "OAMD: object_count=" << audio_object_count
                  << " position_updates=" << audio_objects << "\n";
    }
    std::cout << "decoded audio: " << audio_samples << " samples/channel ("
              << static_cast<double>(audio_samples) / 48000.0 << " s)\n";
    std::printf("presentation 3 peaks: objects=%.6f bed=%.6f\n", object_peak, bed_peak);
    if (decoded < access_units) std::cout << "last decode error: " << last_error << "\n";
    return 0;
}

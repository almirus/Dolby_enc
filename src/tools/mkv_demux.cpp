// Matroska demuxer CLI (plan M0): lists the tracks of an .mkv and writes the
// chosen elementary streams (TrueHD / E-AC-3 / AC-3) to disk.
#include "mkv/demuxer.hpp"

#include <cstdint>
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

bool write_file(const std::filesystem::path& path, std::span<const std::uint8_t> data) {
    std::ofstream stream(path, std::ios::binary);
    if (!stream) return false;
    stream.write(reinterpret_cast<const char*>(data.data()),
        static_cast<std::streamsize>(data.size()));
    return stream.good();
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: mkv-demux <in.mkv> [outdir]\n"
                     "  writes track1_truehd.thd, track2_ac3.ac3, track3_eac3.ec3\n"
                     "  for the audio tracks it finds\n";
        return 2;
    }
    const std::filesystem::path input = argv[1];
    const std::filesystem::path outdir = argc > 2 ? argv[2] : input.parent_path();
    std::filesystem::create_directories(outdir);

    const auto data = read_file(input);
    atmos::mkv::Demuxer demuxer(data);
    std::cout << "tracks in " << input.string() << ":\n";
    for (const auto& track : demuxer.tracks()) {
        std::cout << "  #" << track.number << " type=" << track.type
                  << " codec=" << track.codec_id;
        if (track.type == 2U)
            std::cout << " " << track.channels << "ch " << track.sample_rate << "Hz";
        std::cout << "\n";
    }

    int written = 0;
    for (const auto& track : demuxer.tracks()) {
        std::filesystem::path target;
        if (atmos::mkv::Demuxer::is_truehd(track))
            target = outdir / ("track" + std::to_string(track.number) + "_truehd.thd");
        else if (atmos::mkv::Demuxer::is_eac3(track))
            target = outdir / ("track" + std::to_string(track.number) + "_eac3.ec3");
        else if (atmos::mkv::Demuxer::is_ac3(track))
            target = outdir / ("track" + std::to_string(track.number) + "_ac3.ac3");
        else
            continue;

        std::vector<std::uint8_t> output;
        std::string error;
        if (!demuxer.extract(track.number, output, error)) {
            std::cerr << "  track " << track.number << ": " << error << "\n";
            continue;
        }
        if (!write_file(target, output)) {
            std::cerr << "  cannot write " << target.string() << "\n";
            continue;
        }
        std::cout << "  wrote " << target.string() << " (" << output.size() << " bytes)\n";
        ++written;
    }
    return written > 0 ? 0 : 1;
}

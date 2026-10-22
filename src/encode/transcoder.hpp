#pragma once

#include "model/oamd.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace atmos::encode {

struct Options final {
    std::string codec = "eac3";
    std::string input_layout; // explicit speaker order for a maskless TrueHD PCM input
    std::string input;   // WAV, or raw/MP4/M4A E-AC-3 Atmos for object export. Empty -> synthesise.
    std::string truehd;  // TrueHD/MLP elementary stream or MKV; exclusive with input
    std::string output;  // target .ec3/.thd/container; empty -> export objects only
    std::string dump_bed; // optional: write the rendered 5.1 carrier as WAV
    std::string objects_output_dir; // optional: source bed/stems and object-viewer sidecars
    std::string bed = "5.1";
    std::string fast_atmos; // fixed 5.1.2/5.1.4/7.1.4 output scene (TrueHD only)
    std::uint32_t bitrate = 640000;
    bool bitrate_explicit = false;
    unsigned audio_track = 1;       // one-based audio ordinal in a container
    bool atmos = false;               // opt-in research OAMD/JOC; not production Atmos
    bool with_joc = true;
    bool spdif = false;               // write IEC 61937 bursts instead of a raw .ec3
    bool mkv = false;                 // write a Matroska file instead of a raw .ec3
    bool mp4 = false;                 // write an MP4 file instead of a raw .ec3
    std::vector<model::ObjectPlacement> objects;
    double seconds = 2.0;            // length of the synthesised bed
    double truehd_seconds = 0.0;    // input duration limit; 0 encodes the whole stream
    std::function<void(const char*, int)> progress; // optional stage/percent callback; -1 = unknown
};

// Encodes E-AC-3/TrueHD and/or exports source objects. Returns the frame/AU count (0 for export only), or
// a negative value with `error` set.
int run(const Options& options, std::string& error);

} // namespace atmos::encode

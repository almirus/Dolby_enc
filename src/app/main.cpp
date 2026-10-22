#include "encode/transcoder.hpp"
#include "progress.hpp"
#include "version.hpp"

#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void usage() {
    std::cout << "ddp-atmos " << atmos::app::version << " - E-AC-3 / TrueHD Atmos encoder\n"
        "Usage: ddp-atmos -i input [-o file.ec3] [--objects-output-dir path] [options]\n"
        "  -o, --output <file>       encoded output (optional when exporting objects)\n"
        "      --codec <name>        eac3 (default) or experimental truehd CBI\n"
        "      --input-layout <name> TrueHD WAV: 5.1.2/5.1.4/7.1.4; required without mask\n"
        "                           no mask: L,R,C,LFE,Ls,Rs,Lb,Rb,height pairs as applicable\n"
        "  -i, --input <file>        WAV, TrueHD/MLP, E-AC-3, MKV, MP4/M4A\n"
        "                           no mask: 5.1/7.1/5.1.4/7.1.4/9.1.6\n"
        "                           independent 5.1 E-AC-3 Atmos for object export without -o\n"
        "      --audio-track <N>     audio track number, starting at 1 (default 1)\n"
        "      --duration <TIME>     Decode only this duration: 10s, 10m, 1h2m5s\n"
        "      --objects-output-dir <path>  export source bed/objects for object-viewer\n"
        "                                  TrueHD/E-AC-3 Atmos; new directory\n"
        "  -b, --bitrate <bps>       total bitrate (default 640000)\n"
        "      --bed <layout>        5.1 (default)\n"
        "      --no-atmos            plain E-AC-3 (default)\n"
        "      --atmos               Atmos with recovered JOC analysis\n"
        "      --fast-atmos <layout> render TrueHD to fixed 5.1.2/5.1.4/7.1.4\n"
        "                           replaces source objects with a channel bed\n"
        "      --spdif               write IEC 61937 bursts instead of a raw .ec3\n"
        "      --mkv                 wrap E-AC-3 or TrueHD output in Matroska\n"
        "      --mp4                 write an MP4 file instead of a raw .ec3\n"
        "      --version             show version\n"
        "  -h, --help                show help\n"
        "Existing output files are never overwritten.\n";
}

double duration(const std::string& text) {
    double total=0;
    std::size_t cursor=0;
    unsigned previous=3601;
    while (cursor<text.size()) {
        const auto begin=cursor;
        while (cursor<text.size() && ((text[cursor]>='0' && text[cursor]<='9') || text[cursor]=='.')) ++cursor;
        double value=0;
        const auto parsed=std::from_chars(text.data()+begin,text.data()+cursor,value);
        if (begin==cursor || cursor==text.size() || parsed.ec!=std::errc{} || parsed.ptr!=text.data()+cursor)
            throw std::runtime_error("invalid duration: "+text+" (use 10s, 10m, 1h2m5s)");
        const char unit=text[cursor++];
        const unsigned scale=unit=='h' ? 3600 : unit=='m' ? 60 : unit=='s' ? 1 : 0;
        if (!scale || scale>=previous || !std::isfinite(value))
            throw std::runtime_error("invalid duration: "+text);
        previous=scale;
        total+=value*scale;
    }
    if (!std::isfinite(total) || total<=0 || total>86400)
        throw std::runtime_error("duration must be greater than 0 and at most 24h");
    return total;
}
} // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    if (argc == 1) { usage(); return 0; }
    atmos::encode::Options options;
    try {
        for (int index = 1; index < argc; ++index) {
            const std::string arg = argv[index];
            const auto next = [&]() -> std::string {
                if (index + 1 >= argc) throw std::runtime_error("missing value for " + arg);
                return argv[++index];
            };
            if (arg == "-h" || arg == "--help") { usage(); return 0; }
            if (arg == "--version") { std::cout << "ddp-atmos " << atmos::app::version << '\n'; return 0; }
            if (arg == "-o" || arg == "--output") options.output = next();
            else if (arg == "-i" || arg == "--input") options.input = next();
            else if (arg == "--codec") options.codec = next();
            else if (arg == "--input-layout") options.input_layout = next();
            else if (arg == "--dump-bed") options.dump_bed = next();
            else if (arg == "--objects-output-dir") options.objects_output_dir = next();
            else if (arg == "--duration") options.truehd_seconds = duration(next());
            else if (arg == "--audio-track") {
                const auto value=next();
                const auto parsed=std::from_chars(value.data(),value.data()+value.size(),options.audio_track);
                if (parsed.ec!=std::errc{} || parsed.ptr!=value.data()+value.size() || !options.audio_track)
                    throw std::runtime_error("invalid audio track: "+value+" (starting at 1)");
            }
            else if (arg == "--bed") options.bed = next();
            else if (arg == "--fast-atmos") {
                options.fast_atmos = next(); options.atmos = true; options.with_joc = true;
            }
            else if (arg == "--no-atmos") options.atmos = false;
            else if (arg == "--atmos") { options.atmos = true; options.with_joc = true; }
            else if (arg == "--spdif") options.spdif = true;
            else if (arg == "--mkv") options.mkv = true;
            else if (arg == "--mp4") options.mp4 = true;
            else if (arg == "-b" || arg == "--bitrate") {
                options.bitrate_explicit = true;
                const auto value = next();
                const auto parsed = std::from_chars(value.data(), value.data() + value.size(), options.bitrate);
                if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
                    throw std::runtime_error("invalid bitrate: " + value);
            } else throw std::runtime_error("unknown option: " + arg);
        }
        if (options.output.empty() && options.objects_output_dir.empty())
            throw std::runtime_error("--output or --objects-output-dir is required");
        if (options.input.empty()) throw std::runtime_error("--input is required");
    } catch (const std::exception& exception) {
        std::cerr << "error: " << exception.what() << "\n";
        return 2;
    }
    std::cerr << "ddp-atmos " << atmos::app::version << '\n';
    if (options.codec=="truehd")
        std::cerr << "Experimental TrueHD Atmos encoding with static channels.\n";
    else if (options.atmos && !options.output.empty())
        std::cerr << "Experimental JOC reconstruction; full object fidelity is still under validation.\n";
    std::string error;
    atmos::console_style::enable_virtual_terminal();
    atmos::ProgressReporter progress;
    options.progress = [&](const char* stage, int percent) { progress.update(stage,percent); };
    const int frames = atmos::encode::run(options, error);
    progress.finish();
    if (frames < 0) {
        std::cerr << "error: " << error << "\n";
        return 1;
    }
    if (!options.output.empty()) {
        std::cout << "wrote " << options.output << ": " << frames
            << (options.codec=="truehd" ? " TrueHD access units (CBI)" : " E-AC-3 syncframes");
        if (options.atmos && options.codec=="eac3")
            std::cout << (options.with_joc ? " (research OAMD + JOC)" : " (OAMD only)");
        std::cout << "\n";
    }
    if (!options.objects_output_dir.empty()) std::cout << "wrote objects: " << options.objects_output_dir << "\n";
    return 0;
}

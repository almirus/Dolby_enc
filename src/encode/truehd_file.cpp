#include "truehd_file.hpp"

#include "common/pcm_wav_reader.hpp"
#include "truehd/encoder.hpp"
#include "mkv_writer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace atmos::encode {

int encode_truehd_file(const Options& options, std::string& error) {
    const std::filesystem::path temporary=options.output+".thd.part";
    const std::filesystem::path mux_temporary=options.output+".mkv.part";
    bool owned=false;
    bool mux_owned=false;
    try {
        if(options.input.empty() || !options.truehd.empty() || options.output.empty() || !options.atmos)
            throw std::runtime_error("TrueHD output requires WAV input, --atmos and --output");
        if(options.spdif || options.mp4 || !options.fast_atmos.empty() ||
            !options.objects_output_dir.empty() || !options.dump_bed.empty() || !options.objects.empty() ||
            options.audio_track!=1 || options.bitrate_explicit || options.bitrate!=640000)
            throw std::runtime_error("TrueHD writes raw .thd or --mkv; DD+ bitrate/MP4/SPDIF/object options are unsupported");
        if(!std::isfinite(options.truehd_seconds) || options.truehd_seconds<0 || options.truehd_seconds>86400)
            throw std::runtime_error("invalid TrueHD input duration");
        if(std::filesystem::exists(options.output) || std::filesystem::exists(temporary) ||
            (options.mkv && std::filesystem::exists(mux_temporary)))
            throw std::runtime_error("TrueHD output or temporary file already exists");
        PcmWavReader input(options.input,options.input_layout);
        truehd::Encoder encoder(input.channel_ids());
        auto remaining=input.frames();
        if(options.truehd_seconds>0) remaining=std::min(remaining,
            static_cast<std::uint64_t>(options.truehd_seconds*input.sample_rate()));
        if(!remaining)throw std::runtime_error("TrueHD input duration contains no complete sample frames");
        if((remaining+39)/40>static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
            throw std::runtime_error("TrueHD input exceeds supported access-unit count");
        const auto total=remaining;
        std::FILE* file=nullptr;
#ifdef _WIN32
        _wfopen_s(&file,temporary.c_str(),L"wbx");
#else
        file=std::fopen(temporary.c_str(),"wbx");
#endif
        if(!file)throw std::runtime_error("cannot exclusively create TrueHD temporary output");
        std::unique_ptr<std::FILE,decltype(&std::fclose)> output(file,&std::fclose);
        owned=true;
        int units=0,previous=-1;
        while(remaining) {
            const auto count=static_cast<std::size_t>(std::min<std::uint64_t>(40,remaining));
            const auto samples=input.read(count);
            if(samples.size()!=count*input.channel_ids().size())
                throw std::runtime_error("unexpected PCM WAV EOF");
            const auto bytes=encoder.encode(samples,count==remaining);
            if(std::fwrite(bytes.data(),1,bytes.size(),output.get())!=bytes.size())
                throw std::runtime_error("cannot write TrueHD output");
            remaining-=count;
            ++units;
            const auto percent=static_cast<int>((total-remaining)*100/total);
            if(options.progress && percent!=previous) {
                options.progress("TrueHD",percent);previous=percent;
            }
        }
        if(std::fclose(output.release())!=0)throw std::runtime_error("cannot finish TrueHD output");
        if(options.mkv) {
            mux_owned=true;
            if(!write_truehd_matroska(temporary,mux_temporary,total,
                    static_cast<unsigned>(input.channel_ids().size()),error))
                throw std::runtime_error(error);
        }
        // Publish a completed file without replacing a target created during
        // encoding. Staging on the same volume avoids a cross-volume copy.
        const auto& staged=options.mkv ? mux_temporary : temporary;
#ifdef _WIN32
        if(!MoveFileExW(staged.c_str(),std::filesystem::path(options.output).c_str(),MOVEFILE_WRITE_THROUGH))
            throw std::filesystem::filesystem_error("cannot publish TrueHD output",staged,options.output,
                std::error_code(static_cast<int>(GetLastError()),std::system_category()));
#else
        std::filesystem::create_hard_link(staged,options.output);
#endif
        std::error_code ignored;
        if(mux_owned)std::filesystem::remove(mux_temporary,ignored);
        std::filesystem::remove(temporary,ignored);
        owned=false;
        mux_owned=false;
        error.clear();
        return units;
    } catch(const std::exception& exception) {
        if(owned) { std::error_code ignored;std::filesystem::remove(temporary,ignored); }
        if(mux_owned) { std::error_code ignored;std::filesystem::remove(mux_temporary,ignored); }
        error=exception.what();
        return -1;
    }
}

} // namespace atmos::encode

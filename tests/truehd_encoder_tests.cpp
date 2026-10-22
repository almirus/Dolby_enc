#include "truehd/encoder.hpp"
#include "truehd/decoder.hpp"
#include "truehd/access_unit.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }

int main(int argc,char** argv) {
    try {
        const std::string layout=argc==3 ? argv[2] : "7.1.4";
        const std::vector<std::uint8_t> ids=layout=="5.1.2" ? std::vector<std::uint8_t>{0,1,2,3,4,5,10,11} :
            layout=="5.1.4" ? std::vector<std::uint8_t>{0,1,2,3,4,5,8,9,12,13} :
            std::vector<std::uint8_t>{0,1,2,3,4,5,6,7,8,9,12,13};
        const auto channels=static_cast<unsigned>(ids.size());
        atmos::truehd::Encoder encoder(ids);
        atmos::truehd::BedDecoder decoder(3);
        std::ofstream encoded,source;
        if(argc>=2 && std::string(argv[1])!="--layout") {
            const auto prefix=std::filesystem::path(argv[1]);
            check(!std::filesystem::exists(prefix.string()+".thd") &&
                  !std::filesystem::exists(prefix.string()+".s24"),"fixture already exists");
            encoded.open(prefix.string()+".thd",std::ios::binary);
            source.open(prefix.string()+".s24",std::ios::binary);
            check(bool(encoded) && bool(source),"cannot create fixtures");
        }
        std::uint32_t random=1234;
        // More than one 16-bit timestamp wrap and many restart boundaries.
        for(unsigned au=0;au<1800;++au) {
            std::vector<std::int32_t> pcm(40*channels);
            for(unsigned f=0;f<40;++f)for(unsigned c=0;c<channels;++c) {
                random=random*1664525U+1013904223U;
                auto sample=static_cast<std::int32_t>((random>>16)&1023U)-512;
                if(au<channels) sample=c==au ? (f%2 ? 8388607 : -8388608) : 0;
                if(au>=12 && au<48) sample=static_cast<std::int32_t>(random>>8)-8388608;
                if(au%17==0 && au!=0)sample=0;
                pcm[f*channels+c]=sample;
            }
            const auto bytes=encoder.encode(pcm,au==1799);
            if(au%16==0) {
                // A 10-bit group mask must advertise standard assignment;
                // otherwise readers consume 17 bits and invent a different bed.
                check(bytes.size()>33 && (bytes[33]&8U),"standard bed mask flag missing");
                const auto parsed=atmos::truehd::parse_access_unit(bytes,0);
                check(parsed && parsed->major_sync,"major sync missing");
                check(parsed->major_sync->channel_based_bed_ids==ids,"CBI channel assignment");
                check(parsed->major_sync->channels.size()==8,"7.1 legacy assignment");
            }
            atmos::truehd::PcmFrame frame;std::string error;
            check(decoder.decode(bytes,frame,error),error.c_str());
            check(frame.channels==channels && frame.samples_per_channel==40,"decoded geometry");
            for(unsigned i=0;i<pcm.size();++i)
                check(frame.interleaved[i]*8388608.0F==static_cast<float>(pcm[i]),"PCM changed");
            if(encoded.is_open()) {
                encoded.write(reinterpret_cast<const char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
                for(const auto sample:pcm)for(unsigned i=0;i<3;++i)
                    source.put(static_cast<char>(static_cast<std::uint32_t>(sample)>>(8*i)));
                check(bool(encoded) && bool(source),"fixture write failed");
            }
        }
        bool rejected=false;
        try { static_cast<void>(encoder.encode(std::vector<std::int32_t>(480),true)); }
        catch(const std::runtime_error&) { rejected=true; }
        check(rejected,"encode after final accepted");
        for(const unsigned tail:{1U,7U,8U,39U,40U}) {
            atmos::truehd::Encoder short_encoder(ids);
            const auto bytes=short_encoder.encode(std::vector<std::int32_t>(tail*channels),true);
            check(!bytes.empty(),"short stream failed");
            atmos::truehd::BedDecoder short_decoder(3);
            atmos::truehd::PcmFrame frame;std::string error;
            check(short_decoder.decode(bytes,frame,error),error.c_str());
            check(frame.samples_per_channel==tail,"EOS padding changed source duration");
        }
        // Shared signals and reversible downmix matrices must also preserve
        // simultaneous full-scale samples without exceeding the peak ceiling.
        std::vector<std::int32_t> loud(40*channels);
        for(unsigned i=0;i<loud.size();++i) loud[i]=i%2 ? 8388607 : -8388608;
        atmos::truehd::Encoder limited(ids);
        check(!limited.encode(loud,true).empty(),"full-scale rematrix failed");
        std::cout<<"PASS: CBI PCM24 round trip, channel assignment, restart, timestamp wrap, EOS syntax and peak guard\n";
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 1;
    }
}

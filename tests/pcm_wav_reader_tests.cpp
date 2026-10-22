#include "common/pcm_wav_reader.hpp"

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

using Bytes = std::vector<std::uint8_t>;
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void put(Bytes& b, std::uint64_t value, unsigned count) {
    for (unsigned i=0;i<count;++i) b.push_back(static_cast<std::uint8_t>(value>>(8*i)));
}
void tag(Bytes& b, const char* text) { for (unsigned i=0;i<4;++i) b.push_back(text[i]); }
void patch(Bytes& b, std::size_t at, std::uint64_t value, unsigned n) {
    for (unsigned i=0;i<n;++i) b[at+i]=static_cast<std::uint8_t>(value>>(8*i));
}
Bytes wave(unsigned channels, unsigned bits, std::uint32_t mask, bool rf64=false,
           unsigned valid=0) {
    const bool ext=mask!=0 || valid!=0;
    Bytes b;
    tag(b,rf64?"RF64":"RIFF");put(b,rf64?0xffffffffU:0,4);tag(b,"WAVE");
    if(rf64) { tag(b,"ds64");put(b,28,4);put(b,0,8);put(b,0,8);put(b,3,8);put(b,0,4); }
    tag(b,"fmt ");put(b,ext?40:16,4);put(b,ext?0xfffe:1,2);put(b,channels,2);
    put(b,48000,4);put(b,48000*channels*(bits/8),4);put(b,channels*(bits/8),2);put(b,bits,2);
    if(ext) {
        put(b,22,2);put(b,valid?valid:bits,2);put(b,mask,4);
        const std::uint8_t guid[]{1,0,0,0,0,0,16,0,128,0,0,170,0,56,155,113};
        b.insert(b.end(),std::begin(guid),std::end(guid));
    }
    // Odd-sized unknown chunks must not disturb PCM alignment.
    tag(b,"JUNK");put(b,1,4);put(b,42,1);put(b,0,1);
    tag(b,"data");put(b,rf64?0xffffffffU:3*channels*(bits/8),4);
    for(unsigned f=0;f<3;++f) for(unsigned c=0;c<channels;++c) {
        const auto value=f==0 ? (1U<<((valid?valid:bits)-1)) :
            f==1 ? (1U<<((valid?valid:bits)-1))-1 : c+1;
        put(b,std::uint64_t(value)<<(valid?bits-valid:0),bits/8);
    }
    if(b.size()%2)b.push_back(0);
    if(rf64) { patch(b,20,b.size()-8,8);patch(b,28,3*channels*(bits/8),8); }
    else patch(b,4,b.size()-8,4);
    return b;
}

int main() {
    const auto file=std::filesystem::current_path()/
        ("truehd-pcm-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".wav");
    if(std::filesystem::exists(file)) return 1;
    const auto save=[&](const Bytes& b) {
        std::ofstream out(file,std::ios::binary|std::ios::trunc);
        out.write(reinterpret_cast<const char*>(b.data()),static_cast<std::streamsize>(b.size()));
        require(bool(out),"fixture write failed");
    };
    const auto rejects=[&](const Bytes& b, std::string_view layout) {
        save(b);
        bool rejected=false;
        try { atmos::PcmWavReader reader(file,layout); static_cast<void>(reader.read(10)); }
        catch(const std::runtime_error&) { rejected=true; }
        require(rejected,"malformed/ambiguous input accepted");
    };
    try {
        for(const unsigned bits:{16U,24U})for(const bool rf64:{false,true}) {
            save(wave(12,bits,0x2d63f,rf64));
            atmos::PcmWavReader reader(file,"7.1.4");
            require(reader.frames()==3 && reader.bit_depth()==bits,"PCM properties");
            const auto first=reader.read(1);
            require(first.size()==12 && first[0]==-8388608,"negative full scale");
            const auto rest=reader.read(10);
            require(rest.size()==24 && rest[0]==8388608-(bits==16?256:1),"positive full scale");
            require(rest[12+4]==7*(bits==16?256:1) && rest[12+6]==5*(bits==16?256:1),"side/rear permutation");
            require(reader.read(1).empty(),"EOF");
        }
        save(wave(8,24,0));
        { atmos::PcmWavReader reader(file,"5.1.2");
          require(reader.channel_ids()[6]==10 && reader.channel_ids()[7]==11,"explicit middle heights"); }
        save(wave(12,32,0x2d63f,false,24));
        { atmos::PcmWavReader reader(file);
          require(reader.read(1)[0]==-8388608,"24 valid bits in 32-bit container"); }
        rejects(wave(8,24,0),{});
        rejects(wave(8,24,0),"7.1.4");
        rejects(wave(12,24,0x2d63f),"5.1.4");
        rejects(wave(12,32,0x2d63f),"7.1.4");
        auto b=wave(12,24,0x2d63f);b.pop_back();rejects(b,"7.1.4");
        b=wave(12,24,0x2d63f);patch(b,44,3,4);rejects(b,"7.1.4"); // invalid PCM GUID
        b=wave(12,24,0x2d63f);patch(b,32,0,2);rejects(b,"7.1.4"); // block align
        b=wave(12,24,0x2d63f);patch(b,40,1,4);rejects(b,"7.1.4"); // mask/count
        b=wave(12,24,0x2d63f,true);patch(b,36,4,8);rejects(b,"7.1.4"); // ds64 samples
        b=wave(12,24,0x2d63f,true);patch(b,28,400,8);patch(b,110,432,4);
        rejects(b,"7.1.4"); // explicit data size must still agree with ds64
        b=wave(12,32,0x2d63f,false,24);b[b.size()-4]|=1;rejects(b,"7.1.4"); // nonzero padding
        b=wave(12,24,0x2d63f);patch(b,24,96000,4);rejects(b,"7.1.4");
        std::filesystem::remove(file);
        std::cout<<"PASS: integer PCM16/24, RF64, 24-in-32, masks, layout, channel order, bounds and EOF\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<"; fixture retained at "<<file<<'\n';
        return 1;
    }
}

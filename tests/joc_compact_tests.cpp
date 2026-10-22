#include "encode/joc_compact.hpp"
#include "common/bit_writer.hpp"
#include "eac3/audio_block.hpp"
#include "eac3/audio_frame.hpp"
#include "eac3/joc_decoder.hpp"
#include <filesystem>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

void check(atmos::eac3::JocFrame frame){
    const auto before=frame;const auto size=atmos::encode::write_joc_payload(frame).size();atmos::encode::joc::compact(frame);
    const auto bits=atmos::encode::joc_payload_bit_count(frame);const auto packed=atmos::encode::write_joc_payload(frame);
    if((bits+7)/8!=packed.size() || (bits%8 && (packed.back()&((1U<<(8-bits%8))-1))))throw std::runtime_error("JOC bit count/padding mismatch");
    if(atmos::encode::write_joc_payload(frame).size()>size)throw std::runtime_error("compaction grew payload");
    auto repeated=frame;atmos::encode::joc::compact(repeated);
    if(atmos::encode::write_joc_payload(repeated)!=atmos::encode::write_joc_payload(frame))throw std::runtime_error("compaction is not idempotent");
    const auto bytes=atmos::encode::build_emdf({},frame,false,true);const auto parsed=atmos::eac3::parse_emdf_frame(std::span(bytes).subspan(4));
    if(!parsed.valid || !parsed.has_joc)throw std::runtime_error("compacted roundtrip failed");
    for(unsigned o=0;o<frame.object_count;++o)if(atmos::encode::joc::expand(before.objects[o],frame.channels)!=atmos::encode::joc::expand(parsed.joc.objects[o],frame.channels))throw std::runtime_error("compacted roundtrip changed values");
    atmos::eac3::JocDecoder a,b;unsigned rng=115;
    for(unsigned block=0;block<3;++block){std::vector<float> input(1536*(frame.channels+1));for(auto& x:input){rng=rng*1664525U+1013904223U;x=float((double(rng>>8)/16777216-.5)*.02);}
        std::vector<float> x,y;std::string error;
        if(!a.render(before,input,frame.channels+1,1536,x,error) || !b.render(parsed.joc,input,frame.channels+1,1536,y,error) || (x.size()!=y.size() || std::memcmp(x.data(),y.data(),x.size()*sizeof(float))!=0))throw std::runtime_error("compaction changed decoded PCM");
    }
}
void selftest(){
    unsigned count=0;
    for(unsigned channels:{5U,7U})for(unsigned points:{1U,2U})for(bool fine:{false,true})for(bool sparse:{false,true})for(unsigned grid=0;grid<8;++grid){
        atmos::eac3::JocFrame f;f.channels=static_cast<std::uint8_t>(channels);f.object_count=2;
        for(unsigned object=0;object<2;++object){auto& o=f.objects[object];o.active=true;o.bands_index=static_cast<std::uint8_t>(grid);o.bands=atmos::eac3::joc_tables::band_count[grid];o.data_points=static_cast<std::uint8_t>(points);o.fine_quantization=fine;o.sparse=sparse;o.steep_slope=object;o.timeslot_offsets={8,19};
            const int modulus=fine?192:96,center=modulus/2;
            for(unsigned p=0;p<points;++p){unsigned channel=0;int prev=fine?100:50;std::array<int,7> old{};
                for(unsigned b=0;b<o.bands;++b){if(sparse){const unsigned next=b%channels;const int code=b%3 ? (p?center-1:-center):0;o.sparse_channel[p][b]=static_cast<std::uint8_t>(b?(next+channels-channel)%channels:next);o.sparse_vector[p][b]=static_cast<std::uint8_t>((code+center-prev+modulus)%modulus);channel=next;prev=code+center;}
                    else for(unsigned c=0;c<channels;++c){const int code=object?0:(p?2:4)*(int(c)-2);o.matrix[p][c][b]=static_cast<std::uint8_t>((code-old[c]+modulus)%modulus);old[c]=code;}
                }
            }
        }check(f);++count;
    }
    // Real R109 frame4: byte-rounded costs miss savings until a second pass.
    const std::uint8_t regression[] = {7,8,0,19,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,22,32,0,0,0,0,0,0,0,0,49,98,0,0,0,0,0,0,0,0,3,5,128,0,0,0,0,0,0,0,0,244,176,44,0,0,0,0,0,0,0,3,221,81,98,192,0,0,0,0,123,160,0,0,0,0,0,0,0,0,61,208,176,30,248,0,0,0,0,0,0,0,0,30,248,0,0,0,0,0,0,0,0,0,1,235,180,0,0,0,0,0,0,0,0,61,135,189,0,0,0,0,0,7,189,101,69,64,0,0,0,0,0,0,0,121,194,80,247,192,0,0,0,0,0,0,0,0,247,192,0,0,0,0,0};
    atmos::BitWriter wrapper;wrapper.write(0,5);wrapper.write(14,5);wrapper.write(0,4);wrapper.write_bit(true);
    wrapper.write(sizeof(regression),8);wrapper.write_bit(false);for(auto b:regression)wrapper.write(b,8);
    wrapper.write(0,5);wrapper.write(1,2);wrapper.write(0,2);wrapper.write(0,8);wrapper.align();
    const auto fixture=atmos::eac3::parse_emdf_frame(wrapper.bytes());
    if(!fixture.valid || !fixture.has_joc)throw std::runtime_error("compaction regression fixture invalid");
    check(fixture.joc);++count;
    unsigned rng=119;
    for(unsigned trial=0;trial<64;++trial){
        atmos::eac3::JocFrame f;f.channels=5;f.object_count=16;
        for(auto& o:f.objects){o.active=true;o.bands_index=5;o.bands=12;o.fine_quantization=true;o.data_points=1;
            std::array<int,5> previous{};
            rng=rng*1664525U+1013904223U;const unsigned kind=rng%4;
            for(unsigned b=0;b<12;++b)for(unsigned c=0;c<5;++c){rng=rng*1664525U+1013904223U;const int code=kind==0?0:kind==1?(c==0?2:0):kind==2?2*(int(rng%3)-1):int(rng%9)-4;o.matrix[0][c][b]=static_cast<std::uint8_t>((code-previous[c]+192)%192);previous[c]=code;}
        }check(f);++count;
    }
    auto bad=atmos::eac3::JocFrame{};try{atmos::encode::joc::compact(bad);throw std::logic_error("invalid header accepted");}catch(const std::runtime_error&){}
    auto inactive=atmos::eac3::JocFrame{};inactive.channels=5;inactive.object_count=1;inactive.objects[0].active=false;
    if(atmos::encode::joc_payload_bit_count(inactive)!=31 || atmos::encode::write_joc_payload(inactive).size()!=4)throw std::runtime_error("inactive JOC header bit count");
    std::cout<<count<<" synthetic cases:5/7carriers,1/2points,fine/coarse,dense/sparse,8grids;3block decoder history bitexact;roundtrip and invalid header pass\n";
}

int main() {
    try { selftest(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}

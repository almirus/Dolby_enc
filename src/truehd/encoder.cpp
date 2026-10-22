#include "encoder.hpp"

#include "common/bit_writer.hpp"
#include "crypto/mat_dec_hmac.hpp"
#include "encode/emdf_writer.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <stdexcept>

namespace atmos::truehd {
namespace {

// TrueHD uses direct (non-augmented) CRCs: shift then insert each input bit.
std::uint32_t crc(const BitWriter& bits, unsigned first, unsigned count,
    unsigned width, std::uint32_t polynomial, std::uint32_t initial = 0) {
    auto result = initial;
    for (unsigned i=first;i<first+count;++i) {
        const auto top = result >> (width-1);
        result = ((result << 1) & ((1U << width)-1)) ^ (top ? polynomial : 0) ^
            ((bits.bytes()[i/8] >> (7-i%8)) & 1U);
    }
    return result;
}

std::vector<std::uint8_t> major(unsigned channels, std::uint16_t assignment) {
    BitWriter b;
    b.write(0xf8726fba,32);
    b.write(0,4); b.write(0,4); b.write(0,4); // 48 kHz, channel modifiers
    b.write(15,5); b.write(0,2); b.write(0x4f,13); // 5.1 / 7.1 speaker assignments
    b.write(0xb752,16); b.write(0x1000,16); b.write(0,16);
    b.write(1,1); b.write(6000,15); // VBR, 18 Mbit/s ceiling at 48 kHz
    b.write(4,4); b.write(0,2); b.write(3,2); b.write(0xfc,8);
    b.write(0,17); b.write(31,6); b.write(0,6); b.write(31,5); b.write(0,11);
    b.write(31,5); b.write(0,13); b.write(1,1);
    b.write(2,4); // 48-bit extra channel meaning (including its length)
    b.write(31,5); b.write(0,6); b.write(channels-1,5); b.write(0,1);
    b.write(1,4); b.write(0,3); b.write(1,1); // standard 10-bit bed assignment
    b.write(assignment,10); b.write(0,9);
    b.write(crc(b,0,static_cast<unsigned>(b.position()),16,0x2d),16);
    return std::move(b).take_bytes();
}

void be16(std::vector<std::uint8_t>& bytes, unsigned value) {
    bytes.push_back(static_cast<std::uint8_t>(value>>8));
    bytes.push_back(static_cast<std::uint8_t>(value));
}

struct Matrix {
    unsigned output;
    std::array<int,16> coefficients{};
    unsigned bypass_bits = 0;
};

using Bypass = std::array<std::array<unsigned,16>,40>;

std::vector<std::uint8_t> substream(std::span<const std::int32_t> pcm, unsigned channels,
    unsigned index, unsigned timing, bool restart, int trim, std::uint8_t& lossless,
    unsigned first, std::span<const Matrix> matrices, const Bypass& bypass,
    std::span<const std::int32_t> presented, std::span<const unsigned> assignment) {
    BitWriter b;
    b.write(1,1); b.write(restart,1);
    if (restart) {
        b.write(index == 3 ? 0x31ec : index == 2 ? 0x31eb : 0x31ea,14);
        b.write(timing & 0xffffU,16); b.write(first,4);
        b.write(channels-1,4); b.write(channels-1,4);
        b.write(0,4); b.write(0,23); b.write(0,4);
        b.write(24,5); b.write(24,5); b.write(24,5);
        b.write(0,1); b.write(lossless,8);
        b.write(1,1); b.write(0,15); // No serial high-resolution timing field.
        for (unsigned c=0;c<channels;++c) b.write(assignment.empty() ? c : assignment[c],6);
        b.write(crc(b,2,static_cast<unsigned>(b.position())-2,8,0x1d),8);
        lossless = 0;
    }
    // shortcut: fixed-width residuals; add prediction for better compression.
    std::array<unsigned,16> quant{}, width{};
    for (unsigned c=first;c<channels;++c) {
        std::uint32_t used = 0;
        for (unsigned f=0;f<40;++f) used |= static_cast<std::uint32_t>(pcm[f*channels+c]);
        quant[c] = std::min(15U,static_cast<unsigned>(std::countr_zero(used)));
        for (unsigned f=0;f<40;++f) {
            const auto value = pcm[f*channels+c] >> quant[c];
            const auto magnitude = static_cast<std::uint32_t>(value < 0 ? ~value : value);
            if (value) width[c] = std::max(width[c],std::bit_width(magnitude)+1U);
        }
    }
    if (restart) { b.write(1,1); b.write(0xd0,8); }
    b.write(restart,1);
    if (restart) b.write(40,9);
    b.write(restart && !matrices.empty(),1);
    if (restart && !matrices.empty()) {
        if (index==3) {
            b.write(1,1); b.write(1,1); // new matrix, new configuration
            b.write(static_cast<unsigned>(matrices.size())-1,4);
            for (const auto& matrix:matrices) {
                b.write(matrix.output,4); b.write(3,4); b.write(4,3);
                b.write(matrix.bypass_bits,2); b.write(0,4);
                unsigned mask=0;
                for (unsigned c=0;c<channels;++c) if(matrix.coefficients[c]) mask|=1U<<c;
                b.write(mask,channels);
            }
            for (const auto& matrix:matrices)
                for (unsigned c=0;c<channels;++c) if(matrix.coefficients[c])
                    b.write(static_cast<std::uint32_t>(matrix.coefficients[c]),5);
            b.write(0,1); // no coefficient interpolation
        } else {
            b.write(static_cast<unsigned>(matrices.size()),4);
            for (const auto& matrix:matrices) {
                b.write(matrix.output,4); b.write(0,4); b.write(matrix.bypass_bits!=0,1);
                for (unsigned c=0;c<channels+(index==2 ? 0 : 2);++c) {
                    const auto coefficient=matrix.coefficients[c];
                    b.write(coefficient!=0,1);
                    if(coefficient) b.write(static_cast<std::uint32_t>(coefficient),2);
                }
                if(index==2)b.write(0,4); // no matrix dither
            }
        }
    }
    b.write(1,1);
    for (unsigned c=0;c<channels;++c) b.write(quant[c],4);
    for (unsigned c=first;c<channels;++c) {
        b.write(1,1); b.write(0,2); b.write(width[c]+quant[c],5);
    }
    std::uint32_t check = 0;
    for (unsigned f=0;f<40;++f) {
        for (unsigned m=0;m<matrices.size();++m) b.write(bypass[f][m],matrices[m].bypass_bits);
        for (unsigned c=first;c<channels;++c) {
            const auto sample = pcm[f*channels+c];
            const auto offset = width[c] ? 1U << (width[c]-1) : 0;
            b.write(static_cast<std::uint32_t>(sample >> quant[c])+offset,width[c]);
        }
        for (unsigned c=0;c<channels;++c)
            check ^= (static_cast<std::uint32_t>(presented[f*channels+c]) & 0xffffffU) << (c&7U);
    }
    lossless ^= static_cast<std::uint8_t>(check ^ (check>>8) ^ (check>>16) ^ (check>>24));
    b.write(1,1);
    while (b.position()%16) b.write(0,1);
    if (trim >= 0) b.write(0xd234e000U | static_cast<unsigned>(trim),32);
    std::uint8_t parity = 0xa9;
    for (const auto byte : b.bytes()) parity ^= byte;
    const auto checksum = crc(b,0,static_cast<unsigned>(b.position()),8,0x63,0xa2);
    b.write(parity,8); b.write(checksum,8);
    return std::move(b).take_bytes();
}

} // namespace

Encoder::Encoder(std::span<const std::uint8_t> ids) : ids_(ids.begin(),ids.end()) {
    const std::vector<std::uint8_t> a{0,1,2,3,4,5,10,11},
        b{0,1,2,3,4,5,8,9,12,13}, c{0,1,2,3,4,5,6,7,8,9,12,13};
    if (ids_ != a && ids_ != b && ids_ != c)
        throw std::runtime_error("TrueHD CBI supports 5.1.2, 5.1.4 and 7.1.4");
    constexpr unsigned group_ids[]{0,2,3,4,6,8,10,12,14,16};
    for (unsigned g=0;g<std::size(group_ids);++g)
        if (std::find(ids_.begin(),ids_.end(),group_ids[g]) != ids_.end()) assignment_ |= 1U<<g;
    // The native plugin needs OAMD to expose/render the spatial presentation.
    // CBI channel meaning alone decodes but leaves its output layout unresolved.
    eac3::OamdFrame scene;
    scene.object_count=scene.bed_channels=static_cast<std::uint8_t>(ids_.size());
    scene.bed_channel_ids=ids_;
    const auto payload=encode::write_oamd_payload(scene);
    if(payload.size()>255)throw std::runtime_error("static TrueHD OAMD payload too large");
    BitWriter evo;
    evo.write(0,2); evo.write(0,3); evo.write(11,5);
    evo.write(0,1); evo.write(0,1); evo.write(0,1);
    evo.write(1,1); evo.write(8,8); evo.write(1,1);
    evo.write(static_cast<unsigned>(payload.size()),8); evo.write(0,1);
    for(const auto byte:payload)evo.write(byte,8);
    evo.write(0,5); evo.write(2,2); evo.write(0,2);
    protection_bit_=evo.position();
    evo.write(0,32); evo.align();
    evolution_=std::move(evo).take_bytes();
}

std::vector<std::uint8_t> Encoder::encode(std::span<const std::int32_t> input, bool final) {
    const auto channels = static_cast<unsigned>(ids_.size());
    if (finished_ || input.empty() || input.size()%channels || input.size()>40*channels ||
        (!final && input.size()!=40*channels))
        throw std::runtime_error("invalid TrueHD PCM block or encoder already finished");
    if (std::any_of(input.begin(),input.end(),[](auto x) { return x < -8388608 || x > 8388607; }))
        throw std::runtime_error("TrueHD PCM exceeds signed 24-bit range");
    const auto frames = static_cast<unsigned>(input.size()/channels);
    std::array<std::vector<std::int32_t>,4> pcm;
    constexpr unsigned counts[]{2,6,8};
    for (unsigned p=0;p<3;++p) pcm[p].resize(40*counts[p]);
    pcm[3].resize(40*channels);
    std::copy(input.begin(),input.end(),pcm[3].begin());
    std::array<std::array<std::int32_t,14>,40> scene{};
    for (unsigned f=0;f<frames;++f) {
        std::array<std::int32_t,14> v{};
        for (unsigned c=0;c<channels;++c) v[ids_[c]]=input[f*channels+c];
        scene[f]=v;
        // Rational downmix with headroom. Half sums round down; the stereo
        // quarter sum rounds to nearest, ties up. Preserve remainders below.
        std::array<std::int32_t,8> floor{
            channels==8 ? v[0] : (v[0]+v[8])>>1,
            channels==8 ? v[1] : (v[1]+v[9])>>1,v[2],v[3],v[4],v[5],
            channels==8 ? v[10] : (v[6]+v[12])>>1,
            channels==8 ? v[11] : (v[7]+v[13])>>1};
        std::copy(floor.begin(),floor.end(),pcm[2].begin()+f*8);
        for (unsigned c=0;c<4;++c) pcm[1][f*6+c]=floor[c];
        pcm[1][f*6+4]=(floor[4]+floor[6])>>1;
        pcm[1][f*6+5]=(floor[5]+floor[7])>>1;
        pcm[0][f*2]=(pcm[1][f*6]+floor[2]+pcm[1][f*6+4]+2)>>2;
        pcm[0][f*2+1]=(pcm[1][f*6+1]+floor[2]+pcm[1][f*6+5]+2)>>2;
    }
    std::array<unsigned,10> spatial_assignment{0,1,2,3,4,5,8,9,6,7};
    if(channels==10) {
        // Keep the shared rear-height halves in lanes 6/7; only the front
        // heights need new coded lanes. Restart assignment restores bed order.
        for(unsigned f=0;f<40;++f) {
            pcm[3][f*10+6]=scene[f][12];pcm[3][f*10+7]=scene[f][13];
            pcm[3][f*10+8]=scene[f][8];pcm[3][f*10+9]=scene[f][9];
        }
    }
    auto coded=pcm;
    std::array<std::vector<Matrix>,4> matrices;
    std::array<Bypass,4> bypass{};
    std::array<unsigned,4> first{};
    {
        first={0,2,6,channels==8 ? 6U : 8U};
        // Decode buffers are shared between substreams, but each presentation's
        // rematrix runs on its own output. Repeat the small inverse as needed.
        for(unsigned c=0;c<2;++c) {
            Matrix m{c};m.coefficients[c]=-2;m.bypass_bits=1;
            matrices[1].push_back(m);
            m.coefficients[2]=-1;m.coefficients[c+4]=-1;
            matrices[1].push_back(m);
        }
        matrices[2]=matrices[1];
        for(unsigned c=4;c<6;++c) {
            Matrix m{c};m.coefficients[c]=-2;m.coefficients[c+2]=1;
            matrices[2].push_back(m);
            m.coefficients[c]=-1;m.coefficients[c+2]=0;m.bypass_bits=1;
            matrices[2].push_back(m);
        }
        for(unsigned c=0;c<2;++c) {
            Matrix m{c};m.coefficients[c]=-2;m.bypass_bits=1;
            matrices[3].push_back(m);
            m.coefficients[c]=channels==8 ? -2 : -4;
            m.coefficients[2]=channels==8 ? -1 : -2;
            m.coefficients[c+4]=channels==8 ? -1 : -2;
            if(channels!=8)m.coefficients[c+8]=-1;
            m.bypass_bits=channels==8 ? 1 : 2;
            matrices[3].push_back(m);
        }
        for(unsigned c=4;c<6;++c) {
            Matrix m{c};m.coefficients[c]=2;m.coefficients[c+2]=-1;m.bypass_bits=1;
            matrices[3].push_back(m);
        }
        for(unsigned c=6;channels!=8 && c<8;++c) {
            Matrix m{c};m.coefficients[c]=2;m.coefficients[c+4]=-1;m.bypass_bits=1;
            if(channels==10)m.coefficients[c+4]=0;
            matrices[3].push_back(m);
        }
        for(unsigned f=0;f<40;++f) {
            for(unsigned c=0;c<2;++c) {
                const auto rem=pcm[1][f*6+c]+pcm[1][f*6+2]+
                    pcm[1][f*6+c+4]-4*pcm[0][f*2+c];
                const auto high=rem<0 ? 1U : 0U;
                const auto low=static_cast<unsigned>(rem+2*static_cast<int>(high));
                bypass[1][f][c*2]=bypass[2][f][c*2]=high;
                bypass[1][f][c*2+1]=bypass[2][f][c*2+1]=low;
                bypass[2][f][5+c*2]=static_cast<unsigned>(pcm[2][f*8+c+4]+
                    pcm[2][f*8+c+6]-2*pcm[1][f*6+c+4]);
                bypass[3][f][c*2]=high;
                bypass[3][f][c*2+1]=channels==8 ? low : 2*low+static_cast<unsigned>(scene[f][c]+
                    scene[f][c+8]-2*pcm[2][f*8+c]);
                bypass[3][f][c+4]=bypass[2][f][5+c*2];
                if(channels!=8)bypass[3][f][c+6]=static_cast<unsigned>(scene[f][c+6]+
                    scene[f][c+12]-2*pcm[2][f*8+c+6]);
            }
            for(unsigned p=1;p<4;++p) {
                const unsigned count=p==3 ? channels : counts[p];
                for(unsigned c=0;c<first[p];++c)
                    coded[p][f*count+c]=coded[p-1][f*(p==1?2:counts[p-1])+c];
            }
        }
    }
    const bool restart = units_%16 == 0;
    const auto timing = static_cast<unsigned>((units_*40+1000) & 0xffffU);
    auto next_lossless = lossless_;
    std::array<std::vector<std::uint8_t>,4> subs;
    std::array<unsigned,4> directory{};
    unsigned end = 0;
    for (unsigned p=0;p<4;++p) {
        subs[p]=substream(coded[p],p==3 ? channels : counts[p],p,timing,restart,
            final ? static_cast<int>(40-frames) : -1,next_lossless[p],
            first[p],matrices[p],bypass[p],pcm[p],
            p==3 && channels==10 ? std::span<const unsigned>(spatial_assignment) : std::span<const unsigned>{});
        end += static_cast<unsigned>(subs[p].size());
        directory[p]=(restart ? 0x2000U : 0x6000U) | (end/2);
    }
    const auto sync = restart ? major(channels,assignment_) : std::vector<std::uint8_t>{};
    const auto extra_size=restart ? 2+((evolution_.size()+4)&~std::size_t(1)) : 2;
    const auto bytes = 4+sync.size()+8+end+extra_size;
    if (bytes > 1874 || end/2 > 4095)
        throw std::runtime_error("TrueHD fixed-width residuals exceed 18 Mbit/s; prediction/rematrix required");
    unsigned parity = static_cast<unsigned>(bytes/2) ^ static_cast<unsigned>((units_*40)&0xffffU);
    for (auto d:directory) parity ^= d;
    parity ^= parity>>8; parity ^= parity>>4;
    std::vector<std::uint8_t> output;
    be16(output,static_cast<unsigned>(bytes/2) | (((parity^15)&15)<<12));
    be16(output,static_cast<unsigned>((units_*40)&0xffffU));
    output.insert(output.end(),sync.begin(),sync.end());
    for (auto d:directory) be16(output,d);
    for (const auto& s:subs) output.insert(output.end(),s.begin(),s.end());
    if(restart) {
        auto evo=evolution_;
        crypto::MatDecHmacSha256Context context;
        crypto::DDP_udc_hmac_obf_sha256_init(&context);
        crypto::mat_dec_hmac_sha256_update(&context,output.data(),output.size());
        crypto::mat_dec_hmac_sha256_update(&context,evo.data(),evo.size());
        std::array<std::uint8_t,32> digest{};
        crypto::mat_dec_hmac_sha256_final(&context,digest);
        for(unsigned i=0;i<32;++i) {
            const auto at=protection_bit_+i;
            evo[at/8]|=static_cast<std::uint8_t>(((digest[i/8]>>(7-i%8))&1U)<<(7-at%8));
        }
        std::vector<std::uint8_t> body;
        be16(body,static_cast<unsigned>(evo.size()));
        body.insert(body.end(),evo.begin(),evo.end());
        if(body.size()%2==0)body.push_back(0);
        std::uint8_t check=0xa9;
        for(const auto byte:body)check^=byte;
        body.push_back(check);
        const auto words=static_cast<unsigned>(body.size()/2);
        be16(output,words|(((15^(words>>8)^(words>>4)^words)&15)<<12));
        output.insert(output.end(),body.begin(),body.end());
    } else be16(output,0);
    lossless_=next_lossless;
    ++units_;
    finished_=final;
    return output;
}

} // namespace atmos::truehd

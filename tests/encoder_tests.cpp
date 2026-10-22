// Assert-based self-check for the encoder side. Run after every change:
//   out\encoder-tests.exe
#include "encode/eac3_core.hpp"
#include "encode/emdf_writer.hpp"
#include "encode/joc_encoder.hpp"
#include "encode/joc_refine.hpp"
#include "encode/oamd_reframer.hpp"
#include "encode/transcoder.hpp"
#include "encode/object_export.hpp"
#include "encode/frame_muxer.hpp"
#include "common/wav.hpp"
#include "common/mapped_file.hpp"
#include "common/crc.hpp"
#include "common/bit_reader.hpp"
#include "common/bit_writer.hpp"
#include "eac3/audio_block.hpp"
#include "eac3/bitstream_info.hpp"
#include "eac3/decoder.hpp"
#include "eac3/joc_decoder.hpp"
#include "eac3/joc_surround_phase.hpp"
#include "eac3/sync_frame.hpp"
#include "mkv/demuxer.hpp"
#include "encode/mkv_writer.hpp"
#include "encode/mp4_writer.hpp"
#include "render/object_renderer.hpp"
#include "render/carrier_limiter.hpp"
#include "render/carrier_panner.hpp"
#include "encode/joc_gain.hpp"
#include "truehd/access_unit.hpp"
#include "truehd/decoder.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <random>
#include <optional>
#include <memory>
#include <sstream>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (condition) return;
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::fflush(stderr);
    ++failures;
}

void test_mapped_file() {
    const auto path = std::filesystem::current_path() / std::filesystem::path(u8"ddpatmos-map-\u0442\u0435\u0441\u0442.bin");
    if (std::filesystem::exists(path)) {
        check(false, "mapped file test path must be unused"); return;
    }
    std::vector<std::uint8_t> expected(4097);
    for (std::size_t i = 0; i < expected.size(); ++i)
        expected[i] = static_cast<std::uint8_t>(i);
    {
        std::ofstream output(path, std::ios::binary);
        output.write(reinterpret_cast<const char*>(expected.data()), expected.size());
        check(output.good(), "mapped file fixture written");
    }
    {
        atmos::MappedFile input(path);
        check(std::ranges::equal(input.bytes(), expected), "mapped Unicode file preserves binary bytes across pages");
    }
    {
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        check(output.good(), "mapped file releases its handles before truncation");
    }
    {
        atmos::MappedFile input(path);
        check(input.bytes().empty(), "empty mapped file is accepted");
    }
    check(std::filesystem::remove(path), "mapped file can be removed after destruction");
    bool rejected = false;
    try { atmos::MappedFile input(path); }
    catch (const std::system_error&) { rejected = true; }
    check(rejected, "missing mapped file is rejected");
}

/// Single-tone interleaved generator in AC-3 channel order. Each frame carries
/// six 256-sample blocks.
std::vector<float> make_tone(std::uint32_t rate, unsigned frames, unsigned channels,
    double frequency = 1000.0) {
    constexpr unsigned blocks = 6U;
    const std::size_t per_frame = 256U * blocks * channels;
    const std::size_t frame_samples = 256U * blocks;
    std::vector<float> samples(static_cast<std::size_t>(frames) * per_frame);
    std::size_t index = 0;
    for (std::size_t sample = 0; sample < frame_samples * frames; ++sample) {
        const double t = static_cast<double>(sample) / static_cast<double>(rate);
        for (unsigned channel = 0; channel < channels; ++channel) {
            const double amplitude = 0.1 + 0.07 * channel;
            const double hz = channel == 5 ? 80 : frequency + channel * 211;
            samples[index++] = static_cast<float>(
                amplitude * std::sin(2.0 * std::numbers::pi * hz * t));
        }
    }
    return samples;
}

void test_joc_qmf() {
    using atmos::encode::joc::Analysis;
    std::array<float, 128> impulse{};
    impulse[0] = 1;
    const auto fft = atmos::encode::joc::qmf_detail::fft64(impulse);
    for (unsigned k = 0; k < 64; ++k)
        check(fft[2 * k] == .015625F && fft[2 * k + 1] == 0, "JOC FFT impulse scaling/order");
    Analysis bank, reference;
    std::array<float, 64> input{};
    input[0] = std::numeric_limits<float>::infinity();
    bool rejected = false;
    try { (void)bank.forward(input); } catch (const std::runtime_error&) { rejected = true; }
    check(rejected, "JOC QMF rejects nonfinite input");
    input.fill(0);
    check(bank.forward(input) == reference.forward(input), "JOC QMF rejection preserves state");
    bank = Analysis{};
    // Pinned native QMF impulse capture, including inter-slot state and signed zero.
    std::uint64_t hash = 14695981039346656037ULL;
    for (unsigned slot = 0; slot < 24; ++slot) {
        input.fill(0);
        if (slot == 0) { input[0] = 1; input[63] = -.25F; }
        if (slot == 1) input[0] = .5F;
        if (slot == 23) input[63] = -.3F;
        for (float sample : bank.forward(input)) {
            const auto bits = std::bit_cast<std::uint32_t>(sample);
            for (unsigned byte = 0; byte < 4; ++byte) {
                hash ^= (bits >> (8 * byte)) & 255;
                hash *= 1099511628211ULL;
            }
        }
    }
    check(hash == 0x5e8cdab9c8024d4cULL, "JOC QMF matches native impulse bits");
}

void test_joc_decoder_timing() {
    atmos::eac3::JocFrame joc;
    joc.channels = 5; joc.object_count = 1;
    auto& object = joc.objects[0];
    object.active = object.fine_quantization = object.steep_slope = true;
    object.bands = object.data_points = 1;
    object.timeslot_offsets[0] = 1;
    object.matrix[0][0][0] = 10;
    std::vector<float> bed(1536 * 6), unity, ramp, reset;
    bed[128 * 6] = 1;
    std::string error;
    atmos::eac3::JocDecoder reference, changed;
    std::vector<float> silence(bed.size()),ignored;
    check(reference.render(joc,silence,6,1536,ignored,error) && changed.render(joc,silence,6,1536,ignored,error),
        "gain-ramp controls establish the previous unity gain with silent PCM");
    check(reference.render(joc, bed, 6, 1536, unity, error), "unity JOC impulse renders");
    const auto peak = std::max_element(unity.begin(), unity.end(), [](float a, float b) {
        return std::abs(a) < std::abs(b);
    });
    check(peak != unity.end() && peak - unity.begin() == 128 + 1217,
        "JOC impulse has native 1217-sample reconstruction delay");
    joc.gain = 2;
    check(changed.render(joc, bed, 6, 1536, ramp, error), "changing JOC gain renders");
    double energy = 0, difference = 0;
    for (std::size_t i = 0; i < unity.size() && i < ramp.size(); ++i) {
        energy += double(unity[i]) * unity[i];
        const double delta = ramp[i] - 1.5 * unity[i];
        difference += delta * delta;
    }
    check(energy > 1e-8 && difference < energy * 1e-10,
        "JOC gain at sample128 is halfway through the pre-QMF 256-sample ramp");
    changed.reset();
    check(changed.render(joc, bed, 6, 1536, reset, error),"reset JOC gain renders");
    difference=0;
    for(std::size_t i=0;i<unity.size() && i<reset.size();++i){const double delta=reset[i]-2*unity[i];difference+=delta*delta;}
    check(difference<energy*1e-10,"JOC reset initializes gain directly instead of retaining the previous stream's ramp");
}

void test_joc_two_steep_points() {
    for (unsigned channels : {5U,7U}) {
        atmos::eac3::JocDecoder one_decoder,two_decoder;
        atmos::eac3::JocFrame one;one.channels=static_cast<std::uint8_t>(channels);one.object_count=1;
        auto& object=one.objects[0];
        object.active=object.fine_quantization=object.steep_slope=true;
        object.bands=object.data_points=1;object.timeslot_offsets[0]=17;object.matrix[0][0][0]=10;
        auto two=one;two.objects[0].data_points=2;two.objects[0].timeslot_offsets[0]=5;
        two.objects[0].timeslot_offsets[1]=17;two.objects[0].matrix[0][0][0]=0;two.objects[0].matrix[1][0][0]=10;
        auto initial=one;initial.objects[0].matrix[0][0][0]=0;
        std::vector<float> silence(1536*(channels+1)),ignored;std::string initial_error;
        check(one_decoder.render(initial,silence,static_cast<std::uint8_t>(channels+1),1536,ignored,initial_error) &&
            two_decoder.render(initial,silence,static_cast<std::uint8_t>(channels+1),1536,ignored,initial_error),
            "late steep-point controls establish a real zero previous matrix");
        unsigned random=123;double energy=0;bool equal=true;
        for(unsigned frame=0;frame<3;++frame) {
            if(frame){one.objects[0].steep_slope=false;two=one;}
            std::vector<float> input(1536*(channels+1)),a,b;std::string error;
            for(unsigned t=0;t<1536;++t) {
                random=random*1664525U+1013904223U;
                input[t*(channels+1)]=(float(random>>8)/16777216.0F-.5F)*.25F;
            }
            const bool rendered=one_decoder.render(one,input,static_cast<std::uint8_t>(channels+1),1536,a,error) &&
                two_decoder.render(two,input,static_cast<std::uint8_t>(channels+1),1536,b,error);
            check(rendered,"one/two-point JOC step controls render");
            equal=equal && rendered && a==b;
            for(float value:a)energy+=double(value)*value;
        }
        check(equal && energy>1e-8,"second steep JOC point equals its independent one-point step through QMF drain");
    }
}

void test_joc_inactive_history() {
    for(unsigned channels:{5U,7U})for(unsigned points:{1U,2U})for(bool steep:{false,true}) {
        atmos::eac3::JocDecoder inactive,zero;unsigned random=123;double energy=0;bool equal=true;
        for(unsigned frame=0;frame<6;++frame) {
            atmos::eac3::JocFrame left;left.channels=static_cast<std::uint8_t>(channels);left.object_count=1;
            auto& object=left.objects[0];object.active=object.fine_quantization=object.steep_slope=true;
            object.bands=object.data_points=1;object.timeslot_offsets[0]=1;object.matrix[0][0][0]=8;
            if(frame==2)object.active=false;
            if(frame>2) {
                object.data_points=static_cast<std::uint8_t>(points);object.steep_slope=steep;
                object.timeslot_offsets={5,17};object.matrix[0][0][0]=4;object.matrix[1][0][0]=186;
            }
            auto right=left;
            if(frame==2){right.objects[0].active=true;right.objects[0].matrix[0][0][0]=0;}
            std::vector<float> input(1536*(channels+1)),a,b;std::string error;
            for(unsigned t=0;t<1536;++t){random=random*1664525U+1013904223U;input[t*(channels+1)]=(float(random>>8)/16777216.0F-.5F)*.2F;}
            const bool rendered=inactive.render(left,input,static_cast<std::uint8_t>(channels+1),1536,a,error) &&
                zero.render(right,input,static_cast<std::uint8_t>(channels+1),1536,b,error);
            check(rendered,"inactive and explicit-zero JOC controls render");equal=equal && rendered && a==b;
            for(float value:a)energy+=double(value)*value;
        }
        check(equal && energy>1e-8,"inactive JOC clears interpolation history for one/two-point ramp/steep reactivation");
    }
}

void configure_51(atmos::encode::CoreEncoder& encoder) {
    atmos::encode::CoreConfig config;
    config.sample_rate = 48000;
    config.audio_coding_mode = 7; // 3/2
    config.lfe = true;
    config.blocks = 6;
    config.bitrate = 640000;
    encoder.configure(config);
}

void test_frame_shape() {
    atmos::encode::CoreEncoder encoder;
    configure_51(encoder);
    check(encoder.channels() == 6U, "5.1 channel count");
    check(encoder.samples_per_frame() == 1536U, "6 blocks of 256 samples");
    check(encoder.frame_size() == 2560U, "640 kbps 48 kHz frame is 2560 bytes");

    const auto samples = make_tone(48000, 1, encoder.channels());
    std::vector<std::uint8_t> frame;
    encoder.encode(samples, frame);
    check(frame.size() == 2560U, "encoded frame size");
    check(frame[0] == 0x0BU && frame[1] == 0x77U, "sync word");
    check(atmos::crc16_ansi(std::span<const std::uint8_t>(frame).subspan(2)) == 0,
        "frame CRC is valid");

    
    const auto info = atmos::eac3::parse_bitstream_info(frame, 0U);
    
    check(info.has_value(), "frame header parses");
    if (!info) return;
    check(info->sample_rate == 48000U, "sample rate round-trips");
    check(info->blocks == 6U, "block count round-trips");
    check(info->audio_coding_mode == 7U, "audio coding mode round-trips");
    check(info->lfe, "LFE flag round-trips");
    check(info->bitstream_id == 16U, "E-AC-3 bitstream id");

    const auto syntax = atmos::eac3::parse_audio_frame_syntax(frame, *info);
    check(syntax.has_value(), "audio frame syntax parses");
    if (!syntax) return;
    check(syntax->full_bandwidth_channels == 5U, "five full-bandwidth channels");
    check(!syntax->block_switch_syntax, "no block switching used");
    check(!syntax->parse_aht, "no adaptive hybrid transform used");

    
    
    atmos::eac3::DitherGenerator dither;
    const auto spectral = atmos::eac3::decode_spectral_frame(frame, *info, *syntax, dither);
    check(spectral.has_value(), "spectral payload decodes");
    
}

void test_round_trip() {
    atmos::encode::CoreEncoder encoder;
    configure_51(encoder);
    const unsigned frames = 8U;
    const auto samples = make_tone(48000, frames, encoder.channels());
    atmos::eac3::BedDecoder decoder;
    decoder.reset();
    std::ofstream dump("logs/roundtrip.eac3", std::ios::binary);

    const std::size_t frame_samples =
        static_cast<std::size_t>(encoder.samples_per_frame()) * encoder.channels();
    const std::size_t block_samples = 256U * encoder.channels();
    std::vector<float> decoded_stream(samples.size(), 0.0F);
    for (unsigned frame = 0; frame < frames; ++frame) {
        const auto begin = samples.begin() +
            static_cast<std::ptrdiff_t>(frame) * static_cast<std::ptrdiff_t>(frame_samples);
        const std::vector<float> block(begin, begin +
            static_cast<std::ptrdiff_t>(frame_samples));
        std::vector<std::uint8_t> encoded;
        encoder.encode(block, encoded);
        if (dump) {
            dump.write(reinterpret_cast<const char*>(encoded.data()),
                static_cast<std::streamsize>(encoded.size()));
        }

        atmos::eac3::PcmFrame decoded;
        std::string error;
        if (!decoder.decode(encoded, decoded, error)) {
            std::fprintf(stderr, "FAIL: decode frame %u: %s\n", frame, error.c_str());
            std::fflush(stderr);
            ++failures;
            continue;
        }
        check(decoded.channels == encoder.channels(), "decoded channel count");
        check(decoded.samples_per_channel == 1536U, "decoded sample count");
        std::copy(decoded.interleaved.begin(), decoded.interleaved.end(),
            decoded_stream.begin() + static_cast<std::ptrdiff_t>(frame) * frame_samples);
    }
    if (dump) dump.close();

    // The analysis needs a one-block lookahead, so the decoded stream trails the
    // input by one 256-sample block. The first frame is a fade-in from silence.
    double reference_energy = 0.0;
    double error_energy = 0.0;
    double decoded_peak = 0.0;
    for (std::size_t index = frame_samples; index + block_samples < samples.size(); ++index) {
        const double reference = samples[index];
        constexpr unsigned wav_channels[] = {0,2,1,4,5,3};
        const auto target = index + block_samples;
        const double value = decoded_stream[target / 6 * 6 + wav_channels[target % 6]];
        reference_energy += reference * reference;
        error_energy += (value - reference) * (value - reference);
        decoded_peak = std::max(decoded_peak, std::abs(value));
    }
    check(reference_energy > 0.0, "reference signal is non-trivial");
    const double noise_ratio =
        std::sqrt(error_energy / std::max(1.0e-12, reference_energy));
    std::printf("round-trip relative error: %.6f (decoded peak %.4f)\n",
        noise_ratio, decoded_peak);
    std::fflush(stdout);
    check(noise_ratio < 0.03, "encode/decode preserves channel order and signal level");
}

void test_exponent_reuse_headroom() {
    atmos::encode::CoreEncoder encoder;
    atmos::encode::CoreConfig config;
    config.audio_coding_mode = 1;
    config.lfe = false;
    config.bitrate = 384000;
    encoder.configure(config);
    atmos::eac3::BedDecoder decoder;
    std::vector<float> source, decoded;
    for (unsigned frame = 0; frame < 80; ++frame) {
        std::vector<float> pcm(1536);
        for (unsigned i = 0; i < pcm.size(); ++i) {
            const double gain = .0001 * std::pow(1.08, frame) * std::pow(1.20, i / 256);
            pcm[i] = static_cast<float>(gain) * static_cast<float>(
                std::sin(2 * std::numbers::pi * 17 * (i % 256) / 256.));
        }
        std::vector<std::uint8_t> encoded;
        encoder.encode(pcm, encoded);
        atmos::eac3::PcmFrame output;
        std::string error;
        if (!decoder.decode(encoded, output, error)) {
            check(false, "rising-tone core frame decodes");
            return;
        }
        source.insert(source.end(), pcm.begin(), pcm.end());
        decoded.insert(decoded.end(), output.interleaved.begin(), output.interleaved.end());
    }
    double energy = 0, error = 0;
    for (std::size_t i = 1536; i < source.size() - 1536; ++i) {
        energy += source[i] * source[i];
        const double delta = source[i] - decoded[i + 256];
        error += delta * delta;
    }
    check(energy > 0 && error < energy * 1e-5,
        "reused exponents preserve headroom on a growing tone (SNR above 50 dB)");
}

template<class F> void rejects(F action, const char* message) {
    try { action(); check(false, message); }
    catch (const std::exception&) {}
}

void test_oamd_timing() {
    // Independent syntax vector: two blocks and position-only optional render fields.
    atmos::BitWriter element;
    element.write(0, 1); // element skip
    element.write(2, 2); element.write(5, 5); // explicit sample offset
    element.write(1, 3); // two blocks
    element.write(3, 6); element.write(2, 2); // 1536-sample ramp
    element.write(9, 6); element.write(1, 2); // 512-sample ramp
    element.write(1, 1); // no header extension
    element.write(0, 1); // first block active
    element.write(0, 2); element.write(1, 1); // unity gain, no basic extension
    element.write(31, 6); element.write(0, 6);
    element.write(1, 1); element.write(15, 4); element.write(0, 1); // height, distance
    element.write(0, 4); element.write(0, 2); // zone, size
    element.write(0, 1); element.write(0, 1); element.write(0, 1); // priority, reserved, tail
    element.write(0, 1); element.write(0, 2); // second block active, no basic update
    element.write(3, 2); element.write(1, 4); // optional render: position only
    element.write(0, 1); // absolute position
    element.write(62, 6); element.write(31, 6);
    element.write(1, 1); element.write(0, 4); element.write(0, 1);
    element.write(0, 1); element.write(0, 1); // reserved, tail
    element.align();
    atmos::BitWriter payload;
    payload.write(0, 2); payload.write(0, 5); // version, one object
    payload.write(1, 1); payload.write(0, 1); // dynamic-only, no LFE
    payload.write(0, 1); payload.write(1, 4); // no alternate flag, one element
    payload.write(1, 4);
    payload.write(static_cast<unsigned>(element.bytes().size() - 1), 4);
    payload.write(0, 1); // final variable-length size group
    for (const auto byte : element.bytes()) payload.write(byte, 8);
    payload.align();
    const auto parsed = atmos::eac3::parse_oamd_payload(payload.bytes());
    check(parsed.valid && parsed.oamd.position_updates.size() == 2, "independent OAMD vector parses");
    if (parsed.oamd.position_updates.size() != 2) return;
    const auto& updates = parsed.oamd.position_updates;
    check(updates[0].sample_offset == 101 && updates[1].sample_offset == 293,
        "OAMD block factors use 32 samples plus the explicit offset");
    check(updates[1].x == 1.0F && updates[1].y == 0.5F && updates[1].z == 0.0F,
        "optional render mask 1 selects position without zone, size or screen reference");
    check(parsed.oamd.spread_updates.size() == 2 && parsed.oamd.spread_updates[1].size == parsed.oamd.spread_updates[0].size,
        "position-only update retains size at the new block timing");
    check(parsed.oamd.zone_updates.size() == 2 && !parsed.oamd.zone_updates[0].elevation,
        "independent OAMD vector preserves explicit height exclusion");
    std::vector<atmos::encode::TimedOamdPayload> packets = {{0, parsed.oamd_payload}, {1216, parsed.oamd_payload}};
    for (auto& packet : packets) packet.payload.elements.push_back({5, {0}});
    auto incompatible = packets; incompatible[1].sample = 1240;
    rejects([&] { (void)atmos::encode::reframe_oamd(incompatible,3072); },
        "different timing residues are rejected instead of emitting ignored duplicate elements");
    check(atmos::encode::oamd_alignment_padding(incompatible,3072,0) == 195,
        "minimum common PCM/metadata padding separates incompatible residues exactly");
    unsigned restarts=0;
    check(atmos::encode::oamd_alignment_padding([&] {
        ++restarts;
        return atmos::encode::OamdReader{[&,index=std::size_t{0}](atmos::encode::TimedOamdPayload& output) mutable {
            if(index==incompatible.size()){output={};return false;}
            output=incompatible[index++];return true;
        }};
    },3072,0)==195 && restarts==3,"alignment replays zero, timing filter and minimum surviving candidate");
    rejects([&] { atmos::encode::OamdReframer missing(atmos::encode::OamdReader{},1536); },
        "reframer rejects a missing input callback");
    rejects([&] { (void)atmos::encode::oamd_alignment_padding(atmos::encode::OamdReaderFactory{},1536,0); },
        "alignment rejects a missing source factory");
    unsigned calls=0;
    atmos::encode::OamdReframer failed([&](atmos::encode::TimedOamdPayload& output) {
        if(calls++)throw std::runtime_error("metadata source failed");
        output=packets.front();return true;
    },3072);
    std::vector<std::uint8_t> incomplete;
    rejects([&] { (void)failed.next(incomplete); },"reframer propagates source failure rather than treating it as EOF");
    check(atmos::encode::oamd_alignment_padding(packets,3072,0) == 0,
        "representable metadata adds no latency");
    const std::array<atmos::encode::TimedOamdPayload,3> impossible{{
        {0,packets[0].payload},{40,packets[0].payload},{80,packets[0].payload}}};
    rejects([&] { (void)atmos::encode::oamd_alignment_padding(impossible,1536,0); },
        "unrepresentable dense schedules remain explicit failures after a full phase search");
    incompatible[1].sample = 0;
    const auto simultaneous = atmos::eac3::parse_oamd_payload(
        atmos::encode::reframe_oamd(incompatible,3072)[0]);
    check(simultaneous.valid && simultaneous.oamd.position_updates.size() == 4 &&
        simultaneous.oamd.position_updates[0].sample_offset == 101 &&
        simultaneous.oamd.position_updates[1].sample_offset == 101 &&
        simultaneous.oamd.position_updates[2].sample_offset == 293 &&
        simultaneous.oamd.position_updates[3].sample_offset == 293,
        "simultaneous blocks retain exact times and inherited render state with paired extensions");
    std::vector<atmos::encode::TimedOamdPayload> eight;
    for (unsigned origin : {0U,320U,640U,960U}) eight.push_back({origin,packets[0].payload});
    const auto eight_blocks = atmos::eac3::parse_oamd_payload(atmos::encode::reframe_oamd(eight,1536)[0]);
    check(eight_blocks.valid && eight_blocks.oamd_payload.elements.size() == 2 &&
        eight_blocks.oamd.position_updates.size() == 8 &&
        eight_blocks.oamd.position_updates.back().sample_offset == 1253,
        "eight blocks and paired extensions fit one native object element without rounding");
    const auto reframed = atmos::encode::reframe_oamd(packets, 3072);
    {
        auto overlapping = packets;
        overlapping[0].payload.sample_offset = 3072;
        overlapping[1].sample = 1536;
        overlapping.push_back({3072,packets[0].payload});
        atmos::encode::OamdReframer reader(overlapping,4608);
        std::vector<std::uint8_t> wire;
        for (unsigned frame=0;frame<4;++frame) {
            check(reader.next(wire),"OAMD iterator emits every audio frame and codec flush");
            const auto decoded = atmos::eac3::parse_oamd_payload(wire);
            const auto& positions = decoded.oamd.position_updates;
            check(decoded.valid && positions.size()==(frame==1 ? 2U : frame==2 ? 4U : 0U),
                "future outer offsets do not flush before later packets contribute");
            if (!positions.empty())
                check(positions.front().sample_offset==101 && positions.back().sample_offset==293,
                    "streamed overlapping packets preserve independent 101/293 sample times");
        }
        check(!reader.next(wire) && wire.empty() && !reader.next(wire) && wire.empty(),
            "OAMD iterator EOF is stable and clears output");
        atmos::encode::OamdReframer long_input(packets,1536ULL*100000000);
        check(long_input.next(wire) && wire==reframed.front(),
            "first OAMD frame is available without allocating the declared long duration");
    }
    const auto delayed_oamd = atmos::encode::reframe_oamd(packets, 3072 + 1217, 1217);
    const auto delayed_first = atmos::eac3::parse_oamd_payload(delayed_oamd[0]);
    const auto delayed_second = atmos::eac3::parse_oamd_payload(delayed_oamd[1]);
    check(delayed_first.valid && delayed_first.oamd.position_updates.size() == 2 &&
        delayed_first.oamd.position_updates[0].sample_offset == 1318 &&
        delayed_first.oamd.position_updates[1].sample_offset == 1510 &&
        delayed_second.valid && delayed_second.oamd.position_updates.size() == 2 &&
        delayed_second.oamd.position_updates[0].sample_offset == 998 &&
        delayed_second.oamd.position_updates[1].sample_offset == 1190,
        "OAMD follows the carrier delay across frame boundaries without rounding");
    const auto first = atmos::eac3::parse_oamd_payload(reframed[0]);
    check(first.valid && first.oamd.position_updates.size() == 4,
        "reframing merges both fragments into one object element");
    if (first.oamd.position_updates.size() == 4) {
        const auto& moved = first.oamd.position_updates;
        check(moved[0].sample_offset == 101 && moved[1].sample_offset == 293 &&
            moved[2].sample_offset == 1317 && moved[3].sample_offset == 1509,
            "access-unit origin is added exactly without frame quantization");
        check(moved[2].x == updates[0].x && moved[3].y == updates[1].y &&
            first.oamd.zone_updates.size() == 4 && !first.oamd.zone_updates[1].elevation,
            "reframing preserves position and zone bits");
        check(moved[2].ramp_duration == 1536 && moved[3].ramp_duration == 512,
            "reframing keeps ramps that continue into the next audio frame");
    }
    const auto empty = atmos::eac3::parse_oamd_payload(reframed.back());
    check(empty.valid && empty.oamd.position_updates.empty(), "flush frame does not replay metadata ramps");
    check(first.oamd_payload.elements.size() == 2 &&
        first.oamd_payload.elements[1].body == std::vector<std::uint8_t>{0},
        "empty paired extensions merge without changing fields");
    const auto wrapped = atmos::eac3::parse_emdf(
        atmos::encode::build_emdf(first.oamd, {}, true, false, reframed[0]));
    check(wrapped.valid && wrapped.oamd.position_updates.size() == 4 &&
        wrapped.oamd.position_updates.back().sample_offset == 1509 &&
        wrapped.oamd.position_updates.back().ramp_duration == 512,
        "EMDF writer carries reframed OAMD instead of the static snapshot");
    const std::vector<atmos::encode::TimedOamdPayload> many(15, packets.front());
    rejects([&] { (void)atmos::encode::reframe_oamd(many,1536); },
        "more than eight merged blocks are rejected instead of silently lost by Dolby");
    packets[1].sample = 1408;
    const auto split = atmos::encode::reframe_oamd(packets, 3072);
    const auto moved = atmos::eac3::parse_oamd_payload(split[1]);
    check(moved.valid && moved.oamd.position_updates.size() == 1 &&
        moved.oamd.position_updates.front().sample_offset == 165 && moved.oamd.position_updates.front().x == 1.0F,
        "cross-boundary partial render expands at its exact relocated time");
}

std::vector<std::string> oamd_events(const atmos::eac3::OamdFrame& frame, std::uint64_t origin) {
    std::vector<std::string> result;
    const auto record = [&](const char* type, const auto& update, auto... fields) {
        std::ostringstream text;
        text << type << ' ' << origin + update.sample_offset << ' ' << update.object
             ;
        ((text << ' ' << std::hexfloat << fields), ...);
        result.push_back(text.str());
    };
    for (const auto& u : frame.position_updates)
        record("position", u, u.ramp_duration, u.differential, u.x, u.y, u.z,
            u.distance_specified, u.object_at_infinity, u.distance, u.snap);
    for (const auto& u : frame.gain_updates) record("gain", u, u.ramp_duration, u.gain);
    for (const auto& u : frame.spread_updates) record("spread", u, u.ramp_duration, u.size, u.divergence);
    for (const auto& u : frame.zone_updates) record("zone", u, unsigned(u.horizontal), u.elevation, u.ramp_duration);
    return result;
}

void test_oamd_cross_frame() {
    const auto rejects = [](auto action, const char* message) {
        bool rejected = false;
        try { action(); } catch (const std::exception&) { rejected = true; }
        check(rejected, message);
    };
    // Independent wire vector: LFE + two objects, three full updates spanning three DD+ frames.
    atmos::BitWriter body;
    body.write(0, 1); body.write(2, 2); body.write(5, 5); body.write(2, 3);
    body.write(0, 6); body.write(3, 2); body.write(1, 1); body.write(10, 4); // ramp 1601
    body.write(47, 6); body.write(3, 2); body.write(0, 1); body.write(1111, 11);
    body.write(63, 6); body.write(1, 2); // ramp 512
    body.write(0, 1); body.write(19, 5); // opaque element header extension
    for (unsigned object = 0; object < 3; ++object) {
        for (unsigned block = 0; block < 3; ++block) {
            const bool inactive = object == 2 && block == 1;
            body.write_bit(inactive);
            if (!inactive) {
                if (block) body.write(1, 2); // full basic
                body.write(2, 2); body.write(15 + block, 6); // explicit gain
                body.write(0, 1); body.write(21, 5); // opaque basic extension
                if (object) {
                    if (block) { body.write(1, 2); body.write(0, 1); } // full absolute render
                    body.write(10 * object + block, 6); body.write(20 + block, 6);
                    body.write(1, 1); body.write(5 + block, 4); // nonzero height
                    body.write(1, 1); body.write(0, 1); body.write(3 + block, 4); // distance
                    body.write(object + block, 3); body.write(1, 1); // zone
                    body.write(1, 2); body.write(7 + block, 5); // size
                    body.write(1, 1); body.write(11, 5); // opaque priority
                    body.write(1, 1); // reserved render bit
                }
            }
            body.write(1, 1); body.write(0, 4); body.write(0xa5 + block, 8); // opaque record tail
        }
    }
    body.align();
    atmos::BitWriter payload;
    payload.write(0, 2); payload.write(2, 5); payload.write(1, 1); payload.write(1, 1);
    payload.write(0, 1); payload.write(1, 4); payload.write(1, 4);
    const auto size = static_cast<std::uint32_t>(body.bytes().size() - 1);
    payload.write((size >> 4) - 1, 4); payload.write(1, 1);
    payload.write(size & 15, 4); payload.write(0, 1);
    for (auto byte : body.bytes()) payload.write(byte, 8);
    payload.align();
    auto source = atmos::eac3::parse_oamd_payload(payload.bytes());
    check(source.valid && source.oamd.position_updates.size() == 5,
        "independent three-block OAMD source parses");
    if (!source.valid) return;
    const std::array<atmos::encode::TimedOamdPayload,2> interleaved{{
        {0,source.oamd_payload},{256,source.oamd_payload}}};
    check(atmos::encode::oamd_alignment_padding(interleaved,4608,0) == 0,
        "full interleaved records need no extra latency");
    std::vector<std::string> interleaved_events, interleaved_reference;
    for (unsigned origin : {0U,256U}) {
        const auto events = oamd_events(source.oamd,origin);
        interleaved_reference.insert(interleaved_reference.end(),events.begin(),events.end());
    }
    const auto chronological = atmos::encode::reframe_oamd(interleaved,4608);
    for (unsigned frame = 0; frame < chronological.size(); ++frame) {
        const auto decoded = atmos::eac3::parse_oamd_payload(chronological[frame]);
        check(decoded.valid && decoded.oamd_payload.elements.size() <= 1,
            "interleaved full records form one native object element per frame");
        if (!decoded.oamd_payload.elements.empty()) {
            const auto& merged = decoded.oamd_payload.elements.front();
            for (unsigned object = 0; object < merged.object_records.size(); ++object)
                for (unsigned block = 0; block < merged.object_records[object].size(); ++block) {
                    const unsigned original_block = merged.object_records[0][block].basic.gain_bits-15;
                    check(original_block < 3,"interleaved block retains its original gain code");
                    if (original_block >= 3) continue;
                    atmos::BitReader before(source.oamd_payload.elements.front().body),after(merged.body);
                    before.skip(source.oamd_payload.elements.front().object_records[object][original_block].end-13);
                    after.skip(merged.object_records[object][block].end-13);
                    check(before.read(13) == after.read(13),"interleaved blocks keep opaque record tails bit-exact");
                }
        }
        const auto events = oamd_events(decoded.oamd,frame*1536);
        interleaved_events.insert(interleaved_events.end(),events.begin(),events.end());
    }
    std::sort(interleaved_events.begin(),interleaved_events.end());
    std::sort(interleaved_reference.begin(),interleaved_reference.end());
    check(interleaved_events == interleaved_reference,
        "interleaving preserves full gain/render/inactive fields, opaque tails, timing and ramps");
    const auto reuse_packet = [&](bool hold_last) {
        const auto& original = source.oamd_payload.elements.front();
        const std::vector<unsigned> selected = hold_last ? std::vector<unsigned>{0,1,2} : std::vector<unsigned>{0,2};
        atmos::BitWriter body;
        const auto copy = [&](std::size_t begin,std::size_t end) {
            atmos::BitReader bits(original.body); bits.skip(begin);
            for (auto bit = begin; bit < end; ++bit) body.write_bit(bits.read_bit());
        };
        body.write(0,1); body.write(2,2); body.write(5,5); body.write(static_cast<unsigned>(selected.size()-1),3);
        for (auto block : selected) {
            body.write(block*16,6);
            if (block == 0) { body.write(3,2); body.write_bit(true); body.write(10,4); }
            else if (block == 1) { body.write(3,2); body.write_bit(false); body.write(1111,11); }
            else body.write(1,2);
        }
        copy(original.object_header_begin,original.object_data_begin);
        for (const auto& object : original.object_records) for (auto block : selected) {
            const auto& record = object[block];
            if (hold_last && block == 2) {
                body.write_bit(false); body.write(2,2);
                if (record.dynamic) body.write(2,2);
                copy(record.render_end,record.end);
            } else copy(record.begin,record.end);
        }
        body.align(); atmos::BitWriter payload;
        for (std::size_t bit = 0; bit < source.oamd_payload.header_bits; ++bit)
            payload.write_bit((source.oamd_payload.header[bit/8] >> (7-bit%8)) & 1);
        payload.write(1,4); payload.write(1,4);
        const auto length = static_cast<unsigned>(body.bytes().size()-1);
        check(length >= 16 && length < 272,"focused reuse fixture has a two-group element size");
        payload.write((length>>4)-1,4); payload.write_bit(true); payload.write(length&15,4); payload.write_bit(false);
        for (auto byte : body.bytes()) payload.write(byte,8);
        payload.align(); return atmos::eac3::parse_oamd_payload(payload.bytes());
    };
    const auto reuse_a = reuse_packet(true), reuse_b = reuse_packet(false);
    check(reuse_a.valid && reuse_b.valid,"interleaved reuse-chain fixtures parse independently");
    const std::array<atmos::encode::TimedOamdPayload,2> reuse_input{{{0,reuse_a.oamd_payload},{256,reuse_b.oamd_payload}}};
    check(atmos::encode::oamd_alignment_padding(reuse_input,1536,0) == 0,
        "uninterrupted partial/reuse chains inside an interleaved schedule need no padding");
    const auto reuse_output = atmos::eac3::parse_oamd_payload(atmos::encode::reframe_oamd(reuse_input,1536)[0]);
    auto reuse_expected = oamd_events(reuse_a.oamd,0);
    const auto reuse_second = oamd_events(reuse_b.oamd,256);
    reuse_expected.insert(reuse_expected.end(),reuse_second.begin(),reuse_second.end());
    auto reuse_actual = oamd_events(reuse_output.oamd,0);
    std::sort(reuse_expected.begin(),reuse_expected.end()); std::sort(reuse_actual.begin(),reuse_actual.end());
    check(reuse_output.valid && reuse_output.oamd_payload.elements.size() == 1 && reuse_actual == reuse_expected,
        "adjacent source predecessors preserve gain/render reuse, inactive defaults, times and ramps after interleaving");
    auto initial = source.oamd_payload;
    initial.elements.clear();
    const std::vector<atmos::encode::TimedOamdPayload> packets = {{0, initial}, {1500, source.oamd_payload}};
    const auto output = atmos::encode::reframe_oamd(packets, 4608);
    std::vector<std::string> actual;
    for (unsigned frame = 0; frame < output.size(); ++frame) {
        const auto decoded = atmos::eac3::parse_oamd_payload(output[frame]);
        check(decoded.valid, "split OAMD fragment parses");
        const auto events = oamd_events(decoded.oamd, frame * 1536);
        actual.insert(actual.end(), events.begin(), events.end());
        if (frame < 3) {
            check(decoded.oamd_payload.elements.size() == 1, "one OAMD fragment in each destination frame");
            const auto& records = decoded.oamd_payload.elements.front().object_records;
            const auto& original = source.oamd_payload.elements.front();
            check(records.size() == 3, "split retains all object-major records");
            for (unsigned object = 0; object < records.size(); ++object) {
                const auto& record = records[object][0];
                atmos::BitReader before(original.body), after(decoded.oamd_payload.elements.front().body);
                before.skip(original.object_records[object][frame].end - 13);
                after.skip(record.end - 13);
                check(before.read(13) == after.read(13), "split preserves opaque record tails exactly");
            }
        }
    }
    for (auto& update : source.oamd.position_updates) update.block = 0;
    for (auto& update : source.oamd.gain_updates) update.block = 0;
    for (auto& update : source.oamd.spread_updates) update.block = 0;
    for (auto& update : source.oamd.zone_updates) update.block = 0;
    auto expected = oamd_events(source.oamd, 1500);
    std::sort(actual.begin(), actual.end()); std::sort(expected.begin(), expected.end());
    check(actual == expected, "cross-frame split preserves absolute timing, ramps, gain, position, height, distance, size and zones");
    for (const unsigned origin : {1000U, 31U, 1531U}) {
        auto shifted = packets;
        shifted.back().sample = origin;
        const auto regrouped = atmos::encode::reframe_oamd(shifted, 4608);
        std::vector<std::string> events;
        for (unsigned frame = 0; frame < regrouped.size(); ++frame) {
            auto decoded = atmos::eac3::parse_oamd_payload(regrouped[frame]);
            check(decoded.valid, "regrouped OAMD fragment parses");
            for (auto& update : decoded.oamd.position_updates) update.block = 0;
            for (auto& update : decoded.oamd.gain_updates) update.block = 0;
            for (auto& update : decoded.oamd.spread_updates) update.block = 0;
            for (auto& update : decoded.oamd.zone_updates) update.block = 0;
            const auto fragment = oamd_events(decoded.oamd, frame * 1536);
            events.insert(events.end(), fragment.begin(), fragment.end());
        }
        auto reference = oamd_events(source.oamd, origin);
        std::sort(events.begin(), events.end()); std::sort(reference.begin(), reference.end());
        check(events == reference, "split preserves multi-block fragments and exact boundary events");
    }
    atmos::BitWriter extension;
    extension.write_bit(false); // element prefix
    extension.write_bit(true); // divergence for active dynamic objects only
    extension.write_bit(true); extension.write(0, 2); extension.write(2, 2);
    extension.write_bit(true); extension.write(1, 2); // inherit compact value across the split
    extension.write_bit(true); extension.write(3, 2); extension.write(42, 6);
    extension.write_bit(true); extension.write(2, 2); extension.write(27, 6);
    extension.write_bit(true); extension.write(1, 2); // preceding block is inactive: zero
    extension.write_bit(true); // extended coordinate precision
    for (unsigned object = 1; object < 3; ++object) {
        for (unsigned block = 0; block < 3; ++block) {
            if (object == 2 && block == 1) continue;
            extension.write_bit(true); extension.write(7, 3);
            extension.write(block, 2); extension.write(object, 2); extension.write(3 - block, 2);
        }
    }
    const auto extension_bits = extension.position();
    extension.align();
    atmos::BitWriter extended_payload;
    atmos::BitReader original_payload(payload.bytes());
    for (std::size_t bit = 0; bit < source.oamd_payload.header_bits; ++bit)
        extended_payload.write_bit(original_payload.read_bit());
    original_payload.skip(4); extended_payload.write(2, 4);
    for (std::size_t bit = 0; bit < 14 + body.bytes().size() * 8; ++bit)
        extended_payload.write_bit(original_payload.read_bit());
    extended_payload.write(5, 4);
    extended_payload.write(static_cast<unsigned>(extension.bytes().size() - 1), 4);
    extended_payload.write_bit(false);
    for (auto byte : extension.bytes()) extended_payload.write(byte, 8);
    extended_payload.align();
    auto extended_source = atmos::eac3::parse_oamd_payload(extended_payload.bytes());
    check(extended_source.valid && extended_source.oamd.spread_updates.size() == 10,
        "independent divergence and precision vector parses");
    if (!extended_source.valid) return;
    check(extended_source.oamd.spread_updates.back().divergence == 0.0F,
        "divergence inheritance resets after an inactive block");
    auto with_extension = packets;
    with_extension.back().payload = extended_source.oamd_payload;
    const auto extended_output = atmos::encode::reframe_oamd(with_extension, 4608);
    std::vector<std::string> extended_events;
    for (unsigned frame = 0; frame < extended_output.size(); ++frame) {
        auto decoded = atmos::eac3::parse_oamd_payload(extended_output[frame]);
        check(decoded.valid && (frame >= 3 || decoded.oamd_payload.elements.size() == 2),
            "split extensions remain paired with their object fragment");
        for (auto& update : decoded.oamd.position_updates) update.block = 0;
        for (auto& update : decoded.oamd.gain_updates) update.block = 0;
        for (auto& update : decoded.oamd.spread_updates) update.block = 0;
        for (auto& update : decoded.oamd.zone_updates) update.block = 0;
        const auto events = oamd_events(decoded.oamd, frame * 1536);
        extended_events.insert(extended_events.end(), events.begin(), events.end());
    }
    for (auto& update : extended_source.oamd.position_updates) update.block = 0;
    for (auto& update : extended_source.oamd.gain_updates) update.block = 0;
    for (auto& update : extended_source.oamd.spread_updates) update.block = 0;
    for (auto& update : extended_source.oamd.zone_updates) update.block = 0;
    auto extended_expected = oamd_events(extended_source.oamd, 1500);
    std::sort(extended_events.begin(), extended_events.end());
    std::sort(extended_expected.begin(), extended_expected.end());
    check(extended_events == extended_expected,
        "cross-frame extension split preserves divergence, inherited state and precise coordinates");
    const auto middle = atmos::eac3::parse_oamd_payload(extended_output[1]);
    atmos::BitReader middle_bits(middle.oamd_payload.elements.back().body);
    check(middle_bits.read(7) == 50, // prefix0, divergence1, present1, mode00, compact10
        "first split divergence expands inheritance to the exact original compact code");
    const auto final = atmos::eac3::parse_oamd_payload(extended_output[2]);
    atmos::BitReader final_bits(final.oamd_payload.elements.back().body);
    check(final_bits.read(2) == 1 && final_bits.read(9) == 490 && !final_bits.read_bit(),
        "final split retains mode3/code42 and writes zero after the inactive block");
    for (const unsigned origin : {1000U, 31U, 1531U}) {
        auto shifted = with_extension;
        shifted.back().sample = origin;
        const auto regrouped = atmos::encode::reframe_oamd(shifted, 4608);
        std::vector<std::string> events;
        for (unsigned frame = 0; frame < regrouped.size(); ++frame) {
            auto decoded = atmos::eac3::parse_oamd_payload(regrouped[frame]);
            check(decoded.valid, "regrouped extension fragment parses");
            for (auto& update : decoded.oamd.position_updates) update.block = 0;
            for (auto& update : decoded.oamd.gain_updates) update.block = 0;
            for (auto& update : decoded.oamd.spread_updates) update.block = 0;
            for (auto& update : decoded.oamd.zone_updates) update.block = 0;
            const auto fragment = oamd_events(decoded.oamd, frame * 1536);
            events.insert(events.end(), fragment.begin(), fragment.end());
        }
        auto reference = oamd_events(extended_source.oamd, origin);
        std::sort(events.begin(), events.end()); std::sort(reference.begin(), reference.end());
        check(events == reference, "extension split preserves multi-block fragments and exact boundaries");
    }
    auto empty_extension = with_extension;
    empty_extension.back().payload.elements.back().body = {0};
    const auto empty_output = atmos::encode::reframe_oamd(empty_extension, 4608);
    for (unsigned frame = 0; frame < 3; ++frame) {
        const auto decoded = atmos::eac3::parse_oamd_payload(empty_output[frame]);
        check(decoded.valid && decoded.oamd_payload.elements.back().body == std::vector<std::uint8_t>{0},
            "empty extension splits without inventing divergence or coordinate fields");
    }
    auto bad_extension = with_extension;
    bad_extension.back().payload.elements.back().body.back() |= 1;
    check(extension_bits % 8 != 0, "extension vector has padding to corrupt");
    rejects([&] { static_cast<void>(atmos::encode::reframe_oamd(bad_extension, 4608)); },
        "nonzero extension padding fails explicitly");
    bad_extension = with_extension;
    bad_extension.back().payload.elements.back().body.pop_back();
    rejects([&] { static_cast<void>(atmos::encode::reframe_oamd(bad_extension, 4608)); },
        "truncated split extension fails explicitly");
    // Keep a playable regression artifact with the changed extension path.
    atmos::encode::CoreEncoder core;
    atmos::encode::CoreConfig config;
    config.bitrate = 768000; config.with_additional_bsi = true; config.complexity_index_type_a = 2;
    core.configure(config);
    atmos::eac3::JocFrame joc;
    joc.channels = 5; joc.object_count = 2;
    for (unsigned object = 0; object < 2; ++object) {
        auto& info = joc.objects[object];
        info.active = true; info.bands = 1; info.data_points = 1;
        info.matrix[0][object][0] = 5;
    }
    const auto tone = make_tone(48000, 1, 6);
    std::ofstream stream("logs/oamd_cross_frame_extended.ec3", std::ios::binary);
    for (unsigned frame = 0; frame < extended_output.size(); ++frame) {
        std::vector<std::uint8_t> encoded;
        joc.sequence_count = static_cast<std::uint16_t>(frame);
        const auto metadata = atmos::eac3::parse_oamd_payload(extended_output[frame]);
        atmos::encode::encode_atmos_frame(core, tone, metadata.oamd, joc, true, true, encoded, extended_output[frame]);
        const auto parsed = atmos::eac3::parse_sync_frame(encoded, 0);
        check(parsed && parsed->oamd_payload && parsed->joc_payload &&
            atmos::crc16_ansi(std::span<const std::uint8_t>(encoded).subspan(2)) == 0,
            "split metadata survives authenticated Atmos frame muxing with valid CRC");
        if (stream) stream.write(reinterpret_cast<const char*>(encoded.data()), encoded.size());
    }
    auto dependent = packets;
    dependent.back().payload.elements.front().object_records[1][1].differential = true;
    dependent.back().payload.elements.front().object_records[1][1].render.absolute_position = false;
    rejects([&] { static_cast<void>(atmos::encode::reframe_oamd(dependent, 4608)); },
        "unresolved differential snapshot at the new first block fails explicitly");
    dependent = packets;
    dependent.back().payload.elements.front().object_records[0][1].full_basic = false;
    dependent.back().payload.elements.front().object_records[0][1].basic.gain_index = 3;
    rejects([&] { static_cast<void>(atmos::encode::reframe_oamd(dependent, 4608)); },
        "unresolved basic gain at the new first block fails explicitly");
    auto truncated = packets;
    truncated.back().payload.elements.front().object_records.front().pop_back();
    rejects([&] { static_cast<void>(atmos::encode::reframe_oamd(truncated, 4608)); },
        "inconsistent split record ranges are rejected");
    rejects([&] { static_cast<void>(atmos::encode::reframe_oamd(packets, 1536)); },
        "split events beyond the output duration are rejected");
}

void test_oamd_gain_predictor() {
    const auto make_payload = [](bool repeated, unsigned partial = 0, bool full_render = false, bool snapshots = false, bool ramped = false, unsigned gain_code = 20) {
        atmos::BitWriter body;
        body.write(0, 1); body.write(0, 2); body.write(2, 3); // three blocks
        for (unsigned block = 0; block < 3; ++block) {
            body.write(block * 16, 6); body.write(ramped ? 1U : 0U, 2);
        }
        body.write_bit(true);
        for (unsigned object = 0; object < 3; ++object) {
            for (unsigned block = 0; block < 3; ++block) {
                const bool inactive = object == 1 && block == 2;
                body.write_bit(inactive);
                if (!inactive) {
                    const bool partial_priority = !snapshots && partial == 2 && object == 2 && block == 1;
                    const bool reuse = !snapshots && ((object == 0 && block == 2) || (object == 2 && block == 1)) && !partial_priority;
                    const bool partial_gain = !snapshots && partial == 1 && object == 0 && block == 1;
                    if (block) body.write(reuse ? 2U : (partial_gain || partial_priority) ? 3U : 1U, 2);
                    if (!reuse) {
                        if (partial_gain || partial_priority) body.write(partial_gain ? 1U : 2U, 2);
                        const bool repeat = object < 2 || block == 2;
                        const unsigned code = snapshots ? (object == 2 ? 1U : block ? 2U : 0U) :
                            repeated && repeat && !(object == 0 && block == 1) ? 3U :
                            (object == 2 ? 1U : block == 1 ? 2U : 0U);
                        if (!partial_priority) {
                            body.write(code, 2);
                            if (code == 2) body.write(gain_code, 6); // explicit wire gain
                        }
                        if (partial_priority) { body.write_bit(false); body.write(21, 5); }
                        else if (!partial_gain) body.write_bit(true); // default priority
                    }
                    if (object) {
                        if (block) body.write(full_render ? 1U : 2U, 2);
                        if (!block || full_render) {
                            if (block) body.write_bit(false); // absolute position
                            body.write(object * 20, 6); body.write(20, 6);
                            body.write(1, 1); body.write(10, 4); body.write(0, 1);
                            body.write(0, 3); body.write(1, 1); body.write(0, 2);
                            body.write(0, 1); body.write(0, 1);
                        }
                    }
                }
                body.write_bit(false); // no additional table data
            }
        }
        body.align();
        atmos::BitWriter payload;
        payload.write(0, 2); payload.write(2, 5); // three objects
        payload.write(1, 1); payload.write(1, 1); payload.write(0, 1); // LFE, no alternate
        payload.write(1, 4); payload.write(1, 4);
        const auto size = static_cast<unsigned>(body.bytes().size() - 1);
        check(size >= 16 && size < 272, "gain vector uses two length groups");
        payload.write((size >> 4) - 1, 4); payload.write_bit(true);
        payload.write(size & 15, 4); payload.write_bit(false);
        for (auto byte : body.bytes()) payload.write(byte, 8);
        payload.align();
        return payload.bytes();
    };
    auto repeated = make_payload(true), explicit_gain = make_payload(false);
    auto partial = make_payload(false, 1);
    const auto parsed = atmos::eac3::parse_oamd_payload(repeated);
    const auto reference = atmos::eac3::parse_oamd_payload(explicit_gain);
    const auto partial_metadata = atmos::eac3::parse_oamd_payload(partial);
    const auto priority_metadata = atmos::eac3::parse_oamd_payload(make_payload(false, 2));
    check(parsed.valid && reference.valid, "independent gain predictor vectors parse");
    check(parsed.oamd.gain_updates.size() == 9, "every object block creates a resolved gain target");
    const auto ramped = atmos::eac3::parse_oamd_payload(make_payload(false, 0, false, false, true));
    check(ramped.valid && ramped.oamd.gain_updates.size() == 9 &&
        std::all_of(ramped.oamd.gain_updates.begin(), ramped.oamd.gain_updates.end(),
            [](const auto& update) { return update.ramp_duration == 512; }),
        "gain updates, including inactive/mute records, retain the block ramp duration");
    check(oamd_events(parsed.oamd, 0) == oamd_events(reference.oamd, 0),
        "gain code3 matches explicit values through reuse and inactive predecessors");
    check(partial_metadata.valid && oamd_events(partial_metadata.oamd, 0) == oamd_events(reference.oamd, 0),
        "partial basic mask1 updates gain while retaining priority");
    check(priority_metadata.valid && oamd_events(priority_metadata.oamd, 0) == oamd_events(reference.oamd, 0),
        "partial basic mask2 retains the resolved gain target");
    for (const auto& update : parsed.oamd.gain_updates) {
        const float expected = update.object == 2 || (update.object == 1 && update.block == 2) ? 0.0F :
            update.block == 0 ? 1.0F : 2052.0F / 4096.0F;
        check(std::abs(update.gain - expected) < 1.0e-6F, "gain predictor resolves the same block of the previous object");
    }
    // Independent native OAMDI Q12 captures for all 64 explicit wire codes.
    constexpr unsigned gain_q12[] = {
        23033,20528,18296,16306,14533,12952,11544,10288,
        9169,8172,7283,6491,5785,5156,4595,3650,
        3253,2899,2584,2303,2052,1829,1630,1453,
        1295,1154,1028,916,817,728,649,578,
        515,459,409,365,325,289,258,230,
        205,182,163,145,129,115,102,91,
        81,72,64,57,51,45,40,36,
        32,28,25,23,20,18,16,14
    };
    for (unsigned code = 0; code < 64; ++code) {
        const auto grid = atmos::eac3::parse_oamd_payload(make_payload(false, 0, false, false, false, code));
        check(grid.valid && grid.oamd.gain_updates.size() == 9 &&
            grid.oamd.gain_updates[1].gain == gain_q12[code] / 4096.0F,
            "explicit gain matches the independent native Q12 table");
    }
    for (unsigned origin : {0U,256U}) for (unsigned variant = 0; variant < 4; ++variant) {
        const auto source = atmos::eac3::parse_oamd_payload(
            make_payload(variant == 1, variant >= 2 ? variant - 1 : 0));
        const auto foreign = atmos::eac3::parse_oamd_payload(make_payload(false, 0, false, false, false, 26));
        check(source.valid && foreign.valid, "crossed basic source vectors parse");
        const std::vector<atmos::encode::TimedOamdPayload> packets{
            {0, source.oamd_payload}, {origin, foreign.oamd_payload}};
        check(atmos::encode::oamd_alignment_padding(packets, 1536, 0) == 0,
            "crossed nonunity basic state needs no alignment padding");
        const auto output = atmos::encode::reframe_oamd(packets, 1536);
        const auto merged = atmos::eac3::parse_oamd_payload(output.front());
        check(merged.valid && merged.oamd.gain_updates.size() == 18,
            "crossed basic merge retains all resolved gain targets");
        for (unsigned object = 0; object < 3; ++object) {
            for (unsigned block = 0; block < 6; ++block) {
                const auto& input = block % 2 ? foreign : source;
                const auto& expected = input.oamd_payload.elements.front().object_records[object][block / 2];
                const auto& actual = merged.oamd_payload.elements.front().object_records[object][block];
                const auto& gain = merged.oamd.gain_updates[object * 6 + block];
                check(actual.inactive == expected.inactive && actual.basic.gain_index == expected.basic.gain_index &&
                    actual.basic.gain_bits == expected.basic.gain_bits && actual.basic.priority == expected.basic.priority,
                    "crossed basic snapshot retains its original gain and priority");
                check(gain.sample_offset == (block / 2) * 512 + (block % 2) * origin &&
                    gain.gain == input.oamd.gain_updates[object * 3 + block / 2].gain,
                    "crossed gain target retains the original value and exact time");
            }
        }
    }
    for (const unsigned origin : {1000U, 1500U}) {
        for (unsigned variant = 0; variant < 4; ++variant) {
            const auto raw = make_payload(variant == 1, variant >= 2 ? variant - 1 : 0, true);
            const auto source = atmos::eac3::parse_oamd_payload(raw);
            check(source.valid, "cross-frame basic source parses");
            auto initial = source.oamd_payload;
            initial.elements.clear();
            const auto fragments = atmos::encode::reframe_oamd(
                std::vector<atmos::encode::TimedOamdPayload>{{0, initial}, {origin, source.oamd_payload}}, 3072);
            for (unsigned frame = 0; frame < fragments.size(); ++frame) {
                const auto decoded = atmos::eac3::parse_oamd_payload(fragments[frame]);
                check(decoded.valid, "expanded basic fragment parses");
                if (decoded.oamd_payload.elements.empty()) continue;
                const auto& records = decoded.oamd_payload.elements.front().object_records;
                for (unsigned object = 0; object < records.size(); ++object) {
                    for (unsigned block = 0; block < records[object].size(); ++block) {
                        const auto at = std::find_if(decoded.oamd.gain_updates.begin(), decoded.oamd.gain_updates.end(),
                            [&](const auto& update) { return update.object == object && update.block == block; });
                        unsigned original_block = 0;
                        while ((origin + original_block * 512) / 1536 < frame) ++original_block;
                        original_block += block;
                        const auto& expected = source.oamd_payload.elements.front().object_records[object][original_block];
                        const auto& actual = records[object][block];
                        check(actual.basic.gain_index == expected.basic.gain_index &&
                            actual.basic.gain_bits == expected.basic.gain_bits && actual.basic.priority == expected.basic.priority,
                            "split retains exact resolved gain and priority codes through partial/reuse/default updates");
                        if (block == 0) check(at != decoded.oamd.gain_updates.end(),
                            "new first block explicitly supplies resolved gain including inactivity");
                    }
                }
            }
        }
    }
    // Equal EMDF sizes keep canonical audio identical for the independent DRP comparison.
    const auto bytes = std::max({repeated.size(), explicit_gain.size(), partial.size()});
    repeated.resize(bytes, 0); explicit_gain.resize(bytes, 0); partial.resize(bytes, 0);
    atmos::encode::CoreEncoder a, b, c;
    atmos::encode::CoreConfig config;
    config.bitrate = 768000; config.with_additional_bsi = true; config.complexity_index_type_a = 3;
    a.configure(config); b.configure(config); c.configure(config);
    atmos::eac3::JocFrame joc;
    joc.channels = 5; joc.object_count = 2;
    for (unsigned object = 0; object < 2; ++object) {
        joc.objects[object].active = true; joc.objects[object].bands = 1;
        joc.objects[object].data_points = 1; joc.objects[object].matrix[0][object][0] = 5;
    }
    const auto tone = make_tone(48000, 1, 6);
    std::ofstream repeated_stream("logs/oamd_gain_repeated.ec3", std::ios::binary);
    std::ofstream explicit_stream("logs/oamd_gain_explicit.ec3", std::ios::binary);
    std::ofstream partial_stream("logs/oamd_gain_partial.ec3", std::ios::binary);
    for (unsigned frame = 0; frame < 5; ++frame) {
        joc.sequence_count = static_cast<std::uint16_t>(frame);
        std::vector<std::uint8_t> x, y, z;
        atmos::encode::encode_atmos_frame(a, tone, parsed.oamd, joc, true, true, x, repeated);
        atmos::encode::encode_atmos_frame(b, tone, reference.oamd, joc, true, true, y, explicit_gain);
        atmos::encode::encode_atmos_frame(c, tone, partial_metadata.oamd, joc, true, true, z, partial);
        check(atmos::encode::protection_audio(x) == atmos::encode::protection_audio(y) &&
            atmos::encode::protection_audio(x) == atmos::encode::protection_audio(z),
            "gain predictor comparison keeps canonical audio identical");
        repeated_stream.write(reinterpret_cast<const char*>(x.data()), x.size());
        explicit_stream.write(reinterpret_cast<const char*>(y.data()), y.size());
        partial_stream.write(reinterpret_cast<const char*>(z.data()), z.size());
    }
    for (const unsigned origin : {1000U, 1500U}) {
        const auto split = [&](bool snapshots) {
            const auto source = atmos::eac3::parse_oamd_payload(make_payload(false, 1, true, snapshots));
            auto initial = source.oamd_payload;
            initial.elements.clear();
            return atmos::encode::reframe_oamd(
                std::vector<atmos::encode::TimedOamdPayload>{{0, initial}, {origin, source.oamd_payload}}, 3072);
        };
        auto candidate = split(false), control = split(true);
        atmos::encode::CoreEncoder candidate_encoder, control_encoder;
        candidate_encoder.configure(config); control_encoder.configure(config);
        const auto prefix = "logs/oamd_basic_split_" + std::to_string(origin);
        std::ofstream candidate_stream(prefix + "_partial.ec3", std::ios::binary);
        std::ofstream control_stream(prefix + "_full.ec3", std::ios::binary);
        for (unsigned frame = 0; frame < candidate.size(); ++frame) {
            const auto length = std::max(candidate[frame].size(), control[frame].size());
            candidate[frame].resize(length, 0); control[frame].resize(length, 0);
            const auto candidate_metadata = atmos::eac3::parse_oamd_payload(candidate[frame]);
            const auto control_metadata = atmos::eac3::parse_oamd_payload(control[frame]);
            joc.sequence_count = static_cast<std::uint16_t>(frame);
            std::vector<std::uint8_t> x, y;
            atmos::encode::encode_atmos_frame(candidate_encoder, tone, candidate_metadata.oamd, joc, true, true, x, candidate[frame]);
            atmos::encode::encode_atmos_frame(control_encoder, tone, control_metadata.oamd, joc, true, true, y, control[frame]);
            check(atmos::encode::protection_audio(x) == atmos::encode::protection_audio(y),
                "basic split DRP control has identical canonical audio");
            candidate_stream.write(reinterpret_cast<const char*>(x.data()), x.size());
            control_stream.write(reinterpret_cast<const char*>(y.data()), y.size());
        }
    }
}

void test_oamd_render_state() {
    const auto make_payload = [](bool full) {
        atmos::BitWriter body;
        body.write(0, 1); body.write(0, 2); body.write(3, 3);
        for (unsigned time : {0U, 16U, 32U, 40U}) { body.write(time, 6); body.write(0, 2); }
        body.write_bit(true);
        for (unsigned object = 0; object < 3; ++object) {
            for (unsigned block = 0; block < 4; ++block) {
                body.write_bit(false);
                if (block) body.write(1, 2);
                body.write(0, 2); body.write_bit(true); // full unity gain/default priority
                if (object) {
                    const bool defaults = object == 2 && block == 3;
                    unsigned mask = 15;
                    if (block && !full) {
                        if (object == 2 && block == 2) { body.write(2, 2); mask = 0; }
                        else if (defaults) { body.write(0, 2); mask = 0; }
                        else {
                            body.write(3, 2);
                            mask = block == 1 ? (object == 1 ? 2U : 4U) : block == 2 ? 1U : 8U;
                            body.write(mask, 4);
                        }
                    } else if (block) body.write(1, 2);
                    if (mask & 1) {
                        if (block) body.write_bit(false);
                        body.write(defaults ? 31U : object == 1 && block >= 2 ? 41U : object * 10, 6);
                        body.write(defaults ? 31U : object == 1 && block >= 2 ? 33U : 20U, 6);
                        body.write_bit(defaults || object != 1 || block < 2);
                        body.write(defaults ? 0U : object == 1 && block >= 2 ? 7U : 5U, 4);
                        body.write_bit(!defaults);
                        if (!defaults) {
                            const bool infinity = object == 1 && block >= 2;
                            body.write_bit(infinity);
                            if (!infinity) body.write(3, 4);
                        }
                    }
                    if (mask & 2) {
                        body.write(defaults ? 0U : object == 1 && block ? 3U : object, 3);
                        body.write_bit(defaults || object != 1 || block == 0);
                    }
                    if (mask & 4) {
                        const unsigned size = defaults ? 0U : object == 2 && block ? 1U : 2U;
                        body.write(size, 2);
                        if (size == 1) body.write(13, 5);
                        if (size == 2) for (unsigned code : {7U,8U,9U}) body.write(code, 5);
                    }
                    if (mask & 8) {
                        body.write_bit(!defaults);
                        if (!defaults) {
                            body.write(object == 1 && block == 3 ? 2U : 5U, 3);
                            body.write(object == 1 && block == 3 ? 3U : 1U, 2);
                        }
                    }
                    if (mask) body.write_bit(!defaults && (block == 0 || (object == 1 && block == 2)));
                }
                body.write_bit(true); body.write(0, 4); body.write(0xa0 + block, 8);
            }
        }
        body.align();
        atmos::BitWriter payload;
        payload.write(0, 2); payload.write(2, 5); payload.write(1, 1); payload.write(1, 1);
        payload.write(0, 1); payload.write(1, 4); payload.write(1, 4);
        const auto size = static_cast<unsigned>(body.bytes().size() - 1);
        check(size >= 16 && size < 272, "render vector uses two length groups");
        payload.write((size >> 4) - 1, 4); payload.write_bit(true);
        payload.write(size & 15, 4); payload.write_bit(false);
        for (auto byte : body.bytes()) payload.write(byte, 8);
        payload.align();
        return payload.bytes();
    };
    const auto partial = atmos::eac3::parse_oamd_payload(make_payload(false));
    const auto full = atmos::eac3::parse_oamd_payload(make_payload(true));
    check(partial.valid && full.valid, "independent partial/full render vectors parse");
    if (!partial.valid || !full.valid) return;
    const auto snapshot = [](const atmos::eac3::OamdRenderInfo& info) {
        return std::tuple{info.x, info.y, info.z, info.positive_z, info.absolute_position,
            info.distance_specified, info.infinity, info.distance, info.zone, info.elevation,
            info.size_index, info.size, info.screen_reference, info.screen_factor, info.depth_factor, info.snap};
    };
    for (unsigned object = 0; object < 3; ++object)
        for (unsigned block = 0; block < 4; ++block)
            check(snapshot(partial.oamd_payload.elements.front().object_records[object][block].render) ==
                snapshot(full.oamd_payload.elements.front().object_records[object][block].render),
                "partial masks/reuse/default resolve every exact render code to the independent full snapshot");
    const std::array<atmos::encode::TimedOamdPayload,2> crossed{{{0,partial.oamd_payload},{128,full.oamd_payload}}};
    const std::array<atmos::encode::TimedOamdPayload,2> crossed_reference{{{0,full.oamd_payload},{128,full.oamd_payload}}};
    check(atmos::encode::oamd_alignment_padding(crossed,1536,0) == 0,
        "crossed render masks, reuse and defaults need no padding when basic fields are independent");
    const auto crossed_frames = atmos::encode::reframe_oamd(crossed,1536);
    const auto reference_frames = atmos::encode::reframe_oamd(crossed_reference,1536);
    check(crossed_frames == reference_frames,
        "crossed render materialization is byte-exact to independent full snapshots, including opaque tails");
    const auto crossed_metadata = atmos::eac3::parse_oamd_payload(crossed_frames[0]);
    check(crossed_metadata.valid && crossed_metadata.oamd_payload.elements.size() == 1,
        "all eight crossed render blocks occupy one native object element");
    if (crossed_metadata.valid) {
        const auto& records = crossed_metadata.oamd_payload.elements.front().object_records;
        for (unsigned object = 0; object < records.size(); ++object) for (unsigned block = 0; block < records[object].size(); ++block) {
            const auto& expected = (block%2 ? full : partial).oamd_payload.elements.front().object_records[object][block/2];
            check(snapshot(records[object][block].render) == snapshot(expected.render) &&
                records[object][block].basic.gain_index == expected.basic.gain_index &&
                records[object][block].basic.priority == expected.basic.priority,
                "crossed expansion preserves signed height, distance/infinity, zones, dimensions, screen/depth, snap and basic fields");
        }
    }
    auto corrupted = partial.oamd_payload;
    corrupted.elements.front().object_records[1][1].render.size_index = 3;
    auto initial = corrupted;
    initial.elements.clear();
    bool rejected = false;
    try {
        static_cast<void>(atmos::encode::reframe_oamd(
            std::vector<atmos::encode::TimedOamdPayload>{{0, initial}, {1500, corrupted}}, 3072));
    } catch (const std::exception&) { rejected = true; }
    check(rejected, "reserved size in resolved render state fails explicitly");
    corrupted = partial.oamd_payload;
    corrupted.elements.front().object_records[2][2].render.absolute_position = false;
    rejected = false;
    try {
        static_cast<void>(atmos::encode::reframe_oamd(
            std::vector<atmos::encode::TimedOamdPayload>{{0, initial}, {1000, corrupted}}, 3072));
    } catch (const std::exception&) { rejected = true; }
    check(rejected, "reuse of unresolved differential position at a split fails explicitly");
    for (unsigned origin : {300U,1000U,1500U}) {
        const auto split = [&](const auto& source) {
            auto initial = source.oamd_payload;
            initial.elements.clear();
            return atmos::encode::reframe_oamd(
                std::vector<atmos::encode::TimedOamdPayload>{{0, initial}, {origin, source.oamd_payload}}, 3072);
        };
        auto candidate = split(partial), control = split(full);
        atmos::encode::CoreEncoder a, b;
        atmos::encode::CoreConfig config;
        config.bitrate = 768000; config.with_additional_bsi = true; config.complexity_index_type_a = 3;
        a.configure(config); b.configure(config);
        atmos::eac3::JocFrame joc;
        joc.channels = 5; joc.object_count = 2;
        for (unsigned object = 0; object < 2; ++object) {
            joc.objects[object].active = true; joc.objects[object].bands = 1;
            joc.objects[object].data_points = 1; joc.objects[object].matrix[0][object][0] = 5;
        }
        const auto tone = make_tone(48000, 1, 6);
        const auto prefix = "logs/oamd_render_split_" + std::to_string(origin);
        std::ofstream candidate_stream(prefix + "_partial.ec3", std::ios::binary);
        std::ofstream control_stream(prefix + "_full.ec3", std::ios::binary);
        for (unsigned frame = 0; frame < candidate.size(); ++frame) {
            const auto length = std::max(candidate[frame].size(), control[frame].size());
            candidate[frame].resize(length, 0); control[frame].resize(length, 0);
            const auto x_metadata = atmos::eac3::parse_oamd_payload(candidate[frame]);
            const auto y_metadata = atmos::eac3::parse_oamd_payload(control[frame]);
            check(x_metadata.valid && y_metadata.valid, "partial render fragment parses");
            if (!x_metadata.oamd_payload.elements.empty()) {
                const auto& x = x_metadata.oamd_payload.elements.front().object_records;
                const auto& y = y_metadata.oamd_payload.elements.front().object_records;
                for (unsigned object = 0; object < x.size(); ++object)
                    for (unsigned block = 0; block < x[object].size(); ++block)
                        check(snapshot(x[object][block].render) == snapshot(y[object][block].render),
                            "split partial render retains position, distance, zone, size, screen factors and snap codes");
            }
            joc.sequence_count = static_cast<std::uint16_t>(frame);
            std::vector<std::uint8_t> x, y;
            atmos::encode::encode_atmos_frame(a, tone, x_metadata.oamd, joc, true, true, x, candidate[frame]);
            atmos::encode::encode_atmos_frame(b, tone, y_metadata.oamd, joc, true, true, y, control[frame]);
            check(atmos::encode::protection_audio(x) == atmos::encode::protection_audio(y),
                "render split DRP controls have identical canonical audio");
            candidate_stream.write(reinterpret_cast<const char*>(x.data()), x.size());
            control_stream.write(reinterpret_cast<const char*>(y.data()), y.size());
        }
    }
}

void test_oamd_differential_position() {
    constexpr int positions[2][5][3] = {
        {{61,1,14},{62,0,15},{58,3,11},{58,3,11},{58,0,7}},
        {{1,61,-14},{0,62,-15},{3,58,-12},{3,58,-12},{0,58,-9}}
    };
    const auto make_payload = [&](bool differential, bool ramps, bool extreme = false, bool fine = false, unsigned zone = 0, bool snap = false, int screen = -1, unsigned depth = 0) {
        atmos::BitWriter body;
        body.write(0, 1); body.write(0, 2); body.write(4, 3);
        for (unsigned block = 0; block < 5; ++block) {
            body.write(block * 8, 6);
            if (!ramps || block == 0) body.write(0, 2);
            else if (block < 3) body.write(block, 2); // 512,1536
            else {
                body.write(3, 2); body.write_bit(block == 3);
                body.write(block == 3 ? 0U : 1111U, block == 3 ? 4U : 11U); // 32,1111
            }
        }
        body.write_bit(true);
        for (unsigned object = 0; object < 3; ++object) {
            for (unsigned block = 0; block < 5; ++block) {
                body.write_bit(false);
                if (block) body.write(1, 2);
                body.write(0, 2); body.write_bit(true);
                if (object) {
                    const bool reuse = differential && block == 3;
                    if (block) body.write(reuse ? 2U : 1U, 2);
                    if (!reuse) {
                        if (block) body.write_bit(differential);
                        const auto& position = positions[object - 1][block];
                        if (differential && block) {
                            const int delta[2][3][3] = {
                                {{3,-4,3},{-4,3,-4},{0,-4,-4}},
                                {{-4,3,-4},{3,-4,3},{-4,0,3}}
                            };
                            const unsigned index = block == 4 ? 2U : block - 1;
                            for (int value : delta[object - 1][index]) body.write(static_cast<unsigned>(value) & 7U, 3);
                        } else {
                            body.write(extreme && object == 1 && block == 0 ? 63U : static_cast<unsigned>(position[0]), 6);
                            body.write(static_cast<unsigned>(position[1]), 6);
                            body.write_bit(position[2] >= 0); body.write(static_cast<unsigned>(std::abs(position[2])), 4);
                        }
                        body.write_bit(false); // room distance
                        body.write(zone, 3); body.write_bit(true); // zones
                        body.write(0, 2); body.write_bit(screen >= 0);
                        if (screen >= 0) { body.write(static_cast<unsigned>(screen),3); body.write(depth,2); }
                        body.write_bit(snap); // size/screen/snap
                    }
                }
                body.write_bit(true); body.write(0, 4); body.write(0xb0 + block, 8);
            }
        }
        body.align();
        atmos::BitWriter payload;
        payload.write(0, 2); payload.write(2, 5); payload.write(1, 1); payload.write(1, 1);
        payload.write(0, 1); payload.write(fine ? 2U : 1U, 4); payload.write(1, 4);
        const auto size = static_cast<unsigned>(body.bytes().size() - 1);
        check(size >= 16 && size < 272, "differential vector uses two length groups");
        payload.write((size >> 4) - 1, 4); payload.write_bit(true);
        payload.write(size & 15, 4); payload.write_bit(false);
        for (auto byte : body.bytes()) payload.write(byte, 8);
        if (fine) {
            atmos::BitWriter extension;
            extension.write_bit(false); extension.write_bit(true); // prefix, divergence
            for (unsigned object = 0; object < 2; ++object) {
                for (unsigned block = 0; block < 5; ++block) {
                    const bool zero = object == 0 ? block == 4 : block == 0;
                    const bool inherit = object == 0 ? block == 1 || block == 3 : block == 2 || block == 4;
                    extension.write_bit(!zero);
                    if (zero) continue;
                    if (inherit) extension.write(1, 2);
                    else if (block == 0 || block == 3) {
                        extension.write(0, 2); extension.write(object == 0 ? 2U : 3U, 2);
                    } else {
                        extension.write(object == 0 ? 2U : 3U, 2);
                        extension.write(object == 0 ? 42U : 31U, 6);
                    }
                }
            }
            extension.write_bit(true); // extended position precision
            for (unsigned object = 0; object < 2; ++object) {
                for (unsigned block = 0; block < 5; ++block) {
                    constexpr unsigned masks[] = {7,3,5,6,0};
                    const unsigned mask = masks[block];
                    extension.write_bit(mask != 0);
                    if (!mask) continue;
                    extension.write(mask, 3);
                    if (mask & 1) extension.write((object + block) % 4, 2);
                    if (mask & 2) extension.write((block + 2) % 4, 2);
                    if (mask & 4) extension.write((3 - block) % 4, 2);
                }
            }
            extension.align();
            check(extension.bytes().size() <= 16, "combined extension uses one length group");
            payload.write(5, 4); payload.write(static_cast<unsigned>(extension.bytes().size() - 1), 4);
            payload.write_bit(false);
            for (auto byte : extension.bytes()) payload.write(byte, 8);
        }
        payload.align();
        return payload.bytes();
    };
    for (unsigned zone : {6U,7U}) {
        const auto source=atmos::eac3::parse_oamd_payload(make_payload(true,false,false,true,zone,true));
        check(source.valid, "zone6/7 differential/snap source parses");
        const std::array<atmos::encode::TimedOamdPayload,1> packets{{{0,source.oamd_payload}}};
        const auto frames=atmos::encode::reframe_oamd(packets,3072,768);
        const auto split=atmos::eac3::parse_oamd_payload(frames[1]);
        check(split.valid && !split.oamd.position_updates.empty() && split.oamd.position_updates[0].snap &&
            split.oamd.zone_updates[0].horizontal==zone,
            "split predictor expansion writes exact zone6/7 and snap at a frame boundary");
    }
    atmos::encode::CoreEncoder screen_core;
    atmos::encode::CoreConfig screen_config;
    screen_config.bitrate=768000; screen_config.with_additional_bsi=true; screen_config.complexity_index_type_a=2;
    screen_core.configure(screen_config);
    auto screen_joc=std::make_unique<atmos::eac3::JocFrame>();
    screen_joc->channels=5; screen_joc->object_count=2;
    for(unsigned object=0;object<2;++object) {
        auto& info=screen_joc->objects[object];info.active=info.fine_quantization=true;
        info.bands=info.data_points=1;info.matrix[0][object][0]=10;
    }
    std::ofstream screen_stream("logs/oamd_screen_split.ec3",std::ios::binary);
    if(!screen_stream)throw std::runtime_error("cannot open screen split fixture");
    const std::vector<float> screen_pcm(1536*6,0);
    unsigned screen_sequence=0;
    for (unsigned screen=0;screen<8;++screen) for(unsigned depth=0;depth<4;++depth) {
        const auto source=atmos::eac3::parse_oamd_payload(make_payload(true,false,false,true,7,true,screen,depth));
        check(source.valid, "screen/depth differential source parses");
        const std::array<atmos::encode::TimedOamdPayload,1> packets{{{0,source.oamd_payload}}};
        const auto frames=atmos::encode::reframe_oamd(packets,3072,768);
        for(unsigned frame=0;frame<2;++frame) {
            const auto metadata=atmos::eac3::parse_oamd_payload(frames[frame]);
            screen_joc->sequence_count=static_cast<std::uint16_t>(screen_sequence++);
            std::vector<std::uint8_t> encoded;
            atmos::encode::encode_atmos_frame(screen_core,screen_pcm,metadata.oamd,*screen_joc,true,true,encoded,frames[frame]);
            screen_stream.write(reinterpret_cast<const char*>(encoded.data()),encoded.size());
        }
        const auto split=atmos::eac3::parse_oamd_payload(frames[1]);
        check(split.valid && !split.oamd_payload.elements.empty(), "screen/depth split payload parses");
        if (split.valid && !split.oamd_payload.elements.empty()) {
            const auto& info=split.oamd_payload.elements[0].object_records[1][0].render;
            check(info.screen_reference && info.screen_factor==screen && info.depth_factor==depth && info.snap && info.zone==7,
                "split predictor snapshot preserves every screen/depth code and coupled panning fields");
        }
    }
    screen_stream.close();
    if(!screen_stream)throw std::runtime_error("screen split fixture write failed");
    const auto extreme = atmos::eac3::parse_oamd_payload(make_payload(true, false, true));
    check(extreme.valid && extreme.oamd_payload.elements.front().object_records[1][0].render.x == 62 &&
        extreme.oamd.position_updates.front().x == 1.0F,
        "absolute code63 is clamped before it becomes the differential predictor");
    for (bool fine : {false,true}) for (bool ramps : {false,true}) {
        const auto source = atmos::eac3::parse_oamd_payload(make_payload(true, ramps, false, fine));
        const auto reference = atmos::eac3::parse_oamd_payload(make_payload(false, ramps, false, fine));
        check(source.valid && reference.valid, "differential and independently coded absolute vectors parse");
        if (!source.valid || !reference.valid) continue;
        check(oamd_events(source.oamd, 0) == oamd_events(reference.oamd, 0),
            "resolved renderer events match the explicit absolute source through reuse and nonzero ramps");
        for (unsigned object = 1; object < 3; ++object) {
            for (unsigned block = 0; block < 5; ++block) {
                const auto& record = source.oamd_payload.elements.front().object_records[object][block];
                const auto& expected = positions[object - 1][block];
                check(record.render.absolute_position && record.render.x == expected[0] && record.render.y == expected[1] &&
                    (record.render.positive_z ? 1 : -1) * record.render.z == expected[2],
                    "integer differential targets clamp before the next update and survive reuse");
            }
        }
        for (const auto& update : source.oamd.position_updates) {
            const auto& expected = positions[update.object - 1][update.block];
            constexpr unsigned masks[] = {7,3,5,6,0};
            constexpr float offsets[] = {1,2,-1,-2};
            const unsigned mask = fine ? masks[update.block] : 0;
            const float dx = mask & 1 ? offsets[(update.object - 1 + update.block) % 4] / 310.0F : 0;
            const float dy = mask & 2 ? offsets[(update.block + 2) % 4] / 310.0F : 0;
            const float dz = mask & 4 ? offsets[(3 - update.block) % 4] / 75.0F : 0;
            check(!update.differential && update.x == float(expected[0]) / 62.0F + dx &&
                update.y == float(expected[1]) / 62.0F + dy && update.z == float(expected[2]) / 15.0F + dz,
                "renderer receives an absolute target rather than adding a delta during a ramp");
        }
        for (unsigned origin : {600U,1200U,1500U}) {
            const auto split = [&](const auto& metadata) {
                auto initial = metadata.oamd_payload;
                initial.elements.clear();
                return atmos::encode::reframe_oamd(
                    std::vector<atmos::encode::TimedOamdPayload>{{0, initial}, {origin, metadata.oamd_payload}}, 3072);
            };
            auto candidate = split(source), control = split(reference);
            atmos::encode::CoreEncoder a, b;
            atmos::encode::CoreConfig config;
            config.bitrate = 768000; config.with_additional_bsi = true; config.complexity_index_type_a = 3;
            a.configure(config); b.configure(config);
            atmos::eac3::JocFrame joc;
            joc.channels = 5; joc.object_count = 2;
            for (unsigned object = 0; object < 2; ++object) {
                joc.objects[object].active = true; joc.objects[object].bands = 1;
                joc.objects[object].data_points = 1; joc.objects[object].matrix[0][object][0] = 5;
            }
            const auto tone = make_tone(48000, 1, 6);
            const auto prefix = "logs/oamd_delta_" + std::to_string(origin) + (ramps ? "_ramp" : "_zero") + (fine ? "_fine" : "");
            std::ofstream candidate_stream(prefix + "_delta.ec3", std::ios::binary);
            std::ofstream control_stream(prefix + "_absolute.ec3", std::ios::binary);
            const bool negative = ramps && origin == 600;
            atmos::encode::CoreEncoder negative_encoder;
            std::ofstream negative_stream;
            if (negative) {
                negative_encoder.configure(config);
                negative_stream.open(prefix + "_moved.ec3", std::ios::binary);
            }
            for (unsigned frame = 0; frame < candidate.size(); ++frame) {
                const auto length = std::max(candidate[frame].size(), control[frame].size());
                candidate[frame].resize(length, 0); control[frame].resize(length, 0);
                const auto x_metadata = atmos::eac3::parse_oamd_payload(candidate[frame]);
                const auto y_metadata = atmos::eac3::parse_oamd_payload(control[frame]);
                check(x_metadata.valid && y_metadata.valid, "differential split fragments parse");
                joc.sequence_count = static_cast<std::uint16_t>(frame);
                std::vector<std::uint8_t> x, y;
                atmos::encode::encode_atmos_frame(a, tone, x_metadata.oamd, joc, true, true, x, candidate[frame]);
                atmos::encode::encode_atmos_frame(b, tone, y_metadata.oamd, joc, true, true, y, control[frame]);
                check(atmos::encode::protection_audio(x) == atmos::encode::protection_audio(y),
                    "differential DRP comparison keeps canonical audio identical");
                candidate_stream.write(reinterpret_cast<const char*>(x.data()), x.size());
                control_stream.write(reinterpret_cast<const char*>(y.data()), y.size());
                if (negative) {
                    auto altered = x_metadata.oamd_payload;
                    if (!altered.elements.empty()) {
                        auto& element = altered.elements.front();
                        const auto& record = element.object_records[1][0];
                        const unsigned x_code = record.render.x == 0 ? 62U : 0U;
                        for (unsigned bit = 0; bit < 6; ++bit) {
                            const auto at = record.render_begin + bit;
                            const auto mask = static_cast<std::uint8_t>(1U << (7U - at % 8));
                            element.body[at / 8] = static_cast<std::uint8_t>((element.body[at / 8] & ~mask) |
                                (((x_code >> (5U - bit)) & 1U) ? mask : 0U));
                        }
                    }
                    auto raw = atmos::encode::reframe_oamd(
                        std::vector<atmos::encode::TimedOamdPayload>{{0, altered}}, 1536).front();
                    check(raw.size() <= length, "negative position control fits the same metadata size");
                    raw.resize(length, 0);
                    std::vector<std::uint8_t> moved;
                    const auto metadata = atmos::eac3::parse_oamd_payload(raw);
                    atmos::encode::encode_atmos_frame(negative_encoder, tone, metadata.oamd, joc, true, true, moved, raw);
                    check(atmos::encode::protection_audio(x) == atmos::encode::protection_audio(moved),
                        "changed-position negative control has exactly identical canonical audio");
                    negative_stream.write(reinterpret_cast<const char*>(moved.data()), moved.size());
                }
            }
        }
    }
}

void test_truehd_dialogue_normalization() {
    atmos::BitWriter header;
    header.write(0xf8726fba, 32); header.write(0, 32); // 48kHz format
    header.write(0xb752, 16); header.write(0x1000, 16);
    header.write(0, 16); header.write(0, 16);
    header.write(4, 4); header.write(0, 4); header.write(0x80, 8);
    header.write(0, 17); // DRC/control fields
    header.write(35, 6); header.write(29, 6); // 2ch norm/mix
    header.write(29, 5); header.write(35, 6); header.write(0, 5);
    header.write(27, 5); header.write(35, 6); header.write(0, 6);
    header.write(0, 1); header.write(1, 1);
    header.write(1, 4); // Extended channel meaning: 32 bits including length.
    header.write(25, 5); header.write(35, 6); header.write(15, 5);
    header.write(1, 1); header.write(0, 11); header.write(0, 16); // padding, CRC placeholder
    const auto parsed = atmos::truehd::parse_major_sync(header.bytes(), 0);
    check(parsed && parsed->header_size == 32 && parsed->tracks_in_16ch == 16 &&
        parsed->objects_only && parsed->dialogue_normalization[0] == 35 &&
        parsed->dialogue_normalization[1] == 29 && parsed->dialogue_normalization[2] == 27 &&
        parsed->dialogue_normalization[3] == 25,
        "TrueHD preserves each presentation's distinct dialogue normalization");
    auto missing = header.bytes(); missing[25] &= 0xfe; missing.resize(28);
    const auto base = atmos::truehd::parse_major_sync(missing, 0);
    check(base && base->header_size == 28 && !base->dialogue_normalization[3] &&
        base->dialogue_normalization[1] == 29,
        "missing 16ch channel meaning does not invent a dialogue level");
    auto short_extra = header.bytes(); short_extra[26] &= 15;
    check(!atmos::truehd::parse_major_sync(short_extra, 0),
        "16ch meaning cannot consume checksum bits beyond its declared extension");
    atmos::BitWriter cbi;
    for (unsigned bit = 0; bit < 208; ++bit)
        cbi.write_bit((header.bytes()[bit / 8] >> (7 - bit % 8)) & 1);
    cbi.write(2, 4); // 48-bit extended channel meaning.
    cbi.write(25, 5); cbi.write(35, 6); cbi.write(15, 5); cbi.write_bit(false);
    cbi.write(1, 4); cbi.write(0, 4); cbi.write(511, 10); cbi.write(0, 9); cbi.write(0, 16);
    const auto fixed = atmos::truehd::parse_major_sync(cbi.bytes(), 0);
    check(fixed && fixed->channel_based_bed_ids == std::vector<std::uint8_t>({0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}),
        "CBI 16-channel assignment preserves side/rear/height/wide signal order");
    auto distributed = cbi.bytes(); distributed[233 / 8] |= 1U << (7 - 233 % 8);
    const auto distributed_info = atmos::truehd::parse_major_sync(distributed, 0);
    check(distributed_info && distributed_info->channel_based_bed_ids.empty(),
        "distributed spatial channels are not mistaken for a fixed bed");
    auto truncated_cbi = cbi.bytes(); truncated_cbi[26] = (truncated_cbi[26] & 15) | 16;
    check(!atmos::truehd::parse_major_sync(truncated_cbi, 0),
        "CBI channel assignment must fit its declared extension");
}

std::vector<std::uint8_t> make_truehd_evo_unit(std::span<const std::uint8_t> payload) {
    std::vector<std::uint8_t> unit(6, 0); // AU header, empty substream directory
    const std::size_t words = (payload.size() + 4U) / 2U;
    const unsigned header = static_cast<unsigned>(words) |
        (((15U ^ (words >> 8U) ^ (words >> 4U) ^ words) & 15U) << 12U);
    unit.push_back(static_cast<std::uint8_t>(header >> 8U));
    unit.push_back(static_cast<std::uint8_t>(header));
    unit.push_back(static_cast<std::uint8_t>(payload.size() >> 8U));
    unit.push_back(static_cast<std::uint8_t>(payload.size()));
    unit.insert(unit.end(), payload.begin(), payload.end());
    unit.resize(8U + words * 2U, 0);
    unsigned parity = 0xa9;
    for (std::size_t i = 8U; i + 1U < unit.size(); ++i) parity ^= unit[i];
    unit.back() = static_cast<std::uint8_t>(parity);
    unit[0] = static_cast<std::uint8_t>((unit.size() / 2U) >> 8U);
    unit[1] = static_cast<std::uint8_t>(unit.size() / 2U);
    return unit;
}

void test_evolution_offset() {
    atmos::eac3::OamdFrame scene;
    scene.object_count = 2; scene.bed_channels = 1; scene.bed_channel_ids = {3};
    atmos::eac3::OamdPositionUpdate position;
    position.object = 1; position.x = 1; position.z = 1;
    scene.position_updates.push_back(position);
    const auto oamd = atmos::encode::write_oamd_payload(scene);
    for (const unsigned offset : {17U, 2048U}) {
        atmos::BitWriter evo;
        evo.write(0, 2); evo.write(0, 3); evo.write(11, 5);
        evo.write_bit(true); // outer offset present; frame-aligned bit is absent
        if (offset == 2048) {
            evo.write(0, 11); evo.write_bit(true);
            evo.write(0, 11); evo.write_bit(false);
        } else {
            evo.write(offset, 11); evo.write_bit(false);
        }
        evo.write_bit(false); evo.write_bit(false); // no duration/group
        evo.write_bit(true); evo.write(8, 8); // TrueHD codec data
        evo.write_bit(false); evo.write(0, 5); evo.write(0, 2); // discard/priority/processing
        evo.write(static_cast<unsigned>(oamd.size()), 8); evo.write_bit(false);
        for (const auto byte : oamd) evo.write(byte, 8);
        evo.write(0, 5); evo.write(0, 2); evo.write(1, 2); evo.write(0, 8);
        evo.align();
        const auto records = atmos::truehd::parse_evolution_frame(evo.bytes());
        const auto* parsed = records.empty() ? nullptr : &records.front();
        check(records.size() == 1 && parsed->valid && parsed->oamd_payload.sample_offset == offset,
            "TrueHD EVO keeps one- and two-group outer sample offsets");
        if (!parsed || !parsed->valid) continue;
        const auto output = atmos::encode::reframe_oamd(
            std::vector<atmos::encode::TimedOamdPayload>{{0, parsed->oamd_payload}}, 3072);
        const auto moved = atmos::eac3::parse_oamd_payload(output[offset / 1536]);
        check(moved.valid && moved.oamd.position_updates.size() == 1 &&
            moved.oamd.position_updates[0].sample_offset == offset % 1536 &&
            moved.oamd.position_updates[0].x == 1 && moved.oamd.position_updates[0].z == 1,
            "outer EVO offset becomes exact inner DD+ OAMD timing");
        if (offset >= 1536) {
            const auto initial = atmos::eac3::parse_oamd_payload(output.front());
            check(initial.valid && initial.oamd.object_count == 2 && initial.oamd.position_updates.empty(),
                "delayed first event retains configuration without inventing an event at zero");
        }
        auto truncated = evo.bytes(); truncated.resize(truncated.size() / 2);
        const auto invalid = atmos::truehd::parse_evolution_frame(truncated);
        check(invalid.size() == 1 && !invalid.front().valid && !invalid.front().error.empty(),
            "malformed EVO is reported instead of silently losing metadata");

        atmos::truehd::MajorSync info;
        info.substream_count = 1; info.flags = 0x1000;
        auto unit = make_truehd_evo_unit(evo.bytes());
        const auto wrapped = atmos::truehd::parse_access_unit(unit, 0, &info);
        check(wrapped && wrapped->atmos_metadata.size() == 1 && wrapped->atmos_metadata.front().valid &&
            wrapped->atmos_metadata.front().oamd_payload.sample_offset == offset,
            "validated TrueHD extra-data wrapper preserves OAMD");
        for (const std::size_t at : {std::size_t{6}, std::size_t{10}, unit.size() - 1U}) {
            auto corrupt = unit; corrupt[at] ^= 1U;
            check(!atmos::truehd::parse_access_unit(corrupt, 0, &info),
                "TrueHD rejects corrupt extra-data header, EVO bytes and parity");
        }
        auto oversized = unit;
        oversized[8] = 15; oversized[9] = 255;
        check(!atmos::truehd::parse_access_unit(oversized, 0, &info),
            "TrueHD EVO cannot consume padding or parity outside its declared block");
        auto short_block = unit;
        short_block[6] = 0xd0; short_block[7] = 2; // Valid header nibble, four-byte body.
        check(!atmos::truehd::parse_access_unit(short_block, 0, &info),
            "EVO fitting the AU still cannot exceed its declared extra-data length");
        auto empty = unit;
        std::fill(empty.begin() + 8U, empty.end(), std::uint8_t{0});
        empty.back() = 0xa9;
        const auto no_frame = atmos::truehd::parse_access_unit(empty, 0, &info);
        check(no_frame && no_frame->atmos_metadata.empty(),
            "zero-length EVO is padding, not an invented metadata frame");
        empty[10] = 1; empty.back() ^= 1;
        check(!atmos::truehd::parse_access_unit(empty, 0, &info),
            "nonzero EVO padding fails even with a matching parity byte");
        empty[10] = 0; empty[8] = 0xb0; empty.back() = 0xa9 ^ 0x0b;
        check(atmos::truehd::parse_access_unit(empty, 0, &info).has_value(),
            "EVO reserved nibble uses the LG unshifted parity seed");
    }
}

void test_evolution_payloads() {
    atmos::eac3::OamdFrame scene;
    scene.object_count = 2; scene.bed_channels = 1; scene.bed_channel_ids = {3};
    atmos::eac3::OamdPositionUpdate position;
    position.object = 1; position.z = 1;
    scene.position_updates.push_back(position);
    const auto first = atmos::encode::write_oamd_payload(scene);
    scene.position_updates[0].x = 1; scene.position_updates[0].z = 0;
    const auto second = atmos::encode::write_oamd_payload(scene);
    const auto append = [](atmos::BitWriter& bits, const std::vector<std::uint8_t>& payload,
                           unsigned offset, std::optional<unsigned> presentations) {
        bits.write(11, 5);
        bits.write_bit(true); bits.write(offset, 11); bits.write_bit(false);
        bits.write_bit(false); bits.write_bit(false); // no duration/group
        bits.write_bit(presentations.has_value());
        if (presentations) bits.write(*presentations, 8);
        bits.write_bit(false); bits.write(0, 7); // priority/processing
        bits.write(static_cast<unsigned>(payload.size()), 8); bits.write_bit(false);
        for (auto byte : payload) bits.write(byte, 8);
    };
    atmos::BitWriter evo;
    evo.write(0, 5); // version/key
    append(evo, second, 0, 1); // Belongs to the 2ch presentation, not object presentation 3.
    append(evo, first, 17, 8);
    append(evo, second, 1600, 8);
    evo.write(0, 5); evo.write(3, 2); evo.write(2, 2);
    for (unsigned i = 0; i < 20; ++i) evo.write(0xa0 + i, 8);
    evo.align();
    const auto parsed = atmos::truehd::parse_evolution_frame(evo.bytes());
    atmos::truehd::MajorSync info;
    info.substream_count = 1; info.flags = 0x1000;
    const auto unit = atmos::truehd::parse_access_unit(make_truehd_evo_unit(evo.bytes()), 0, &info);
    check(unit && unit->atmos_metadata.size() == 2 && unit->atmos_metadata[0].valid &&
        unit->atmos_metadata[1].valid, "TrueHD AU preserves both selected EVO payloads");
    check(parsed.size() == 2 && parsed[0].valid && parsed[1].valid &&
        parsed[0].oamd_payload.sample_offset == 17 && parsed[1].oamd_payload.sample_offset == 1600,
        "all object-presentation OAMD payloads survive one EVO frame in order");
    if (parsed.size() == 2 && parsed[0].valid && parsed[1].valid) {
        check(parsed[0].oamd_payload.elements[0].body ==
                atmos::eac3::parse_oamd_payload(first).oamd_payload.elements[0].body &&
            parsed[1].oamd_payload.elements[0].body ==
                atmos::eac3::parse_oamd_payload(second).oamd_payload.elements[0].body,
            "multiple EVO payloads retain their original object record bits");
        const auto output = atmos::encode::reframe_oamd(
            std::vector<atmos::encode::TimedOamdPayload>{{0, parsed[0].oamd_payload}, {0, parsed[1].oamd_payload}}, 3072);
        const auto before = atmos::eac3::parse_oamd_payload(output[0]);
        const auto after = atmos::eac3::parse_oamd_payload(output[1]);
        check(before.valid && after.valid && before.oamd.position_updates.size() == 1 &&
            after.oamd.position_updates.size() == 1 &&
            before.oamd.position_updates[0].sample_offset == 17 && before.oamd.position_updates[0].z == 1 &&
            after.oamd.position_updates[0].sample_offset == 64 && after.oamd.position_updates[0].x == 1,
            "both EVO payloads reach their exact DD+ frame and sample");
        atmos::encode::CoreEncoder core;
        atmos::encode::CoreConfig config;
        config.bitrate = 768000; config.with_additional_bsi = true; config.complexity_index_type_a = 2;
        core.configure(config);
        atmos::eac3::JocFrame joc;
        joc.channels = 5; joc.object_count = 1;
        joc.objects[0].active = true; joc.objects[0].bands = 1;
        joc.objects[0].data_points = 1; joc.objects[0].matrix[0][0][0] = 5;
        const auto tone = make_tone(48000, 1, 6);
        std::ofstream stream("logs/evo_multiple_oamd.ec3", std::ios::binary);
        for (unsigned frame = 0; frame < output.size(); ++frame) {
            std::vector<std::uint8_t> encoded;
            joc.sequence_count = static_cast<std::uint16_t>(frame);
            const auto metadata = atmos::eac3::parse_oamd_payload(output[frame]);
            atmos::encode::encode_atmos_frame(core, tone, metadata.oamd, joc, true, true, encoded, output[frame]);
            stream.write(reinterpret_cast<const char*>(encoded.data()), static_cast<std::streamsize>(encoded.size()));
        }
        check(static_cast<bool>(stream), "multi-payload EVO regression artifact is written");
    }
    const auto invalid = [](const std::vector<std::uint8_t>& bytes) {
        const auto records = atmos::truehd::parse_evolution_frame(bytes);
        return records.size() == 1 && !records[0].valid && !records[0].error.empty();
    };
    auto truncated = evo.bytes(); truncated.pop_back();
    check(invalid(truncated), "truncated EVO protection cannot hide behind a valid first OAMD");
    auto trailing = evo.bytes(); trailing.push_back(1);
    check(invalid(trailing), "nonzero trailing EVO padding is rejected");
    atmos::BitWriter common;
    common.write(0, 5); append(common, first, 17, std::nullopt); common.write(0, 9); common.align();
    const auto shared = atmos::truehd::parse_evolution_frame(common.bytes());
    check(shared.size() == 1 && shared[0].valid && shared[0].oamd_payload.sample_offset == 17,
        "OAMD without a presentation mask applies to the object presentation");
    atmos::BitWriter missing;
    missing.write(0, 5); append(missing, first, 17, 8); missing.align();
    check(invalid(missing.bytes()), "EVO requires a terminator and protection length fields");
    atmos::BitWriter runaway;
    runaway.write(0, 5); append(runaway, first, 17, 8);
    runaway.write(11, 5); runaway.write_bit(true);
    for (unsigned i = 0; i < 2; ++i) { runaway.write(0, 11); runaway.write_bit(true); }
    runaway.align();
    check(invalid(runaway.bytes()), "unterminated later offset cannot silently discard an OAMD");
    atmos::BitWriter overflow;
    overflow.write(0, 5); overflow.write(11, 5);
    overflow.write_bit(false); overflow.write_bit(false); overflow.write_bit(true);
    for (unsigned i = 0; i < 16; ++i) { overflow.write(3, 2); overflow.write_bit(true); }
    overflow.align();
    check(invalid(overflow.bytes()), "EVO variable-bit groups cannot wrap uint32");
}



void test_joc_surround_phase() {
    // Native inverse phase oracle: impulse (0.25,-0.125) in QMF band zero.
    constexpr std::array<std::uint32_t, 42> expected{
        0x3a0e89a8U, 0xba2d8bfcU, 0x3abc930aU, 0xbafbdb94U, 0x3b3049f5U, 0xbb836dcfU,
        0x3b889072U, 0xbbeb91aeU, 0x3bb79042U, 0xbc4099f6U, 0x3bd6786aU, 0xbc947594U,
        0x3bd33c82U, 0xbcdac65eU, 0x3ba33850U, 0xbd18fcaeU, 0x3b239a28U, 0xbd46dc46U,
        0x38df9d00U, 0xbd6b089eU, 0xbe00e7d0U, 0xbe9f1d14U, 0x38df9d00U, 0xbd6b089eU,
        0x3b239a28U, 0xbd46dc46U, 0x3ba33850U, 0xbd18fcaeU, 0x3bd33c82U, 0xbcdac65eU,
        0x3bd6786aU, 0xbc947594U, 0x3bb79042U, 0xbc4099f6U, 0x3b889072U, 0xbbeb91aeU,
        0x3b3049f5U, 0xbb836dcfU, 0x3abc930aU, 0xbafbdb94U, 0x3a0e89a8U, 0xba2d8bfcU,
    };
    atmos::eac3::qmf::SurroundPhase phase, plain;
    for (unsigned slot = 0; slot < 32; ++slot) {
        atmos::eac3::qmf::ComplexBands input;
        if (!slot) { input.real[0] = .25F; input.imaginary[0] = -.125F; input.real[7] = .25F; }
        const auto output = phase.process(input, true);
        const auto delayed = plain.process(input, false);
        check(delayed.real[7] == (slot == 10 ? .25F : 0), "unshifted surround has ten QMF slots of delay");
        check(output.real[7] == 0 && output.imaginary[7] == (slot == 10 ? -.25F : 0),
            "inverse surround phase rotates delayed upper bands by minus ninety degrees");
        if (slot < 21)
            check(std::bit_cast<std::uint32_t>(output.real[0]) == expected[2*slot] &&
                std::bit_cast<std::uint32_t>(output.imaginary[0]) == expected[2*slot+1],
                "surround low-band FIR reproduces the native impulse including state history");
        else check(output.real[0] == 0 && output.imaginary[0] == 0, "surround impulse drains completely");
    }
}

void test_emdf() {
    {
        // Native first10 23-band wrap case: float modulo flips band19 from -9.6 to +9.6.
        constexpr std::array<std::uint8_t,23> deltas{
            0,0,0,191,1,0,0,0,0,0,95,97,0,0,0,0,0,0,0,96,169,119,0};
        atmos::eac3::JocFrame dense;
        dense.channels=5; dense.object_count=1;
        auto& object=dense.objects[0];
        object.active=object.fine_quantization=true;
        object.bands_index=7; object.bands=23; object.data_points=1;
        object.matrix[0][1]=deltas;
        auto sparse=dense; sparse.objects[0].sparse=true;
        sparse.objects[0].sparse_channel[0][0]=1;
        unsigned value=0,previous=100;
        for (unsigned band=0;band<23;++band) {
            value=(value+deltas[band])%192;
            const auto current=(value+96)%192;
            sparse.objects[0].sparse_vector[0][band]=static_cast<std::uint8_t>((current+192-previous)%192);
            previous=current;
        }
        atmos::eac3::JocDecoder dense_decoder,sparse_decoder;
        std::vector<float> pcm(1536*6),a,b;
        std::uint32_t random=7319;
        for (auto& sample:pcm) { random=random*1664525U+1013904223U; sample=(float(random>>8)/16777216-.5F)*.01F; }
        std::string error;
        for (unsigned frame=0;frame<3;++frame) {
            check(dense_decoder.render(dense,pcm,6,1536,a,error) &&
                sparse_decoder.render(sparse,pcm,6,1536,b,error) && a==b,
                "dense integer predictor wraps before float conversion, matching sparse native codes");
        }
    }
    for (unsigned selector = 0; selector <= 4; ++selector) {
        atmos::BitWriter payload;
        payload.write(selector, 3); payload.write(0, 9);
        payload.write(0x80, 8); payload.write(0, 10);
        payload.write_bit(false); payload.align();
        atmos::BitWriter frame;
        frame.write(0, 5); frame.write(14, 5); frame.write(0, 4); frame.write_bit(true);
        frame.write(static_cast<unsigned>(payload.bytes().size()), 8); frame.write_bit(false);
        for (auto byte : payload.bytes()) frame.write(byte, 8);
        frame.write(0, 5); frame.write(1, 2); frame.write(0, 2); frame.write(0, 8); frame.align();
        const auto parsed = atmos::eac3::parse_emdf_frame(frame.bytes());
        check(parsed.valid && parsed.has_joc &&
            atmos::encode::write_joc_payload(parsed.joc) == payload.bytes(),
            "JOC preserves the wire downmix selector, not only the channel count");
        auto inconsistent = parsed.joc;
        inconsistent.channels = inconsistent.channels == 5 ? 7 : 5;
        rejects([&] { static_cast<void>(atmos::encode::write_joc_payload(inconsistent)); },
            "JOC writer rejects a preserved selector with inconsistent carriers");
        inconsistent = parsed.joc; inconsistent.downmix_config = 5;
        rejects([&] { static_cast<void>(atmos::encode::write_joc_payload(inconsistent)); },
            "JOC writer rejects reserved downmix selectors");
        if (selector >= 3) {
            atmos::eac3::JocDecoder decoder;
            std::vector<float> bed(1536 * (parsed.joc.channels + 1), .1F), objects;
            std::string error;
            check(decoder.render(parsed.joc, bed, parsed.joc.channels + 1, 1536, objects, error) &&
                std::all_of(objects.begin(), objects.end(), [](float x) { return x == 0; }),
                "JOC renderer supports phase-coded downmix with inactive objects");
        }
    }
    for (const unsigned channels : {5U, 7U}) for (const bool fine : {false, true}) for (const unsigned points : {1U, 2U}) {
        atmos::eac3::JocFrame sparse;
        sparse.channels = static_cast<std::uint8_t>(channels); sparse.object_count = 1;
        auto& object = sparse.objects[0];
        object.active = object.sparse = true; object.fine_quantization = fine;
        object.bands_index = 2; object.bands = 5; object.data_points = static_cast<std::uint8_t>(points);
        const unsigned modulus = fine ? 192 : 96;
        for (unsigned point = 0; point < points; ++point) {
            object.sparse_channel[point][0] = static_cast<std::uint8_t>(channels - 1);
            for (unsigned band = 0; band < 5; ++band) {
                if (band) object.sparse_channel[point][band] = static_cast<std::uint8_t>((band + point) % channels);
                object.sparse_vector[point][band] = static_cast<std::uint8_t>((modulus - 1 + band + point) % modulus);
            }
        }
        const auto payload = atmos::encode::write_joc_payload(sparse);
        const auto parsed = atmos::eac3::parse_emdf(atmos::encode::build_emdf({}, sparse, false, true));
        check(parsed.valid && parsed.has_joc && parsed.joc.objects[0].sparse &&
            atmos::encode::write_joc_payload(parsed.joc) == payload, "sparse JOC 5/7channel fine/coarse one/two-point round-trip");
        for (unsigned point = 0; point < points; ++point) {
            object.sparse_vector[point].fill(0);
            object.sparse_vector[point][0] = static_cast<std::uint8_t>(modulus - (fine ? 4 : 2));
        }
        atmos::eac3::JocDecoder decoder;
        std::vector<float> carrier(1536 * (channels + 1), .1F), decoded;
        std::string error;
        check(decoder.render(sparse, carrier, static_cast<std::uint8_t>(channels + 1), 1536, decoded, error) &&
            std::all_of(decoded.begin(), decoded.end(), [](float x) { return x == 0; }),
            "sparse zero coefficients must not leak unselected carriers");
        auto dense = sparse;
        dense.objects[0].sparse = false;
        constexpr std::array<int, 5> gains{5, 3, -2, 1, -5};
        for (unsigned point = 0; point < points; ++point) {
            int previous_value = fine ? 4 : 2;
            unsigned selected = object.sparse_channel[point][0];
            std::array<int, 7> previous{};
            for (unsigned band = 0; band < 5; ++band) {
                const int value = gains[band] * (fine ? 2 : 1);
                object.sparse_vector[point][band] = static_cast<std::uint8_t>((value - previous_value + int(modulus)) % int(modulus));
                previous_value = value;
                if (band) selected = (selected + object.sparse_channel[point][band]) % channels;
                for (unsigned channel = 0; channel < channels; ++channel) {
                    const int current = channel == selected ? value : 0;
                    dense.objects[0].matrix[point][channel][band] = static_cast<std::uint8_t>((current - previous[channel] + int(modulus)) % int(modulus));
                    previous[channel] = current;
                }
            }
        }
        std::uint32_t random = 41;
        for (auto& sample : carrier) {
            random = random * 1664525U + 1013904223U;
            sample = (float(random >> 8) / 16777216.0F - .5F) * .2F;
        }
        atmos::eac3::JocDecoder sparse_decoder, dense_decoder;
        std::vector<float> sparse_pcm, dense_pcm;
        const bool rendered = sparse_decoder.render(sparse, carrier, static_cast<std::uint8_t>(channels + 1), 1536, sparse_pcm, error) &&
            dense_decoder.render(dense, carrier, static_cast<std::uint8_t>(channels + 1), 1536, dense_pcm, error);
        double energy = 0, difference = 0;
        if (rendered) for (unsigned sample = 0; sample < sparse_pcm.size(); ++sample) {
            energy += double(dense_pcm[sample]) * dense_pcm[sample];
            const double delta = double(dense_pcm[sample]) - sparse_pcm[sample]; difference += delta * delta;
        }
        check(rendered && energy > 1e-8 && difference < energy * 1e-8,
            "sparse channel/gain predictors match the equivalent dense matrix");
        object.sparse_channel[0][0] = static_cast<std::uint8_t>(channels);
        rejects([&] { static_cast<void>(atmos::encode::write_joc_payload(sparse)); }, "sparse initial channel bound validated");
        object.sparse_channel[0][0] = 0; object.sparse_channel[0][1] = static_cast<std::uint8_t>(channels);
        rejects([&] { static_cast<void>(atmos::encode::write_joc_payload(sparse)); }, "sparse channel delta bound validated");
        object.sparse_channel[0][1] = 0; object.sparse_vector[0][0] = 255;
        rejects([&] { static_cast<void>(atmos::encode::write_joc_payload(sparse)); }, "sparse vector code bound validated");
    }
    for (const auto [code, gain] : std::array<std::pair<unsigned, float>, 6>{{
             {0x00, .0625F}, {0x7f, .984375F}, {0x80, 1.0F},
             {0xa2, 2.125F}, {0xc2, 4.25F}, {0xff, 15.75F}}}) {
        atmos::BitWriter payload;
        payload.write(0, 12); // 5-channel downmix, one object, no extension
        payload.write(code, 8);
        payload.write(0, 10); // sequence
        payload.write_bit(false); // inactive object
        payload.align();
        atmos::BitWriter frame;
        frame.write(0, 5); // EMDF version/key
        frame.write(14, 5); frame.write(0, 4); frame.write_bit(true);
        frame.write(static_cast<unsigned>(payload.bytes().size()), 8); frame.write_bit(false);
        for (auto byte : payload.bytes()) frame.write(byte, 8);
        frame.write(0, 5); frame.write(1, 2); frame.write(0, 2); frame.write(0, 8);
        frame.align();
        const auto decoded = atmos::eac3::parse_emdf_frame(frame.bytes());
        check(decoded.valid && decoded.has_joc && decoded.joc_gain == gain && decoded.joc.gain == gain,
            "JOC gain exponent scales both the implicit one and the fractional mantissa");
        check(atmos::encode::write_joc_payload(decoded.joc) == payload.bytes(),
            "JOC writer reproduces native gain header bits exactly");
    }
    for (unsigned code = 0; code < 256; ++code) {
        atmos::eac3::JocFrame joc;
        joc.channels = 5; joc.object_count = 1;
        joc.gain = std::ldexp(1.0F + static_cast<float>(code & 31) / 32, static_cast<int>(code >> 5) - 4);
        const auto payload = atmos::encode::write_joc_payload(joc);
        check(payload.size() == 4 && ((unsigned(payload[1] & 15) << 4) | (payload[2] >> 4)) == code,
            "JOC writer preserves every exponent/mantissa gain code");
    }
    for (const float gain : {0.0F, .03125F, 16.0F, 1.01F, std::numeric_limits<float>::infinity(),
                            std::numeric_limits<float>::quiet_NaN()}) {
        atmos::eac3::JocFrame joc;
        joc.channels = 5; joc.object_count = 1; joc.gain = gain;
        rejects([&] { static_cast<void>(atmos::encode::write_joc_payload(joc)); },
            "JOC writer rejects invalid or unrepresentable gain without quantizing it");
    }
    atmos::eac3::OamdFrame oamd;
    oamd.object_count = 9; // six 5.1 bed channels plus three dynamic objects
    oamd.bed_channels = 6;
    for (unsigned object = 6; object < 9U; ++object) {
        atmos::eac3::OamdPositionUpdate update;
        update.object = static_cast<std::uint16_t>(object);
        update.x = 0.1F + 0.3F * static_cast<float>(object - 6U);
        update.y = 0.5F;
        update.z = object == 7U ? -0.5F : 0.0F;
        oamd.position_updates.push_back(update);
    }
    atmos::eac3::JocFrame joc;
    joc.channels = 5;
    joc.object_count = 3;
    for (unsigned i = 0; i < 3; ++i) {
        joc.objects[i].active = true;
        joc.objects[i].bands_index = 0;
        joc.objects[i].bands = 1;
        joc.objects[i].data_points = 1;
        joc.objects[i].matrix[0][i][0] = 5; // +1.0, coarse differential code
    }

    const auto emdf = atmos::encode::build_emdf(oamd, joc, true, true);
    check(emdf.size() > 4U, "emdf has a body");
    check(emdf[0] == 0x58U && emdf[1] == 0x38U, "emdf sync word");
    const auto parsed = atmos::eac3::parse_emdf(emdf);
    check(parsed.valid, "emdf container parses");
    check(parsed.has_oamd, "OAMD payload present");
    check(parsed.has_joc, "JOC payload present");
    check(parsed.oamd_objects == 9U, "OAMD object count round-trips");
    check(parsed.oamd_position_updates == 3U, "OAMD positions round-trip");
    check(parsed.joc_objects == 3U, "JOC object count round-trips");
    check(parsed.joc_channels == 5U, "JOC bed channel count round-trips");
    auto truncated = emdf;
    truncated.pop_back();
    check(!atmos::eac3::parse_emdf(truncated).valid, "truncated EMDF is rejected");
    check(parsed.joc.objects[1].matrix[0][1][0] == 5, "JOC matrix survives serialization");
    if (parsed.oamd.position_updates.size() > 1)
        check(parsed.oamd.position_updates[1].z < 0, "negative OAMD height survives serialization");
    check(parsed.oamd.zone_updates.size() == 3 && parsed.oamd.zone_updates[0].elevation,
        "default OAMD zones allow height rendering");
    oamd.zone_updates.push_back({7, 0, 0, 2, false});
    const auto constrained = atmos::eac3::parse_emdf(atmos::encode::build_emdf(oamd, joc, true, true));
    check(constrained.valid && constrained.oamd.zone_updates.size() == 3 &&
        constrained.oamd.zone_updates[1].horizontal == 2 && !constrained.oamd.zone_updates[1].elevation,
        "explicit horizontal and height zone constraints survive serialization");

    atmos::encode::CoreEncoder encoder;
    configure_51(encoder);
    const auto tone = make_tone(48000, 1, encoder.channels());
    encoder.set_aux_payload(emdf);
    std::vector<std::uint8_t> frame;
    encoder.encode(tone, frame);
    atmos::eac3::BedDecoder decoder;
    decoder.reset();
    atmos::eac3::PcmFrame pcm;
    std::string decode_error;
    check(decoder.decode(frame, pcm, decode_error), "Atmos frame decodes");
    check(pcm.metadata.has_oamd, "decoded frame carries OAMD");
    check(pcm.metadata.has_joc, "decoded frame carries JOC");

    // Dependent substream (canonical DD+ Atmos carries JOC/OAMD here).
    atmos::encode::CoreEncoder object_encoder;
    atmos::encode::CoreConfig object_config;
    object_config.sample_rate = 48000;
    object_config.audio_coding_mode = 7;
    object_config.lfe = true;
    object_config.blocks = 6;
    object_config.bitrate = 640000;
    object_config.stream_type = 1;
    object_encoder.configure(object_config);
    object_encoder.set_aux_payload(emdf);
    std::vector<std::uint8_t> object_frame;
    object_encoder.encode(tone, object_frame);
    const auto parsed_frame = atmos::eac3::parse_sync_frame(object_frame, 0U);
    check(parsed_frame.has_value(), "dependent substream frame parses");
    if (parsed_frame) {
        check(parsed_frame->stream_type == 1U, "frame is a dependent substream");
        check(parsed_frame->joc_payload && parsed_frame->oamd_payload,
            "dependent substream carries Atmos");
    }
}

void test_oamd_final_element_length() {
    const std::vector<std::uint8_t> source={23,132,70,34,128,52,96,137,192,2,1,3,252,4,2,7,124,8,4,14,7,208,8,31,239,160,16,56,15,192,32,127,159,128,64,224,1,240,129,254,3,225,3,129,247,194,7,251,239,132,0};
    const auto parsed=atmos::eac3::parse_oamd_payload(source);
    check(parsed.valid && parsed.oamd.object_count==12 && parsed.oamd.bed_channel_ids==std::vector<std::uint8_t>{3} && parsed.oamd.position_updates.size()==11,
        "Lucy complete final object element accepts native-compatible understated length");
    if(parsed.valid){
        check(parsed.oamd_payload.elements.size()==1 && parsed.oamd_payload.elements[0].body.size()==47,
            "understated last element retains all parsed object bytes with canonical padding");
        check(parsed.oamd.position_updates[0].sample_offset==16 && parsed.oamd.position_updates[0].ramp_duration==1120,
            "Lucy first native metadata timing retained");
        const atmos::encode::TimedOamdPayload packet{0,parsed.oamd_payload};
        const auto reframed=atmos::encode::reframe_oamd(std::span(&packet,1),1536,0);
        check(!reframed.empty() && atmos::eac3::parse_oamd_payload(reframed[0]).valid,
            "accepted final object element can be serialized canonically");
    }
    auto nonfinal=source;nonfinal[1]=static_cast<std::uint8_t>((nonfinal[1]&0xc3)|8);
    check(!atmos::eac3::parse_oamd_payload(nonfinal).valid,"understated intermediate element remains rejected");
    auto truncated=source;truncated.pop_back();
    check(!atmos::eac3::parse_oamd_payload(truncated).valid,"truncated final object data remains rejected");
    auto overstated=source;
    for(unsigned bit=0;bit<10;++bit){const unsigned at=18+bit,mask=1U<<(7-at%8);
        overstated[at/8]=static_cast<std::uint8_t>((overstated[at/8]&~mask)|(((358U>>(9-bit))&1U)?mask:0));}
    const auto large=atmos::eac3::parse_oamd_payload(overstated);
    check(large.valid && large.oamd.position_updates.size()==11 && large.oamd_payload.elements[0].body.size()==47,
        "native final object length recovery is bounded by actual data even when size is overstated");
}

void test_oamd_bed_headers() {
    // Independent wire vectors: content bits 0=bed, 1=ISF, 2=dynamic; masks use speaker ID as bit index.
    const std::array<std::uint8_t, 5> explicit_51{0x0a, 0x10, 0x00, 0x1f, 0x80};
    const std::array<std::uint8_t, 4> standard_51{0x0a, 0x11, 0x03, 0xc0};
    for (const auto bytes : {std::span<const std::uint8_t>(explicit_51), std::span<const std::uint8_t>(standard_51)}) {
        const auto parsed = atmos::eac3::parse_oamd_payload(bytes);
        check(parsed.valid && parsed.oamd.object_count == 6 && parsed.oamd.bed_channels == 6 &&
            parsed.oamd.bed_channel_ids == std::vector<std::uint8_t>{0,1,2,3,4,5},
            "independent standard and explicit 5.1 assignments use numeric bitmasks");
    }
    const std::array<std::uint8_t, 3> dynamic_only{0x02, 0x40, 0x80};
    const auto dynamic = atmos::eac3::parse_oamd_payload(dynamic_only);
    check(dynamic.valid && dynamic.oamd.object_count == 2 && dynamic.oamd.bed_channels == 0 &&
        dynamic.oamd_payload.header_bits == 18, "content bit 2 selects the dynamic object count");
    const std::array<std::uint8_t, 3> isf_only{0x06, 0x20, 0x00};
    const auto isf = atmos::eac3::parse_oamd_payload(isf_only);
    check(isf.valid && isf.oamd.object_count == 4 && isf.oamd.isf_channels == 4 && isf.oamd_payload.header_bits == 16,
        "content bit 1 selects ISF rather than dynamic objects");
    rejects([&] { static_cast<void>(atmos::encode::joc_target_channels(isf.oamd)); },
        "ISF signals must not be encoded as dynamic JOC objects");
    rejects([&] { static_cast<void>(atmos::encode::write_oamd_payload(isf.oamd)); },
        "ISF signals must not be rewritten as dynamic OAMD objects");
    atmos::render::Layout isf_layout;
    static_cast<void>(atmos::render::parse_layout("5.1", isf_layout));
    const std::array<atmos::render::MetadataFrame, 1> isf_timeline{{{0,isf.oamd}}};
    bool read_isf = false, wrote_isf = false;
    std::string isf_error;
    check(!atmos::render::render_objects_stream(4, 1536, isf_timeline, isf_layout,
        [&](std::span<float>) {read_isf=true;},
        [&](std::span<const float>, const atmos::render::CarrierHistory*) {wrote_isf=true;}, isf_error) &&
        isf_error.find("ISF") != std::string::npos && !read_isf && !wrote_isf,
        "renderer rejects ISF before consuming PCM or producing a wrong scene");
    const std::array<std::uint8_t, 4> reserved_only{0x00, 0x80, 0xab, 0x00};
    const auto reserved = atmos::eac3::parse_oamd_payload(reserved_only);
    check(reserved.valid && reserved.oamd_payload.header_bits == 25,
        "content bit 3 skips reserved bytes rather than parsing a bed");
    atmos::eac3::OamdFrame metadata;
    metadata.object_count = metadata.bed_channels = 6;
    metadata.bed_channel_ids = {0,1,2,3,4,5};
    const auto written = atmos::encode::write_oamd_payload(metadata);
    const auto initialized_bed = atmos::eac3::parse_oamd_payload(written);
    check(initialized_bed.valid && std::equal(initialized_bed.oamd_payload.header.begin(),
        initialized_bed.oamd_payload.header.end(), explicit_51.begin(), explicit_51.end()),
        "5.1 OAMD writer header matches an independent wire vector");
    check(initialized_bed.oamd.gain_updates.size() == 6,
        "bed-only programs initialize basic object state instead of emitting a silent header-only payload");
    metadata.object_count = 7;
    check(atmos::encode::joc_target_channels(metadata) == std::vector<unsigned>{0,1,2,4,5,6},
        "5.1 bed and dynamic signals preserve order while LFE at index 3 is excluded");
    metadata.object_count = 9; metadata.bed_channels = 8; metadata.bed_channel_ids = {0,1,2,3,4,5,6,7};
    check(atmos::encode::joc_target_channels(metadata) == std::vector<unsigned>{0,1,2,4,5,6,7,8},
        "7.1 rear bed signals are included in JOC targets");
    metadata.object_count = 3; metadata.bed_channels = 1; metadata.bed_channel_ids = {3};
    check(atmos::encode::joc_target_channels(metadata) == std::vector<unsigned>{1,2},
        "existing LFE-only layout keeps the same target order");
    metadata.bed_channel_ids = {16};
    rejects([&] { static_cast<void>(atmos::encode::joc_target_channels(metadata)); }, "LFE2 is not silently dropped");
    metadata.bed_channel_ids = {3,3}; metadata.bed_channels = 2;
    rejects([&] { static_cast<void>(atmos::encode::joc_target_channels(metadata)); }, "multiple LFE signals are rejected");
    metadata.bed_channels = 0; metadata.bed_channel_ids.clear(); metadata.object_count = 2;
    check(atmos::encode::joc_target_channels(metadata) == std::vector<unsigned>{0,1},
        "a dynamic-only presentation keeps its first full-band signal");
    metadata.object_count = 15;
    check(atmos::encode::joc_target_channels(metadata).size() == 15,
        "native-compatible JOC accepts 15 full-band signals");
    metadata.object_count = 16;
    rejects([&] { static_cast<void>(atmos::encode::joc_target_channels(metadata)); },
        "16 full-band signals exceed the native decoder limit");
    metadata.bed_channels = 1; metadata.bed_channel_ids = {3};
    check(atmos::encode::joc_target_channels(metadata).size() == 15,
        "16 source signals including LFE retain all 15 full-band targets");
    metadata.object_count = 17;
    rejects([&] { static_cast<void>(atmos::encode::joc_target_channels(metadata)); },
        "LFE plus 16 full-band signals exceed the native decoder limit");
    metadata.bed_channels = 0; metadata.bed_channel_ids.clear();
    rejects([&] { static_cast<void>(atmos::encode::joc_target_channels(metadata)); }, "more than 16 JOC signals are rejected");
    metadata.object_count = metadata.bed_channels = 1; metadata.bed_channel_ids = {3};
    rejects([&] { static_cast<void>(atmos::encode::joc_target_channels(metadata)); }, "LFE alone has no JOC full-band target");
    metadata.bed_channel_ids.clear();
    rejects([&] { static_cast<void>(atmos::encode::joc_target_channels(metadata)); }, "bed channel IDs must match the declared count");
}

void test_carrier_gain_ramps() {
    atmos::render::Layout layout;
    check(atmos::render::parse_layout("5.1", layout), "gain-ramp carrier layout parses");
    atmos::render::MetadataFrame metadata;
    metadata.oamd.object_count = 1;
    metadata.oamd.position_updates.emplace_back();
    metadata.oamd.gain_updates = {{0,256,0,.5F,512}, {0,448,0,2,256}, {0,1152,0,0,384}};
    std::vector<float> output;
    std::string error;
    check(atmos::render::render_objects(std::vector<float>(3072, 1), 1, 3072,
        std::span(&metadata, 1), layout, output, error), "gain-only carrier scene renders");
    // Independent initialized DEE renderer: carrier-gain-production-verified/gain-only.f32.
    check(output[255 * 6] == 1 && output[256 * 6] == .9990234375F &&
        output[447 * 6] == .8125F && output[448 * 6] == .817138671875F &&
        output[703 * 6] == 2 && output[1536 * 6] == 0,
        "gain-only ramps retain time and retarget from the current channel gain before muting");
    for (std::uint8_t channel = 0; channel < 6; ++channel) {
        auto bed = metadata;
        bed.oamd.bed_channels = 1;
        bed.oamd.bed_channel_ids = {channel};
        bed.oamd.position_updates.clear();
        bed.oamd.gain_updates.insert(bed.oamd.gain_updates.begin(), {0,0,0,1,256});
        check(atmos::render::render_objects(std::vector<float>(3072, 1), 1, 3072,
            std::span(&bed, 1), layout, output, error), "matching bed carrier scene renders");
        const float level = channel < 4 ? 1.0F : .750109851360321F;
        check(std::abs(output[channel] - .00390625F * level) < 1e-7F &&
            std::abs(output[255 * 6 + channel] - level) < 1e-6F &&
            std::abs(output[256 * 6 + channel] - .9990234375F * level) < 1e-6F &&
            std::abs(output[448 * 6 + channel] - .817138671875F * level) < 1e-6F &&
            std::abs(output[703 * 6 + channel] - 2 * level) < 1e-6F && output[1536 * 6 + channel] == 0,
            "native bed startup, gain retarget and mute use the channel-coefficient ramp");
        for (unsigned other = 0; other < 6; ++other) if (other != channel)
            check(output[512 * 6 + other] == 0, "bed gain stays in its assigned carrier");
    }
    auto timed_gain = metadata.oamd;
    timed_gain.gain_updates.resize(1);
    timed_gain.gain_updates.front().sample_offset = 0;
    rejects([&] { static_cast<void>(atmos::encode::write_oamd_payload(timed_gain)); },
        "the static OAMD writer refuses timed gain updates instead of silently discarding ramps");
    metadata.oamd.gain_updates[0].gain = std::numeric_limits<float>::infinity();
    check(!atmos::render::render_objects(std::vector<float>(32, 1), 1, 32,
        std::span(&metadata, 1), layout, output, error) && error == "invalid object gain",
        "invalid gain is rejected at the renderer boundary");
}

void test_carrier_gain() {
    atmos::encode::joc::CarrierGain gain;
    std::vector<float> pcm(1536 * 6);
    pcm[0] = .125F;
    pcm[1535 * 6 + 5] = -.125F;
    check(gain.process(pcm) == 128 && pcm[0] == 0 && pcm[256 * 6] == .5F,
        "native mode2 carrier gain applies quarter-headroom compensation after a 256-sample delay");
    std::fill(pcm.begin(), pcm.end(), 0.0F);
    check(gain.process(pcm) == 128 && pcm[255 * 6 + 5] == -.5F,
        "carrier gain retains the delayed frame tail");
    atmos::encode::joc::CarrierGain lfe;
    std::fill(pcm.begin(), pcm.end(), 0.0F);
    for (unsigned t = 0; t < 1536; ++t) pcm[t * 6 + 3] = 1;
    check(lfe.process(pcm) == 128 && pcm[256 * 6 + 3] == 4,
        "LFE is delayed and scaled but excluded from JOC normalization peak detection");
    auto bad = pcm;
    bad.back() = std::numeric_limits<float>::quiet_NaN();
    rejects([&] { gain.process(bad); }, "carrier gain rejects nonfinite frames");
    rejects([&] { gain.process(std::span<float>(pcm).first(pcm.size() - 1)); },
        "carrier gain rejects incomplete frames");
    rejects([&] { atmos::encode::joc::CarrierGain invalid(5); }, "carrier gain rejects reserved headroom modes");
    atmos::encode::joc::CarrierGain loud;
    std::fill(pcm.begin(), pcm.end(), 1.0F);
    check(loud.process(pcm) > 128, "carrier gain encodes non-unity gain under overload");
}

void test_carrier_limiter() {
    atmos::render::CarrierLimiter limiter;
    std::vector<float> impulse(32 * 6 * 3);
    impulse[5] = .5F;
    limiter.process(impulse);
    check(impulse[5] == 0 && impulse[32 * 6 + 5] == .5F && impulse[64 * 6 + 5] == 0,
        "native limiter delays each carrier exactly 32 samples");
    std::vector<float> loud(32 * 6 * 32, 4);
    limiter.process(loud);
    check(std::all_of(loud.begin(), loud.end(), [](float x) { return std::abs(x) <= 1.000001F; }),
        "native lookahead limits a sustained overloaded carrier before core clipping");
    auto bad = std::vector<float>(32 * 6, 0);
    bad[0] = std::numeric_limits<float>::infinity();
    rejects([&] { limiter.process(bad); }, "limiter rejects nonfinite input before updating its state");
    rejects([&] { limiter.process(std::span<float>(loud).first(191)); }, "limiter rejects an incomplete six-channel block");
    atmos::render::CarrierLimiter whole, pieces;
    auto a = make_tone(48000, 1, 6);
    auto b = a;
    whole.process(a);
    for (std::size_t offset = 0; offset < b.size(); offset += 32 * 6)
        pieces.process(std::span<float>(b).subspan(offset, 32 * 6));
    check(a == b, "limiter state is independent of the caller's chunk boundaries");
}

void test_carrier_history() {
    atmos::encode::joc::HistoryDelay delay;
    atmos::encode::joc::History markers{};
    for (unsigned t = 0; t < 24; ++t) markers[t * 128] = float(t + 1);
    const auto first = delay.advance(markers);
    for (unsigned t = 0; t < 24; ++t) markers[t * 128] = float(t + 101);
    const auto second = delay.advance(markers);
    check(first[17 * 128] == 0 && first[18 * 128] == 1 && first[23 * 128] == 6 &&
        second[0] == 7 && second[17 * 128] == 24 && second[18 * 128] == 101,
        "native JOC history alignment delays 18 slots and retains the preceding frame tail");
    atmos::render::MetadataFrame metadata;
    metadata.oamd.object_count = 3;
    metadata.oamd.bed_channels = 2;
    metadata.oamd.bed_channel_ids = {3,0};
    metadata.oamd.gain_updates = {{0,0,0,1,0}, {1,0,0,.5F,256}};
    atmos::eac3::OamdPositionUpdate position;
    position.object = 2; position.x = 32767.0F / 32768;
    metadata.oamd.position_updates.push_back(position);
    atmos::render::Layout layout;
    check(atmos::render::parse_layout("5.1", layout), "history carrier layout parses");
    std::vector<float> output;
    std::vector<atmos::render::CarrierHistory> history;
    std::string error;
    check(atmos::render::render_objects(std::vector<float>(300, 1), 3, 100,
        std::span(&metadata, 1), layout, output, error, 0, &history), "short mixed bed/object history renders");
    check(output.size() == 600 && history.size() == 1 && history[0][0] == .09375F &&
        history[0][3] == 0 && history[0][9] == 1 && history[0][23 * 128] == .5F &&
        history[0][23 * 128 + 9] == 1,
        "history compacts LFE, pairs block-end gains and fills the padded frame without extending PCM");
    check(!atmos::render::render_objects(std::vector<float>(300, 1), 3, 100,
        std::span(&metadata, 1), layout, output, error, 1, &history), "history refuses a partial carrier mix");
    metadata.oamd.bed_channels = 0; metadata.oamd.bed_channel_ids.clear();
    check(!atmos::render::render_objects(std::vector<float>(17, 1), 17, 1,
        std::span(&metadata, 1), layout, output, error, 0, &history), "history refuses more than 16 full-band signals");
}

void test_carrier_panning() {
    using atmos::render::carrier_panner::width_gains;
    const std::array<float, 6> native_width{.429754674F,0,.429754674F,0,.73366487F,.303904235F};
    // Exercise runtime trigonometry rather than MSVC constant folding.
    volatile int runtime_coordinate = 8192;
    check(width_gains({runtime_coordinate,runtime_coordinate,0},runtime_coordinate) == native_width,
        "Cartesian width gains match independent native vector");
    const std::array<float, 6> native_edge{.707106769F,.707106769F,0,0,0,0};
    check(width_gains({16384,0,0},32767) == native_edge,
        "Cartesian width clamps split points at room boundaries");
    check(width_gains({8192,8192,0},32767,4096,false,true) == std::array<float,6>{},
        "Cartesian width respects mute");
    rejects([&] { (void)width_gains({0,0,0},-1); }, "Cartesian width rejects negative values");
    rejects([&] { (void)width_gains({0,0,0},32768); }, "Cartesian width rejects overflow");

    atmos::render::Layout layout;
    check(atmos::render::parse_layout("5.1", layout), "carrier layout parses");
    atmos::render::MetadataFrame metadata;
    metadata.oamd.object_count = 1;
    atmos::eac3::OamdPositionUpdate first, second;
    second.sample_offset = 1; second.x = second.y = 0.25F;
    metadata.oamd.position_updates = {first, second};
    const std::vector<float> samples(33, 1);
    std::vector<float> output;
    std::string error;
    check(atmos::render::render_objects(samples, 1, samples.size(), std::span(&metadata, 1), layout, output, error),
        "point carrier scene renders");
    const std::array<float, 12> native{1, 0, 0, 0, 0, 0,
        .4332621693611145F,0,.4332621693611145F,0,.5661085844039917F,.2344978153705597F};
    const std::array<float, 12> selected{output[0], output[1], output[2], output[3], output[4], output[5],
        output[192], output[193], output[194], output[195], output[196], output[197]};
    check(std::equal(selected.begin(), selected.end(), native.begin(),
        [](float a, float b) { return std::abs(a - b) < 1e-7F; }),
        "carrier point gains match independent DEE vectors without nearest-speaker leakage");
    check(output[6] == 1 && output[7] == 0,
        "point update waits for the next native 32-sample boundary");

    first.ramp_duration = 256;
    second.sample_offset = 512; second.ramp_duration = 384; second.x = 32767.0F / 32768; second.y = 0;
    metadata.oamd.position_updates = {first, second};
    check(atmos::render::render_objects(std::vector<float>(1536, 1), 1, 1536,
        std::span(&metadata, 1), layout, output, error), "native moving carrier scene renders");
    // Independent initialized DEE renderer: carrier-production-verified/left-right.f32.
    check(output[0] == 0.00390625F && output[255 * 6] == 1 &&
        output[512 * 6] == 0.9973958134651184F && output[512 * 6 + 1] == 0.0026041667442768812F &&
        output[896 * 6] == 0 && output[896 * 6 + 1] == 1,
        "carrier ramps interpolate channel gains from startup and retain their block state");
    metadata.oamd.position_updates[0].x = std::numeric_limits<float>::quiet_NaN();
    check(!atmos::render::render_objects(samples, 1, samples.size(), std::span(&metadata, 1), layout, output, error),
        "nonfinite carrier positions fail before coordinate conversion");
}

void test_carrier_width_rendering() {
    atmos::render::Layout layout;
    check(atmos::render::parse_layout("5.1", layout), "width layout parses");
    atmos::render::MetadataFrame metadata;
    metadata.oamd.object_count = 1;
    atmos::eac3::OamdPositionUpdate position;
    position.x = position.y = .25F; position.ramp_duration = 256;
    metadata.oamd.position_updates.push_back(position);
    metadata.oamd.spread_updates.push_back({0,0,256,0,-1,.5F});
    std::vector<float> output;
    std::string error;
    const std::vector<float> source(257,1);
    check(atmos::render::render_objects(source,1,source.size(),std::span(&metadata,1),layout,output,error),
        "Cartesian width scene renders");
    const std::array<float,6> native{.37240180373191833F,0,.37240180373191833F,0,.6357536911964417F,.26334673166275024F};
    check(output.size() == 257 * 6 && std::equal(native.begin(),native.end(),output.end()-6),
        "production maps divergence to half-range native width and gain ramps");
    metadata.oamd.spread_updates[0].size = 1;
    check(atmos::render::render_objects(source,1,source.size(),std::span(&metadata,1),layout,output,error) &&
        std::equal(native.begin(),native.end(),output.end()-6),
        "nonzero size preserves native width gains in canonical5.1");
    metadata.oamd.spread_updates[0].divergence = 0;
    const std::array<float,6> native_volume{.4332621693611145F,0,.4332621693611145F,0,.5661085844039917F,.2344978153705597F};
    check(atmos::render::render_objects(source,1,source.size(),std::span(&metadata,1),layout,output,error) &&
        std::equal(native_volume.begin(),native_volume.end(),output.end()-6),
        "native canonical5.1 volume retains point gains instead of diffuse leakage");
    metadata.oamd.position_updates[0].distance_specified = true;
    metadata.oamd.position_updates[0].distance = 50.1F;
    for (bool infinity : {false,true}) {
        metadata.oamd.position_updates[0].object_at_infinity = infinity;
        check(atmos::render::render_objects(source,1,source.size(),std::span(&metadata,1),layout,output,error) &&
            std::equal(native_volume.begin(),native_volume.end(),output.end()-6),
            "explicit distance and infinity retain native canonical5.1 gains");
    }
    metadata.oamd.spread_updates[0].divergence = std::numeric_limits<float>::quiet_NaN();
    check(!atmos::render::render_objects(source,1,source.size(),std::span(&metadata,1),layout,output,error),
        "width renderer rejects nonfinite divergence before integer conversion");
    metadata.oamd.spread_updates[0].divergence = 1.1F;
    check(!atmos::render::render_objects(source,1,source.size(),std::span(&metadata,1),layout,output,error),
        "width renderer rejects divergence overflow");
}

void test_carrier_zone_snap() {
    atmos::eac3::OamdFrame scene;
    scene.object_count=2; scene.bed_channels=1; scene.bed_channel_ids={3};
    atmos::eac3::OamdPositionUpdate position;
    position.object=1; position.y=.0625F; position.z=.5F; position.snap=true;
    scene.position_updates.push_back(position);
    scene.zone_updates.push_back({1,0,0,6,false,0});
    for (unsigned code : {6U,7U}) {
        scene.zone_updates[0].horizontal=static_cast<std::uint8_t>(code);
        const auto parsed=atmos::eac3::parse_oamd_payload(atmos::encode::write_oamd_payload(scene));
        check(parsed.valid && parsed.oamd.position_updates[0].snap && parsed.oamd.zone_updates[0].horizontal==code,
            "writer/parser preserve snap and source zone6/7");
        const std::array<atmos::encode::TimedOamdPayload,2> packets{{{0,parsed.oamd_payload},{1600,parsed.oamd_payload}}};
        unsigned active_frames=0;
        for (const auto& bytes : atmos::encode::reframe_oamd(packets,3072)) {
            const auto split=atmos::eac3::parse_oamd_payload(bytes);
            check(split.valid, "reframed zone/snap payload parses");
            if (split.oamd.position_updates.empty()) continue;
            ++active_frames;
            check(split.oamd.position_updates[0].snap && split.oamd.zone_updates[0].horizontal==code,
                "reframing retains snap and exact source zone6/7");
        }
        check(active_frames==2, "zone/snap updates reach both output frames");

    }
    atmos::render::Layout layout;
    check(atmos::render::parse_layout("5.1",layout), "zone layout parses");
    atmos::render::MetadataFrame metadata; metadata.oamd=scene;
    const std::array<float,2> source{0,1};
    std::vector<float> output;std::string error;
    const auto render=[&] { return atmos::render::render_objects(source,2,1,std::span(&metadata,1),layout,output,error); };
    check(render() && output==std::vector<float>({1,0,0,0,0,0}),
        "elevation exclusion allows native nearby-speaker snap");
    metadata.oamd.zone_updates[0].elevation=true;
    check(render() && output==std::vector<float>({.8317376971244812F,0,0,0,.1654481142759323F,0}),
        "elevated object outside snap radius retains point gains");
    metadata.oamd.position_updates[0].snap=false;metadata.oamd.position_updates[0].y=1;
    metadata.oamd.position_updates[0].z=0;metadata.oamd.zone_updates[0].horizontal=6;
    check(render() && output==std::vector<float>({.7079470157623291F,0,0,0,0,0}),
        "source zone6 selects the native front-only layout");
    metadata.oamd.zone_updates[0].horizontal=7;
    check(render() && output==std::vector<float>({0,0,0,0,.7079470157623291F,0}),
        "source zone7 uses the default layout");
    metadata.oamd.zone_updates[0].horizontal=8;
    check(!render(), "renderer rejects out-of-range source zone");
    scene.zone_updates[0].horizontal=8;
    rejects([&] { (void)atmos::encode::write_oamd_payload(scene); }, "writer rejects out-of-range source zone");
    rejects([&] { (void)atmos::render::carrier_panner::width_gains({0,0,0},0,4096,false,false,true,7); },
        "native panner rejects out-of-range native zone");
}

void test_joc_analysis() {
    const auto source = make_tone(48000, 1, 6);
    std::vector<float> targets(1536 * 2);
    for (unsigned t = 0; t < 1536; ++t) {
        targets[t * 2] = source[t * 6] * 0.5F;
        targets[t * 2 + 1] = source[t * 6 + 1] * -0.7F;
    }
    atmos::encode::JocEncoder encoder;
    auto joc = encoder.encode(source, targets, 2, 360);
    check(joc.objects[0].matrix[0][0][0] == 0, "initial delayed JOC window is silent");
    for (unsigned warmup = 0; warmup < 2; ++warmup) joc = encoder.encode(source, targets, 2, 360);
    check(joc.objects[0].fine_quantization, "JOC analysis uses fine quantization");
    const auto emdf = atmos::encode::build_emdf({}, joc, false, true);
    const auto parsed = atmos::eac3::parse_emdf(emdf);
    check(parsed.valid && parsed.has_joc, "analyzed JOC serializes and parses");
    for (unsigned object = 0; object < 2; ++object) {
        int value = 0;
        const unsigned channel = object;
        bool matched = false;
        for (unsigned band = 0; band < joc.objects[object].bands; ++band) {
            value = (value + parsed.joc.objects[object].matrix[0][channel][band] + 192) % 192;
            const int signed_value = value >= 96 ? value - 192 : value;
            if (signed_value == (object == 0 ? 5 : -7)) matched = true;
        }
        check(matched, "JOC matrix follows object audio gain and polarity");
    }
    std::vector<float> silence(1536 * 6), silent_object(1536);
    rejects([&] { static_cast<void>(encoder.encode(source, silent_object, 1, 360)); },
        "JOC rejects object-count changes without a new stream");
    atmos::encode::JocEncoder silent_encoder;
    const auto silent = silent_encoder.encode(silence, silent_object, 1, 360);
    check(silent.objects[0].matrix[0][0][0] == 0, "silent JOC analysis is well-defined");
    check(silent.sequence_count == 0, "first JOC frame starts a sequence");
    const auto following = silent_encoder.encode(silence, silent_object, 1, 360);
    const auto following_emdf = atmos::encode::build_emdf({}, following, false, true);
    check(atmos::eac3::parse_emdf(following_emdf).joc.sequence_count == 1,
        "JOC sequence increments and survives serialization");
    rejects([&] { static_cast<void>(encoder.encode({}, targets, 2, 360)); }, "JOC rejects wrong shape");
    auto correlated = source;
    std::vector<float> half_gain(1536);
    for (unsigned t = 0; t < 1536; ++t) {
        half_gain[t] = correlated[t * 6] * 0.5F;
        correlated[t * 6 + 1] = correlated[t * 6];
        for (unsigned c = 2; c < 6; ++c) correlated[t * 6 + c] = 0;
    }
    atmos::encode::JocEncoder correlated_encoder;
    auto correlated_joc = correlated_encoder.encode(correlated, half_gain, 1, 360);
    for (unsigned warmup = 0; warmup < 2; ++warmup)
        correlated_joc = correlated_encoder.encode(correlated, half_gain, 1, 360);
    std::array<int, 2> coefficients{};
    bool correlated_matched = false;
    for (unsigned band = 0; band < correlated_joc.objects[0].bands; ++band) {
        for (unsigned channel = 0; channel < 2; ++channel)
            coefficients[channel] = (coefficients[channel] +
                correlated_joc.objects[0].matrix[0][channel][band] + 192) % 192;
        const auto signed_code = [](int value) { return value >= 96 ? value - 192 : value; };
        const int sum = signed_code(coefficients[0]) + signed_code(coefficients[1]);
        correlated_matched |= sum == 5;
        check(sum >= 0 && sum <= 5, "correlated JOC carriers do not duplicate the half gain");
    }
    check(correlated_matched, "correlated JOC retains half gain in the signal bands");
    atmos::encode::joc::History right_history{};
    for (unsigned t = 0; t < 24; ++t) right_history[t * 128 + 1] = 1;
    auto screened = std::make_unique<atmos::encode::JocEncoder>();
    auto screened_joc = screened->encode(correlated, half_gain, 1, 360, 5, right_history);
    for (unsigned warmup = 0; warmup < 2; ++warmup)
        screened_joc = screened->encode(correlated, half_gain, 1, 360, 5, right_history);
    coefficients = {};
    bool right_matched = false;
    for (unsigned band = 0; band < screened_joc.objects[0].bands; ++band) {
        for (unsigned channel = 0; channel < 2; ++channel)
            coefficients[channel] = (coefficients[channel] +
                screened_joc.objects[0].matrix[0][channel][band]) % 192;
        check(coefficients[0] == 0, "history screening does not seed the correlated left carrier");
        right_matched |= coefficients[1] == 5;
    }
    check(right_matched, "production JOC fit uses the delayed right-only carrier history");
    rejects([&] { static_cast<void>(screened->encode(correlated, half_gain, 1, 360)); },
        "JOC rejects removal of history in the middle of a stream");
    rejects([&] { static_cast<void>(correlated_encoder.encode(correlated, half_gain, 1, 360, 5, right_history)); },
        "JOC rejects introducing history in the middle of a stream");
    auto invalid_history = std::make_unique<atmos::encode::JocEncoder>();
    rejects([&] { static_cast<void>(invalid_history->encode(correlated, half_gain, 1, 360, 5,
        std::span<const float>(right_history).first(10))); }, "JOC rejects truncated history before analysis");
    right_history[0] = std::numeric_limits<float>::quiet_NaN();
    rejects([&] { static_cast<void>(invalid_history->encode(correlated, half_gain, 1, 360, 5, right_history)); },
        "JOC rejects nonfinite history before analysis");
    auto full_precision = std::make_unique<atmos::encode::JocEncoder>();
    auto limited_precision = std::make_unique<atmos::encode::JocEncoder>();
    auto sparse_precision = std::make_unique<atmos::encode::JocEncoder>();
    auto remapped_precision = std::make_unique<atmos::encode::JocEncoder>();
    std::vector<float> multi_carrier(1536 * 6), multi_targets(1536 * 15);
    std::uint32_t noise_state = 39;
    for (unsigned frame = 0; frame < 4; ++frame) {
        for (unsigned t = 0; t < 1536; ++t) {
            for (unsigned channel = 0; channel < 6; ++channel) {
                noise_state = noise_state * 1664525U + 1013904223U;
                multi_carrier[t * 6 + channel] = channel == 3 ? 0.0F :
                    (float(noise_state >> 8) / 16777216.0F - .5F) * (frame < 3 ? .00003F : .3F);
            }
            for (unsigned object = 0; object < 15; ++object) {
                float target = 0;
                for (unsigned channel = 0; channel < 6; ++channel)
                    target += multi_carrier[t * 6 + channel] * float(int((object * 3 + channel * 5) % 13) - 6) / 8.0F;
                multi_targets[t * 15 + object] = target;
            }
        }
        const auto fine = full_precision->encode(multi_carrier, multi_targets, 15, 8191);
        const auto budget = frame == 3 ? atmos::encode::write_joc_payload(fine).size() - 1 : 8191;
        const auto limited = limited_precision->encode(multi_carrier, multi_targets, 15, budget);
        const auto sparse_budget = frame == 3 ? atmos::encode::write_joc_payload(limited).size() - 1 : 8191;
        const auto sparse = sparse_precision->encode(multi_carrier, multi_targets, 15, sparse_budget);
        const auto remap_budget = frame == 3 ? atmos::encode::write_joc_payload(sparse).size() - 1 : 8191;
        const auto remapped = remapped_precision->encode(multi_carrier, multi_targets, 15, remap_budget);
        if (frame == 3) {
            check(atmos::encode::write_joc_payload(limited).size() <= budget, "coarse JOC fits the byte budget");
            for (unsigned object = 0; object < 15; ++object)
                check(limited.objects[object].bands == 12 && !limited.objects[object].fine_quantization,
                    "JOC lowers coefficient precision before losing frequency bands");
            check(atmos::encode::write_joc_payload(sparse).size() <= sparse_budget, "sparse JOC fits the byte budget");
            for (unsigned object = 0; object < 15; ++object)
                check(sparse.objects[object].bands == 12 && sparse.objects[object].sparse && !sparse.objects[object].fine_quantization,
                    "JOC uses sparse first-candidate gains before losing frequency bands");
            check(atmos::encode::write_joc_payload(remapped).size() <= remap_budget, "remapped JOC fits the byte budget");
            for (unsigned object = 0; object < 15; ++object)
                check(remapped.objects[object].bands < 12 && remapped.objects[object].sparse && !remapped.objects[object].fine_quantization,
                    "JOC band reduction preserves sparse coding instead of refitting");
            for (unsigned object = 0; object < 15; ++object)
                for (unsigned band = 0; band < remapped.objects[object].bands; ++band)
                    check(remapped.objects[object].sparse_channel[0][band] == sparse.objects[object].sparse_channel[0][band],
                        "JOC band remapping retains the original sparse channel indices");
        }
    }
    atmos::encode::JocEncoder bounded;
    const auto small = bounded.encode(source, targets, 2, 12);
    check(atmos::encode::write_joc_payload(small).size() <= 12, "JOC coarsens bands to fit the actual byte budget");
    check(small.objects[0].bands < joc.objects[0].bands, "bounded JOC reduces frequency resolution");
    rejects([&] { static_cast<void>(bounded.encode(source, targets, 2, 1)); }, "impossible JOC budget rejected");
    for (const int invalid_band : {-1, 8})
        rejects([&] { static_cast<void>(bounded.encode(source, targets, 2, 360, invalid_band)); }, "JOC rejects invalid initial band indices");
    auto invalid = joc;
    invalid.object_count = 0;
    rejects([&] { static_cast<void>(atmos::encode::write_joc_payload(invalid)); }, "JOC rejects zero objects");
}

void test_joc_initial_history() {
    for(unsigned channels : {5U,7U})for(unsigned points : {1U,2U})for(bool steep : {false,true})for(float gain : {1.0F,.25F}) {
        atmos::eac3::JocDecoder fresh,primed;
        atmos::eac3::JocFrame frame;frame.channels=static_cast<std::uint8_t>(channels);frame.object_count=1;frame.gain=gain;
        auto& object=frame.objects[0];object.active=object.fine_quantization=object.steep_slope=true;
        object.bands=object.data_points=1;object.timeslot_offsets[0]=1;object.matrix[0][0][0]=gain==1 ? 10 : 40;
        const unsigned physical=channels+1;std::vector<float> silence(1536*physical),unused;
        std::string error;
        check(primed.render(frame,silence,static_cast<std::uint8_t>(physical),1536,unused,error),"initial-history reference primes with silent PCM");
        object.data_points=static_cast<std::uint8_t>(points);object.steep_slope=steep;
        object.timeslot_offsets={static_cast<std::uint8_t>(points==1 ? 23 : 5),17};
        object.matrix[1][0][0]=20;
        unsigned random=111;bool exact=true;double energy=0;
        for(unsigned index=0;index<3;++index){
            auto input=silence;
            for(unsigned t=0;t<1536;++t){random=random*1664525U+1013904223U;input[t*physical]=(float(random>>8)/16777216.0F-.5F)*.2F;}
            std::vector<float> a,b;
            const bool ok=fresh.render(frame,input,static_cast<std::uint8_t>(physical),1536,a,error) &&
                primed.render(frame,input,static_cast<std::uint8_t>(physical),1536,b,error);
            check(ok,"initial-history controls render");exact=exact && ok && a==b;
            for(float value:b)energy+=double(value)*value;
        }
        check(exact && energy>1e-8,"first JOC matrix/gain equals a silent reference primed with that matrix/gain");
    }
    for(unsigned channels : {5U,7U})for(bool steep : {false,true}) {
        atmos::eac3::JocDecoder unseen,primed;unsigned random=112;bool exact=true;double energy=0;
        for(unsigned index=0;index<7;++index){
            atmos::eac3::JocFrame frame;frame.channels=static_cast<std::uint8_t>(channels);frame.object_count=2;
            for(unsigned object=0;object<2;++object){auto& o=frame.objects[object];o.active=o.fine_quantization=true;o.bands=o.data_points=1;o.steep_slope=steep;o.timeslot_offsets[0]=23;o.matrix[0][object?3:0][0]=10;}
            auto reference=frame;if(index<4)frame.objects[1].active=false;
            std::vector<float> input(1536*(channels+1));
            for(unsigned t=0;t<1536;++t){random=random*1664525U+1013904223U;const float value=(float(random>>8)/16777216.0F-.5F)*.2F;input[t*(channels+1)]=value;if(index>=4)input[t*(channels+1)+4]=value*.7F;}
            std::vector<float> a,b;std::string error;
            const bool ok=unseen.render(frame,input,static_cast<std::uint8_t>(channels+1),1536,a,error) && primed.render(reference,input,static_cast<std::uint8_t>(channels+1),1536,b,error);
            exact=exact && ok && a==b;for(float value:b)energy+=double(value)*value;
        }
        check(exact && energy>1e-8,"first matrix history is tracked independently for a later object's activation");
    }
}

void test_joc_refinement() {
    namespace joc = atmos::encode::joc;
    joc::Window window{5,0,2};window.target.resize(48);
    for(unsigned i=0;i<5;++i)window.carrier[i].resize(48);
    unsigned random=109;
    for(unsigned t=0;t<48;++t)for(unsigned k=0;k<4;++k){
        for(unsigned i=0;i<5;++i){
            random=random*1664525U+1013904223U;
            window.carrier[i][t][k]=(float(random>>8)/16777216.0F-.5F)*.2F;
        }
        window.target[t][k]=.3F*window.carrier[0][t][k]-.2F*window.carrier[1][t][k];
    }
    joc::Result native;native.coefficients={.06F,.02F,0,0,0,0,0};
    native.first_gain=.06F;native.first_channel=0;
    const auto error=[&](const joc::Result& fit){
        const auto coefficients=joc::quantize(fit.coefficients,true,5);double total=0;
        for(unsigned t=0;t<48;++t)for(unsigned k=0;k<4;++k){
            double residual=-window.target[t][k];
            for(unsigned i=0;i<5;++i)residual+=double(coefficients[i])*window.carrier[i][t][k];
            total+=residual*residual;
        }
        return total;
    };
    auto refined=joc::refine_quantized(window,native);
    check(error(refined)<error(native)*.001,"JOC refinement lowers actual quantized weighted error");
    check(refined.first_gain==native.first_gain && refined.first_channel==native.first_channel,
        "JOC refinement preserves the recovered sparse fallback");
    for(unsigned i=2;i<7;++i)check(refined.coefficients[i]==0,"JOC refinement retains the selected carrier support");
    for(unsigned t=0;t<48;++t)for(unsigned k=0;k<4;++k){
        window.carrier[1][t][k]=window.carrier[0][t][k];
        window.target[t][k]=.3F*window.carrier[0][t][k];
    }
    refined=joc::refine_quantized(window,native);
    check(error(refined)<=error(native),"JOC refinement handles rank-deficient correlated carriers");
    for(auto& slot:window.target)slot={};
    for(auto& channel:window.carrier)for(auto& slot:channel)slot={};
    refined=joc::refine_quantized(window,{});
    check(std::all_of(refined.coefficients.begin(),refined.coefficients.end(),[](float x){return x==0;}),
        "silent JOC refinement remains finite and zero");
    window.channels=7;
    rejects([&]{(void)joc::refine_quantized(window,{});},"JOC refinement rejects uncalibrated carrier counts");
}

void test_joc_zero_compaction() {
    for (unsigned budget : {24U,64U,360U}) {
        atmos::encode::JocEncoder encoder;atmos::eac3::JocDecoder compact_decoder,full_decoder;
        unsigned random=123;std::size_t saved=0;bool equal=true,protected_zero=false;
        bool previous_nonzero=false,sparse=false,coarse=false;
        for (unsigned frame=0;frame<32;++frame) {
            std::vector<float> carrier(1536*6),targets(1536*3),excitation(1536*6);
            const bool audible=frame<5 || (frame>=15 && frame<20);
            for (unsigned t=0;t<1536;++t) {
                random=random*1664525U+1013904223U;
                const float noise=(float(random>>8)/16777216.0F-.5F)*.2F;
                targets[t*3]=carrier[t*6]=audible ? noise : 0;excitation[t*6]=noise;
            }
            const auto compact=encoder.encode(carrier,targets,3,budget);
            auto full=compact;for(unsigned object=0;object<3;++object)full.objects[object].active=true;
            const auto a=atmos::encode::write_joc_payload(compact),b=atmos::encode::write_joc_payload(full);
            check(a.size()<=b.size() && a.size()<=budget,"compacted JOC respects the original payload budget");
            saved+=b.size()-a.size();
            const auto& o=full.objects[0];bool zero=true;
            sparse=sparse || o.sparse;coarse=coarse || !o.fine_quantization;
            const unsigned modulus=o.fine_quantization ? 192 : 96;
            if(o.sparse) {
                unsigned value=o.fine_quantization ? 100 : 50;
                for(unsigned band=0;band<o.bands;++band){value=(value+o.sparse_vector[0][band])%modulus;zero=zero && value==modulus/2;}
            } else for(unsigned channel=0;channel<5;++channel) {
                unsigned value=0;for(unsigned band=0;band<o.bands;++band){value=(value+o.matrix[0][channel][band])%modulus;zero=zero && !value;}
            }
            if(zero && previous_nonzero){protected_zero=true;check(compact.objects[0].active,"first zero JOC matrix retains its fade");}
            previous_nonzero=!zero;
            std::vector<float> x,y;std::string error;
            const bool rendered=compact_decoder.render(compact,excitation,6,1536,x,error)&&full_decoder.render(full,excitation,6,1536,y,error);
            check(rendered,"compacted and expanded JOC controls render");equal=equal && rendered && x==y;
        }
        check(equal && saved>0 && protected_zero,"zero compaction saves metadata with bit-exact matrix trajectories");
        if(budget==24)check(sparse && coarse,"zero compaction also covers coarse sparse rate-control output");
    }
    for(unsigned count : {1U,3U,5U,15U,16U}) {
        atmos::encode::JocEncoder encoder;
        const auto frame=encoder.encode(std::vector<float>(1536*6),std::vector<float>(1536*count),count,360);
        check(std::all_of(frame.objects.begin(),frame.objects.begin()+count,[](const auto& o){return o.active;}),
            "initial zero JOC matrices remain explicit for native surround initialization");
    }
}

void test_joc_carrier_correction() {
    atmos::encode::JocEncoder encoder;
    std::vector<float> carrier(1536 * 6), target(1536);
    std::uint32_t noise = 55;
    for (unsigned frame = 0; frame < 4; ++frame) {
        for (unsigned t = 0; t < 1536; ++t) {
            noise = noise * 1664525U + 1013904223U;
            target[t] = (float(noise >> 8) / 16777216.0F - .5F) * .2F;
            carrier[t * 6 + 4] = target[t] * .750109851360321F;
        }
        (void)encoder.encode(carrier, target, 1, 360);
    }
    const auto gains = encoder.carrier_correction();
    const float reconstructed = .750109851360321F * (13 * 205.0F / 2048);
    check(std::abs(reconstructed - 1) > .02F && std::abs(reconstructed * gains[3] - 1) < .001F,
        "quantized JOC correction recovers the isolated auto-trim level");
    for (unsigned channel : {0U,1U,2U,4U})
        check(gains[channel] == 1, "silent carriers retain unity correction");
    atmos::encode::JocEncoder silent;
    (void)silent.encode(std::vector<float>(1536 * 6), std::vector<float>(1536), 1, 360);
    check(silent.carrier_correction() == std::array<float,5>{1,1,1,1,1},
        "silent JOC correction is unity");
    atmos::encode::joc::CarrierCorrection ramp;
    std::vector<float> pcm(1536 * 6, 1);
    const std::array<float,5> extremes{.95F,1.05F,1,.95F,1.05F};
    ramp.process(pcm, extremes);
    check(pcm[0] == 1 && pcm[256 * 6] == .95F && pcm[256 * 6 + 1] == 1.05F,
        "carrier correction ramps from the previous frame over256 samples");
    for (unsigned t = 0; t < 1536; ++t) check(pcm[t * 6 + 3] == 1, "carrier correction preserves LFE");
    pcm.assign(pcm.size(), 1);
    ramp.process(pcm, extremes);
    check(pcm[0] == .95F && pcm[1] == 1.05F, "carrier correction retains its ramp state");
    rejects([&] { ramp.process(std::span<float>(pcm).first(6), extremes); },
        "carrier correction rejects incomplete frames");
    auto invalid = extremes; invalid[0] = std::numeric_limits<float>::quiet_NaN();
    rejects([&] { ramp.process(pcm, invalid); }, "carrier correction rejects nonfinite gains");
    invalid[0] = 1.06F;
    rejects([&] { ramp.process(pcm, invalid); }, "carrier correction enforces the downmix limit");
    pcm[0] = std::numeric_limits<float>::infinity();
    rejects([&] { ramp.process(pcm, extremes); }, "carrier correction rejects nonfinite PCM");
}

void test_fast_atmos() {
    atmos::encode::Options options;
    options.fast_atmos = "9.1.6";
    std::string error;
    check(atmos::encode::run(options,error)<0 && error.find("5.1.2")!=std::string::npos,
        "fast Atmos rejects unsupported output scenes");
    options.fast_atmos = "5.1.2";
    check(atmos::encode::run(options,error)<0 && error.find("TrueHD input")!=std::string::npos,
        "fast Atmos requires an explicit TrueHD input and output");
    options.truehd = "test_file/trueHD Atmos.mkv";
    options.output = "ddpatmos-fast-stream.ec3";
    check(atmos::encode::run(options,error)<0 && error.find("JOC")!=std::string::npos,
        "fast Atmos rejects conflicting plain/OAMD-only modes");
    const auto buffered_path=std::filesystem::path("ddpatmos-fast-buffer.ec3");
    const auto dump=std::filesystem::path("ddpatmos-fast-bed.wav");
    const auto stream_path=std::filesystem::path(options.output);
    if (std::filesystem::exists(options.output) || std::filesystem::exists(buffered_path) || std::filesystem::exists(dump)) {
        check(false,"fast Atmos test paths must be unused");return;
    }
    options.atmos=true;options.truehd_seconds=.16;options.bitrate=384000;
    const std::pair<const char*,std::vector<std::uint8_t>> layouts[]{
        {"5.1.2",{0,1,2,3,4,5,10,11}},
        {"5.1.4",{0,1,2,3,4,5,8,9,12,13}},
        {"7.1.4",{0,1,2,3,4,5,6,7,8,9,12,13}}
    };
    for (const auto& [layout,ids] : layouts) {
        for (const double seconds : {.16,.155,.15999}) {
        options.truehd_seconds=seconds;
        options.fast_atmos=layout;options.output=stream_path.string();options.dump_bed.clear();
        bool prescan=false;
        options.progress=[&](const char* stage,int) {
            prescan=prescan || std::string_view(stage)=="metadata" || std::string_view(stage)=="alignment";
        };
        const auto count=atmos::encode::run(options,error);
        check(!prescan,"fast Atmos starts without the metadata/alignment prescan");
        options.progress={};
        check(count>0,"fast Atmos streams a TrueHD/MKV source");
        if (count<=0) {std::fprintf(stderr,"fast Atmos: %s\n",error.c_str());return;}
        std::vector<std::uint8_t> expected;
        {
            const atmos::MappedFile stream(options.output);
            expected.assign(stream.bytes().begin(),stream.bytes().end());
        }
        const auto info=atmos::eac3::parse_bitstream_info(expected,0);
        atmos::eac3::BedDecoder decoder;
        atmos::eac3::PcmFrame decoded;
        check(info && decoder.decode(std::span(expected).first(info->frame_size),decoded,error),
            "fast Atmos output decodes with JOC metadata");
        check(decoded.metadata.valid && decoded.metadata.has_joc && decoded.metadata.oamd.bed_channels==ids.size() &&
            decoded.metadata.oamd.bed_channel_ids==ids &&
            decoded.metadata.joc_objects==ids.size()-1 && decoded.metadata.oamd.position_updates.empty(),
            "fast Atmos preserves the selected fixed bed and every non-LFE target");
        options.output=buffered_path.string();options.dump_bed=dump.string();
        check(atmos::encode::run(options,error)==count,"fast Atmos buffered and streamed durations agree");
        {
            const atmos::MappedFile buffered(buffered_path);
            check(std::equal(buffered.bytes().begin(),buffered.bytes().end(),expected.begin(),expected.end()),
                "fast Atmos streamed constant downmix matches buffered rendering byte-for-byte");
        }
        // Release the memory mapping before removing the output on Windows.
        std::filesystem::remove(buffered_path);std::filesystem::remove(dump);
        std::filesystem::remove(stream_path);
        }
    }
}

void test_live_renderer() {
    constexpr unsigned samples=3183;
    std::vector<float> pcm(samples*2);
    for (unsigned t=0;t<samples;++t) {pcm[t*2]=.03F;pcm[t*2+1]=float(int(t%97)-48)/100;}
    std::array<atmos::render::MetadataFrame,3> timeline;
    for (auto& f:timeline) {f.oamd.object_count=2;f.oamd.bed_channels=1;f.oamd.bed_channel_ids={3};}
    timeline[0].oamd.position_updates={{1,0,0,0,false,.2F,.1F,1}};
    timeline[0].oamd.gain_updates={{1,0,0,1,0}};
    timeline[1].sample=800;
    timeline[1].oamd.position_updates={{1,600,256,0,false,.8F,.9F,.5F}};
    timeline[2].sample=2200;
    timeline[2].oamd.gain_updates={{1,0,0,.5F,0}};
    for (const auto* name : {"5.1.2","5.1.4","7.1.4"}) {
        atmos::render::Layout layout;static_cast<void>(atmos::render::parse_layout(name,layout));
        std::vector<float> expected,actual;std::string error;
        check(atmos::render::render_objects(pcm,2,samples,timeline,layout,expected,error),
            "buffered live-renderer reference renders");
        std::size_t available=0,next=0,first_output=0;
        const auto ok=atmos::render::render_objects_live(2,
            [&](atmos::render::MetadataPacket& packet) {
                if (next<timeline.size() && (!next || timeline[next].sample<available)) {
                    packet={timeline[next].sample,timeline[next]};++next;return true;
                }
                if (available==samples) return false;
                packet={available,{available,{}}};return true;
            },layout,[&](std::span<float> target) {
                const auto count=std::min(target.size()/2,samples-available);
                std::copy_n(pcm.begin()+available*2,count*2,target.begin());available+=count;return count;
            },[&](std::span<const float> block,const atmos::render::CarrierHistory*) {
                if (!first_output) first_output=available;
                actual.insert(actual.end(),block.begin(),block.end());
            },error);
        check(ok && actual==expected,"one-pass renderer preserves motion across blocks and partial EOF exactly");
        check(first_output==1536 && next==timeline.size(),"live renderer produces its first block before consuming the source");
    }
}

void test_boundaries() {
    atmos::encode::CoreEncoder encoder;
    atmos::encode::CoreConfig config;
    config.sample_rate = 0;
    rejects([&] { encoder.configure(config); }, "zero sample rate rejected");
    config.sample_rate = 48000;
    config.audio_coding_mode = 8;
    rejects([&] { encoder.configure(config); }, "invalid acmod rejected");
    configure_51(encoder);
    std::vector<std::uint8_t> frame;
    rejects([&] { encoder.encode({}, frame); }, "wrong PCM shape rejected");
    auto pcm = make_tone(48000, 1, 6);
    pcm[0] = std::numeric_limits<float>::quiet_NaN();
    rejects([&] { encoder.encode(pcm, frame); }, "NaN PCM rejected");
    std::vector<std::uint8_t> oversized(511 * 6 + 1);
    rejects([&] { encoder.set_aux_payload(oversized); }, "oversized EMDF rejected");

    const auto path = std::filesystem::current_path() / "ddpatmos-tail-test.ec3";
    const auto wav = std::filesystem::current_path() / "ddpatmos-tail-test.wav";
    if (std::filesystem::exists(path) || std::filesystem::exists(wav)) {
        check(false, "test output paths must be unused"); return;
    }
    const auto short_pcm = make_tone(48000, 1, 6);
    atmos::write_float_wav(wav, 48000, 6, std::span<const float>(short_pcm).first(6 * 100));
    atmos::encode::Options options;
    options.input = wav.string(); options.output = path.string();
    std::string error;
    check(atmos::encode::run(options, error) == 2, "short input emits padded frame and filter flush");
    check(std::filesystem::file_size(path) == 5120, "plain output contains no duplicate dependent substream");
    check(atmos::encode::run(options, error) < 0, "existing output protected");
    std::filesystem::remove(path);
    options.atmos = true;
    check(atmos::encode::run(options, error) == 3, "short Atmos input drains JOC synthesis and the core filter");
    {
        std::ifstream encoded(path, std::ios::binary);
        const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(encoded), {}};
        const auto info = atmos::eac3::parse_bitstream_info(bytes, 0);
        check(info && info->complexity_index_type_a == 6,
            "Atmos complexity index matches the six coded OAMD objects");
    }
    std::filesystem::remove(path);
    const auto other_rate = std::filesystem::current_path() / "ddpatmos-other-rate-test.wav";
    if (!std::filesystem::exists(other_rate)) {
        atmos::write_float_wav(other_rate, 44100, 6, std::span<const float>(short_pcm).first(6 * 100));
        options.input = other_rate.string();
        check(atmos::encode::run(options, error) < 0 && error.find("48 kHz") != std::string::npos &&
            !std::filesystem::exists(path), "unverified JOC sample-rate profiles fail before writing output");
        std::filesystem::remove(other_rate);
        options.input = wav.string();
    } else check(false, "sample-rate test output path must be unused");
    options.output = wav.string();
    check(atmos::encode::run(options, error) < 0, "input cannot be overwritten");
    options.output = path.string();
    options.dump_bed = wav.string();
    const auto source = [&] {
        std::ifstream in(wav, std::ios::binary);
        return std::vector<char>(std::istreambuf_iterator<char>(in), {});
    };
    const auto original = source();
    check(atmos::encode::run(options, error) < 0 && error.find("dump") != std::string::npos,
        "bed dump cannot overwrite input");
    check(source() == original && !std::filesystem::exists(path), "rejected bed dump preserves input and creates no output");
    std::filesystem::remove(wav);
}

void test_spatial_wav() {
    const auto root = std::filesystem::current_path();
    const auto wav = root / "ddpatmos-height-test.wav";
    const auto output = root / "ddpatmos-height-test.ec3";
    const auto dump = root / "ddpatmos-height-test-bed.wav";
    if (std::filesystem::exists(wav) || std::filesystem::exists(output) || std::filesystem::exists(dump)) {
        check(false, "height test paths must be unused"); return;
    }
    // Each channel has its own level; the masked WAV swaps rear/side order.
    std::vector<float> pcm(100 * 12);
    for (unsigned t = 0; t < 100; ++t)
        for (unsigned c = 0; c < 12; ++c) pcm[t * 12 + c] = float(c + 1) / 100;
    atmos::encode::Options options;
    options.input = wav.string(); options.output = output.string(); options.dump_bed = dump.string();
    std::string error;
    atmos::write_float_wav(wav, 48000, 12, pcm);
    check(atmos::encode::run(options, error) > 0, "unmasked 7.1.4 WAV downmixes to plain 5.1");
    std::ifstream baseline(dump, std::ios::binary);
    const std::vector<char> expected{std::istreambuf_iterator<char>(baseline), {}};
    baseline.close();
    std::filesystem::remove(output); std::filesystem::remove(dump);
    // Build WAVE_FORMAT_EXTENSIBLE with the same speaker mask as Nature's Kiss.
    for (unsigned t = 0; t < 100; ++t) {
        std::swap(pcm[t * 12 + 4], pcm[t * 12 + 6]);
        std::swap(pcm[t * 12 + 5], pcm[t * 12 + 7]);
    }
    atmos::write_float_wav(wav, 48000, 12, pcm);
    std::ifstream source(wav, std::ios::binary);
    std::vector<unsigned char> bytes{std::istreambuf_iterator<char>(source), {}};
    source.close();
    bytes.insert(bytes.begin() + 36, 24, 0);
    const auto u16 = [&](unsigned at, unsigned value) { bytes[at] = static_cast<unsigned char>(value & 255); bytes[at + 1] = static_cast<unsigned char>(value >> 8); };
    const auto u32 = [&](unsigned at, unsigned value) { u16(at, value & 65535); u16(at + 2, value >> 16); };
    u32(4, static_cast<unsigned>(bytes.size() - 8)); u32(16, 40); u16(20, 0xfffe);
    u16(36, 22); u16(38, 32); u32(40, 0x2d63f); u32(44, 3);
    bytes[50] = 0x10; bytes[52] = 0x80; bytes[55] = 0xaa;
    bytes[57] = 0x38; bytes[58] = 0x9b; bytes[59] = 0x71;
    const auto save = [&] {
        std::ofstream file(wav, std::ios::binary);
        file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    };
    save();
    check(atmos::encode::run(options, error) > 0, "masked 7.1.4 WAV accepted");
    std::ifstream actual(dump, std::ios::binary);
    check(std::vector<char>(std::istreambuf_iterator<char>(actual), {}) == expected,
        "WAV speaker mask reorders side and rear channels before rendering");
    actual.close(); std::filesystem::remove(output); std::filesystem::remove(dump);
    options.atmos = true; options.dump_bed.clear();
    check(atmos::encode::run(options, error) > 0, "height WAV encodes with JOC and limiter tail");
    {
    atmos::MappedFile encoded(output);
    atmos::eac3::BedDecoder decoder;
    atmos::eac3::PcmFrame decoded;
    const auto info = atmos::eac3::parse_bitstream_info(encoded.bytes(), 0);
    check(info && decoder.decode(encoded.bytes().first(info->frame_size), decoded, error), "height output decodes");
    const auto& metadata = decoded.metadata;
    check(metadata.valid && metadata.has_joc && metadata.has_oamd && metadata.oamd_objects == 12 &&
        metadata.oamd.bed_channel_ids == std::vector<std::uint8_t>({0,1,2,3,4,5,6,7,8,9,12,13}) &&
        metadata.joc_objects == 11, "JOC retains eleven full-band channels plus LFE and four height positions");
    }
    std::filesystem::remove(output);
    options.output = (root / "ddpatmos-height-invalid.ec3").string();
    u32(40, 0x3f); save();
    check(atmos::encode::run(options, error) < 0 && error.find("mask") != std::string::npos &&
        !std::filesystem::exists(options.output), "mismatched WAV channel mask fails before output creation");
    std::filesystem::remove(wav);
}

void test_carrier_trim_boundary() {
    atmos::eac3::OamdFrame scene;
    scene.object_count = 1;
    scene.position_updates.emplace_back();
    const auto base = atmos::eac3::parse_oamd_payload(atmos::encode::write_oamd_payload(scene));
    check(base.valid && !base.oamd.trim_present, "absent trim selects the default policy");
    atmos::BitWriter packet;
    for (std::size_t bit = 0; bit < base.oamd_payload.header_bits; ++bit)
        packet.write((base.oamd_payload.header[bit/8] >> (7-bit%8)) & 1, 1);
    packet.write(2, 4);
    for (const auto& element : base.oamd_payload.elements) {
        packet.write(element.id, 4); packet.write(static_cast<unsigned>(element.body.size()-1), 4);
        packet.write_bit(false);
        for (auto byte : element.body) packet.write(byte, 8);
    }
    packet.write(2, 4); packet.write(0, 4); packet.write_bit(false); packet.write(0x20, 8); packet.align();
    const auto explicit_trim = atmos::eac3::parse_oamd_payload(packet.bytes());
    check(explicit_trim.valid && explicit_trim.oamd.trim_present,
        "trim extension is distinguished from absent trim without discarding its wire body");
    atmos::render::Layout layout;
    check(atmos::render::parse_layout("5.1", layout), "trim carrier layout parses");
    layout.name.clear();
    atmos::render::MetadataFrame frame; frame.oamd = explicit_trim.oamd;
    std::vector<float> output; std::string error;
    check(atmos::render::render_objects(std::vector<float>(32,1),1,32,std::span(&frame,1),layout,output,error),
        "canonical renderer accepts supported explicit trim metadata");
    frame.oamd.warp_mode=2;
    check(!atmos::render::render_objects(std::vector<float>(32,1),1,32,std::span(&frame,1),layout,output,error) &&
        error.find("warp mode0/1")!=std::string::npos,"renderer rejects uncalibrated warp modes");
    frame.oamd=explicit_trim.oamd;
    const auto written = atmos::eac3::parse_oamd_payload(atmos::encode::write_oamd_payload(explicit_trim.oamd));
    check(written.valid && written.oamd.trim_present &&
        written.oamd_payload.elements.back().body == explicit_trim.oamd_payload.elements.back().body,
        "static writer preserves explicit trim semantics");
    const auto wrapped = atmos::eac3::parse_emdf(atmos::encode::build_emdf(explicit_trim.oamd,{},true,false));
    check(wrapped.valid && wrapped.oamd.trim_present && wrapped.oamd.trims == explicit_trim.oamd.trims,
        "EMDF writer preserves explicit trim semantics");
    const std::array<atmos::encode::TimedOamdPayload,1> timed{{{0,explicit_trim.oamd_payload}}};
    const auto reframed=atmos::encode::reframe_oamd(timed,1536);
    unsigned cursor=0;
    atmos::encode::OamdReframer owned([&](atmos::encode::TimedOamdPayload& value) {
        if(cursor++){value={};return false;}
        value=timed[0];return true;
    },1536);
    std::vector<std::uint8_t> callback_frame;
    for(const auto& expected:reframed)
        check(owned.next(callback_frame) && callback_frame==expected,
            "callback header and trim survive replacement and clearing of the source packet at EOF");
    check(!owned.next(callback_frame) && callback_frame.empty(),"callback reframer EOF clears output");
    const auto moved=atmos::eac3::parse_oamd_payload(reframed.front());
    check(moved.valid && moved.oamd.trim_present &&
        moved.oamd_payload.elements.back().body==explicit_trim.oamd_payload.elements.back().body,
        "reframer preserves a constant trim element without requantization");
    std::array<atmos::encode::TimedOamdPayload,2> changing{{timed[0],{768,base.oamd_payload}}};
    rejects([&] { (void)atmos::encode::reframe_oamd(changing,1536); },
        "reframer rejects changing trim presence rather than merging incompatible profiles");
    changing[1].payload=explicit_trim.oamd_payload;changing[1].payload.elements.back().body[0]^=0x20;
    rejects([&] { (void)atmos::encode::reframe_oamd(changing,1536); },"reframer rejects changing trim wire fields");
    changing[1].sample=1536;
    const auto different_frames=atmos::encode::reframe_oamd(changing,3072);
    const auto second_profile=atmos::eac3::parse_oamd_payload(different_frames[1]);
    check(second_profile.valid && second_profile.oamd.trim_present && second_profile.oamd.warp_mode==0,
        "trim profiles may change when their object blocks land in different DD+ frames");
    changing[1].sample=768;
    changing[1].payload=explicit_trim.oamd_payload;changing[1].payload.elements.erase(changing[1].payload.elements.begin());
    changing[1].payload.elements.back().body[0]^=0x20;
    check(atmos::encode::reframe_oamd(changing,1536)==reframed,
        "trim-only packets do not change the reframed object profile");
    atmos::BitWriter short_body;
    for (std::size_t bit = 0; bit < base.oamd_payload.header_bits; ++bit)
        short_body.write((base.oamd_payload.header[bit/8] >> (7-bit%8)) & 1, 1);
    short_body.write(2, 4);
    short_body.write(2, 4); short_body.write(0, 4); short_body.write_bit(false); short_body.write(0x24, 8);
    const auto& object = base.oamd_payload.elements.front();
    short_body.write(1, 4); short_body.write(static_cast<unsigned>(object.body.size()-1), 4); short_body.write_bit(false);
    for (const auto byte : object.body) short_body.write(byte, 8);
    short_body.align();
    check(!atmos::eac3::parse_oamd_payload(short_body.bytes()).valid,
        "truncated trim cannot consume the next element's body");
}

void test_oamd_trim_fields() {
    atmos::eac3::OamdFrame scene;
    scene.object_count = 2; scene.position_updates.resize(2); scene.position_updates[1].object = 1;
    scene.trim_present = true;
    const auto roundtrip = [&] {
        const auto raw = atmos::encode::write_oamd_payload(scene);
        const auto parsed = atmos::eac3::parse_oamd_payload(raw);
        check(parsed.valid && parsed.oamd.trim_present && parsed.oamd.warp_mode == scene.warp_mode &&
            parsed.oamd.trim_reserved == scene.trim_reserved && parsed.oamd.global_trim_mode == scene.global_trim_mode &&
            parsed.oamd.trims == scene.trims && parsed.oamd.trim_bypass == scene.trim_bypass,
            "all trim fields survive serialization");
        auto truncated = raw; truncated.pop_back();
        check(!atmos::eac3::parse_oamd_payload(truncated).valid, "truncated trim body is rejected");
    };
    for (unsigned global = 0; global < 3; ++global) for (unsigned warp = 0; warp < 4; ++warp) {
        scene.global_trim_mode = static_cast<std::uint8_t>(global);
        scene.warp_mode = static_cast<std::uint8_t>(warp);
        scene.trims = {};
        for (auto& trim : scene.trims) trim.mode = static_cast<std::uint8_t>(global);
        scene.trim_bypass = {false,true}; roundtrip();
    }
    scene.global_trim_mode = 2; scene.warp_mode = 1; scene.trim_reserved = 3;
    for (unsigned config = 0; config < 9; ++config) {
        scene.trims = {}; scene.trims[config].mode = 2;
        for (unsigned field = 0; field < 3; ++field) for (unsigned code = 0; code <= 16; ++code) {
            scene.trims[config].levels = {16,16,16};
            scene.trims[config].levels[field] = static_cast<std::uint8_t>(code); roundtrip();
        }
        scene.trims[config].levels = {16,16,16};
        for (unsigned field = 0; field < 2; ++field) for (int value = -16; value <= 16; ++value) {
            scene.trims[config].balance = {};
            scene.trims[config].balance[field] = static_cast<std::int8_t>(value); roundtrip();
        }
    }
    scene.trims = {}; scene.trim_bypass.clear(); roundtrip();
    scene.position_updates.clear();
    for (unsigned count : {1U,31U,32U,159U}) {
        scene.object_count = static_cast<std::uint16_t>(count);
        scene.trim_bypass.resize(count);
        for (unsigned i = 0; i < count; ++i) scene.trim_bypass[i] = bool(i % 2);
        for (auto& trim : scene.trims) { trim.mode = 2; trim.levels = {0,8,15}; trim.balance = {-16,16}; }
        roundtrip();
    }
    scene.object_count = 2; scene.trims = {};
    scene.trim_bypass = {true};
    rejects([&] { (void)atmos::encode::write_oamd_payload(scene); }, "trim bypass count matches the object count");
    scene.trim_bypass.clear(); scene.trims[0].mode = 2; scene.trims[0].levels[0] = 17;
    rejects([&] { (void)atmos::encode::write_oamd_payload(scene); }, "trim level code is bounded");
    scene.trims[0].levels[0] = 16; scene.trims[0].balance[0] = 17;
    rejects([&] { (void)atmos::encode::write_oamd_payload(scene); }, "trim balance is bounded");
    scene.trims[0].balance[0] = 0; scene.trims[0].mode = 0; scene.trims[0].levels[0] = 8;
    rejects([&] { (void)atmos::encode::write_oamd_payload(scene); }, "auto mode cannot silently discard explicit values");
    scene.trims = {}; scene.global_trim_mode = 3;
    rejects([&] { (void)atmos::encode::write_oamd_payload(scene); }, "reserved global trim mode is rejected");
}

void test_carrier_explicit_trim_gains() {
    using namespace atmos::render::carrier_panner;
    atmos::eac3::OamdTrimConfig trim; trim.mode=2; trim.levels[2]=8;
    const std::array<float,6> height{0,0,0,0,.5005821585655212F,.500606119632721F};
    check(trim_gains({16384,16384,16384},0,trim,true)==height,
        "explicit minus6 height gives native interpolated minus3dB at half elevation");
    const std::array<float,6> bypass{0,0,0,0,.7070898413658142F,.7071236968040466F};
    check(trim_gains({16384,16384,16384},0,trim,true,4096,false,true)==bypass,
        "per-object bypass removes explicit attenuation");
    const std::array<float,6> side_normal{.7070898413658142F,0,0,0,.7071236968040466F,0};
    trim.levels={16,16,16};
    check(trim_bed_gains(10,trim,false)==side_normal,
        "normal warp maps the top-side bed to both front and surround");
    check(trim_bed_gains(10,trim,true)==std::array<float,6>{0,0,0,0,1,0},
        "warping maps the top-side bed to surround");
    trim.levels={0,15,15};
    check(trim_bed_gains(3,trim,false)==std::array<float,6>{0,0,0,1,0,0},
        "dimensional trims preserve the LFE channel");
    check(trim_gains({8192,8192,16384},32767,trim,true,4096,true)==std::array<float,6>{},
        "explicit trims preserve mute");
    trim.balance[0]=17;
    rejects([&] { (void)trim_gains({0,0,0},0,trim,true); },"trim panner rejects invalid balances");
    trim.balance[0]=0;trim.levels[1]=17;
    rejects([&] { (void)trim_bed_gains(0,trim,true); },"trim bed panner rejects invalid level codes");
    trim.levels={16,16,16};trim.mode=3;
    rejects([&] { (void)trim_gains({0,0,0},0,trim,true); },"trim panner rejects unknown modes");
}

void test_carrier_trim_gain_order() {
    atmos::render::Layout layout;
    check(atmos::render::parse_layout("5.1",layout),"trim gain-order layout parses");
    atmos::render::MetadataFrame frame;
    frame.oamd.object_count=1; frame.oamd.trim_present=true; frame.oamd.global_trim_mode=2;
    for(auto& trim:frame.oamd.trims)trim.mode=2;
    frame.oamd.trims[1].levels={16,6,8};
    atmos::eac3::OamdPositionUpdate point;
    point.y=16.0F/62.0F; point.z=8.0F/15.0F;
    frame.oamd.position_updates={point};
    frame.oamd.gain_updates={{0,0,0,1028.0F/4096.0F,0},{0,32,1,0,0}};
    std::vector<float> output; std::string error;
    std::vector<atmos::render::CarrierHistory> history;
    check(atmos::render::render_objects(std::vector<float>(64,1),1,64,std::span(&frame,1),layout,
        output,error,0,&history),"nonunity explicit-trim scene renders");
    // Independent DEE renderer targets: renderer64-final-simultaneous-zero-last/2/native.f32.
    constexpr std::array<float,6> native{.11002509295940399F,0,0,0,.11574631184339523F,0};
    for(unsigned channel=0;channel<6;++channel) {
        check(output[channel]==native[channel] && output[31*6+channel]==native[channel] &&
            output[32*6+channel]==0,"explicit trim and nonunity gain retain the native rounding before mute");
        check(history[0][channel]==.5F*native[channel],"trim gain order retains the native packed history");
    }
    auto wide=frame;
    wide.oamd.trim_present=false;wide.oamd.bed_channels=1;wide.oamd.bed_channel_ids={14};
    wide.oamd.position_updates.clear();wide.oamd.gain_updates={{0,0,0,14533.0F/4096.0F,0}};
    check(atmos::render::render_objects(std::vector<float>(64,1),1,64,std::span(&wide,1),layout,
        output,error,0,&history),"nonunity default-trim wide bed renders");
    // Independent DEE wire-gain-bed-wide target at frame4/sample32.
    constexpr std::array<float,6> native_wide{2.828871250152588F,0,0,0,1.570151686668396F,0};
    for(unsigned channel=0;channel<6;++channel)
        check(output[channel]==native_wide[channel] && history[0][channel]==native_wide[channel],
            "default wide-bed gain uses the native panner rounding in PCM and packed history");
    using namespace atmos::render::carrier_panner;
    const auto nan=std::numeric_limits<float>::quiet_NaN();
    rejects([&]{(void)point_gains({0,0,0},nan);},"point panner rejects nonfinite gain");
    rejects([&]{(void)trim_gains({0,0,0},0,frame.oamd.trims[1],true,nan);},"trim panner rejects nonfinite gain");
    rejects([&]{(void)trim_bed_gains(0,frame.oamd.trims[1],true,nan);},"trim bed panner rejects nonfinite gain");
}

void test_renderer_event_order() {
    atmos::render::Layout layout;check(atmos::render::parse_layout("5.1",layout),"event order layout parses");
    atmos::render::MetadataFrame base;
    base.oamd.object_count=1;base.oamd.bed_channels=1;base.oamd.bed_channel_ids={3};
    auto late=base;late.sample=128;late.oamd.gain_updates={{0,128,0,0,0},{0,0,0,.75F,0}};
    auto early=base;early.oamd.gain_updates={{0,256,0,.5F,0},{0,0,0,1,0}};
    auto middle=base;middle.sample=64;middle.oamd.gain_updates={{0,192,0,0,0},{0,0,0,.5F,0},{0,192,0,.25F,0}};
    const std::array<atmos::render::MetadataFrame,3> timeline{late,early,middle};
    std::vector<float> output;std::string error;
    check(atmos::render::render_objects(std::vector<float>(320,1),1,320,timeline,layout,output,error),
        "unsorted frames and unsorted per-frame updates render");
    for(const auto [sample,gain]:std::array<std::pair<unsigned,float>,4>{{{0,1.0F},{64,.5F},{128,.75F},{256,.25F}}})
        check(output[sample*6+3]==gain,"event time wins first; original frame and update order break ties exactly");
    unsigned callbacks=0;
    const auto rejected=[&](const auto& metadata) {
        return !atmos::render::render_objects_stream(1,1,metadata,layout,
            [&](std::span<float>){++callbacks;},
            [&](std::span<const float>,const atmos::render::CarrierHistory*){++callbacks;},error);
    };
    auto future=base;future.sample=50000;
    future.oamd.gain_updates={{0,0,0,std::numeric_limits<float>::quiet_NaN(),0}};
    check(rejected(std::span(&future,1)) && callbacks==0 && error=="invalid object gain",
        "invalid future metadata still fails before any block callback");
    for(unsigned kind=0;kind<4;++kind)for(bool ramp:{false,true}) {
        auto bad=base;bad.sample=std::numeric_limits<std::uint64_t>::max()-4;
        atmos::eac3::OamdPositionUpdate p;p.sample_offset=ramp ? 3 : 5;p.ramp_duration=ramp ? 2 : 0;
        if(kind==0)bad.oamd.position_updates={p};
        if(kind==1)bad.oamd.gain_updates={{0,p.sample_offset,0,1,p.ramp_duration}};
        if(kind==2)bad.oamd.spread_updates={{0,p.sample_offset,p.ramp_duration,0,-1,-1}};
        if(kind==3)bad.oamd.zone_updates={{0,p.sample_offset,0,0,true,p.ramp_duration}};
        check(rejected(std::span(&bad,1)) && callbacks==0 && error.find("timestamp overflow")!=std::string::npos,
            "each event family rejects timestamp and ramp overflow before callbacks");
    }
}

void test_renderer_metadata_source() {
    using namespace atmos::render;
    Layout layout;check(parse_layout("5.1",layout),"metadata source layout parses");
    MetadataFrame base;base.oamd.object_count=1;base.oamd.bed_channels=1;base.oamd.bed_channel_ids={3};
    auto late=base;late.sample=128;late.oamd.gain_updates={{0,128,0,0,0},{0,0,0,.75F,0}};
    auto early=base;early.oamd.gain_updates={{0,256,0,.5F,0},{0,0,0,1,0}};
    auto middle=base;middle.sample=64;middle.oamd.gain_updates={{0,192,0,.25F,0},{0,0,0,.5F,0}};
    const std::array<MetadataPacket,3> packets{{{0,late},{0,early},{64,middle}}};
    unsigned factories=0,callbacks=0;bool validated=false;std::string error;std::vector<float> output;
    const MetadataReaderFactory source=[&] {
        const auto pass=++factories;
        return MetadataReader{[&,pass,index=std::size_t{0}](MetadataPacket& packet) mutable {
            if(index==packets.size()) {packet={};if(pass==1)validated=true;return false;}
            packet=packets[index++];return true;
        }};
    };
    check(render_objects_stream(1,320,source,layout,
        [&](std::span<float> block){check(validated,"source validation finishes before reading PCM");++callbacks;std::fill(block.begin(),block.end(),1.0F);},
        [&](std::span<const float> block,const CarrierHistory*){++callbacks;output.insert(output.end(),block.begin(),block.end());},error) && factories==2 && callbacks==2,
        "renderer uses separate validation and playback sources");
    for(const auto [sample,gain]:std::array<std::pair<unsigned,float>,4>{{{0,1.0F},{64,.5F},{128,.75F},{256,.25F}}})
        check(output[sample*6+3]==gain,"source watermark supports reordered frame timestamps and original tie order");
    const auto no_read=[&](std::span<float>){++callbacks;};
    const auto no_write=[&](std::span<const float>,const CarrierHistory*){++callbacks;};
    callbacks=0;
    check(!render_objects_stream(1,1,MetadataReaderFactory{},layout,no_read,no_write,error) && callbacks==0,
        "renderer rejects a missing metadata source factory before callbacks");
    check(!render_objects_stream(1,1,MetadataReaderFactory{[]{return MetadataReader{};}},layout,no_read,no_write,error) && callbacks==0,
        "renderer rejects a missing metadata reader before callbacks");
    for(bool backwards:{false,true}) {
        check(!render_objects_stream(1,1,MetadataReaderFactory{[&,backwards] {
            return MetadataReader{[&,backwards,index=0U](MetadataPacket& packet) mutable {
                if(index==2)return false;
                packet={index==0 ? 64U : backwards ? 32U : 128U,base};
                packet.frame.sample=backwards ? packet.source_sample : 0;
                ++index;return true;
            }};
        }},layout,no_read,no_write,error) && callbacks==0 && error.find("source timestamp")!=std::string::npos,
            "invalid source watermark or scheduled time is rejected even past render duration");
    }
    rejects([&]{(void)render_objects_stream(1,1,MetadataReaderFactory{[] {
        return MetadataReader{[](MetadataPacket&)->bool{throw std::runtime_error("metadata read failed");}};
    }},layout,no_read,no_write,error);},"metadata validation read failures propagate");
    check(callbacks==0,"failed source validation does not invoke PCM callbacks");
}

void test_streaming_renderer_blocks() {
    atmos::render::Layout layout;check(atmos::render::parse_layout("5.1",layout),"streaming layout parses");
    atmos::render::MetadataFrame frame;
    frame.oamd.object_count=1;frame.oamd.bed_channels=1;frame.oamd.bed_channel_ids={14};
    frame.oamd.gain_updates={{0,0,0,14533.0F/4096.0F,0}};
    constexpr std::array<float,6> native{2.828871250152588F,0,0,0,1.570151686668396F,0};
    unsigned phase=0,reads=0,writes=0;std::string error;
    check(atmos::render::render_objects_stream(1,1553,std::span(&frame,1),layout,
        [&](std::span<float> block) {
            check(phase==reads*2 && block.size()==(reads ? 17U : 1536U),"stream reader receives exact bounded blocks in order");
            std::fill(block.begin(),block.end(),1.0F);++reads;++phase;
        },
        [&](std::span<const float> block,const atmos::render::CarrierHistory* history) {
            check(phase==writes*2+1 && block.size()==(writes ? 17U : 1536U)*6 && history,
                "stream writer receives each carrier block and history before the next read");
            for(std::size_t i=0;i<block.size();++i)check(block[i]==native[i%6],"streamed PCM retains independent native wide-bed gains");
            for(unsigned slot=0;slot<24;++slot)for(unsigned channel=0;channel<6;++channel)
                check((*history)[slot*128+channel]==native[channel],"partial final block retains the native padded history");
            ++writes;++phase;
        },error,0,true) && reads==2 && writes==2,"stream renderer closes the partial final block exactly once");
    check(!atmos::render::render_objects_stream(1,1553,std::span(&frame,1),layout,{},
        [](std::span<const float>,const atmos::render::CarrierHistory*){},error),"stream renderer rejects missing callbacks");
    check(!atmos::render::render_objects_stream(1,std::numeric_limits<std::uint64_t>::max(),std::span(&frame,1),layout,
        [](std::span<float>){throw std::runtime_error("unexpected read");},
        [](std::span<const float>,const atmos::render::CarrierHistory*){throw std::runtime_error("unexpected write");},
        error,0,true) && error.find("overflow")!=std::string::npos,"history frame rounding rejects sample-count overflow before callbacks");
    unsigned callbacks=0;
    rejects([&]{(void)atmos::render::render_objects_stream(1,3072,std::span(&frame,1),layout,
        [&](std::span<float> block){++callbacks;std::fill(block.begin(),block.end(),1.0F);},
        [&](std::span<const float>,const atmos::render::CarrierHistory*){++callbacks;throw std::runtime_error("writer failed");},error);},
        "stream renderer propagates writer failures to the output owner");
    check(callbacks==2,"writer failure stops all subsequent reads");
}

void test_carrier_trim_timing() {
    atmos::render::Layout layout;check(atmos::render::parse_layout("5.1",layout),"trim timing layout parses");
    atmos::render::MetadataFrame first;first.oamd.object_count=1;
    atmos::eac3::OamdPositionUpdate p;p.x=p.y=.25F;p.z=.5F;p.sample_offset=512;p.ramp_duration=256;
    first.oamd.position_updates={p};
    auto next=first;next.sample=1536;next.oamd.trim_present=true;next.oamd.global_trim_mode=2;next.oamd.trims[1].mode=1;
    auto only=next;only.sample=1024;only.oamd.position_updates.clear();only.oamd.trims[1].mode=2;only.oamd.trims[1].levels[2]=15;
    const std::array<atmos::render::MetadataFrame,3> timeline{first,only,next};
    std::vector<float> output;std::string error;
    check(atmos::render::render_objects(std::vector<float>(3072,1),1,3072,timeline,layout,output,error),
        "trim timing scene renders");
    // Native trim-timing58 offset512 control: profile changes only at the first object block.
    check(output[1200*6]==.38050714135169983F && output[2047*6]==.38050714135169983F,
        "trim-only packets and early packet headers do not retarget the carrier");
    check(output[2304*6]==.49998801946640015F,"trim profile follows the object-block ramp");
    const auto expected=output;output.clear();
    const atmos::render::MetadataReaderFactory source=[&] {
        return atmos::render::MetadataReader{[&,index=std::size_t{0}](atmos::render::MetadataPacket& packet) mutable {
            if(index==timeline.size()){packet={};return false;}
            packet={timeline[index].sample,timeline[index]};++index;return true;
        }};
    };
    check(atmos::render::render_objects_stream(1,3072,source,layout,
        [](std::span<float> block){std::fill(block.begin(),block.end(),1.0F);},
        [&](std::span<const float> block,const atmos::render::CarrierHistory*){output.insert(output.end(),block.begin(),block.end());},error) && output==expected,
        "pending and active trim profiles survive source packet replacement and clearing at EOF");
    next.oamd.trim_bypass={true,false};
    check(!atmos::render::render_objects(std::vector<float>(3072,1),1,3072,std::span(&next,1),layout,output,error),
        "renderer rejects a bypass array that does not match decoded signals");
}

void test_core_modes() {
    for (const unsigned mode : {1U, 2U, 7U}) {
        for (const unsigned blocks : {1U, 2U, 3U, 6U}) {
            atmos::encode::CoreEncoder encoder;
            atmos::encode::CoreConfig config;
            config.audio_coding_mode = static_cast<std::uint8_t>(mode);
            config.blocks = static_cast<std::uint8_t>(blocks);
            config.sample_rate = mode == 1 ? 32000 : mode == 2 ? 44100 : 48000;
            encoder.configure(config);
            std::vector<float> input(encoder.samples_per_frame() * encoder.channels(), 0);
            std::vector<std::uint8_t> encoded;
            encoder.encode(input, encoded);
            atmos::eac3::BedDecoder decoder;
            decoder.reset();
            atmos::eac3::PcmFrame decoded;
            std::string error;
            check(decoder.decode(encoded, decoded, error), "mono/stereo/5.1 block and rate combinations decode");
            check(decoded.samples_per_channel == blocks * 256, "configured block count survives decoding");
        }
    }
}

void test_protection() {
    atmos::encode::CoreEncoder encoder;
    configure_51(encoder);
    atmos::eac3::OamdFrame oamd;
    oamd.object_count = 6; oamd.bed_channels = 6;
    const auto pcm = make_tone(48000, 1, 6);
    std::vector<std::uint8_t> frame;
    atmos::encode::encode_atmos_frame(encoder, pcm, oamd, {}, true, false, frame);
    const auto info = atmos::eac3::parse_bitstream_info(frame, 0);
    const auto syntax = atmos::eac3::parse_audio_frame_syntax(frame, *info);
    atmos::eac3::DitherGenerator dither;
    dither.reset();
    const auto spectral = atmos::eac3::decode_spectral_frame(frame, *info, *syntax, dither);
    check(spectral.has_value(), "protected frame still decodes");
    if (!spectral) return;
    const auto audio = atmos::encode::protection_audio(frame);
    auto emdf = spectral->auxiliary_data;
    check(atmos::encode::verify_emdf_primary(emdf, audio), "muxer signs the encoded audio and EMDF");
    auto changed_audio = audio;
    changed_audio.back() ^= 1;
    check(!atmos::encode::verify_emdf_primary(emdf, changed_audio), "changed audio fails authentication");
    emdf[0] ^= 1;
    rejects([&] { atmos::encode::sign_emdf(emdf, {}); }, "malformed metadata is not signed");

    std::ifstream reference("../atmos/reference/invasion.eac3", std::ios::binary);
    if (!reference) { std::printf("external DD+ authentication fixture absent, skipping\n"); return; }
    for (unsigned index = 0; index < 8; ++index) {
        std::vector<std::uint8_t> bytes(4);
        reference.read(reinterpret_cast<char*>(bytes.data()), 4);
        if (!reference) { check(false, "authentication fixture is complete"); break; }
        const auto size = ((static_cast<unsigned>(bytes[2] & 7U) << 8U) | bytes[3]) * 2U + 2U;
        bytes.resize(size);
        reference.read(reinterpret_cast<char*>(bytes.data() + 4), size - 4);
        const auto header = atmos::eac3::parse_bitstream_info(bytes, 0);
        const auto shape = atmos::eac3::parse_audio_frame_syntax(bytes, *header);
        dither.reset();
        const auto decoded = atmos::eac3::decode_spectral_frame(bytes, *header, *shape, dither);
        check(decoded.has_value(), "reference spectral frame parses");
        if (decoded) {
            const bool valid = atmos::encode::verify_emdf_primary(decoded->auxiliary_data,
                atmos::encode::protection_audio(bytes));
            if (!valid) std::fprintf(stderr, "native authentication frame %u failed\n", index);
            check(valid, "native Dolby primary protection matches");
        }
    }
}

void test_skip_fragments() {
    for (const unsigned rate : {32000U, 44100U, 48000U}) {
        for (const unsigned blocks : {1U, 2U, 3U, 6U}) {
            atmos::encode::CoreConfig config;
            config.sample_rate = rate; config.bitrate = rate * 20U;
            config.blocks = static_cast<std::uint8_t>(blocks);
            config.audio_coding_mode = 1; config.lfe = false;
            atmos::encode::CoreEncoder encoder;
            encoder.configure(config);
            const auto capacity = std::min<std::size_t>(2047, blocks * 511U);
            rejects([&] { encoder.set_skip_payload(std::vector<std::uint8_t>(capacity + 1)); },
                "skip payload rejects frame/block capacity overflow");
            const std::vector<float> pcm(blocks * 256U);
            const auto decode = [&](const std::vector<std::uint8_t>& frame) {
                const auto info = atmos::eac3::parse_bitstream_info(frame, 0);
                check(info.has_value(), "fragmented header parses");
                if (!info) return std::optional<atmos::eac3::SpectralFrame>{};
                const auto syntax = atmos::eac3::parse_audio_frame_syntax(frame, *info);
                check(syntax.has_value(), "fragmented syntax parses");
                if (!syntax) return std::optional<atmos::eac3::SpectralFrame>{};
                atmos::eac3::DitherGenerator dither; dither.reset();
                return atmos::eac3::decode_spectral_frame(frame, *info, *syntax, dither);
            };
            for (const std::size_t size : {0U, 1U, 510U, 511U, 512U, 1021U, 1022U,
                                          1023U, 1532U, 1533U, 1534U, 2046U, 2047U}) {
                if (size > capacity) continue;
                std::vector<std::uint8_t> payload(size);
                for (std::size_t i = 0; i < size; ++i) payload[i] = static_cast<std::uint8_t>(i * 37U + 11U);
                std::vector<std::uint8_t> frame;
                try { encoder.set_skip_payload(payload); encoder.encode(pcm, frame); }
                catch (const std::exception&) { check(false, "bounded skip payload encodes"); continue; }
                check(atmos::crc16_ansi(std::span<const std::uint8_t>(frame).subspan(2)) == 0,
                    "fragmented frame CRC is valid");
                const auto spectral = decode(frame);
                check(spectral.has_value(), "fragmented core decodes");
                if (!spectral) continue;
                check(spectral->auxiliary_data == payload, "all skip fragments concatenate without data loss");
                std::size_t count = 0;
                for (const auto& [first, last] : spectral->skip_ranges) {
                    if (last - first <= 1) continue;
                    ++count;
                    check(last - first >= 10 && (last - first - 10) % 8 == 0 &&
                          (last - first - 10) / 8 <= 511, "each skip fragment has a bounded byte length");
                }
                check(count == (size + 510) / 511, "skip fragment count follows payload boundaries");
            }
            std::vector<std::uint8_t> frame;
            encoder.encode(pcm, frame);
            const auto empty = decode(frame);
            check(empty && empty->auxiliary_data.empty(), "successful encode clears fragmented payload state");
        }
    }
}

void test_dense_oamd_budget() {
    using atmos::encode::joc_payload_budget;
    check(joc_payload_budget(80,15,768000) == 423, "ordinary OAMD keeps the historical JOC budget");
    check(joc_payload_budget(568,15,768000) == 372, "dense OAMD retains the high-rate native fit budget");
    for (unsigned objects=1;objects<=16;++objects) {
        const auto floor=4+4*objects;
        check(joc_payload_budget(1000,objects,448000)==floor, "dense OAMD reserves a bounded minimal JOC at low rates");
        check(joc_payload_budget(2047-8-floor,objects,768000)==floor, "last representable dense OAMD budget fits");
        rejects([&]{joc_payload_budget(2048-8-floor,objects,768000);}, "dense OAMD beyond skip capacity rejects");
        for (unsigned symbol=0;symbol<96;++symbol) {
            atmos::eac3::JocFrame joc; joc.channels=5; joc.object_count=static_cast<std::uint8_t>(objects);
            for (unsigned i=0;i<objects;++i) {
                auto& object=joc.objects[i];
                object.active=object.sparse=object.steep_slope=true;
                object.bands=object.data_points=1; object.timeslot_offsets[0]=24;
                object.sparse_channel[0][0]=4; object.sparse_vector[0][0]=static_cast<std::uint8_t>(symbol);
            }
            check(atmos::encode::write_joc_payload(joc).size()<=floor,
                "all coarse sparse Huffman symbols fit the minimum reserve");
        }
    }
    rejects([&]{joc_payload_budget(0,0,768000);}, "empty JOC budget layout rejects");
    rejects([&]{joc_payload_budget(0,17,768000);}, "oversized JOC budget layout rejects");
    rejects([&]{joc_payload_budget(std::numeric_limits<std::size_t>::max(),15,768000);}, "OAMD budget overflow rejects");
}

void test_fragmented_protection() {
    atmos::encode::CoreConfig config;
    config.bitrate = 1024000; config.with_additional_bsi = true; config.complexity_index_type_a = 16;
    atmos::encode::CoreEncoder encoder; encoder.configure(config);
    atmos::eac3::OamdFrame oamd;
    oamd.object_count = 16; oamd.bed_channels = 1; oamd.bed_channel_ids = {3};
    for (unsigned i = 1; i < 16; ++i) {
        atmos::eac3::OamdPositionUpdate position;
        position.object = static_cast<std::uint16_t>(i);
        position.x = float(i % 3) / 2; position.y = float(i % 5) / 4; position.z = float(i % 2);
        oamd.position_updates.push_back(position);
    }
    atmos::eac3::JocFrame joc; joc.channels = 5; joc.object_count = 15;
    std::mt19937 random(92);
    for (unsigned obj = 0; obj < 15; ++obj) {
        auto& o = joc.objects[obj];
        o.active = o.fine_quantization = o.steep_slope = true;
        o.bands_index = 7; o.bands = 23; o.data_points = 1; o.timeslot_offsets[0] = 1;
        for (unsigned channel = 0; channel < 5; ++channel) {
            int previous = 0;
            for (unsigned band = 0; band < 23; ++band) {
                const int value = int(random() % 17) - 8;
                o.matrix[0][channel][band] = static_cast<std::uint8_t>((value - previous + 192) % 192);
                previous = value;
            }
        }
    }
    std::vector<std::uint8_t> frame;
    try { atmos::encode::encode_atmos_frame(encoder, std::vector<float>(1536 * 6), oamd, joc, true, true, frame); }
    catch (const std::exception&) { check(false, "large authenticated EMDF encodes"); return; }
    const auto info = atmos::eac3::parse_bitstream_info(frame, 0);
    check(info.has_value(), "large protected header parses"); if (!info) return;
    const auto syntax = atmos::eac3::parse_audio_frame_syntax(frame, *info);
    check(syntax.has_value(), "large protected syntax parses"); if (!syntax) return;
    atmos::eac3::DitherGenerator dither; dither.reset();
    const auto spectral = atmos::eac3::decode_spectral_frame(frame, *info, *syntax, dither);
    check(spectral.has_value(), "large protected core decodes"); if (!spectral) return;
    const auto& emdf = spectral->auxiliary_data;
    check(emdf.size() == 1642, "stress EMDF reaches the fourth skip field");
    const auto audio = atmos::encode::protection_audio(frame);
    check(atmos::encode::verify_emdf_primary(emdf, audio), "muxer signs all EMDF fragments");
    const auto metadata = atmos::eac3::parse_emdf(emdf);
    check(metadata.valid && !metadata.primary_protection.empty(), "fragmented protection parses");
    if (!metadata.valid || metadata.primary_protection.empty()) return;
    auto damaged = emdf;
    const auto bit = 32U + metadata.primary_protection_bit;
    check(bit >= 3U * 511U * 8U && bit < damaged.size() * 8U, "primary protection lies in a later fragment");
    if (bit >= damaged.size() * 8U) return;
    damaged[bit / 8] ^= static_cast<std::uint8_t>(1U << (7U - bit % 8));
    check(!atmos::encode::verify_emdf_primary(damaged, audio), "late-fragment protection corruption is rejected");
    check(atmos::crc16_ansi(std::span<const std::uint8_t>(frame).subspan(2)) == 0,
        "authenticated fragmented frame CRC is valid");
}

void test_mkv_demux() {
    std::filesystem::path mkv;
    for (const char* candidate : {"test_file/00006 ATMOS+EAC+DD.mkv",
            "../test_file/00006 ATMOS+EAC+DD.mkv"}) {
        if (std::filesystem::exists(candidate)) { mkv = candidate; break; }
    }
    if (mkv.empty()) { std::printf("mkv fixture absent, skipping demux test\n"); return; }

    std::ifstream stream(mkv, std::ios::binary);
    if (!stream) { std::printf("mkv unreadable, skipping demux test\n"); return; }
    const std::vector<std::uint8_t> data{std::istreambuf_iterator<char>(stream),
        std::istreambuf_iterator<char>()};
    atmos::mkv::Demuxer demuxer(data);

    std::uint64_t truehd_track = 0;
    std::uint64_t eac3_track = 0;
    std::uint64_t ac3_track = 0;
    for (const auto& track : demuxer.tracks()) {
        if (atmos::mkv::Demuxer::is_truehd(track)) {
            truehd_track = track.number;
            check(track.channels == 8U, "TrueHD track is 7.1");
        } else if (atmos::mkv::Demuxer::is_eac3(track)) {
            eac3_track = track.number;
            check(track.channels == 8U, "E-AC-3 track is 7.1");
        } else if (atmos::mkv::Demuxer::is_ac3(track)) {
            ac3_track = track.number;
            check(track.channels == 6U, "AC-3 track is 5.1");
        }
    }
    check(truehd_track != 0U && eac3_track != 0U && ac3_track != 0U,
        "all three audio tracks are found");

    std::vector<std::uint8_t> eac3;
    std::string error;
    std::vector<std::uint8_t> truehd;
    check(demuxer.extract(truehd_track, truehd, error), "demux the TrueHD track");
    const auto source_unit = atmos::truehd::parse_access_unit(truehd, 0);
    check(source_unit && source_unit->atmos_metadata.size() == 1 && source_unit->atmos_metadata.front().has_oamd,
        "independent TrueHD fixture contains initial Atmos metadata");
    if (source_unit && !source_unit->atmos_metadata.empty()) {
        const auto& source_oamd = source_unit->atmos_metadata.front().oamd;
        check(source_oamd.zone_updates.size() == 15,
            "TrueHD metadata preserves zone fields for all fifteen dynamic objects");
        const auto upper = std::find_if(source_oamd.zone_updates.begin(), source_oamd.zone_updates.end(),
            [](const auto& zone) { return zone.object == 8; });
        check(upper != source_oamd.zone_updates.end() && upper->elevation,
            "native TrueHD upper object explicitly allows the height zone");
    }
    check(source_unit && source_unit->major_sync &&
        source_unit->major_sync->dialogue_normalization[3] == 29,
        "independent DRP fixture reports -29dBFS for the 16ch presentation");
    const auto loudness_path = std::filesystem::current_path() / "ddpatmos-dialnorm-test.ec3";
    if (!std::filesystem::exists(loudness_path)) {
        atmos::encode::Options options;
        options.truehd = mkv.string(); options.truehd_seconds = 0.032;
        options.atmos = true; options.output = loudness_path.string();
        check(atmos::encode::run(options, error) == 4, "TrueHD loudness fixture drains JOC analysis, synthesis and codec tail");
        std::ifstream encoded(loudness_path, std::ios::binary);
        const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(encoded), {}};
        const auto output_info = atmos::eac3::parse_bitstream_info(bytes, 0);
        check(output_info && output_info->dialog_normalization[0] == -29,
            "transcoder carries source dialogue normalization into DD+ BSI");
        encoded.close();
        std::filesystem::remove(loudness_path);
    } else check(false, "loudness test output path must be unused");
    constexpr unsigned metadata_delay = atmos::encode::JocEncoder::carrier_delay + atmos::render::CarrierLimiter::delay;
    std::vector<atmos::encode::TimedOamdPayload> packets, clock_packets;
    std::vector<std::string> expected, actual;
    atmos::truehd::MajorSync stream_info{};
    atmos::truehd::BedDecoder object_decoder(3);
    std::uint64_t object_samples = 0;
    std::uint64_t object_pcm_hash = 14695981039346656037ULL;
    std::uint64_t sample = 0;
    std::uint16_t last_timing = source_unit ? source_unit->input_timing : 0;
    for (std::size_t at = 0; at + 8 <= truehd.size();) {
        const auto unit = atmos::truehd::parse_access_unit(truehd, at, at ? &stream_info : nullptr);
        if (!unit) { check(false, "complete TrueHD metadata fixture parses"); break; }
        if (unit->major_sync) stream_info = *unit->major_sync;
        check(stream_info.dialogue_normalization[3] == 29,
            "full source fixture retains its measured 16ch dialogue level");
        atmos::truehd::PcmFrame objects;
        if (!object_decoder.decode(std::span<const std::uint8_t>(truehd).subspan(at, unit->size), objects, error)) {
            check(false, "full native object presentation decodes without concealment");
            break;
        }
        check(objects.channels == 16 && objects.sample_rate == 48000,
            "object presentation matches the independent TrueHD layout");
        object_samples += objects.samples_per_channel;
        if (!std::all_of(objects.interleaved.begin(), objects.interleaved.end(),
                [](float value) { return value >= -1.0F && value < 1.0F; })) {
            check(false, "object PCM is finite and representable as signed24");
            return;
        }
        for (const float value : objects.interleaved) {
            const auto word = static_cast<std::uint32_t>(static_cast<std::int32_t>(value * 8388608.0F));
            for (const unsigned shift : {16U, 8U, 0U}) {
                object_pcm_hash ^= (word >> shift) & 255U;
                object_pcm_hash *= 1099511628211ULL;
            }
        }
        sample += static_cast<std::uint16_t>(unit->input_timing - last_timing);
        last_timing = unit->input_timing;
        for (const auto& metadata : unit->atmos_metadata) {
            check(metadata.valid, "complete source EVO parses through protection and padding");
            if (!metadata.has_oamd) continue;
            clock_packets.push_back({sample,metadata.oamd_payload});
            const auto decoded_origin = object_samples-objects.samples_per_channel;
            packets.push_back({decoded_origin, metadata.oamd_payload});
            const auto events = oamd_events(metadata.oamd,
                decoded_origin + metadata.oamd_payload.sample_offset + metadata_delay);
            expected.insert(expected.end(), events.begin(), events.end());
        }
        at += unit->size;
    }
    // Whole-file signed24 big-endian PCM fingerprint from truehdd 02d29cd0; provenance in TRUEHD_REFERENCE.txt.
    check(packets.size() == 1535, "full fixture retains every independently audited OAMD payload");
    check(object_samples == 2356000 && object_pcm_hash == 0x0cd188ae78563251ULL,
        "all sixteen native TrueHD object channels match the independent whole-file decode");
    std::printf("TrueHD P3 reference: %llu samples/channel, PCM fingerprint %016llx\n",
        static_cast<unsigned long long>(object_samples), static_cast<unsigned long long>(object_pcm_hash));
    rejects([&] { (void)atmos::encode::reframe_oamd(clock_packets,static_cast<std::size_t>(sample+40)); },
        "input-clock fixture rejects incompatible timing instead of emitting ignored duplicate elements");
    const auto reframed = atmos::encode::reframe_oamd(packets, static_cast<std::size_t>(object_samples+metadata_delay),metadata_delay);
    for (std::size_t index = 0; index < reframed.size(); ++index) {
        const auto parsed = atmos::eac3::parse_oamd_payload(reframed[index]);
        check(parsed.valid, "full fixture reframed OAMD parses");
        const auto events = oamd_events(parsed.oamd, index * 1536);
        actual.insert(actual.end(), events.begin(), events.end());
    }
    std::sort(expected.begin(), expected.end());
    std::sort(actual.begin(), actual.end());
    check(!expected.empty() && expected == actual,
        "actual decoded-PCM timeline preserves every modeled OAMD field, time and ramp");
    std::printf("OAMD reframing: %zu source payloads, %zu unchanged events\n", packets.size(), actual.size());
    check(demuxer.extract(eac3_track, eac3, error), "demux the E-AC-3 track");
    std::size_t offset = 0;
    while (offset + 8U <= eac3.size()) {
        const auto candidate = atmos::eac3::parse_sync_frame(eac3, offset);
        if (candidate && candidate->joc_payload && candidate->oamd_payload) break;
        offset += candidate ? candidate->size : 1U;
    }
    const auto frame = atmos::eac3::parse_sync_frame(eac3, offset);
    check(frame.has_value(), "reference E-AC-3 frame parses");
    if (frame) check(frame->joc_payload && frame->oamd_payload,
        "reference E-AC-3 frame is Atmos");
    std::printf("mkv demux: eac3 track %llu bytes, first frame Atmos=%d\n",
        static_cast<unsigned long long>(eac3.size()),
        frame ? (frame->joc_payload && frame->oamd_payload) : 0);
}

} // namespace

void test_spdif() {
    atmos::encode::CoreEncoder encoder;
    configure_51(encoder);
    atmos::eac3::OamdFrame oamd;
    oamd.object_count = 6; oamd.bed_channels = 6;
    const auto pcm = make_tone(48000, 1, 6);
    std::vector<std::uint8_t> frame;
    atmos::encode::encode_atmos_frame(encoder, pcm, oamd, {}, true, false, frame);
    const auto burst = atmos::encode::iec61937_burst(frame);
    constexpr std::size_t slot_bytes = 24576;
    check(burst.size() == slot_bytes, "one IEC 61937 slot holds one E-AC-3 frame");
    const auto word = [&](std::size_t at) {
        return static_cast<unsigned>(burst[at]) | (static_cast<unsigned>(burst[at + 1U]) << 8U);
    };
    check(word(0) == 0xF872U && word(2) == 0x4E1FU, "IEC 61937 preambles are present");
    check(word(4) == 0x0015U, "IEC 61937 data type is E-AC-3");
    check(word(6) == frame.size(), "IEC 61937 payload size matches the frame");
    bool swapped = true;
    for (std::size_t index = 0; index < frame.size(); index += 2U)
        swapped = swapped && burst[8U + index] == frame[index + 1U] &&
            burst[9U + index] == frame[index];
    check(swapped, "IEC 61937 payload is written word-swapped");
    bool padded = true;
    for (std::size_t index = 8U + frame.size(); index < burst.size(); ++index)
        padded = padded && burst[index] == 0U;
    check(padded, "the rest of the slot is zero padding");
    rejects([&] {
        std::vector<std::uint8_t> odd(frame.begin(), frame.begin() + 3);
        static_cast<void>(atmos::encode::iec61937_burst(odd));
    }, "an odd frame cannot be framed");
}

void test_mkv_packets() {
    using Bytes=std::vector<std::uint8_t>;
    const auto element=[](Bytes id,const Bytes& body,bool unknown=false) {
        if (unknown) id.push_back(0xff);
        else {
            unsigned width=1;
            while (body.size()>=(1ULL<<(7*width))-1) ++width;
            const auto size=(1ULL<<(7*width))|body.size();
            for(unsigned i=width;i--;)id.push_back(static_cast<std::uint8_t>(size>>(i*8)));
        }
        id.insert(id.end(),body.begin(),body.end());return id;
    };
    const auto wrap=[&](const Bytes& body) {
        return element({0x18,0x53,0x80,0x67},element({0x1f,0x43,0xb6,0x75},body));
    };
    const auto verify=[&](const Bytes& data,const std::vector<Bytes>& expected) {
        atmos::mkv::PacketReader reader(data,1);std::span<const std::uint8_t> packet;std::string error;
        std::vector<Bytes> result;
        while(reader.next(packet,error)) {
            const auto address=reinterpret_cast<std::uintptr_t>(packet.data());
            const auto begin=reinterpret_cast<std::uintptr_t>(data.data());
            check(address>=begin && address-begin<=data.size() && packet.size()<=data.size()-(address-begin),
                "Matroska packets borrow the input bytes");
            result.emplace_back(packet.begin(),packet.end());
        }
        check(error.empty() && result==expected,"packet bytes and lace sizes match independent vectors");
        check(!reader.next(packet,error) && error.empty() && packet.empty(),"packet EOF is stable and clears the span");
        Bytes flat,reference;
        for(const auto& bytes:expected)reference.insert(reference.end(),bytes.begin(),bytes.end());
        atmos::mkv::Demuxer demuxer(data);
        check(demuxer.extract(1,flat,error) && flat==reference,"legacy extraction preserves packet order");
    };
    verify(wrap(element({0xa3},{0x81,0,0,0x80,1,2,3})),{{1,2,3}});
    verify(wrap(element({0xa3},{0x81,0,0,0x82,2,2,3,1,2,3,4,5,6})),{{1,2},{3,4,5},{6}});
    verify(wrap(element({0xa3},{0x81,0,0,0x84,2,1,2,3,4,5,6})),{{1,2},{3,4},{5,6}});
    verify(wrap(element({0xa3},{0x81,0,0,0x86,2,0x82,0xc0,1,2,3,4,5,6})),{{1,2},{3,4,5},{6}});
    verify(wrap(element({0xa3},{0x81,0,0,0x86,2,0x83,0xbd,1,2,3,4,5,6})),{{1,2,3},{4},{5,6}});
    verify(wrap(element({0xa3},{0x81,0,0,0x86,0,1,2,3})),{{1,2,3}});
    verify(wrap(element({0xa3},{0x81,0,0,0x86,2,0x80,0xc1,1,2,3})),{{},{1,2},{3}});
    Bytes xiph{0x81,0,0,0x82,1,255,1};xiph.insert(xiph.end(),256,7);xiph.push_back(8);
    verify(wrap(element({0xa3},xiph)),{Bytes(256,7),{8}});
    verify(wrap(element({0xa3},{0x40,0x01,0,0,0x80,1,2})),{{1,2}});
    Bytes maximum{0x81,0,0,0x84,255};maximum.insert(maximum.end(),256,9);
    verify(wrap(element({0xa3},maximum)),std::vector<Bytes>(256,Bytes{9}));
    Bytes mixed=element({0xa3},{0x82,0,0,0x80,99});
    const auto group=element({0xa0},element({0xa1},{0x81,0,0,0x84,1,1,2,3,4}));
    mixed.insert(mixed.end(),group.begin(),group.end());
    auto clusters=element({0x1f,0x43,0xb6,0x75},mixed,true);
    const auto cues=element({0x1c,0x53,0xbb,0x6b},{});clusters.insert(clusters.end(),cues.begin(),cues.end());
    const auto second=element({0x1f,0x43,0xb6,0x75},element({0xa3},{0x81,0,0,0x80,5,6}),true);
    clusters.insert(clusters.end(),second.begin(),second.end());
    verify(element({0x18,0x53,0x80,0x67},clusters,true),{{1,2},{3,4},{5,6}});
    const auto bad=[&](const Bytes& data) {
        atmos::mkv::PacketReader reader(data,1);std::span<const std::uint8_t> packet;std::string error;
        check(!reader.next(packet,error) && !error.empty() && packet.empty(),"bad lacing fails before exposing clipped packets");
        const auto first=error;
        check(!reader.next(packet,error) && error==first,"packet errors are distinct from clean EOF");
    };
    for(const auto& body:std::vector<Bytes>{{},{0x40},{0x81,0,0},
        {0x81,0,0,0x82,1,255},{0x81,0,0,0x82,1,4,1,2},
        {0x81,0,0,0x84,1,1,2,3},{0x81,0,0,0x86,1,0x40},
        {0x81,0,0,0x86,1,0x85,1,2},{0x81,0,0,0x86,2,0x80,0xbe,1,2},
        {0x81,0,0,0x86,2,0x81,0xc0,1,2}})bad(wrap(element({0xa3},body)));
    bad(wrap({0xa3,0x90,0x81,0,0,0x80,1}));
    bad(wrap(element({0xa3},{0x81,0,0,0x80,1},true)));
    bad(wrap(element({0xa0},element({0xa1},{0x81,0,0,0x80,1}),true)));
    auto truncated=wrap(element({0xa3},{0x81,0,0,0x80,1}));truncated.pop_back();bad(truncated);
}

void test_mkv_writer() {
    atmos::encode::CoreEncoder encoder;
    configure_51(encoder);
    atmos::eac3::OamdFrame oamd;
    oamd.object_count = 6; oamd.bed_channels = 6;
    const auto pcm = make_tone(48000, 1, 6);
    std::vector<std::uint8_t> frame;
    atmos::encode::encode_atmos_frame(encoder, pcm, oamd, {}, true, false, frame);
    const std::filesystem::path ec3 = "mkv-writer-test.ec3";
    const std::filesystem::path mkv = "mkv-writer-test.mkv";
    {
        std::ofstream out(ec3, std::ios::binary);
        out.write(reinterpret_cast<const char*>(frame.data()),
            static_cast<std::streamsize>(frame.size()));
    }
    std::string error;
    const bool written = atmos::encode::write_matroska(ec3, mkv, error);
    std::filesystem::remove(ec3);
    check(written, "matroska muxer accepts an E-AC-3 stream");
    if (!written) {
        std::printf("matroska muxer error: %s\n", error.c_str());
        std::filesystem::remove(mkv);
        return;
    }
    std::ifstream in(mkv, std::ios::binary);
    const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(in),
        std::istreambuf_iterator<char>()};
    in.close();
    std::filesystem::remove(mkv);
    check(bytes.size() > frame.size() + 64U, "matroska file contains the frame");
    check(bytes.size() > 4U && bytes[0] == 0x1AU && bytes[1] == 0x45U &&
        bytes[2] == 0xDFU && bytes[3] == 0xA3U, "matroska file starts with the EBML header");
    atmos::mkv::Demuxer demuxer(bytes);
    check(demuxer.tracks().size() == 1U, "matroska file has one track");
    if (demuxer.tracks().size() == 1U) {
        const auto& track = demuxer.tracks()[0];
        check(atmos::mkv::Demuxer::is_eac3(track), "matroska track is E-AC-3");
        check(track.channels == 6U && track.sample_rate == 48000.0, "matroska track is 5.1 at 48 kHz");
        std::vector<std::uint8_t> extracted;
        std::string extract_error;
        check(demuxer.extract(track.number, extracted, extract_error),
            "matroska track extracts back");
        check(extracted == frame, "extracted frame matches the encoded frame");
    }
}

void test_mp4_writer() {
    atmos::encode::CoreEncoder encoder;
    configure_51(encoder);
    atmos::eac3::OamdFrame oamd;
    oamd.object_count = 6; oamd.bed_channels = 6;
    const auto pcm = make_tone(48000, 1, 6);
    std::vector<std::uint8_t> frame;
    atmos::encode::encode_atmos_frame(encoder, pcm, oamd, {}, true, false, frame);
    const std::filesystem::path ec3 = "mp4-writer-test.ec3";
    const std::filesystem::path mp4 = "mp4-writer-test.mp4";
    {
        std::ofstream out(ec3, std::ios::binary);
        out.write(reinterpret_cast<const char*>(frame.data()),
            static_cast<std::streamsize>(frame.size()));
    }
    std::string error;
    const bool written = atmos::encode::write_mp4(ec3, mp4, 768000U, error);
    std::filesystem::remove(ec3);
    check(written, "mp4 muxer accepts an E-AC-3 stream");
    if (!written) {
        std::printf("mp4 muxer error: %s\n", error.c_str());
        std::filesystem::remove(mp4);
        return;
    }
    std::ifstream in(mp4, std::ios::binary);
    const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(in),
        std::istreambuf_iterator<char>()};
    in.close();
    std::filesystem::remove(mp4);
    const auto has = [&](const char* text, std::size_t length) {
        return std::search(bytes.begin(), bytes.end(), text, text + length) != bytes.end();
    };
    check(bytes.size() > frame.size() + 128U, "mp4 file contains the frame");
    check(has("ftyp", 4) && has("moov", 4) && has("mdat", 4), "mp4 has the mandatory boxes");
    check(has("ec-3", 4) && has("dec3", 4), "mp4 carries the E-AC-3 sample entry");
    check(std::search(bytes.begin(), bytes.end(), frame.begin(), frame.end()) != bytes.end(),
        "mp4 mdat carries the encoded frame verbatim");
}

void test_object_export() {
    const auto dir=std::filesystem::current_path()/"ddpatmos-object-export-test";
    if (std::filesystem::exists(dir)) { check(false,"object export test path must be unused"); return; }
    atmos::eac3::OamdFrame layout;
    layout.object_count=2; layout.bed_channels=1; layout.bed_channel_ids={3};
    {
        atmos::encode::ObjectExport output(dir,48000,layout);
        atmos::eac3::OamdPositionUpdate p;
        p.object=1; p.x=1; p.y=.5F;
        layout.position_updates.push_back(p);
        layout.gain_updates.push_back({1,64,0,.5F,0});
        output.metadata(0,layout);
        std::array<float,256> pcm{};
        for (unsigned i=0;i<128;++i) { pcm[2*i]=.25F; pcm[2*i+1]=.75F; }
        output.write(pcm); output.finish();
    }
    const auto read=[](const std::filesystem::path& path) {
        std::ifstream input(path,std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(input),{});
    };
    const auto bed=read(dir/"bed.wav"), object=read(dir/"object_1.wav");
    check(bed.size()==80+128*4 && object.size()==bed.size(),"bed and objects retain source duration");
    float bed_sample=0, object_sample=0;
    if (bed.size()>=84 && object.size()>=84) {
        std::memcpy(&bed_sample,bed.data()+80,4); std::memcpy(&object_sample,object.data()+80,4);
    }
    check(bed_sample==.25F && object_sample==.75F,"bed excludes objects; stems retain original PCM without metadata gain");
    check(read(dir/"bed.json").find("\"channels\":[\"LFE\"]")!=std::string::npos,"viewer bed schema");
    const auto json=read(dir/"object_1.coordinates.jsonl");
    check(json.find("\"azimuthDeg\":90")!=std::string::npos &&
        json.find("\"ptsSamples\":64")!=std::string::npos &&
        json.find("\"metadataGainLinear\":0.5")!=std::string::npos,"viewer coordinates and sample-accurate gain changes");
    bool refused=false;
    try { atmos::encode::ObjectExport duplicate(dir,48000,layout); }
    catch (const std::exception&) { refused=true; }
    check(refused && read(dir/"bed.wav")==bed,"existing object export is never overwritten");
    for (const auto* name:{"bed.wav","bed.json","object_1.wav","object_1.coordinates.jsonl"})
        std::filesystem::remove(dir/name);
    std::filesystem::remove(dir);
}

int main() {
    try {
    test_mapped_file();
    test_object_export();
    test_frame_shape();
    test_round_trip();
    test_exponent_reuse_headroom();
    test_emdf();
    test_oamd_timing();
    test_oamd_cross_frame();
    test_oamd_gain_predictor();
    test_oamd_render_state();
    test_oamd_differential_position();
    test_evolution_offset();
    test_evolution_payloads();
    test_truehd_dialogue_normalization();
    test_joc_qmf();
    test_joc_surround_phase();
    test_joc_decoder_timing();
    test_joc_two_steep_points();
    test_joc_inactive_history();
    test_joc_analysis();
    test_joc_carrier_correction();
    test_joc_initial_history();
    test_joc_refinement();
    test_joc_zero_compaction();
    test_carrier_panning();
    test_carrier_width_rendering();
    test_carrier_zone_snap();
    test_carrier_trim_boundary();
    test_oamd_trim_fields();
    test_carrier_explicit_trim_gains();
    test_carrier_trim_gain_order();
    test_renderer_event_order();
    test_renderer_metadata_source();
    test_streaming_renderer_blocks();
    test_carrier_trim_timing();
    test_carrier_gain_ramps();
    test_carrier_history();
    test_carrier_gain();
    test_carrier_limiter();
    test_oamd_bed_headers();
    test_oamd_final_element_length();
    test_spatial_wav();
    test_fast_atmos();
    test_live_renderer();
    test_boundaries();
    test_core_modes();
    test_protection();
    test_skip_fragments();
    test_dense_oamd_budget();
    test_fragmented_protection();
    test_spdif();
    test_mkv_packets();
    test_mkv_writer();
    test_mp4_writer();
    test_mkv_demux();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "unexpected exception: %s\n", error.what());
        return 1;
    }
    if (failures == 0) {
        std::printf("encoder tests: all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "encoder tests: %d failure(s)\n", failures);
    return 1;
}

// Recovered DEE 48 kHz, six-channel limiter with 32-sample lookahead.
#pragma once
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <span>
#include <stdexcept>

namespace atmos::render {
class CarrierLimiter final {
public:
    static constexpr unsigned delay = 32;

    void process(std::span<float> pcm) {
        if (pcm.size() % (delay * 6)) throw std::runtime_error("invalid limiter block shape");
        if (!std::all_of(pcm.begin(), pcm.end(), [](float x) { return std::isfinite(x); }))
            throw std::runtime_error("nonfinite limiter input");
        for (std::size_t offset = 0; offset < pcm.size(); offset += delay * 6) {
            float peak = 0;
            for (unsigned i = 0; i < delay * 6; ++i) peak = std::max(peak, std::abs(pcm[offset + i]));
            const float previous_gain = gain_;
            advance(peak);
            const float before = previous_gain * (1.0F / 256);
            const float after = gain_ * (1.0F / 256);
            for (unsigned t = 0; t < delay; ++t) {
                for (unsigned channel = 0; channel < 6; ++channel) {
                    auto& input = pcm[offset + t * 6 + channel];
                    const float old = samples_[t * 6 + channel];
                    samples_[t * 6 + channel] = input;
                    input = (((old * before) * window_[31 - t]) + ((old * after) * window_[t])) * 256.0F;
                }
            }
        }
    }

private:
    void advance(float peak) {
        float value = std::max(peak, previous_peak_) * makeup_;
        previous_peak_ = peak;
        float attenuation = 1;
        if (value > 1) {
            int exponent = 0;
            while (value >= 1) { value *= .5F; --exponent; }
            float reciprocal = value <= .707122802734375F ? (1.0F - value) * 2 : 1.0F - .5F * value;
            for (unsigned i = 0; i < 4; ++i)
                reciprocal += ((.5F - reciprocal * value) * reciprocal) * 2;
            attenuation = reciprocal * std::bit_cast<float>(static_cast<std::uint32_t>(exponent + 128) << 23);
        }
        const float target = makeup_ * attenuation;
        if (gain_ > target) {
            smooth_ = attenuation;
            hold_ = 0;
            gain_ = target;
        } else if (hold_ < 3) {
            ++hold_;
        } else {
            smooth_ = (attenuation - .911712646484375F * attenuation) + .911712646484375F * smooth_;
            const float released = smooth_ * makeup_;
            if (released > gain_) gain_ = released;
            if (gain_ > target) gain_ = target;
        }
        if (smooth_ < .841400146484375F)
            makeup_ = ((((smooth_ * makeup_) * .59423828125F) * 2) * .011474609375F)
                + makeup_ * .988525390625F;
        else makeup_ = makeup_ * .997100830078125F + .002899169921875F;
    }

    float previous_peak_ = 0, makeup_ = 1, gain_ = 1, smooth_ = 1;
    unsigned hold_ = 0;
    std::array<float, delay * 6> samples_{};
    static constexpr std::array<float, delay> window_{
        .0006103515625F,.005401611328125F,.014984130859375F,.02923583984375F,
        .048004150390625F,.071136474609375F,.098388671875F,.1295166015625F,
        .164215087890625F,.2021484375F,.242950439453125F,.286224365234375F,
        .33154296875F,.378509521484375F,.4266357421875F,.4754638671875F,
        .5245361328125F,.5733642578125F,.621490478515625F,.66845703125F,
        .713775634765625F,.757049560546875F,.7978515625F,.835784912109375F,
        .8704833984375F,.901611328125F,.928863525390625F,.951995849609375F,
        .97076416015625F,.985015869140625F,.994598388671875F,.9993896484375F};
};
} // namespace atmos::render

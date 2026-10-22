// Recovered DEE carrier normalization after JOC analysis, before core encoding.
#pragma once
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <span>
#include <stdexcept>

namespace atmos::encode::joc {
class CarrierCorrection final {
public:
    void process(std::span<float> pcm, const std::array<float, 5>& gains) {
        if (pcm.size() != 1536 * 6 ||
            !std::all_of(pcm.begin(), pcm.end(), [](float x) { return std::isfinite(x); }) ||
            !std::all_of(gains.begin(), gains.end(), [](float x) {
                return std::isfinite(x) && x >= .95F && x <= 1.05F;
            })) throw std::runtime_error("invalid JOC carrier correction");
        constexpr unsigned channels[] = {0,1,2,4,5};
        // Limit downmix changes to 5% and ramp over the normalizer's 256-sample transition.
        for (unsigned t = 0; t < 1536; ++t) {
            const float mix = float(std::min(t, 256U)) / 256;
            for (unsigned channel = 0; channel < 5; ++channel)
                pcm[t * 6 + channels[channel]] *= previous_[channel] * (1-mix) + gains[channel] * mix;
        }
        previous_ = gains;
    }
private:
    std::array<float, 5> previous_{1,1,1,1,1};
};

class CarrierGain final {
public:
    static constexpr unsigned delay = 256;
    explicit CarrierGain(unsigned mode = 2) : mode_(mode) {
        if (mode > 4) throw std::runtime_error("invalid JOC headroom mode");
    }

    std::uint8_t process(std::span<float> pcm) {
        if (pcm.size() != 1536 * 6) throw std::runtime_error("invalid JOC gain frame shape");
        if (!std::all_of(pcm.begin(), pcm.end(), [](float x) { return std::isfinite(x); }))
            throw std::runtime_error("nonfinite JOC gain input");
        float peak = tail_peak_;
        tail_peak_ = 0;
        for (unsigned t = 0; t < 1536; ++t)
            for (unsigned c = 0; c < 6; ++c) if (c != 3) {
                const float value = std::abs(pcm[t * 6 + c]);
                if (t < 1280) peak = std::max(peak, value);
                else tail_peak_ = std::max(tail_peak_, value);
            }
        peak = std::max(peak, tail_peak_);
        const float log_peak = logarithm(peak) - static_cast<float>(mode_) / 48.0F;
        float target = static_cast<float>(std::clamp(static_cast<double>(log_peak) * 6, -1.0, 1.0)) - .0078125F;
        const float released = envelope_ + .0015F;
        if (released > target) hold_ = 5;
        else if (hold_) { --hold_; target = envelope_; }
        else target = released;
        envelope_ = target;
        unsigned index = 0;
        const float level = std::min(target, 0.0F);
        while (index != 127 && level < thresholds_[index]) ++index;
        const unsigned code = index + 128;
        const float gain = std::ldexp(1.0F + static_cast<float>(code & 31) / 32, static_cast<int>(code >> 5) - 8);
        float current = gain_;
        const float increment = (gain - current) * (1.0F / 256);
        const float scale = static_cast<float>(1U << mode_);
        for (unsigned t = 0; t < 1536; ++t) {
            const float multiplier = .0625F / (t < 256 ? current : gain);
            for (unsigned c = 0; c < 6; ++c) {
                auto& sample = pcm[t * 6 + c];
                auto& delayed = delay_[(t % 256) * 6 + c];
                const float old = delayed;
                delayed = sample;
                sample = (multiplier * old) * scale;
            }
            if (t < 256) current += increment;
        }
        gain_ = gain;
        return static_cast<std::uint8_t>(code);
    }

private:
    static float logarithm(float value) {
        const float clamped = std::clamp(value, -1.0F, 1.0F);
        const std::uint32_t bits = std::bit_cast<std::uint32_t>(clamped) * 2U;
        int exponent = bits ? 126 - static_cast<int>(bits >> 24) : 127;
        exponent = std::clamp(exponent, 0, 60);
        if (exponent >= 48 || value == 0) return 1;
        const float mantissa = value * std::bit_cast<float>(static_cast<std::uint32_t>(exponent + 127) << 23);
        const float polynomial = (.013804385F - mantissa * .020795746F) + (mantissa * mantissa) * .0070254644F;
        return polynomial * 4 + static_cast<float>(exponent) * .020833334F;
    }

    unsigned mode_, hold_ = 0;
    float tail_peak_ = 0, envelope_ = 0, gain_ = .0625F;
    std::array<float, 256 * 6> delay_{};
    // Native table at RVA 0x688F30; retain its float rounding.
    static constexpr std::array<float, 128> thresholds_{
        -0.F,-0.00554926461F,-0.0109328553F,-0.0161603782F,-0.0212406255F,-0.0261816699F,-0.0309909396F,-0.0356752761F,
        -0.0402410142F,-0.0446940027F,-0.0490396768F,-0.0532830916F,-0.0574289523F,-0.0614816397F,-0.0654452443F,-0.0693236068F,
        -0.0731203109F,-0.0768387318F,-0.0804820284F,-0.0840531737F,-0.0875549689F,-0.0909900591F,-0.0943609402F,-0.0976699665F,
        -0.100919366F,-0.104111254F,-0.107247621F,-0.11033038F,-0.113361329F,-0.116342165F,-0.119274534F,-0.122159995F,
        -0.125F,-0.130549267F,-0.135932848F,-0.141160369F,-0.146240622F,-0.151181668F,-0.155990943F,-0.160675272F,
        -0.165241018F,-0.169693992F,-0.174039677F,-0.178283095F,-0.182428956F,-0.18648164F,-0.190445244F,-0.194323614F,
        -0.198120311F,-0.201838732F,-0.205482021F,-0.209053174F,-0.212554976F,-0.215990067F,-0.219360933F,-0.222669974F,
        -0.225919366F,-0.229111254F,-0.232247621F,-0.235330388F,-0.238361329F,-0.241342157F,-0.244274527F,-0.247159988F,
        -0.25F,-0.255549282F,-0.260932863F,-0.266160369F,-0.271240622F,-0.276181668F,-0.280990928F,-0.285675287F,
        -0.290241033F,-0.294694006F,-0.299039662F,-0.303283095F,-0.307428956F,-0.311481655F,-0.315445244F,-0.319323599F,
        -0.323120326F,-0.326838732F,-0.330482036F,-0.334053159F,-0.337554961F,-0.340990067F,-0.344360948F,-0.347669959F,
        -0.350919366F,-0.354111254F,-0.357247621F,-0.360330373F,-0.363361329F,-0.366342157F,-0.369274557F,-0.372159988F,
        -0.375F,-0.380549282F,-0.385932863F,-0.391160399F,-0.396240622F,-0.401181668F,-0.405990928F,-0.410675287F,
        -0.415241003F,-0.419693977F,-0.424039692F,-0.428283066F,-0.432428956F,-0.436481625F,-0.440445215F,-0.444323599F,
        -0.448120296F,-0.451838732F,-0.455482006F,-0.459053189F,-0.462554961F,-0.465990067F,-0.469360948F,-0.472669959F,
        -0.475919366F,-0.479111284F,-0.482247621F,-0.485330373F,-0.488361359F,-0.491342187F,-0.494274527F,-0.497159988F,
    };
};
} // namespace atmos::encode::joc

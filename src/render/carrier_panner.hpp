// Recovered 5.1 point/Cartesian-width gains; neutral mode preserves unit-power carriers.
#pragma once
#include "eac3/emdf.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace atmos::render::carrier_panner {
inline std::pair<float, float> power_pair(float fraction) {
    const float t = fraction <= 0.5F ? fraction : 1.0F - fraction;
    const float angle = ((t * 0.78539819F) * 0.5F) * 4.0F;
    const float c = std::cos(angle), s = std::sin(angle);
    return fraction <= 0.5F ? std::pair{c, s} : std::pair{s, c};
}

inline float distance_gain(float y, float z) {
    const float fy = y >= 0.6F ? 1.0F : y <= 0 ? 0 : float(double(y) / double(0.6F));
    const float fz = z >= 1 ? 1.0F : z <= 0.2F ? 0 : float(double(z - 0.2F) / double(0.8F));
    const float exponent = (std::clamp(fy + fz, 0.0F, 1.0F) * -3.0F) * 0.16609640419483185F;
    const float integral = std::floor(exponent), t = exponent - integral, squared = t * t;
    const float polynomial = (((t * 0.6862793F) + 1.0F) + squared * 0.25482178F) +
        (t * squared) * 0.058898926F;
    return std::ldexp(polynomial, static_cast<int>(integral));
}

inline constexpr std::array<unsigned,7> zone_masks{0x37,0x07,0x37,0x37,0x34,0x30,0x07};

inline std::array<float, 6> point_geometry(float x, float y, bool side_surrounds, unsigned zone = 0) {
    constexpr float end = 32767.0F / 32768.0F;
    x = std::min(x, end);
    std::array<float, 6> front{}, back{}, result{};
    if (x <= 0.5F) {
        const auto [l, c] = power_pair(x / 0.5F);
        front[0] = l; front[2] = c;
    } else {
        const auto [c, r] = power_pair((x - 0.5F) / (end - 0.5F));
        front[2] = c; front[1] = r;
    }
    if (zone == 4) { front = {}; front[2] = 1; }
    const auto [l, r] = power_pair(x / end);
    back[4] = l; back[5] = r;
    if (zone == 1 || zone == 6) return front;
    if (zone == 5) return back;
    const float room_y = side_surrounds ? std::min(2.0F * y, 1.0F) : y;
    const auto [f, b] = power_pair(std::min(room_y / end, 1.0F));
    for (unsigned i = 0; i < 6; ++i) result[i] = front[i] * f + back[i] * b;
    return result;
}

inline std::array<float, 6> point_gains(const std::array<int, 3>& position,
    float gain = 4096, bool attenuate = false, bool muted = false, bool side_surrounds = true, unsigned zone = 0, bool snap = false) {
    if (position[0] < 0 || position[0] > 32767 || position[1] < 0 || position[1] > 32767 ||
        position[2] < -32768 || position[2] > 32767 || !std::isfinite(gain) || gain < 0 || gain > 32767 || zone > 6)
        throw std::runtime_error("invalid carrier point");
    if (muted) return {};
    float x = position[0] * 0.000030517578125F, y = position[1] * 0.000030517578125F;
    float z = std::max(position[2] * 0.000030517578125F, 0.0F);
    auto result = point_geometry(x, y, side_surrounds, zone);
    if (snap) {
        unsigned best = 0; float maximum = 0;
        for (unsigned channel : {0U,1U,2U,4U,5U})
            if ((zone_masks[zone] & (1U << channel)) && result[channel] > maximum) {
                best = channel; maximum = result[channel];
            }
        constexpr float end = 32767.0F / 32768.0F;
        constexpr std::array<float,6> xs{0,end,.5F,0,0,end}, ys{0,0,0,0,end,end};
        const float scale_y = side_surrounds ? .5F : 1.0F;
        const float dx = x - xs[best], dy = std::min(y, scale_y) - scale_y * ys[best];
        if (z * z + (dx * dx + dy * dy) < .04F) {
            result = {}; result[best] = 1;
            x = xs[best]; y = ys[best]; z = 0;
        }
    }
    const float scale = (attenuate ? distance_gain(y, z) : 1) * (gain * 0.000030517578125F);
    for (unsigned i = 0; i < 6; ++i) result[i] = (scale * result[i]) * 8.0F;
    return result;
}

inline std::array<float, 6> normalize_width(std::array<float, 6> result, unsigned zone, float level) {
    const auto shift = [](float value) {
        const auto bits = std::bit_cast<std::uint32_t>(value);
        return std::clamp(bits * 2U ? 126 - static_cast<int>((bits * 2U) >> 24) : 127, 0, 60);
    };
    // Preserve native power-of-two accumulation and sqrt rounding.
    float energy = 0, scale = .5F;
    int started = 0, reductions = 0;
    for (unsigned channel : {0U, 1U, 2U, 4U, 5U}) {
        if (!(zone_masks[zone] & (1U << channel))) continue;
        const float square = result[channel] * result[channel];
        float term;
        if (shift(square) || started) term = std::ldexp(square, -started - reductions);
        else { started = 1; term = .5F * energy; energy = scale * square; }
        energy += term;
        if (!shift(energy)) { ++reductions; energy *= .5F; scale *= .5F; }
    }
    const int exponent = started + reductions;
    float norm = std::sqrt(energy);
    if (!exponent && norm < .00001F) norm = .00001F;
    const int norm_shift = shift(norm);
    norm = std::ldexp(norm, norm_shift);
    const int power = exponent - 2 * norm_shift;
    const int half = static_cast<int>(std::floor(power / 2.0));
    const bool odd = power - 2 * half != 0;
    for (auto& value : result) {
        if (odd) value *= .707106769F;
        value = std::ldexp(value, -half);
        value = value > norm ? 1.0F : value / norm;
        value = (level * value) * 8.0F;
    }
    return result;
}

// Native word7 Cartesian width; OAMD size/divergence mapping is separate.
inline std::array<float, 6> width_gains(const std::array<int, 3>& position, int width,
    float gain = 4096, bool attenuate = false, bool muted = false, bool side_surrounds = true, unsigned zone = 0, bool snap = false) {
    auto result = point_gains(position, gain, attenuate, muted, side_surrounds, zone, snap);
    if (width < 0 || width > 32767) throw std::runtime_error("invalid carrier width");
    if (!width || muted || snap) return result;
    const float x = position[0] * 0.000030517578125F, y = position[1] * 0.000030517578125F;
    const float z = std::max(position[2] * 0.000030517578125F, 0.0F);
    const float radius = std::min(width * 0.000030517578125F, x > .5F ? 1.0F - x : x);
    const auto left = point_geometry(std::max(x - radius, 0.0F), y, side_surrounds, zone);
    const auto right = point_geometry(std::min(x + radius, 1.0F), y, side_surrounds, zone);
    for (unsigned i = 0; i < 6; ++i) result[i] = right[i] + left[i];
    return normalize_width(result, zone, (attenuate ? distance_gain(y, z) : 1) * (gain * 0.000030517578125F));
}

inline std::array<float, 6> bed_gains(unsigned speaker, bool attenuate = false) {
    // Native neutral 5.1 profile; wide channels split between front and surround.
    constexpr std::array<unsigned, 17> channel{0,1,2,3,4,5,4,5,0,1,4,5,4,5,0,1,3};
    if (speaker >= channel.size()) throw std::runtime_error("invalid carrier bed speaker");
    std::array<float, 6> result{};
    result[channel[speaker]] = 1;
    if (speaker == 14 || speaker == 15) {
        result[channel[speaker]] = 0.8743466138839722F;
        result[channel[speaker] + 4] = 0.4853019714355469F;
    }
    if (attenuate) {
        constexpr std::array<float, 17> level{1,1,1,1,
            .750109851360321F,.750109851360321F,.7079470157623291F,.7079470157623291F,
            .7079563736915588F,.7079563736915588F,.7079470157623291F,.7079470157623291F,
            .7079470157623291F,.7079470157623291F,1,1,1};
        for (auto& gain : result) gain *= level[speaker];
        if (speaker == 14 || speaker == 15) {
            result[channel[speaker]] = .7972927689552307F;
            result[channel[speaker] + 4] = .44253361225128174F;
        }
    }
    return result;
}

inline void validate_trim(const eac3::OamdTrimConfig& trim) {
    eac3::OamdTrimConfig empty; empty.mode = trim.mode;
    if (trim.mode > 2 || (trim.mode != 2 && trim != empty) ||
        std::any_of(trim.levels.begin(), trim.levels.end(), [](auto v) { return v > 16; }) ||
        std::any_of(trim.balance.begin(), trim.balance.end(), [](int v) { return v < -16 || v > 16; }))
        throw std::runtime_error("invalid carrier trim");
}

inline float balanced_y(float y, float z, const std::array<std::int8_t, 2>& balance) {
    if (y <= .05F || y >= .95F || (balance[0] == 0 && balance[1] == 0)) return y;
    const auto coefficients = [](int value) {
        if (!value) return std::array<float,4>{1,0,1,.5F};
        const float small = (float(16 - std::abs(value)) * .0625F) * .8888889F + .11111111F;
        const float large = 2.0F - small, shift = .05F - small * .05F;
        return value < 0 ? std::array<float,4>{small,shift,large,.5F * small + shift} :
            std::array<float,4>{large,-shift,small,.5F * large - shift};
    };
    const auto listener = coefficients(balance[1]), overhead = coefficients(balance[0]);
    const auto mapped = [y](const auto& c) {
        return y < .5F ? y * c[0] + c[1] : c[2] * (y - .5F) + c[3];
    };
    return (1.0F-z) * mapped(listener) + mapped(overhead) * z;
}

inline float explicit_trim_level(float y, float z, const eac3::OamdTrimConfig& trim) {
    constexpr std::array<float,17> db{6,3,1.5F,.75F,-.75F,-1.5F,-3,-4.5F,-6,-7.5F,-9,-10.5F,-12,-13.5F,-15,-36,0};
    const float surround = std::min(db[trim.levels[1]], 0.0F), height = std::min(db[trim.levels[2]], 0.0F);
    // Canonical5.1 has a physical centre; its native panner ignores the centre trim.
    const float listener = y < .5F ? (y * surround) * 2.0F : surround;
    const float exponent = (listener * (1.0F-z) + z * height) * .16609640419483185F;
    const float integral = std::floor(exponent), t = exponent - integral, square = t * t;
    return std::ldexp(((t * .6862793F + 1.0F) + square * .25482178F) + (t * square) * .058898926F,
        static_cast<int>(integral));
}

inline std::array<float,6> trim_gains(const std::array<int,3>& position, int width,
    const eac3::OamdTrimConfig& trim, bool warping, float gain = 4096,
    bool muted = false, bool bypass = false, unsigned zone = 0, bool snap = false) {
    validate_trim(trim);
    if (trim.mode != 2 || bypass)
        return width_gains(position,width,gain,trim.mode == 0 && !bypass,muted,warping,zone,snap);
    if (position[0] < 0 || position[0] > 32767 || position[1] < 0 || position[1] > 32767 ||
        position[2] < -32768 || position[2] > 32767 || !std::isfinite(gain) || gain < 0 || gain > 32767 ||
        width < 0 || width > 32767 || zone > 6) throw std::runtime_error("invalid trimmed carrier point");
    if (muted) return {};
    float x = position[0] * .000030517578125F, z = std::max(position[2] * .000030517578125F, 0.0F);
    const float source_y = position[1] * .000030517578125F;
    float y = balanced_y(source_y,z,trim.balance);
    const float geometry_y = balanced_y(warping ? std::min(2.0F * source_y,1.0F) : source_y,z,trim.balance);
    auto result = point_geometry(x,geometry_y,false,zone);
    if (snap) {
        unsigned best = 0; float maximum = 0;
        for (unsigned channel : {0U,1U,2U,4U,5U})
            if ((zone_masks[zone] & (1U << channel)) && result[channel] > maximum) {
                best = channel; maximum = result[channel];
            }
        constexpr float end = 32767.0F / 32768.0F;
        constexpr std::array<float,6> xs{0,end,.5F,0,0,end}, ys{0,0,0,0,end,end};
        const float scale_y = warping ? .5F : 1.0F;
        const float dx = x-xs[best], dy = std::min(y,scale_y) - scale_y * ys[best];
        if (z*z + (dx*dx + dy*dy) < .04F) {
            result = {}; result[best] = 1;
            x = xs[best]; y = ys[best]; z = 0;
        }
    }
    const float level = explicit_trim_level(y,z,trim) * (gain * .000030517578125F);
    if (width && !snap) {
        const float radius = std::min(width * .000030517578125F, x > .5F ? 1.0F-x : x);
        const auto left = point_geometry(std::max(x-radius,0.0F),geometry_y,false,zone);
        const auto right = point_geometry(std::min(x+radius,1.0F),geometry_y,false,zone);
        for (unsigned i=0;i<6;++i) result[i] = right[i] + left[i];
        return normalize_width(result,zone,level);
    }
    for (auto& value : result) value = (level * value) * 8.0F;
    return result;
}

inline std::array<float,6> trim_bed_gains(unsigned speaker, const eac3::OamdTrimConfig& trim,
    bool warping, float gain = 4096, bool bypass = false) {
    validate_trim(trim);
    if (speaker > 16 || !std::isfinite(gain) || gain < 0 || gain > 32767) throw std::runtime_error("invalid trimmed bed speaker");
    constexpr std::array<std::array<int,3>,17> positions{{
        {0,0,0},{32767,0,0},{16384,0,0},{0,0,0}, {0,16384,0},{32767,16384,0},
        {0,32767,0},{32767,32767,0},{7928,0,32767},{24840,0,32767},
        {7928,16384,32767},{24840,16384,32767},{7928,32767,32767},{24840,32767,32767},
        {0,5285,0},{32767,5285,0},{0,0,0}}};
    if (speaker == 14 || speaker == 15) {
        // Bed fallback uses the source position but does not apply object balances.
        auto bed_trim = trim; bed_trim.balance = {};
        return trim_gains(positions[speaker],0,bed_trim,warping,gain,false,bypass);
    }
    auto result = bed_gains(speaker,false);
    if (!warping && (speaker == 10 || speaker == 11))
        result = point_geometry(speaker == 10 ? 0.0F : 32767.0F/32768, .5F, false);
    float level = 1;
    if (!bypass && speaker != 3 && speaker != 16) {
        const float y = positions[speaker][1] * .000030517578125F, z = positions[speaker][2] * .000030517578125F;
        if (trim.mode == 0) level = distance_gain(y,z);
        if (trim.mode == 2) level = explicit_trim_level(y,z,trim);
    }
    level *= gain * .000244140625F;
    for (auto& value : result) value *= level;
    return result;
}

struct GainRamp final {
    std::array<float, 6> current{}, increment{};
    std::array<unsigned, 6> remaining{};

    std::array<std::array<float, 6>, 32> block(const std::array<float, 6>& target,
        bool changed, unsigned duration) {
        std::array<std::array<float, 6>, 32> output{};
        for (unsigned channel = 0; channel < 6; ++channel) {
            if (std::max(std::abs(target[channel]), std::abs(current[channel])) < 0.000001F) continue;
            const float difference = target[channel] - current[channel];
            if (std::abs(difference) < 0.0001F) {
                remaining[channel] = 0;
                current[channel] = target[channel];
            } else if (changed) {
                remaining[channel] = duration;
                if (duration) increment[channel] = difference * static_cast<float>(1.0 / duration);
            }
            if (remaining[channel]) {
                for (unsigned sample = 0; sample < 32; ++sample)
                    output[sample][channel] = (static_cast<float>(sample + 1) / 32 * increment[channel]) + current[channel];
                current[channel] = increment[channel] + current[channel];
                --remaining[channel];
            } else {
                for (auto& sample : output) sample[channel] = target[channel];
                current[channel] = target[channel];
            }
        }
        return output;
    }
};
} // namespace atmos::render::carrier_panner

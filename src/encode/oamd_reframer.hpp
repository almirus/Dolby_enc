#pragma once

#include "eac3/emdf.hpp"

#include <map>
#include <functional>
#include <optional>

namespace atmos::encode {

struct TimedOamdPayload final {
    std::uint64_t sample = 0;
    eac3::OamdPayload payload;
};

// Assign one packet; false is EOF, read failures throw.
using OamdReader = std::function<bool(TimedOamdPayload&)>;
using OamdReaderFactory = std::function<OamdReader()>;

// Emits one frame at a time, including codec flush. Span input remains borrowed;
// callback packets and pending trim profiles are owned by the reframer.
class OamdReframer final {
public:
    OamdReframer(std::span<const TimedOamdPayload> input, std::size_t samples, unsigned delay = 0);
    OamdReframer(OamdReader read, std::size_t samples, unsigned delay = 0);
    // False clears output at EOF; malformed or unrepresentable metadata throws.
    bool next(std::vector<std::uint8_t>& output);
private:
    struct Frame {
        std::vector<eac3::OamdElement> elements;
        std::optional<eac3::OamdElement> trim;
        bool profile_seen = false;
    };
    OamdReader read_;
    TimedOamdPayload packet_;
    eac3::OamdPayload header_;
    bool have_packet_ = false;
    std::size_t frames_, frame_ = 0;
    unsigned delay_;
    std::uint64_t previous_ = 0;
    std::vector<std::uint8_t> last_validated_trim_;
    std::map<std::size_t, Frame> pending_;
};

// One OAMD payload per output frame,including codec flush;delay follows the carrier PCM.
// Cross-frame elements resolve absolute state and relocate paired type-5 extensions.
// Compatible fragments merge into one object element; mixed timing residues and >8 blocks are rejected.
// shortcut: conflicting trims within one frame are rejected; upgrade after validating multi-profile OAMD transport.
[[nodiscard]] std::vector<std::vector<std::uint8_t>> reframe_oamd(
    std::span<const TimedOamdPayload> input, std::size_t samples, unsigned delay = 0);

// Minimum common PCM/metadata padding that gives an exact supported frame grouping.
[[nodiscard]] unsigned oamd_alignment_padding(std::span<const TimedOamdPayload> input,
    std::size_t samples, unsigned delay);

// Each candidate gets a fresh sequential source; the factory must replay identical input.
[[nodiscard]] unsigned oamd_alignment_padding(const OamdReaderFactory& input,
    std::size_t samples, unsigned delay);

} // namespace atmos::encode

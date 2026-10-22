#include "app/version.hpp"

#include <iostream>

using namespace std::chrono;
constexpr sys_seconds built=sys_days{year{2026}/October/21};
static_assert(!atmos::app::expired("0.0.1 alfa",built,built));
static_assert(!atmos::app::expired("0.0.1 alfa",built,built+hours{360}-seconds{1}));
static_assert(atmos::app::expired("0.0.1 alfa",built,built+hours{360}));
static_assert(atmos::app::expired("0.0.1 alfa",built,built+days{30}));
static_assert(!atmos::app::expired("0.0.1",built,built+days{3650}));
static_assert(!atmos::app::expired("0.0.1 beta",built,built+days{3650}));
static_assert(!atmos::app::expired("0.0.1 alfa",built+days{14},built+days{15}));

int main() {
    struct Case { const char* date; int year, month, day; };
    for (const auto& test:{Case{"Jan  3 2026",126,0,3},Case{"Oct 21 2026",126,9,21},Case{"Feb 29 2024",124,1,29}}) {
        const auto timestamp=system_clock::to_time_t(atmos::app::build_time(test.date,"12:34:56"));
        const auto* calendar=std::localtime(&timestamp);
        if (!calendar || calendar->tm_year!=test.year || calendar->tm_mon!=test.month || calendar->tm_mday!=test.day ||
            calendar->tm_hour!=12 || calendar->tm_min!=34 || calendar->tm_sec!=56) return 1;
    }
    std::cout << "PASS: alfa expiry boundary, release exemption, rebuild renewal, build timestamp\n";
}

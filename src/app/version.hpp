#pragma once

#include <chrono>
#include <ctime>
#include <string_view>

namespace atmos::app {

inline constexpr std::string_view version="0.0.1 alfa";
inline constexpr auto alfa_lifetime=std::chrono::hours{15*24};

constexpr bool expired(std::string_view label, std::chrono::sys_seconds built,
    std::chrono::sys_seconds now) {
    return label.find("alfa")!=std::string_view::npos && now>=built+alfa_lifetime;
}

// Compiler date/time are in the build machine's local timezone.
inline std::chrono::sys_seconds build_time(std::string_view date, std::string_view time) {
    constexpr std::string_view months="JanFebMarAprMayJunJulAugSepOctNovDec";
    std::tm calendar{};
    calendar.tm_year=(date[7]-'0')*1000+(date[8]-'0')*100+(date[9]-'0')*10+date[10]-'0'-1900;
    calendar.tm_mon=static_cast<int>(months.find(date.substr(0,3))/3);
    calendar.tm_mday=(date[4]==' ' ? 0 : date[4]-'0')*10+date[5]-'0';
    calendar.tm_hour=(time[0]-'0')*10+time[1]-'0';
    calendar.tm_min=(time[3]-'0')*10+time[4]-'0';
    calendar.tm_sec=(time[6]-'0')*10+time[7]-'0';
    calendar.tm_isdst=-1;
    return std::chrono::time_point_cast<std::chrono::seconds>(
        std::chrono::system_clock::from_time_t(std::mktime(&calendar)));
}

} // namespace atmos::app

#pragma once

#include "transcoder.hpp"

namespace atmos::encode {
int encode_truehd_file(const Options& options, std::string& error);
}

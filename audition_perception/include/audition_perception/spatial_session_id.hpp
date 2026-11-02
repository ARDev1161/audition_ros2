#pragma once
#include <cstdint>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>

namespace audition_perception {
// Per-spatial-process incarnation discriminator; NOT an authentication token.
inline std::string makeSpatialSessionId() {
  std::random_device random;
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) {
    out << std::setw(8) << static_cast<std::uint32_t>(random());
  }
  return out.str();
}
}  // namespace audition_perception

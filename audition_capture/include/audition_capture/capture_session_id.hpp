#pragma once

#include <cstdint>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>

namespace audition_capture {

// Per-capture-process incarnation discriminator, not a secret or authentication.
// Keep the same ID across ALSA XRUN recovery and device reconnect.
inline std::string makeCaptureSessionId() {
  std::random_device random;
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) {
    out << std::setw(8) << static_cast<std::uint32_t>(random());
  }
  return out.str();
}

}  // namespace audition_capture

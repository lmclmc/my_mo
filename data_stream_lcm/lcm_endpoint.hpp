#pragma once

#include <charconv>
#include <optional>
#include <string>
#include <string_view>

namespace lcm_stream {

inline constexpr int kDefaultLcmPort = 7667;

inline std::string lcmUrl(int port) {
  return "udpm://239.255.76.67:" + std::to_string(port);
}

inline std::optional<int> parseLcmPort(std::string_view text) {
  if (text.empty()) {
    return kDefaultLcmPort;
  }
  int port = 0;
  const auto end = text.data() + text.size();
  const auto [next, error] = std::from_chars(text.data(), end, port);
  if (error != std::errc{} || next != end || port < 1 || port > 65535) {
    return std::nullopt;
  }
  return port;
}

}  // namespace lcm_stream

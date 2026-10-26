#pragma once
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

namespace plan_env { namespace trace {
inline long long steadyNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
// Opt in per process. Buffered JSONL avoids synchronous per-event disk flush.
// Only use this when collecting measurements; tracing adds measurement overhead.
inline std::ofstream& stream() {
  static std::ofstream out = [] {
    std::ofstream f;
    const char* dir = std::getenv("M20_PIPELINE_TRACE_DIR");
    if (dir && *dir) {
      try {
        std::filesystem::create_directories(dir);
        f.open(std::string(dir) + "/process_" + std::to_string(getpid()) + ".jsonl");
      } catch (...) { /* A diagnostic path must not terminate navigation. */ }
    }
    return f;
  }();
  return out;
}
inline bool enabled() { return stream().is_open(); }
inline void event(const char* name, const std::string& fields) {
  auto& out = stream();
  if (!out.is_open()) return;
  const auto ros_independent_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  out << "{\"event\":\"" << name << "\",\"pid\":" << getpid()
      << ",\"steady_ns\":" << steadyNs() << ",\"system_ns\":" << ros_independent_wall_ns
      << fields << "}\n";
}
} }  // namespace plan_env::trace

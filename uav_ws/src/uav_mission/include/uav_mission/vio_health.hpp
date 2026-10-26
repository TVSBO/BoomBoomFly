#pragma once

#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <system_error>
#include <unordered_map>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace uav_mission
{

struct VioHealthSample
{
  bool timestamp_accepted{false};
  bool healthy{false};
  bool enabled{false};
  bool faulted{false};
  bool paused{false};
  bool visual_fresh{false};
  bool input_fresh{false};
  int64_t stamp_ns{0};
  double visual_age_s{-1.0};
  double input_age_s{-1.0};
  uint64_t received{0};
  uint64_t adapted{0};
  uint64_t published{0};
  uint64_t rejected{0};
  uint64_t rate_limited{0};
  uint64_t recovery_visual_count{0};
  uint64_t recovery_odom_count{0};
  std::string backend;
  std::string pause_reason;
  std::string last_rejection;
  std::string reason;
};

inline bool parse_vio_bool(const std::string & text, bool & value)
{
  if (text == "true") {
    value = true;
    return true;
  }
  if (text == "false") {
    value = false;
    return true;
  }
  return false;
}

inline bool parse_vio_counter(const std::string & text, uint64_t & value)
{
  if (text.empty()) {
    return false;
  }
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

inline bool parse_vio_age(const std::string & text, double & value)
{
  char * end = nullptr;
  value = std::strtod(text.c_str(), &end);
  return end != text.c_str() && end == text.c_str() + text.size() && std::isfinite(value);
}

inline bool parse_vio_duration(const std::string & text, double & value)
{
  return parse_vio_age(text, value) && value >= 0.0;
}

inline bool valid_vio_sample_age(double age_s, bool fresh)
{
  return !fresh || age_s >= -0.01;
}

inline VioHealthSample evaluate_vio_diagnostics(
  const diagnostic_msgs::msg::DiagnosticArray & message,
  int64_t now_ns, int64_t previous_stamp_ns, double timeout_s)
{
  VioHealthSample result;
  result.stamp_ns = previous_stamp_ns;
  const auto & stamp = message.header.stamp;
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
    (stamp.sec == 0 && stamp.nanosec == 0))
  {
    result.reason = "invalid bridge diagnostic timestamp";
    return result;
  }
  const int64_t stamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  const double age_s = static_cast<double>(now_ns - stamp_ns) / 1e9;
  if (stamp_ns <= previous_stamp_ns) {
    result.reason = "bridge diagnostic timestamp is repeated or out of order";
    return result;
  }
  if (age_s < -0.1 || age_s > timeout_s) {
    result.reason = "bridge diagnostic timestamp is future-dated or stale";
    return result;
  }
  result.timestamp_accepted = true;
  result.stamp_ns = stamp_ns;

  const diagnostic_msgs::msg::DiagnosticStatus * bridge = nullptr;
  size_t matching_statuses = 0;
  for (const auto & status : message.status) {
    if (status.name == "uav_bridge/odom_bridge") {
      bridge = &status;
      ++matching_statuses;
    }
  }
  if (matching_statuses != 1 || bridge == nullptr) {
    result.reason = "expected exactly one uav_bridge/odom_bridge diagnostic";
    return result;
  }

  std::unordered_map<std::string, std::string> values;
  for (const auto & item : bridge->values) {
    if (!values.emplace(item.key, item.value).second) {
      result.reason = "bridge diagnostic contains a duplicate key: " + item.key;
      return result;
    }
  }
  const auto get = [&values](const char * key) -> const std::string * {
      const auto found = values.find(key);
      return found == values.end() ? nullptr : &found->second;
    };
  const auto required = {
    "contract_version", "backend", "enabled", "faulted", "paused", "pause_reason",
    "visual_fresh", "visual_age_s", "visual_stamp_ns", "input_fresh", "input_age_s",
    "received", "adapted", "published", "rejected", "rate_limited",
    "recovery_visual_count", "recovery_odom_count", "last_process_result",
    "last_rejection", "last_process_ms", "max_process_ms"};
  for (const auto * key : required) {
    if (get(key) == nullptr) {
      result.reason = "bridge diagnostic contract is missing key: " + std::string(key);
      return result;
    }
  }
  if (*get("contract_version") != "1" ||
    (*get("backend") != "dds" && *get("backend") != "mavros"))
  {
    result.reason = "invalid uav_bridge diagnostic v1 contract";
    return result;
  }

  bool visual_stamp_present = false;
  uint64_t visual_stamp_ns = 0;
  double last_process_ms = 0.0;
  double max_process_ms = 0.0;
  if (!parse_vio_bool(*get("enabled"), result.enabled) ||
    !parse_vio_bool(*get("faulted"), result.faulted) ||
    !parse_vio_bool(*get("paused"), result.paused) ||
    !parse_vio_bool(*get("visual_fresh"), result.visual_fresh) ||
    !parse_vio_bool(*get("input_fresh"), result.input_fresh) ||
    !parse_vio_age(*get("visual_age_s"), result.visual_age_s) ||
    !parse_vio_age(*get("input_age_s"), result.input_age_s) ||
    !parse_vio_counter(*get("visual_stamp_ns"), visual_stamp_ns) ||
    !parse_vio_counter(*get("received"), result.received) ||
    !parse_vio_counter(*get("adapted"), result.adapted) ||
    !parse_vio_counter(*get("published"), result.published) ||
    !parse_vio_counter(*get("rejected"), result.rejected) ||
    !parse_vio_counter(*get("rate_limited"), result.rate_limited) ||
    !parse_vio_counter(*get("recovery_visual_count"), result.recovery_visual_count) ||
    !parse_vio_counter(*get("recovery_odom_count"), result.recovery_odom_count) ||
    !parse_vio_duration(*get("last_process_ms"), last_process_ms) ||
    !parse_vio_duration(*get("max_process_ms"), max_process_ms))
  {
    result.reason = "bridge diagnostic contract contains malformed values";
    return result;
  }
  visual_stamp_present = visual_stamp_ns > 0;
  result.backend = *get("backend");
  result.pause_reason = *get("pause_reason");
  result.last_rejection = *get("last_rejection");

  if (result.faulted != result.paused ||
    (result.visual_fresh && !visual_stamp_present) ||
    !valid_vio_sample_age(result.visual_age_s, result.visual_fresh) ||
    !valid_vio_sample_age(result.input_age_s, result.input_fresh))
  {
    result.reason = "bridge diagnostic freshness or pause fields are inconsistent";
    return result;
  }
  if (!result.enabled) {
    result.reason = "bridge output is disabled";
  } else if (result.faulted || result.paused) {
    result.reason = result.pause_reason.empty() ? "bridge is paused" :
      "bridge is paused: " + result.pause_reason;
  } else if (!result.visual_fresh) {
    result.reason = "bridge has no fresh accepted visual update";
  } else if (!result.input_fresh) {
    result.reason = "bridge odometry input is stale";
  } else if (bridge->level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
    result.reason = bridge->message.empty() ? "bridge diagnostic is not OK" : bridge->message;
  } else {
    result.healthy = true;
    result.reason = "bridge healthy (backend=" + result.backend + ")";
  }
  return result;
}

}  // namespace uav_mission

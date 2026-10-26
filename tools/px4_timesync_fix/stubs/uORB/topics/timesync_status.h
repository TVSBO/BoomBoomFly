#pragma once
#include <cstdint>
struct timesync_status_s {
    static constexpr uint8_t SOURCE_PROTOCOL_UNKNOWN = 0;
    uint8_t source_protocol{};
    uint64_t timestamp{}, remote_timestamp{}, round_trip_time{};
    int64_t observed_offset{}, estimated_offset{};
};

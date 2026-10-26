#pragma once
#include <cstdint>
#include <ctime>
#define PX4_DEBUG(...) ((void)0)
namespace time_literals {
constexpr uint64_t operator""_ms(unsigned long long x) { return x * 1000; }
}
inline uint64_t hrt_absolute_time() { return 1; }

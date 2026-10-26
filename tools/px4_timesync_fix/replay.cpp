// Runs the unchanged upstream Timesync implementation; only cadence differs.
#include "Timesync.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
    if (argc < 4) return 2;
    const bool fixed = std::string(argv[1]) == "fixed";
    const double drift = std::atof(argv[2]); // us / second
    const double jitter = std::atof(argv[3]); // us, bounded deterministic jitter
    const bool schedule_jitter = argc > 5;
    std::vector<double> noise;
    if (argc > 4) {
        std::ifstream input(argv[4]);
        double value;
        while (input >> value) noise.push_back(value);
        if (noise.empty()) return 3;
    }
    Timesync filter;
    uint64_t now = 1000000;
    unsigned resets = 0, samples = 0;
    bool previously_converged = false;
    double peak = 0, peak_hold = 0;
    std::vector<double> errors;
    while (now < 651000000) {
        uint64_t step = fixed ? 100000 : (filter.sync_converged() ? 1000000 : 13000);
        if (schedule_jitter && filter.sync_converged()) {
            // Poll scheduling latency and an occasional missed synchronization.
            step += (samples * 37 % 11) * 1000;
            if (samples % 500 == 0) step += 100000;
        }
        now += step;
        const double truth = -1792578057000000.0 + drift * (now - 1000000) / 1e6;
        // A reproducible sample sequence; optional noise comes from captured DDS offsets.
        const double error = noise.empty() ? jitter * std::sin(samples * 1.73) : noise[samples % noise.size()];
        const int64_t observed = std::llround(truth + error);
        const uint64_t rtt = 3000;
        const int64_t remote = static_cast<int64_t>(now - rtt / 2) - observed;
        filter.update(now, remote * 1000, (now - rtt) * 1000);
        const bool converged = filter.sync_converged();
        if (previously_converged && !converged) ++resets;
        if (converged) {
            const double residual = truth - filter.offset();
            errors.push_back(std::abs(residual));
            peak = std::max(peak, std::abs(residual));
            // Outgoing stamps use a held offset until the next synchronization.
            peak_hold = std::max(peak_hold, std::abs(residual + drift * step / 1e6));
        }
        previously_converged = converged;
        ++samples;
    }
    std::sort(errors.begin(), errors.end());
    if (errors.empty()) return 4;
    std::cout << "{\"fixed\":" << (fixed ? "true" : "false")
              << ",\"drift_us_per_s\":" << drift << ",\"jitter_us\":" << jitter
              << ",\"samples\":" << samples << ",\"resets\":" << resets
              << ",\"schedule_jitter\":" << (schedule_jitter ? "true" : "false")
              << ",\"max_error_us\":" << peak
              << ",\"p95_error_us\":" << errors[errors.size()*95/100]
              << ",\"max_hold_error_us\":" << peak_hold << "}\n";
    return fixed && (resets || peak_hold > 10000) ? 1 : 0;
}

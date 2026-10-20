// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
inline bool valid_external_arm_target(const std::vector<double>& q,
                                      const std::vector<double>& limits) {
    if (q.size() != 10 || limits.size() != 46) return false;
    for (size_t i = 0; i < 10; ++i) {
        const size_t j = 13 + i;
        if (!std::isfinite(q[i]) || q[i] < limits[2*j] || q[i] > limits[2*j+1]) return false;
    }
    return true;
}
inline float limit_external_arm_step(float previous, float target, float max_delta) {
    return previous + std::clamp(target - previous, -max_delta, max_delta);
}

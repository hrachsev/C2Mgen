#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>

struct JobProgress {
  std::function<void(float fraction, const char* label)> report;

  bool active() const { return static_cast<bool>(report); }

  void set(float fraction, const char* label = nullptr) const {
    if (report) report(std::clamp(fraction, 0.f, 1.f), label);
  }

  void setRange(float base, float span, float local, const char* label) const {
    set(base + span * std::clamp(local, 0.f, 1.f), label);
  }

  JobProgress segment(float base, float span) const {
    JobProgress child;
    if (!report) return child;
    const auto parent = report;
    child.report = [parent, base, span](float local, const char* label) {
      parent(base + span * std::clamp(local, 0.f, 1.f), label);
    };
    return child;
  }

  void setIndexed(size_t index, size_t total, float base, float span,
                  const char* label, size_t throttle = 64) const {
    if (!active() || total == 0) return;
    if (index % throttle != 0 && index + 1 != total) return;
    setRange(base, span, static_cast<float>(index + 1) /
                              static_cast<float>(total),
             label);
  }
};

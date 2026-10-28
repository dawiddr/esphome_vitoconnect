#pragma once

#include <cmath>
#include <cstdint>

namespace vitoconnect_site {

// Site-only estimate. This class has no access to boiler controls or transport.
class GasEstimator {
 public:
  void set_power(float percent, uint32_t now) {
    power_ = percent;
    power_at_ = now;
    power_received_ = true;
  }

  void set_activity(float raw_status, uint32_t now) {
    activity_ = raw_status;
    activity_at_ = now;
    activity_received_ = true;
  }

  static bool calibration_set(float full_power_flow) {
    return std::isfinite(full_power_flow) && full_power_flow > 0.0f && full_power_flow <= 10.0f;
  }

  void tick(uint32_t now, float full_power_flow, double &equivalent_hours, double &gas_volume) {
    // Latch expiry so a timestamp cannot become fresh again after clock rollover.
    if (static_cast<uint32_t>(now - power_at_) > MAX_INPUT_AGE_MS) power_received_ = false;
    if (static_cast<uint32_t>(now - activity_at_) > MAX_INPUT_AGE_MS) activity_received_ = false;
    inputs_valid_ = power_received_ && activity_received_ &&
                    std::isfinite(power_) && power_ >= 0.0f && power_ <= 100.0f &&
                    std::isfinite(activity_) && activity_ >= 0.0f && activity_ <= 255.0f;
    const float load = inputs_valid_ && activity_ != 0.0f ? power_ / 100.0f : 0.0f;
    const uint32_t elapsed = now - last_tick_;

    // Never fill a boot gap, an input outage, or a long scheduler pause.
    if (inputs_valid_ && previous_valid_ && elapsed <= MAX_TICK_GAP_MS) {
      const double hours = static_cast<double>(elapsed) / 3600000.0;
      const double increment = hours * (static_cast<double>(previous_load_) + load) / 2.0;
      equivalent_hours += increment;
      // A new calibration applies only to subsequent intervals, not old totals.
      if (calibration_set(full_power_flow) && full_power_flow == previous_calibration_) {
        gas_volume += increment * full_power_flow;
      }
    }

    last_tick_ = now;
    previous_valid_ = inputs_valid_;
    previous_load_ = load;
    previous_calibration_ = full_power_flow;
    flow_ = inputs_valid_ && calibration_set(full_power_flow) ? load * full_power_flow : NAN;
  }

  bool inputs_valid() const { return inputs_valid_; }
  float flow() const { return flow_; }

 private:
  // The site polls every 60 seconds and evaluates this estimate every 5 seconds.
  static constexpr uint32_t MAX_INPUT_AGE_MS = 90000;
  static constexpr uint32_t MAX_TICK_GAP_MS = 10000;
  float power_{NAN};
  float activity_{NAN};
  uint32_t power_at_{0};
  uint32_t activity_at_{0};
  uint32_t last_tick_{0};
  bool power_received_{false};
  bool activity_received_{false};
  bool inputs_valid_{false};
  bool previous_valid_{false};
  float previous_load_{0.0f};
  float previous_calibration_{0.0f};
  float flow_{NAN};
};

inline GasEstimator gas_estimator;

}  // namespace vitoconnect_site

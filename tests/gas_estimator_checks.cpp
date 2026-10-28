// Run: g++ -std=c++17 tests/gas_estimator_checks.cpp -o /tmp/vito-gas-checks && /tmp/vito-gas-checks
#include "../gas_estimator.h"

#include <cassert>
#include <iostream>
#include <limits>

using vitoconnect_site::GasEstimator;

void near(double actual, double expected) { assert(std::fabs(actual - expected) < 1e-8); }

int main() {
  {
    GasEstimator estimate;
    double hours = 0, volume = 0;
    estimate.tick(0, 2, hours, volume);
    assert(!estimate.inputs_valid() && std::isnan(estimate.flow()));
    estimate.set_power(50, 0);
    estimate.set_activity(1, 0);
    estimate.tick(0, 2, hours, volume);
    for (uint32_t now = 5000; now <= 3600000; now += 5000) {
      if (now % 60000 == 0) {
        // Repeated identical wire values must keep the estimate valid.
        estimate.set_power(50, now);
        estimate.set_activity(1, now);
      }
      estimate.tick(now, 2, hours, volume);
    }
    near(hours, 0.5);
    near(volume, 1.0);
    near(estimate.flow(), 1.0);
    estimate.set_activity(0, 3600000);
    estimate.tick(3600000, 2, hours, volume);
    estimate.tick(3605000, 2, hours, volume);
    near(hours, 0.5);
    near(volume, 1.0);
    near(estimate.flow(), 0);
    std::cout << "PASS: known volume, unchanged reads, tick zero and burner off\n";
  }
  {
    GasEstimator estimate;
    double hours = 0, volume = 0;
    estimate.set_power(100, 0);
    estimate.set_activity(1, 0);
    estimate.tick(0, 0, hours, volume);
    estimate.tick(5000, 0, hours, volume);
    near(hours, 5.0 / 3600);
    near(volume, 0);
    assert(std::isnan(estimate.flow()));
    estimate.tick(10000, 2, hours, volume);
    near(volume, 0);  // Calibration must not fill an uncalibrated interval.
    estimate.tick(15000, 2, hours, volume);
    near(volume, 10.0 / 3600);
    estimate.tick(20000, 3, hours, volume);
    near(volume, 10.0 / 3600);  // Changing calibration must not rescale history.
    estimate.tick(25000, 3, hours, volume);
    near(volume, 25.0 / 3600);
    estimate.tick(30000, 0, hours, volume);
    near(volume, 25.0 / 3600);
    near(hours, 30.0 / 3600);
    std::cout << "PASS: meter calibration counter, calibration changes and disabling\n";
  }
  {
    GasEstimator estimate;
    double hours = 1, volume = 2;  // Restored persisted totals.
    estimate.set_power(100, 0);
    estimate.set_activity(1, 0);
    estimate.tick(0, 2, hours, volume);
    estimate.tick(20000, 2, hours, volume);
    near(hours, 1);
    near(volume, 2);  // A long event-loop pause cannot fill the gap.
    estimate.tick(25000, 2, hours, volume);
    const double before_outage = volume;
    estimate.tick(95000, 2, hours, volume);
    assert(!estimate.inputs_valid());
    estimate.tick(100000, 2, hours, volume);
    near(volume, before_outage);
    estimate.set_power(100, 100000);  // One recovered input is insufficient.
    estimate.tick(100000, 2, hours, volume);
    assert(!estimate.inputs_valid());
    estimate.set_activity(1, 100000);
    estimate.tick(100000, 2, hours, volume);
    near(volume, before_outage);
    estimate.tick(105000, 2, hours, volume);
    near(volume, before_outage + 10.0 / 3600);
    std::cout << "PASS: persistence, long pauses, stale inputs and recovery without backfill\n";
  }
  {
    for (float power : {NAN, INFINITY, -1.0f, 101.0f}) {
      GasEstimator estimate;
      double hours = 0, volume = 0;
      estimate.set_power(power, 0);
      estimate.set_activity(1, 0);
      estimate.tick(0, 2, hours, volume);
      estimate.tick(5000, 2, hours, volume);
      assert(!estimate.inputs_valid() && std::isnan(estimate.flow()));
      near(hours, 0);
      near(volume, 0);
    }
    for (float calibration : {NAN, INFINITY, -1.0f, 11.0f}) {
      assert(!GasEstimator::calibration_set(calibration));
    }
    GasEstimator estimate;
    double hours = 0, volume = 0;
    const uint32_t start = std::numeric_limits<uint32_t>::max() - 2000;
    estimate.set_power(100, start);
    estimate.set_activity(1, start);
    estimate.tick(start, 2, hours, volume);
    estimate.tick(start + 5000, 2, hours, volume);
    near(volume, 10.0 / 3600);
    estimate.tick(start + 95000, 2, hours, volume);
    estimate.tick(start, 2, hours, volume);  // Ancient input cannot revive on wrap.
    assert(!estimate.inputs_valid());
    near(volume, 10.0 / 3600);
    std::cout << "PASS: invalid inputs, invalid calibration and clock rollover\n";
  }
}

#pragma once

#include "esphome/components/number/number.h"
#include "../vitoconnect_datapoint.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace vitoconnect {

class OPTOLINKNumber : public number::Number, public Datapoint {

  public:
    OPTOLINKNumber();
    ~OPTOLINKNumber();

    virtual void control(float value) override;

    // For an uncommissioned stopped controller only. Drop unsent intent/retries,
    // not an already queued/transmitted request. hasPendingCommand() stays true
    // until that request completes. Only post-fence reads can renew wire truth.
    void cancelPendingCommand() {
      this->_has_command_value = false;
      this->clearLastUpdate();
      this->clearVerifyPending();
      this->resetWriteFailCount();
      this->invalidateReadRequests();
      this->_read_expired = true;
    }

    void decode(uint8_t* data, uint8_t length, Datapoint* dp = nullptr) override;
    void encode(uint8_t* raw, uint8_t length, void* data) override;
    void encode(uint8_t* raw, uint8_t length, float data);
    void encode(uint8_t* raw, uint8_t length) override;

    void setDivRatio(float div) { this->_div_ratio = div; }
    void setSigned(bool is_signed) { this->_signed = is_signed; }
    uint32_t getLastReadMs() const { return this->_last_read_ms; }
    uint32_t getReadSequence() const { return this->_read_sequence; }
    bool readIsFresh(uint32_t max_age_ms) const {
      return this->_read_sequence != 0 && !this->_read_expired &&
             static_cast<uint32_t>(millis() - this->_last_read_ms) <= max_age_ms;
    }
    // Latch expiry until a successful read. Prevent an ancient timestamp from
    // looking fresh again after the 32-bit millisecond clock wraps.
    void expireRead(uint32_t max_age_ms) {
      if (!this->readIsFresh(max_age_ms)) this->_read_expired = true;
    }
    bool hasPendingCommand() const {
      return this->_has_command_value || this->_last_update != 0 ||
             this->_write_in_flight || this->_verify_pending;
    }
    bool hasActiveWriteCommand() const {
      return this->_write_in_flight || this->_verify_pending;
    }
    float getPendingCommandValue() const { return this->_command_value; }

  private:
    float _div_ratio = 1.0f;
    bool _signed = false;
    float _command_value = 0.0f;
    bool _has_command_value = false;
    uint32_t _last_read_ms = 0;
    uint32_t _read_sequence = 0;
    uint32_t _command_sequence = 0;
    bool _read_expired = true;
};

}  // namespace vitoconnect
}  // namespace esphome

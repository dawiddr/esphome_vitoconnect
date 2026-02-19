#include "vitoconnect_binary_sensor.h"

namespace esphome {
namespace vitoconnect {

static const char *TAG = "vitoconnect.binary_sensor";

OPTOLINKBinarySensor::OPTOLINKBinarySensor(){
  // empty
}

OPTOLINKBinarySensor::~OPTOLINKBinarySensor() {
  // empty
}

void OPTOLINKBinarySensor::decode(uint8_t* data, uint8_t length, Datapoint* dp) {
  if (length < _length) {
    ESP_LOGW(TAG, "decode length mismatch for %s: got=%u expected=%u",
             this->get_name().c_str(), (unsigned) length, (unsigned) _length);
    return;
  }

  if (!dp) dp = this;

  publish_state(data[0]);
}

void OPTOLINKBinarySensor::encode(uint8_t* raw, uint8_t length, void* data) {
  float value = *reinterpret_cast<float*>(data);
  encode(raw, length, value);
}

void OPTOLINKBinarySensor::encode(uint8_t* raw, uint8_t length, float data) {
  if (length < _length) {
    ESP_LOGW(TAG, "encode length mismatch for %s: got=%u expected=%u",
             this->get_name().c_str(), (unsigned) length, (unsigned) _length);
    return;
  }

}

}  // namespace vitoconnect
}  // namespace esphome

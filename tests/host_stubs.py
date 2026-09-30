"""Minimal ESPHome interface stubs for host execution; not a firmware build."""

STUBS = {
    'esphome/core/log.h': r"""#pragma once
#define ESP_LOGD(...) do {} while(0)
#define ESP_LOGI(...) do {} while(0)
#define ESP_LOGW(...) do {} while(0)
#define ESP_LOGE(...) do {} while(0)
#define ESP_LOGV(...) do {} while(0)
#define ESP_LOGVV(...) do {} while(0)
#define ESP_LOGCONFIG(...) do {} while(0)
""",
    'esphome/core/hal.h': r"""#pragma once
#include <cstdint>
extern uint32_t fake_millis;
namespace esphome { inline uint32_t millis() { return fake_millis; } }
""",
    'esphome/core/component.h': r"""#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
namespace esphome {
class Component { public: virtual ~Component() = default; virtual void setup() {} virtual void loop() {} };
class PollingComponent : public Component { public: PollingComponent(uint32_t) {} virtual void update() {} };
}
""",
    'esphome/components/uart/uart_component.h': r"""#pragma once
namespace esphome::uart { constexpr int UART_CONFIG_PARITY_EVEN=1; class UARTComponent {}; }
""",
    'esphome/components/uart/uart.h': r"""#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "uart_component.h"
namespace esphome::uart {
class UARTDevice { public: virtual ~UARTDevice() = default; int available() {return 0;} int read() {return -1;} int peek() {return -1;}
void write_array(const uint8_t*, size_t) {} void flush() {} void check_uart_settings(int,int,int,int) {} };
}
""",
    'esphome/components/number/number.h': r"""#pragma once
#include <cmath>
#include <string>
#include "esphome/core/log.h"
namespace esphome::number {
class NumberTraits { public: float min=3, max=30, step=1;
float get_min_value() const{return min;} float get_max_value() const{return max;} float get_step() const{return step;} };
class Number { public: virtual ~Number()=default; virtual void control(float)=0; NumberTraits traits; float state=NAN;
bool has_state() const{return !std::isnan(state);} void publish_state(float v) {state=v;} std::string get_name() const{return "fake-number";} };
}
""",
    'esphome/components/sensor/sensor.h': r"""#pragma once
namespace esphome::sensor { class Sensor {}; }
""",
}

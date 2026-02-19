/* VitoWiFi

Copyright 2019 Bert Melis

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include "vitoconnect_optolink.h"

namespace esphome {
namespace vitoconnect {

Optolink::Optolink(uart::UARTDevice* uart) :
  _uart(uart),
  _queue(VITOWIFI_MAX_QUEUE_LENGTH),
  _onData(nullptr),
  _onError(nullptr),
  _onDataNoArg(nullptr),
  _onErrorNoArg(nullptr) {}

Optolink::~Optolink() {
  // nothing to do
}

void Optolink::onData(void (*callback)(uint8_t* data, uint8_t len)) {
  _onDataNoArg = callback;
  _onData = nullptr;
}

void Optolink::onData(OnDataArgCallback callback) {
  _onData = callback;
  _onDataNoArg = nullptr;
}

void Optolink::onError(void (*callback)(uint8_t error)) {
  _onErrorNoArg = callback;
  _onError = nullptr;
}

void Optolink::onError(OnErrorArgCallback callback) {
  _onError = callback;
  _onErrorNoArg = nullptr;
}

bool Optolink::read(uint16_t address, uint8_t length, void* arg) {
  if (length == 0 || length > MAX_DP_LENGTH) {
    return false;
  }
  OptolinkDP dp(address, length, false, nullptr, arg);
  return _queue.push(dp);
}

bool Optolink::write(uint16_t address, uint8_t length, uint8_t* data, void* arg) {
  if (length == 0 || length > MAX_DP_LENGTH || data == nullptr) {
    return false;
  }
  OptolinkDP dp(address, length, true, data, arg);
  return _queue.push(dp);
}

void Optolink::_tryOnData(uint8_t* data, uint8_t len) {
  OptolinkDP* dp = _queue.front();
  if (dp == nullptr) return;
  if (_onData) _onData(data, len, dp->arg);
  else if (_onDataNoArg) _onDataNoArg(data, len);
  _queue.pop();
}

void Optolink::_tryOnError(uint8_t error) {
  OptolinkDP* dp = _queue.front();
  if (dp == nullptr) return;
  if (_onError) _onError(error, dp->arg);
  else if (_onErrorNoArg) _onErrorNoArg(error);
  _queue.pop();
}

}  // namespace vitoconnect
}  // namespace esphome

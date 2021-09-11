#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace wiegand {

class WiegandTrigger;

// Based on:
// https://github.com/paulo-raca/YetAnotherArduinoWiegandLibrary

class Wiegand : public PollingComponent {
 public:
  enum DataError { Communication, SizeTooBig, SizeUnexpected, DecodeFailed, VerificationFailed };

  static const uint8_t LENGTH_ANY = 0xFF;
  static const uint8_t TIMEOUT = 25;
  static const uint8_t MAX_BITS = 64;
  static const uint8_t MAX_BYTES = ((MAX_BITS + 7) / 8);
  static const uint8_t PIN_0 = 0x01;
  static const uint8_t PIN_1 = 0x02;
  static const uint8_t MASK_PINS = (PIN_0 | PIN_1);
  static const uint8_t DEVICE_CONNECTED = 0x04;
  static const uint8_t DEVICE_INITIALIZED = 0x08;
  static const uint8_t ERROR_TRANSMISSION = 0x10;
  static const uint8_t ERROR_TOO_BIG = 0x20;
  static const uint8_t MASK_STATE = 0x0F;
  static const uint8_t MASK_ERRORS = 0xF0;

  operator bool();

  void setup() override;
  void update() override;
  inline float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  inline void register_trigger(WiegandTrigger *trig) { this->triggers_.push_back(trig); }

  static void pin_state_changed(Wiegand *arg);

  inline void set_pin_d0(GPIOPin *_pin_d0) { this->pin_d0 = _pin_d0; }
  inline void set_pin_d1(GPIOPin *_pin_d1) { this->pin_d1 = _pin_d1; }
  inline void set_pin0_state(bool state) { set_pin_state(0, state); }
  inline void set_pin1_state(bool state) { set_pin_state(1, state); }
  inline void received_bit(bool bitValue) {
    set_pin_state(0, true);
    set_pin_state(1, true);
    set_pin_state(bitValue, false);
    set_pin_state(bitValue, true);
  }
  void set_pin_state(uint8_t pin, bool pin_state);
  void add_bit_internal(bool value);
  const char *DataErrorStr(DataError error);
  void begin(uint8_t expected_bits = LENGTH_ANY, bool decode_messages = true);
  void end();
  void reset();
  void flush();
  void flush_now();
  void flush_data();

 protected:
  int8_t read_state_{-1};
  uint8_t buffer_[6]{};
  std::vector<WiegandTrigger *> triggers_;
  uint32_t last_id_{0};

  GPIOPin *pin_d0;
  GPIOPin *pin_d1;
  uint8_t expected_bits;
  bool decode_messages;
  uint8_t bits;
  uint8_t state;
  unsigned long timestamp;
  uint8_t data[MAX_BYTES];
};

class WiegandTrigger : public Trigger<std::string> {
 public:
  void process(uint8_t *data, uint8_t bits);
};

}  // namespace wiegand
}  // namespace esphome

#include "wiegand.h"
#include "esphome/core/log.h"

namespace esphome {
namespace wiegand {

const char *const TAG = "wiegand";

void write_bit(uint8_t *data, uint8_t i, bool value) {
  if (value) {
    data[i >> 3] |= (0x80 >> (i & 7));
  } else {
    data[i >> 3] &= ~(0x80 >> (i & 7));
  }
}

std::string format_data(uint8_t *data, uint8_t bits) {
  char buf[32];
  char format[] = "%02x-";

  uint8_t num_bytes = (bits + 7) / 8;
  uint8_t padding = Wiegand::MAX_BYTES - num_bytes;

  uint8_t j = 0;
  for (int i = 0; i < Wiegand::MAX_BYTES; i++) {
    if (i == Wiegand::MAX_BYTES - 1)
      format[strlen(format) - 1] = '\0';

    if (i < padding)
      sprintf(buf + j, format, 0x00);
    else
      sprintf(buf + j, format, data[i - padding]);

    j = j + 3;
  }

  return std::string(buf);
}

bool read_bit(uint8_t *data, uint8_t i) { return bool(data[i >> 3] & (0x80 >> (i & 7))); }

inline uint8_t align_data(uint8_t *data, uint8_t start, uint8_t end) {
  uint8_t aligned_data[Wiegand::MAX_BYTES];
  uint8_t aligned_bits = end - start;
  uint8_t aligned_bytes = (aligned_bits + 7) / 8;
  uint8_t aligned_offset = 8 * aligned_bytes - aligned_bits;

  aligned_data[0] = 0;
  for (int bit = 0; bit < aligned_bits; bit++) {
    write_bit(aligned_data, bit + aligned_offset, read_bit(data, bit + start));
  }
  for (int i = 0; i < aligned_bytes; i++) {
    data[i] = aligned_data[i];
  }
  return aligned_bits;
}

Wiegand::operator bool() {
  return (state & (Wiegand::DEVICE_CONNECTED | Wiegand::DEVICE_INITIALIZED)) ==
         (Wiegand::DEVICE_CONNECTED | Wiegand::DEVICE_INITIALIZED);
}

void Wiegand::setup() {
  begin(Wiegand::LENGTH_ANY, true);
  pin_d0->setup();
  pin_d1->setup();

  pin_d0->attach_interrupt(pin_state_changed, this, gpio::INTERRUPT_ANY_EDGE);
  pin_d1->attach_interrupt(pin_state_changed, this, gpio::INTERRUPT_ANY_EDGE);
  pin_state_changed(this);
}

void Wiegand::update() {
  {
    InterruptLock lock;
    flush();
  }
}

void IRAM_ATTR Wiegand::pin_state_changed(Wiegand *arg) {
  arg->set_pin0_state(arg->pin_d0->digital_read());
  arg->set_pin1_state(arg->pin_d1->digital_read());
}

/**
 * Updates the state of a pin.
 *
 * It will trigger adding bits to the payload, device connection/disconnection,
 * dispatching the payload to the callback, etc
 */
void Wiegand::set_pin_state(uint8_t pin, bool pin_state) {
  uint8_t pin_mask = pin ? Wiegand::PIN_1 : Wiegand::PIN_0;

  flush();

  // No change? Abort!
  if (bool(state & pin_mask) == pin_state) {
    return;
  }

  timestamp = millis();
  if (pin_state) {
    state |= pin_mask;
  } else {
    state &= ~pin_mask;
  }

  // Both pins on: bit received
  if ((state & MASK_PINS) == MASK_PINS) {
    // If the device wasn't ready before -- Enable it, and marks state as INVALID until is settles.
    if (state & DEVICE_CONNECTED) {
      add_bit_internal(pin);
    } else {
      // Device connection was detected right now!
      // Set the device as connected, but unstable
      state = (state & Wiegand::MASK_STATE) | Wiegand::DEVICE_CONNECTED | Wiegand::ERROR_TRANSMISSION;
      ESP_LOGW(TAG, "State changed");
    }

    // Both pins off - Device is unplugged
  } else if ((state & Wiegand::MASK_PINS) == 0) {
    if (state & Wiegand::DEVICE_CONNECTED) {
      // Flush truncated message, if any, and resets state
      state |= ERROR_TRANSMISSION;
      flush_now();

      // Set the state as disconnected
      state = (state & Wiegand::MASK_STATE & ~Wiegand::DEVICE_CONNECTED);
      ESP_LOGW(TAG, "State changed. Disconnected");
    }
  }
}

/**
 * Adds a new bit to the payload
 */
void Wiegand::add_bit_internal(bool value) {
  // Skip if we have too much data
  if (bits >= Wiegand::MAX_BITS) {
    state |= Wiegand::ERROR_TOO_BIG;
  } else {
    write_bit(data, bits++, value);
  }

  // If we know the number of bits, there is no need to wait for the timeout to send the data
  if (expected_bits > 0 && (bits == expected_bits)) {
    flush_data();
    reset();
  }
}

const char *Wiegand::DataErrorStr(Wiegand::DataError error) {
  switch (error) {
    case Communication:
      return "Communication Error";
    case SizeTooBig:
      return "Message size too big";
    case SizeUnexpected:
      return "Message size unexpected";
    case DecodeFailed:
      return "Unsupported message format";
    case VerificationFailed:
      return "Message verification failed";
    default:
      return "Unknown";
  }
}

void Wiegand::begin(uint8_t expected_bits, bool decode_messages) {
  this->expected_bits = expected_bits;
  this->decode_messages = decode_messages;

  // Set state as "INVALID", so that data can only be received after a few millis in "ready" state
  bits = 0;
  timestamp = millis();
  state = (state & Wiegand::MASK_STATE) | Wiegand::DEVICE_INITIALIZED | Wiegand::ERROR_TRANSMISSION;
}

void Wiegand::end() {
  expected_bits = 0;

  bits = 0;
  timestamp = millis();
  state &= MASK_STATE & ~Wiegand::DEVICE_INITIALIZED;
}

void Wiegand::reset() {
  bits = 0;
  state &= Wiegand::MASK_STATE;
  // A transmission must start with D0=1, D1=1
  if ((state & Wiegand::MASK_PINS) != Wiegand::MASK_PINS) {
    state |= Wiegand::ERROR_TRANSMISSION;
  }
}

/**
 * Clean up state after `TIMEOUT` milliseconds without events
 *
 * This means sending out any pending message and calling `reset()`
 */
void Wiegand::flush() {
  unsigned long elapsed = millis() - timestamp;
  // Resets state if nothing happened in a few milliseconds
  if (elapsed > TIMEOUT) {
    // Might have a pending data package
    flush_data();
    reset();
  }
}

/**
 * Immediately cleans up state, sending out pending messages and calling `reset()`
 */
void Wiegand::flush_now() {
  flush_data();
  reset();
}

void Wiegand::flush_data() {
  // Ignore empty messages
  if ((bits == 0) || (expected_bits == 0)) {
    return;
  }

  // Check for pending errors
  ESP_LOGV(TAG, "Checking for pending errors");
  if (state & Wiegand::MASK_ERRORS) {
    bits = align_data(data, 0, bits);
    if (state & Wiegand::ERROR_TOO_BIG) {
      ESP_LOGW(TAG, "Data error, size too big");
    } else {
      ESP_LOGW(TAG, "Communication error");
    }

    return;
  }

  // Validate the message size
  if ((expected_bits != bits) && (expected_bits != Wiegand::LENGTH_ANY)) {
    ESP_LOGW(TAG, "Card Read Error");
    return;
  }

  // Decode the message
  if (!decode_messages) {
    bits = align_data(data, 0, bits);
    ESP_LOGI(TAG, "Decoded message bits: %d data: %d", bits, data);
  } else {
    // 4-bit keycode: No check necessary
    if ((bits == 4)) {
      bits = align_data(data, 0, bits);
      for (auto *trigger : this->triggers_)
        trigger->process(data, bits);

      ESP_LOGI(TAG, "Found tag: 4bit: %s", format_data(data, bits).c_str());

      // 8-bit keybode: UpperNibble = ~lowerNibble
    } else if ((bits == 8)) {
      uint8_t value = data[0] & 0xF;
      if (data[0] == (value | ((0xF & ~value) << 4))) {
        for (auto *trigger : this->triggers_)
          trigger->process(data, bits);

        ESP_LOGI(TAG, "Found tag: 8bit: %s", format_data(data, bits).c_str());
      } else {
        ESP_LOGW(TAG, "Card Read Error (8bit)");
      }

      // 26 or 34-bits: First and last bits are used for parity
    } else if ((bits == 26) || (bits == 34)) {
      // FIXME: The parity check doesn't seem to work for a 34-bit reader I have,
      // but I suspect that the reader is non-complaint

      bool left_parity = false;
      bool right_parity = false;
      for (int i = 0; i < (bits + 1) / 2; i++) {
        left_parity = (left_parity != read_bit(data, i));
      }
      for (int i = bits / 2; i < bits; i++) {
        right_parity = (right_parity != read_bit(data, i));
      }

      if (!left_parity && right_parity) {
        bits = align_data(data, 1, bits - 1);

        for (auto *trigger : this->triggers_)
          trigger->process(data, bits);

        ESP_LOGI(TAG, "Found tag: 26/34bit: %s", format_data(data, bits).c_str());
      } else {
        ESP_LOGW(TAG, "Card Read Error (verification failed, parity mismatch)");
      }

    } else {
      ESP_LOGW(TAG, "Card Read Error (decode failed)");
    }
  }
}

void WiegandTrigger::process(uint8_t *data, uint8_t bits) { this->trigger(format_data(data, bits)); }

}  // namespace wiegand
}  // namespace esphome

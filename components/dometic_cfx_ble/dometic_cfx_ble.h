#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/entity_base.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_bt_main.h>
#include <esp_gatt_defs.h>

#include <cstdint>
#include <cstring>
#include <queue>
#include <map>
#include <string>
#include <vector>
#include <type_traits>

namespace esphome {
namespace dometic_cfx_ble {

static const char *const TAG = "dometic_cfx_ble";

// Simple description of a protocol topic.
// For now we only need the 4-byte "param" key; type/desc are informational.
struct TopicInfo {
  uint8_t param[4];
  const char *type;
  const char *description;
};

// Action codes in the DDM protocol. Values here are placeholders; adjust if needed
// to match the real protocol constants from your bundle.js.
enum : uint8_t {
  ACTION_PUB   = 0x01,
  ACTION_SUB   = 0x02,
  ACTION_PING  = 0x03,
  ACTION_HELLO = 0x04,
  ACTION_ACK   = 0x05,
  ACTION_NAK   = 0x06,
  ACTION_NOP   = 0x07,
};

// Minimal topic map; you can extend this with the full set from your Python map.
extern const std::map<std::string, TopicInfo> TOPICS;

class DometicCfxBle : public Component {
 public:
  float get_setup_priority() const override { return setup_priority::BLUETOOTH; }

  void set_mac_address(const uint8_t *mac);
  void set_product_type(uint8_t type) { this->product_type_ = type; }

  template<typename T>
  void add_entity(const std::string &topic, T *entity) {
    // Classify entity at compile-time based on base class.
    if constexpr (std::is_base_of<sensor::Sensor, T>::value) {
      sensors_[topic] = entity;
    } else if constexpr (std::is_base_of<binary_sensor::BinarySensor, T>::value) {
      binary_sensors_[topic] = entity;
    } else if constexpr (std::is_base_of<switch_::Switch, T>::value) {
      switches_[topic] = entity;
    } else if constexpr (std::is_base_of<number::Number, T>::value) {
      numbers_[topic] = entity;
    } else if constexpr (std::is_base_of<text_sensor::TextSensor, T>::value) {
      text_sensors_[topic] = entity;
    } else {
      ESP_LOGW(TAG, "Unknown entity type for topic %s", topic.c_str());
    }
  }

  void setup() override;
  void loop() override;
  void dump_config() override;

  // Low-level protocol helpers used by the per-entity classes.
  void send_pub(const std::string &topic, const std::vector<uint8_t> &value);
  void send_sub(const std::string &topic);
  void send_ping();

  // Static entry points for the ESP-IDF callbacks.
  static DometicCfxBle *instance_;

  static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gatt_if, esp_ble_gattc_cb_param_t *param);

  // Instance handlers where we can safely touch members.
  void handle_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  void handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gatt_if, esp_ble_gattc_cb_param_t *param);

  // Exposed for the per-entity classes.
  bool is_connected() const { return connected_; }

 protected:
  void start_scan_();
  void connect_();
  void handle_notify_(const uint8_t *data, size_t len);
  void update_entity_(const std::string &topic, const std::vector<uint8_t> &value);

  float decode_to_float_(const std::vector<uint8_t> &bytes, const std::string &type_hint);
  bool decode_to_bool_(const std::vector<uint8_t> &bytes, const std::string &type_hint);
  std::string decode_to_string_(const std::vector<uint8_t> &bytes, const std::string &type_hint);

  std::vector<uint8_t> encode_from_bool_(bool value, const std::string &type_hint);
  std::vector<uint8_t> encode_from_float_(float value, const std::string &type_hint);

  uint8_t mac_address_[6] = {0};
  uint8_t product_type_{0};

  esp_gatt_if_t gattc_if_{ESP_GATT_IF_NONE};
  uint16_t conn_id_{0};
  bool connected_{false};

  uint16_t write_handle_{0};
  uint16_t notify_handle_{0};

  uint32_t last_activity_ms_{0};
  bool scan_in_progress_{false};

  std::queue<std::vector<uint8_t>> send_queue_;

  std::map<std::string, sensor::Sensor *> sensors_;
  std::map<std::string, binary_sensor::BinarySensor *> binary_sensors_;
  std::map<std::string, switch_::Switch *> switches_;
  std::map<std::string, number::Number *> numbers_;
  std::map<std::string, text_sensor::TextSensor *> text_sensors_;
};

// Per-platform helper classes. These are deliberately kept very small and
// delegate all protocol work back to the hub.

class DometicCfxBleSensor : public sensor::Sensor, public PollingComponent {
 public:
  void set_parent(DometicCfxBle *parent) { parent_ = parent; }
  void set_topic(const std::string &topic) { topic_ = topic; }
  void update() override {}  // sensors are push-driven by notifications

 protected:
  DometicCfxBle *parent_{nullptr};
  std::string topic_;
};

class DometicCfxBleBinarySensor : public binary_sensor::BinarySensor, public PollingComponent {
 public:
  void set_parent(DometicCfxBle *parent) { parent_ = parent; }
  void set_topic(const std::string &topic) { topic_ = topic; }
  void update() override {}  // push-only

 protected:
  DometicCfxBle *parent_{nullptr};
  std::string topic_;
};

class DometicCfxBleSwitch : public switch_::Switch, public PollingComponent {
 public:
  void set_parent(DometicCfxBle *parent) { parent_ = parent; }
  void set_topic(const std::string &topic) { topic_ = topic; }

  void write_state(bool state) override;
  void update() override {}  // no periodic polling

 protected:
  DometicCfxBle *parent_{nullptr};
  std::string topic_;
};

class DometicCfxBleNumber : public number::Number, public PollingComponent {
 public:
  void set_parent(DometicCfxBle *parent) { parent_ = parent; }
  void set_topic(const std::string &topic) { topic_ = topic; }

  void control(float value) override;
  void update() override {}  // no periodic polling

 protected:
  DometicCfxBle *parent_{nullptr};
  std::string topic_;
};

class DometicCfxBleTextSensor : public text_sensor::TextSensor, public PollingComponent {
 public:
  void set_parent(DometicCfxBle *parent) { parent_ = parent; }
  void set_topic(const std::string &topic) { topic_ = topic; }
  void update() override {}  // push-only

 protected:
  DometicCfxBle *parent_{nullptr};
  std::string topic_;
};

}  // namespace dometic_cfx_ble
}  // namespace esphome

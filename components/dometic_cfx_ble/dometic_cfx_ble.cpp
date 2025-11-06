#include "dometic_cfx_ble.h"

#include <esp_err.h>
#include <esp_bt.h>
#include <cmath>

namespace esphome {
namespace dometic_cfx_ble {

DometicCfxBle *DometicCfxBle::instance_ = nullptr;

// UUIDs for the CFX3 BLE service and characteristics.
// Replace these with the exact values from your reverse-engineering notes
// if they differ.
static const uint8_t SERVICE_UUID_128[16] = {
    0x15, 0xd5, 0x3f, 0xe2, 0x04, 0x16, 0x6c, 0x92,
    0x1f, 0x48, 0x95, 0x09, 0x00, 0x03, 0x7a, 0x53,
};

static const uint8_t WRITE_CHAR_UUID_128[16] = {
    0x15, 0xd5, 0x3f, 0xe2, 0x04, 0x16, 0x6c, 0x92,
    0x1f, 0x48, 0x95, 0x09, 0x01, 0x03, 0x7a, 0x53,
};

static const uint8_t NOTIFY_CHAR_UUID_128[16] = {
    0x15, 0xd5, 0x3f, 0xe2, 0x04, 0x16, 0x6c, 0x92,
    0x1f, 0x48, 0x95, 0x09, 0x02, 0x03, 0x7a, 0x53,
};

// Minimal topic map - enough to get the basic subscription handshake going.
// Fill this out with the full table from your Python implementation.
const std::map<std::string, TopicInfo> TOPICS = {
    {"SUBSCRIBE_APP_SZ",  {{0x01, 0x00, 0x00, 0x81}, "EMPTY", "Subscribe single-zone app topics"}},
    {"SUBSCRIBE_APP_SZI", {{0x02, 0x00, 0x00, 0x81}, "EMPTY", "Subscribe single-zone + ice maker app topics"}},
    {"SUBSCRIBE_APP_DZ",  {{0x03, 0x00, 0x00, 0x81}, "EMPTY", "Subscribe dual-zone app topics"}},
};

void DometicCfxBle::set_mac_address(const uint8_t *mac) {
  if (mac == nullptr) return;
  std::memcpy(this->mac_address_, mac, 6);
}

void DometicCfxBle::setup() {
  ESP_LOGI(TAG, "Initializing Dometic CFX3 BLE hub");

  instance_ = this;

  esp_err_t err;

  err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "esp_bt_controller_mem_release failed: %d", err);
  }

  err = esp_ble_gattc_register_callback(&DometicCfxBle::gattc_event_handler);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gattc_register_callback failed: %d", err);
  }

  err = esp_ble_gattc_app_register(0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gattc_app_register failed: %d", err);
  }

  err = esp_ble_gap_register_callback(&DometicCfxBle::gap_event_handler);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %d", err);
  }

  this->start_scan_();
}

void DometicCfxBle::dump_config() {
  ESP_LOGCONFIG(TAG, "Dometic CFX3 BLE:");
  ESP_LOGCONFIG(TAG, "  MAC address: %02X:%02X:%02X:%02X:%02X:%02X",
                mac_address_[0], mac_address_[1], mac_address_[2],
                mac_address_[3], mac_address_[4], mac_address_[5]);
  ESP_LOGCONFIG(TAG, "  Product type: %u", product_type_);
}

void DometicCfxBle::loop() {
  const uint32_t now = millis();

  // Periodic ping to keep the link alive if we are connected.
  if (connected_ && (now - last_activity_ms_ > 15000U)) {
    this->send_ping();
  }

  // Flush send queue.
  if (connected_ && write_handle_ != 0 && !send_queue_.empty()) {
    auto packet = std::move(send_queue_.front());
    send_queue_.pop();

    esp_err_t err = esp_ble_gattc_write_char(
        gattc_if_, conn_id_, write_handle_,
        static_cast<uint16_t>(packet.size()), packet.data(),
        ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);

    if (err != ESP_OK) {
      ESP_LOGW(TAG, "esp_ble_gattc_write_char failed: %d", err);
    } else {
      last_activity_ms_ = now;
    }
  }
}

void DometicCfxBle::send_pub(const std::string &topic, const std::vector<uint8_t> &value) {
  auto it = TOPICS.find(topic);
  if (it == TOPICS.end()) {
    ESP_LOGW(TAG, "send_pub: unknown topic '%s'", topic.c_str());
    return;
  }
  const TopicInfo &info = it->second;

  std::vector<uint8_t> packet;
  packet.reserve(1 + 4 + value.size());
  packet.push_back(ACTION_PUB);
  packet.insert(packet.end(), info.param, info.param + 4);
  packet.insert(packet.end(), value.begin(), value.end());

  send_queue_.push(std::move(packet));
}

void DometicCfxBle::send_sub(const std::string &topic) {
  auto it = TOPICS.find(topic);
  if (it == TOPICS.end()) {
    ESP_LOGW(TAG, "send_sub: unknown topic '%s'", topic.c_str());
    return;
  }
  const TopicInfo &info = it->second;

  std::vector<uint8_t> packet;
  packet.reserve(1 + 4);
  packet.push_back(ACTION_SUB);
  packet.insert(packet.end(), info.param, info.param + 4);

  send_queue_.push(std::move(packet));
}

void DometicCfxBle::send_ping() {
  std::vector<uint8_t> packet;
  packet.reserve(1);
  packet.push_back(ACTION_PING);
  send_queue_.push(std::move(packet));
}

void DometicCfxBle::start_scan_() {
  if (scan_in_progress_) return;

  esp_ble_scan_params_t scan_params{};
  scan_params.scan_type          = BLE_SCAN_TYPE_ACTIVE;
  scan_params.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
  scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
  scan_params.scan_interval      = 0x50;
  scan_params.scan_window        = 0x30;
  scan_params.scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE;

  esp_err_t err = esp_ble_gap_set_scan_params(&scan_params);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_set_scan_params failed: %d", err);
    return;
  }

  // We'll actually start scanning from the SCAN_PARAM_SET_COMPLETE event.
  scan_in_progress_ = true;
}

void DometicCfxBle::connect_() {
  if (gattc_if_ == ESP_GATT_IF_NONE) {
    ESP_LOGW(TAG, "connect_: GATTC interface not ready yet");
    return;
  }

  ESP_LOGI(TAG, "Opening GATT connection to CFX3");

  esp_err_t err = esp_ble_gattc_open(
      gattc_if_, mac_address_, BLE_ADDR_TYPE_PUBLIC, true);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gattc_open failed: %d", err);
  }
}

// Static trampoline callbacks

void DometicCfxBle::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (instance_ != nullptr) {
    instance_->handle_gap_event(event, param);
  }
}

void DometicCfxBle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gatt_if, esp_ble_gattc_cb_param_t *param) {
  if (instance_ != nullptr) {
    instance_->handle_gattc_event(event, gatt_if, param);
  }
}

// Instance-level handlers

void DometicCfxBle::handle_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
      ESP_LOGI(TAG, "BLE scan params set, starting scan");
      esp_ble_gap_start_scanning(30);  // seconds
      break;
    }

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
      auto &r = param->scan_rst;
      if (r.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT)
        break;

      if (std::memcmp(r.bda, mac_address_, 6) == 0) {
        ESP_LOGI(TAG, "Found target CFX3, stopping scan and connecting");
        esp_ble_gap_stop_scanning();
        this->connect_();
      }
      break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
      ESP_LOGD(TAG, "BLE scan stopped");
      scan_in_progress_ = false;
      break;
    }

    default:
      break;
  }
}

void DometicCfxBle::handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gatt_if, esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_REG_EVT: {
      gattc_if_ = gatt_if;
      ESP_LOGI(TAG, "GATTC registered, app_id=%d", param->reg.app_id);
      break;
    }

    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        conn_id_ = param->open.conn_id;
        connected_ = true;
        last_activity_ms_ = millis();
        ESP_LOGI(TAG, "GATTC connected, conn_id=%d", conn_id_);

        // Discover primary service and characteristics – intentionally left
        // minimal here. You can extend this with esp_ble_gattc_search_service
        // and get_all_char to locate WRITE/NOTIFY handles, then enable CCCD.
      } else {
        ESP_LOGW(TAG, "GATTC open failed, status=%d", param->open.status);
      }
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.is_notify && param->notify.value != nullptr && param->notify.value_len > 0) {
        this->handle_notify_(param->notify.value, param->notify.value_len);
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "GATTC disconnected, reason=%d", param->disconnect.reason);
      connected_ = false;
      write_handle_ = 0;
      notify_handle_ = 0;
      break;
    }

    default:
      break;
  }
}

// Very small and conservative notify handler – it only logs frames for now.
// You can extend this to fully decode the DDM protocol and call update_entity_.
void DometicCfxBle::handle_notify_(const uint8_t *data, size_t len) {
  if (len == 0 || data == nullptr) return;

  uint8_t action = data[0];
  ESP_LOGV(TAG, "Notify: action=0x%02X, len=%u", action, (unsigned) len);

  // For now we only bump the activity timer; all detailed decoding is left
  // for the next iteration once the GATT plumbing is stable.
  last_activity_ms_ = millis();
}

void DometicCfxBle::update_entity_(const std::string &topic, const std::vector<uint8_t> &value) {
  auto it_s = sensors_.find(topic);
  if (it_s != sensors_.end()) {
    float v = decode_to_float_(value, "sensor");
    it_s->second->publish_state(v);
    return;
  }

  auto it_b = binary_sensors_.find(topic);
  if (it_b != binary_sensors_.end()) {
    bool v = decode_to_bool_(value, "binary_sensor");
    it_b->second->publish_state(v);
    return;
  }

  auto it_sw = switches_.find(topic);
  if (it_sw != switches_.end()) {
    bool v = decode_to_bool_(value, "switch");
    it_sw->second->publish_state(v);
    return;
  }

  auto it_n = numbers_.find(topic);
  if (it_n != numbers_.end()) {
    float v = decode_to_float_(value, "number");
    it_n->second->publish_state(v);
    return;
  }

  auto it_t = text_sensors_.find(topic);
  if (it_t != text_sensors_.end()) {
    std::string s = decode_to_string_(value, "text_sensor");
    it_t->second->publish_state(s);
    return;
  }

  ESP_LOGV(TAG, "update_entity_: no entity registered for topic '%s'", topic.c_str());
}

// Extremely conservative default decoding helpers.
// These don't assume detailed knowledge of the protocol; they just apply
// basic interpretations so that at least something sensible is shown.
// You can replace these with the exact rules from your DDM implementation.

float DometicCfxBle::decode_to_float_(const std::vector<uint8_t> &bytes, const std::string & /*type_hint*/) {
  if (bytes.empty())
    return NAN;

  if (bytes.size() == 1) {
    int8_t v = static_cast<int8_t>(bytes[0]);
    return static_cast<float>(v);
  }

  if (bytes.size() == 2) {
    int16_t v = static_cast<int16_t>(bytes[0] | (static_cast<int16_t>(bytes[1]) << 8));
    return static_cast<float>(v) / 10.0f;  // assume fixed-point 0.1
  }

  // Fallback: treat first 4 bytes as signed 32-bit integer, little endian.
  int32_t v = static_cast<int32_t>(
      bytes[0] |
      (static_cast<int32_t>(bytes[1]) << 8) |
      (static_cast<int32_t>(bytes[2]) << 16) |
      (static_cast<int32_t>(bytes[3]) << 24));
  return static_cast<float>(v);
}

bool DometicCfxBle::decode_to_bool_(const std::vector<uint8_t> &bytes, const std::string & /*type_hint*/) {
  if (bytes.empty()) return false;
  return bytes[0] != 0;
}

std::string DometicCfxBle::decode_to_string_(const std::vector<uint8_t> &bytes, const std::string & /*type_hint*/) {
  return std::string(bytes.begin(), bytes.end());
}

std::vector<uint8_t> DometicCfxBle::encode_from_bool_(bool value, const std::string & /*type_hint*/) {
  return {static_cast<uint8_t>(value ? 1 : 0)};
}

std::vector<uint8_t> DometicCfxBle::encode_from_float_(float value, const std::string & /*type_hint*/) {
  // Default: encode as signed 16-bit fixed-point with 0.1 resolution.
  int16_t v = static_cast<int16_t>(value * 10.0f);
  std::vector<uint8_t> out;
  out.reserve(2);
  out.push_back(static_cast<uint8_t>(v & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  return out;
}

// Per-platform classes – just thin wrappers around send_pub / decode helpers.

void DometicCfxBleSwitch::write_state(bool state) {
  if (parent_ == nullptr) {
    ESP_LOGW(TAG, "Switch write_state without parent");
    publish_state(state);
    return;
  }

  auto payload = parent_->encode_from_bool_(state, "switch");
  parent_->send_pub(topic_, payload);
  publish_state(state);
}

void DometicCfxBleNumber::control(float value) {
  if (parent_ == nullptr) {
    ESP_LOGW(TAG, "Number control without parent");
    publish_state(value);
    return;
  }

  auto payload = parent_->encode_from_float_(value, "number");
  parent_->send_pub(topic_, payload);
  publish_state(value);
}

}  // namespace dometic_cfx_ble
}  // namespace esphome

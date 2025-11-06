#include "dometic_cfx_ble.h"

#include <esp_err.h>
#include <esp_bt.h>
#include <cmath>

namespace esphome {
namespace dometic_cfx_ble {

// UUIDs for the CFX3 BLE service and characteristics.
// Swap these if your bundle.js shows different values.
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

// Minimal topic map so the DDM plumbing is wired. You should extend this
// with the full table from your Python DDM/TOPIC_TYPES so nothing is lost.
const std::map<std::string, TopicInfo> TOPICS = {
    {"SUBSCRIBE_APP_SZ",  {{0x01, 0x00, 0x00, 0x81}, "EMPTY", "Subscribe single-zone app topics"}},
    {"SUBSCRIBE_APP_SZI", {{0x02, 0x00, 0x00, 0x81}, "EMPTY", "Subscribe single-zone+ice app"}},
    {"SUBSCRIBE_APP_DZ",  {{0x03, 0x00, 0x00, 0x81}, "EMPTY", "Subscribe dual-zone app topics"}},
    // Add all the other topics here; keep param[] EXACT as per bundle.js.
};

DometicCfxBle *DometicCfxBle::instance_ = nullptr;

void DometicCfxBle::set_mac_address(const uint8_t *mac) {
  if (mac == nullptr) return;
  std::memcpy(this->mac_address_, mac, 6);
}

void DometicCfxBle::setup() {
  ESP_LOGI(TAG, "Initializing Dometic CFX3 BLE hub");

  instance_ = this;

  esp_err_t err;

  // Free classic BT RAM
  err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "esp_bt_controller_mem_release failed: %d", err);
  }

  err = esp_ble_gap_register_callback(&DometicCfxBle::gap_event_handler);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %d", err);
  }

  err = esp_ble_gattc_register_callback(&DometicCfxBle::gattc_event_handler);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gattc_register_callback failed: %d", err);
  }

  err = esp_ble_gattc_app_register(0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gattc_app_register failed: %d", err);
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

  // Keepalive ping
  if (connected_ && (now - last_activity_ms_ > 15000U)) {
    this->send_ping();
  }

  // Flush send queue
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

// ----------------- Frame-level DDM helpers -----------------------------------

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
  std::vector<uint8_t> packet(1);
  packet[0] = ACTION_PING;
  send_queue_.push(std::move(packet));
}

void DometicCfxBle::send_switch(const std::string &topic, bool value) {
  std::string type_hint = "BOOL";
  auto it = TOPICS.find(topic);
  if (it != TOPICS.end() && it->second.type != nullptr) {
    type_hint = it->second.type;
  }
  auto payload = this->encode_from_bool_(value, type_hint);
  this->send_pub(topic, payload);
}

void DometicCfxBle::send_number(const std::string &topic, float value) {
  std::string type_hint = "NUMBER";
  auto it = TOPICS.find(topic);
  if (it != TOPICS.end() && it->second.type != nullptr) {
    type_hint = it->second.type;
  }
  auto payload = this->encode_from_float_(value, type_hint);
  this->send_pub(topic, payload);
}

// ----------------- GAP / GATTC plumbing -------------------------------------

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

  // Actual start happens in SCAN_PARAM_SET_COMPLETE
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

// Static trampolines
void DometicCfxBle::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (instance_ != nullptr) {
    instance_->handle_gap_event(event, param);
  }
}

void DometicCfxBle::gattc_event_handler(esp_gattc_cb_event_t event,
                                        esp_gatt_if_t gatt_if,
                                        esp_ble_gattc_cb_param_t *param) {
  if (instance_ != nullptr) {
    instance_->handle_gattc_event(event, gatt_if, param);
  }
}

// Instance handlers
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

void DometicCfxBle::handle_gattc_event(esp_gattc_cb_event_t event,
                                       esp_gatt_if_t gatt_if,
                                       esp_ble_gattc_cb_param_t *param) {
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

        esp_bt_uuid_t service_uuid{};
        service_uuid.len = ESP_UUID_LEN_128;
        std::memcpy(service_uuid.uuid.uuid128, SERVICE_UUID_128, sizeof(SERVICE_UUID_128));
        esp_ble_gattc_search_service(gatt_if, conn_id_, &service_uuid);
      } else {
        ESP_LOGW(TAG, "GATTC open failed, status=%d", param->open.status);
      }
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      // Find write + notify chars
      esp_bt_uuid_t write_uuid{};
      write_uuid.len = ESP_UUID_LEN_128;
      std::memcpy(write_uuid.uuid.uuid128, WRITE_CHAR_UUID_128, sizeof(WRITE_CHAR_UUID_128));

      esp_bt_uuid_t notify_uuid{};
      notify_uuid.len = ESP_UUID_LEN_128;
      std::memcpy(notify_uuid.uuid.uuid128, NOTIFY_CHAR_UUID_128, sizeof(NOTIFY_CHAR_UUID_128));

      esp_gattc_char_elem_t char_elem[2];
      uint16_t count = 0;

      if (esp_ble_gattc_get_char_by_uuid(
              gatt_if, conn_id_, 0, 0xFFFF, write_uuid, char_elem, &count) == ESP_OK &&
          count > 0) {
        write_handle_ = char_elem[0].char_handle;
      }

      count = 0;
      if (esp_ble_gattc_get_char_by_uuid(
              gatt_if, conn_id_, 0, 0xFFFF, notify_uuid, char_elem, &count) == ESP_OK &&
          count > 0) {
        notify_handle_ = char_elem[0].char_handle;
      }

      if (notify_handle_ != 0) {
        uint8_t cccd[2] = {0x01, 0x00};
        esp_err_t err = esp_ble_gattc_write_char_descr(
            gatt_if, conn_id_, notify_handle_ + 1, sizeof(cccd), cccd,
            ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (err != ESP_OK) {
          ESP_LOGW(TAG, "Failed to enable notifications, err=%d", err);
        }
      }

      // Kick protocol: ping + subscribe group
      this->send_ping();
      std::string sub_key = (product_type_ == 1)
                                ? "SUBSCRIBE_APP_SZ"
                                : (product_type_ == 2) ? "SUBSCRIBE_APP_SZI" : "SUBSCRIBE_APP_DZ";
      this->send_sub(sub_key);

      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.is_notify &&
          param->notify.value != nullptr &&
          param->notify.value_len > 0) {
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

    case ESP_GATTC_WRITE_CHAR_EVT: {
      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Write char status=%d", param->write.status);
      }
      break;
    }

    default:
      break;
  }
}

// ----------------- DDM frame handling ---------------------------------------

void DometicCfxBle::handle_notify_(const uint8_t *data, size_t len) {
  if (data == nullptr || len < 1) return;

  uint8_t action = data[0];
  ESP_LOGVV(TAG, "Notify frame: action=0x%02X len=%u", action, (unsigned) len);

  // ACK / NAK in response to our PUB/SUB/PING
  if (action == ACTION_ACK || action == ACTION_NAK) {
    if (!send_queue_.empty()) {
      send_queue_.pop();
    }
    if (action == ACTION_NAK) {
      ESP_LOGW(TAG, "Fridge returned NAK");
    }
    last_activity_ms_ = millis();
    return;
  }

  if (action != ACTION_PUB) {
    ESP_LOGV(TAG, "Unhandled DDM action 0x%02X", action);
    last_activity_ms_ = millis();
    return;
  }

  if (len < 5) {
    ESP_LOGW(TAG, "PUB frame too short: %u", (unsigned) len);
    return;
  }

  // 4-byte param key, little-endian
  uint32_t key = static_cast<uint32_t>(data[1]) |
                 (static_cast<uint32_t>(data[2]) << 8) |
                 (static_cast<uint32_t>(data[3]) << 16) |
                 (static_cast<uint32_t>(data[4]) << 24);

  std::string topic;
  const TopicInfo *topic_info = nullptr;

  for (const auto &kv : TOPICS) {
    const TopicInfo &info = kv.second;
    uint32_t tk = static_cast<uint32_t>(info.param[0]) |
                  (static_cast<uint32_t>(info.param[1]) << 8) |
                  (static_cast<uint32_t>(info.param[2]) << 16) |
                  (static_cast<uint32_t>(info.param[3]) << 24);
    if (tk == key) {
      topic = kv.first;
      topic_info = &info;
      break;
    }
  }

  if (topic_info == nullptr) {
    ESP_LOGV(TAG, "Unknown DDM param key 0x%08X (len=%u)", (unsigned) key, (unsigned) len);
    // Still ACK to keep the protocol happy
    uint8_t ack = ACTION_ACK;
    esp_ble_gattc_write_char(
        gattc_if_, conn_id_, write_handle_, 1, &ack,
        ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
    last_activity_ms_ = millis();
    return;
  }

  std::vector<uint8_t> payload;
  if (len > 5) {
    payload.assign(data + 5, data + len);
  }

  ESP_LOGV(TAG, "PUB %s (%s) payload_len=%u",
           topic.c_str(),
           topic_info->type ? topic_info->type : "",
           (unsigned) payload.size());

  this->update_entity_(topic, payload);

  // ACK this publish
  uint8_t ack = ACTION_ACK;
  esp_ble_gattc_write_char(
      gattc_if_, conn_id_, write_handle_, 1, &ack,
      ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);

  last_activity_ms_ = millis();
}

void DometicCfxBle::update_entity_(const std::string &topic, const std::vector<uint8_t> &value) {
  std::string type_hint = "RAW";
  auto ti = TOPICS.find(topic);
  if (ti != TOPICS.end() && ti->second.type != nullptr) {
    type_hint = ti->second.type;
  }

  // Sensor
  if (auto it = sensors_.find(topic); it != sensors_.end()) {
    float v = decode_to_float_(value, type_hint);
    it->second->publish_state(v);
    return;
  }

  // Binary sensor
  if (auto it = binary_sensors_.find(topic); it != binary_sensors_.end()) {
    bool v = decode_to_bool_(value, type_hint);
    it->second->publish_state(v);
    return;
  }

  // Switch
  if (auto it = switches_.find(topic); it != switches_.end()) {
    bool v = decode_to_bool_(value, type_hint);
    it->second->publish_state(v);
    return;
  }

  // Number
  if (auto it = numbers_.find(topic); it != numbers_.end()) {
    float v = decode_to_float_(value, type_hint);
    it->second->publish_state(v);
    return;
  }

  // Text sensor
  if (auto it = text_sensors_.find(topic); it != text_sensors_.end()) {
    std::string s = decode_to_string_(value, type_hint);
    it->second->publish_state(s);
    return;
  }

  ESP_LOGV(TAG, "No entity registered for topic '%s'", topic.c_str());
}

// ----------------- Generic encode/decode ------------------------------------

float DometicCfxBle::decode_to_float_(const std::vector<uint8_t> &bytes, const std::string &type_hint) {
  if (bytes.empty())
    return NAN;

  bool use_fixed_01 = false;
  if (type_hint.find("DECI") != std::string::npos ||
      type_hint.find("_0_1") != std::string::npos ||
      type_hint.find("0.1") != std::string::npos) {
    use_fixed_01 = true;
  }

  if (bytes.size() == 1) {
    int8_t v = static_cast<int8_t>(bytes[0]);
    return static_cast<float>(v);
  }

  if (bytes.size() >= 2) {
    int16_t v = static_cast<int16_t>(bytes[0] |
                                     (static_cast<int16_t>(bytes[1]) << 8));
    if (use_fixed_01) {
      return static_cast<float>(v) / 10.0f;
    }
    return static_cast<float>(v);
  }

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

std::vector<uint8_t> DometicCfxBle::encode_from_float_(float value, const std::string &type_hint) {
  bool use_fixed_01 = false;
  if (type_hint.find("DECI") != std::string::npos ||
      type_hint.find("_0_1") != std::string::npos ||
      type_hint.find("0.1") != std::string::npos) {
    use_fixed_01 = true;
  }

  float scaled = value;
  if (use_fixed_01) {
    scaled = value * 10.0f;
  }

  int16_t v = static_cast<int16_t>(scaled);
  std::vector<uint8_t> out;
  out.reserve(2);
  out.push_back(static_cast<uint8_t>(v & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  return out;
}

// ----------------- Entity helpers -------------------------------------------

void DometicCfxBleSwitch::write_state(bool state) {
  if (parent_ == nullptr) {
    ESP_LOGW(TAG, "Switch write_state without parent");
    publish_state(state);
    return;
  }

  parent_->send_switch(topic_, state);
  publish_state(state);
}

void DometicCfxBleNumber::control(float value) {
  if (parent_ == nullptr) {
    ESP_LOGW(TAG, "Number control without parent");
    publish_state(value);
    return;
  }

  parent_->send_number(topic_, value);
  publish_state(value);
}

}  // namespace dometic_cfx_ble
}  // namespace esphome

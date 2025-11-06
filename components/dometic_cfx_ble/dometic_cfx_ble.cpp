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

      // Kick protoc

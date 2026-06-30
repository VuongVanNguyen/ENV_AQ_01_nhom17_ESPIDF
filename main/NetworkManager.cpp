// ============================================================
// NetworkManager.cpp
//   Wi-Fi STA + MQTT (espressif/mqtt) + SNTP + cJSON payload.
//
//   Toàn bộ event xử lý theo mô hình event-driven của esp_event:
//   không có polling loop — đáp ứng NFR công suất 2W và non-blocking.
//   ESP32 dual-core: event callback có thể chạy trên core khác so với
//   taskNetwork → dùng std::atomic cho các flag trạng thái.
// ============================================================

#include "NetworkManager.hpp"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"  // esp_netif_sntp_init/deinit, esp_sntp_config_t
#include "esp_event.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include <algorithm>
#include <cstring>
#include <cstdio>

static const char *TAG = "NetworkManager";

// File-scope: SNTP là singleton trong ESP-IDF, flag này cũng là singleton.
// Được set trong sntpSyncCb() (static) → không thể dùng instance member.
static std::atomic<bool> s_sntp_synced{false};

// ============================================================
// Constructor / Destructor
// ============================================================
NetworkManager::NetworkManager()
    : netif_(nullptr),
      mqtt_(nullptr),
      wifi_connected_(false),
      mqtt_connected_(false),
      mqtt_client_id_{},
      cmd_callback_(nullptr) {}

NetworkManager::~NetworkManager() {
    if (mqtt_) {
        esp_mqtt_client_stop(mqtt_);
        esp_mqtt_client_destroy(mqtt_);
    }
    esp_netif_sntp_deinit();  // giải phóng SNTP handle (bắt cặp với initSntp)
    // Wi-Fi & netif không tear-down ở dtor — chúng là singleton hệ thống;
    // lifecycle thuộc app_main, không phải NetworkManager.
}

// ============================================================
// Public init
// ============================================================
esp_err_t NetworkManager::init() {
    // esp_netif_init() và default event loop là singleton hệ thống —
    // gọi lần 2 trả ESP_ERR_INVALID_STATE → bỏ qua an toàn.
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    err = initWifi();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi khởi tạo thất bại: %s", esp_err_to_name(err));
        return err;
    }

    err = initMqtt();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT khởi tạo thất bại: %s", esp_err_to_name(err));
        return err;
    }

    err = initSntp();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SNTP khởi tạo thất bại: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "NetworkManager sẵn sàng — SSID='%s' broker='%s' ntp='%s'",
             Cfg::WIFI_SSID, Cfg::MQTT_BROKER_URL, Cfg::NTP_SERVER_URL);
    return ESP_OK;
}

// ============================================================
// Wi-Fi STA
// ============================================================
esp_err_t NetworkManager::initWifi() {
    netif_ = esp_netif_create_default_wifi_sta();
    if (!netif_) return ESP_FAIL;

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) return err;

    // Đăng ký handler — `this` truyền qua `arg` để static handler truy cập instance
    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &NetworkManager::wifiEventHandler,
                                              this, nullptr);
    if (err != ESP_OK) return err;

    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &NetworkManager::ipEventHandler,
                                              this, nullptr);
    if (err != ESP_OK) return err;

    wifi_config_t wifi_cfg = {};
    std::strncpy(reinterpret_cast<char *>(wifi_cfg.sta.ssid),
                 Cfg::WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    std::strncpy(reinterpret_cast<char *>(wifi_cfg.sta.password),
                 Cfg::WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);
    // WPA2-PSK minimum: chặn kết nối vào AP không mã hoá / WEP yếu
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);  // tránh ghi NVS mỗi lần connect
    if (err != ESP_OK) return err;
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (err != ESP_OK) return err;
    err = esp_wifi_start();  // phát WIFI_EVENT_STA_START → handler gọi connect()
    if (err != ESP_OK) return err;

    // NFR công suất 2W: bật Modem-sleep — PHẢI gọi sau esp_wifi_start()
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    return ESP_OK;
}

// ============================================================
// MQTT
// ============================================================
esp_err_t NetworkManager::initMqtt() {
    // Tạo client ID = "aq01_<MAC WiFi STA>" — duy nhất mỗi thiết bị.
    // Tránh broker disconnect client cũ khi nhiều trạm kết nối cùng broker.
    // esp_wifi_get_mac() an toàn sau esp_wifi_init() (không cần wifi_start).
    uint8_t mac[6] = {};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(mqtt_client_id_, sizeof(mqtt_client_id_),
             "aq01_%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri    = Cfg::MQTT_BROKER_URL;
    cfg.credentials.client_id = mqtt_client_id_;  // trỏ vào member — vòng đời hợp lệ
    // ThingsBoard (và các dashboard cloud khác) xác thực bằng username = access token,
    // không dùng password. Để trống nếu broker không yêu cầu auth (vd. broker.hivemq.com).
    if (Cfg::MQTT_ACCESS_TOKEN[0] != '\0') {
        cfg.credentials.username = Cfg::MQTT_ACCESS_TOKEN;
    }
    // Auto-reconnect mặc định 10s — đủ cho NFR cảnh báo <3s sau khi kết nối lại
    cfg.network.disable_auto_reconnect = false;

    mqtt_ = esp_mqtt_client_init(&cfg);
    if (!mqtt_) return ESP_FAIL;

    esp_err_t err = esp_mqtt_client_register_event(
        mqtt_,
        static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
        &NetworkManager::mqttEventHandler,
        this);
    if (err != ESP_OK) return err;

    // start() không block — MQTT_EVENT_CONNECTED phát khi broker chấp nhận.
    // Nếu Wi-Fi chưa có IP, client tự retry → không cần xử lý thêm.
    return esp_mqtt_client_start(mqtt_);
}

// ============================================================
// SNTP
// ============================================================
esp_err_t NetworkManager::initSntp() {
    // esp_netif_sntp (v5.1+): tự đồng bộ khi mạng sẵn sàng, retry theo
    // CONFIG_LWIP_SNTP_UPDATE_DELAY (mặc định 1 giờ). Không cần trigger thủ công.
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(Cfg::NTP_SERVER_URL);
    cfg.sync_cb = &NetworkManager::sntpSyncCb;  // callback báo khi đồng bộ xong

    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        // SNTP đã được init trước đó (singleton) — không phải lỗi
        ESP_LOGW(TAG, "SNTP đã được khởi tạo trước đó — bỏ qua");
        return ESP_OK;
    }
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "SNTP khởi tạo thành công — server='%s'", Cfg::NTP_SERVER_URL);
    return ESP_OK;
}

// ============================================================
// Public API
// ============================================================
esp_err_t NetworkManager::publishData(const AirData &data) {
    return publishJson("data", data);
}

esp_err_t NetworkManager::publishAlert(const AirData &data) {
    return publishJson("alert", data);
}

esp_err_t NetworkManager::publishJson(const char *msg_type, const AirData &data) {
    if (!mqtt_connected_.load()) return ESP_ERR_INVALID_STATE;

    char json[Cfg::MQTT_JSON_BUF_LEN];
    size_t n = buildJson(data, msg_type, json, sizeof(json));
    if (n == 0) return ESP_FAIL;

    // QoS 1: đảm bảo broker nhận ít nhất 1 lần — nếu PUBACK mất, client retransmit
    // và subscriber có thể nhận duplicate. Dedup phía subscriber bằng (ts, client_id).
    int msg_id = esp_mqtt_client_publish(mqtt_, Cfg::MQTT_TOPIC_DATA, json,
                                          static_cast<int>(n), 1, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

bool NetworkManager::isConnected() const {
    return wifi_connected_.load() && mqtt_connected_.load();
}

bool NetworkManager::isTimeSynced() const {
    return s_sntp_synced.load();  // true sau khi sntpSyncCb() được gọi lần đầu
}

// Hàm này nhận các tham số là một đối tượng có thể gọi được (callable) như hàm, lambda, hoặc std::function.
// Nó cho phép người dùng đăng ký một callback để xử lý các lệnh nhận được từ broker qua MQTT_TOPIC_CMD.
void NetworkManager::setCommandCallback(CommandCallback cb) {   
    cmd_callback_ = std::move(cb);
}

// ============================================================
// Event handlers
// ============================================================
void NetworkManager::wifiEventHandler(void *arg, esp_event_base_t,
                                      int32_t id, void *event_data) {
    auto *self = static_cast<NetworkManager *>(arg);
    switch (id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "Wi-Fi khởi động — kết nối tới '%s'...", Cfg::WIFI_SSID);
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_CONNECTED: {
            auto *evt = static_cast<wifi_event_sta_connected_t *>(event_data);
            ESP_LOGI(TAG, "Wi-Fi kết nối thành công (SSID='%s', kênh=%d) — chờ IP...",
                     (char *)evt->ssid, evt->channel);
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED: {
            // Log reason giúp phân biệt: AUTH_FAIL (sai pass) vs AP_NOT_FOUND vs timeout
            auto *evt = static_cast<wifi_event_sta_disconnected_t *>(event_data);
            self->wifi_connected_.store(false);
            ESP_LOGW(TAG, "Wi-Fi mất kết nối (lí do=%d) — chờ kết nối lại", (int)evt->reason);
            esp_wifi_connect();  // reconnect không block; driver tự retry
            break;
        }

        default: break;
    }
}

void NetworkManager::ipEventHandler(void *arg, esp_event_base_t,
                                    int32_t id, void *event_data) {
    auto *self = static_cast<NetworkManager *>(arg);
    if (id == IP_EVENT_STA_GOT_IP) {
        auto *evt = static_cast<ip_event_got_ip_t *>(event_data);
        ESP_LOGI(TAG, "Có IP " IPSTR, IP2STR(&evt->ip_info.ip));
        self->wifi_connected_.store(true);
        // SNTP tự bắt đầu sync khi mạng available — esp_netif_sntp xử lý nội bộ
    }
}

void NetworkManager::mqttEventHandler(void *arg, esp_event_base_t,
                                      int32_t id, void *event_data) {
    auto *self = static_cast<NetworkManager *>(arg);
    auto *evt  = static_cast<esp_mqtt_event_handle_t>(event_data);
    switch (static_cast<esp_mqtt_event_id_t>(id)) {
        case MQTT_EVENT_CONNECTED: {
            self->mqtt_connected_.store(true);
            ESP_LOGI(TAG, "MQTT kết nối thành công (client_id='%s')", self->mqtt_client_id_);
            // Subscribe topic lệnh mỗi lần reconnect — broker không giữ subscription sau disconnect
            // ThingsBoard RPC: requestId thay đổi mỗi lần gọi → subscribe theo wildcard "<prefix>/+"
            char cmd_sub_topic[64];
            snprintf(cmd_sub_topic, sizeof(cmd_sub_topic), "%s/+", Cfg::MQTT_TOPIC_CMD);
            esp_mqtt_client_subscribe(self->mqtt_, cmd_sub_topic, 1);
            ESP_LOGI(TAG, "Đã subscribe topic lệnh '%s'", cmd_sub_topic);
            break;
        }

        case MQTT_EVENT_DISCONNECTED:
            self->mqtt_connected_.store(false);
            ESP_LOGW(TAG, "MQTT mất kết nối — client tự động kết nối lại nếu Wi-Fi sẵn sàng");
            break;

        case MQTT_EVENT_DATA: {
            if (!evt || !evt->topic || !evt->data) break;

            size_t cmd_prefix_len = std::strlen(Cfg::MQTT_TOPIC_CMD);
            if (static_cast<size_t>(evt->topic_len) <= cmd_prefix_len ||
                std::strncmp(evt->topic, Cfg::MQTT_TOPIC_CMD, cmd_prefix_len) != 0 ||
                evt->topic[cmd_prefix_len] != '/') {
                break;
            }

            // Copy payload vào buffer cục bộ để null-terminate
            char payload[64];
            size_t copy_len = std::min(static_cast<size_t>(evt->data_len), sizeof(payload) - 1);
            std::memcpy(payload, evt->data, copy_len);
            payload[copy_len] = '\0';

            cJSON *root = cJSON_Parse(payload);
            if (!root) {
                ESP_LOGW(TAG, "Lệnh nhận được không phải JSON hợp lệ: %s", payload);
                break;
            }

            // ThingsBoard RPC payload: {"method":"confirm_calib","params":{}}
            cJSON *cmd_item = cJSON_GetObjectItem(root, "method");
            if (cJSON_IsString(cmd_item)) {
                ESP_LOGI(TAG, "Nhận lệnh RPC từ ThingsBoard: '%s'", cmd_item->valuestring);
                self->dispatchCommand(cmd_item->valuestring);
            }
            cJSON_Delete(root);
            break;
        }

        case MQTT_EVENT_ERROR:
            if (evt && evt->error_handle) {
                // error_type: 0=TCP transport, 1=PAHO, 2=TLS, 3=DNS
                ESP_LOGE(TAG, "MQTT loại lỗi=%d", (int)evt->error_handle->error_type);
            }
            break;

        default: break;
    }
}

// ============================================================
// SNTP sync callback
// ============================================================
void NetworkManager::sntpSyncCb(struct timeval *) {
    // Được lwip gọi một lần khi sync xong — set flag vĩnh viễn.
    // time(NULL) từ đây trở đi trả Unix timestamp UTC hợp lệ.
    s_sntp_synced.store(true);
    ESP_LOGI(TAG, "SNTP đã đồng bộ — Unix time hợp lệ, time(NULL) sẵn sàng");
}

// ============================================================
// Command dispatch
// ============================================================
void NetworkManager::dispatchCommand(const char *cmd) {
    // Built-in: lệnh hệ thống xử lý trực tiếp tại đây.
    // Nghiệp vụ (confirm_calib, set_interval...): delegate sang cmd_callback_.
    if (std::strcmp(cmd, "reboot") == 0) {
        ESP_LOGW(TAG, "Thực thi lệnh reboot...");
        esp_restart();  // không trở về
    } else if (cmd_callback_) {
        cmd_callback_(cmd);
    } else {
        ESP_LOGW(TAG, "Lệnh '%s' không có handler — đăng ký setCommandCallback()", cmd);
    }
}

// ============================================================
// JSON builder — cJSON_PrintPreallocated: không alloc heap động
// ============================================================
size_t NetworkManager::buildJson(const AirData &data, const char *msg_type, char *out, size_t out_sz) {
    static const char *AQI_LABELS[]     = {"Tốt", "Trung bình", "Kém", "Xấu", "Rất xấu", "Nguy hại"};
    static const char *COMFORT_LABELS[] = {"Dễ chịu", "Hơi nóng", "Nóng khó chịu", "Rất khó chịu", "Stress nhiệt", "Cấp cứu"};
    static const char *CO2_LABELS[]     = {"Tốt", "Trung bình", "Xấu"};

    cJSON *root = cJSON_CreateObject();
    if (!root) return 0;  // heap cạn kiệt — caller log và xử lý

    // --- Metadata ---
    // msg_type: "data" (publishData, periodic) hoặc "alert" (publishAlert,
    // edge-triggered khi alert_level đổi mức) — cả 2 cùng publish lên
    // Cfg::MQTT_TOPIC_DATA, field này là cách DUY NHẤT để dashboard phân biệt
    // lại 2 luồng vì ThingsBoard gộp telemetry theo (device, key, ts), không
    // giữ lại topic MQTT gốc.
    cJSON_AddStringToObject(root, "msg_type", msg_type);
    cJSON_AddNumberToObject(root, "ts",      (double)data.timestamp);
    cJSON_AddBoolToObject  (root, "valid",   data.data_valid);
    // cycle_ms: thời gian taskSensor xử lý chu kỳ này — cho dashboard giám sát
    // NFR pipeline ≤300ms (CLAUDE.md §3) mà không cần đọc log debug UART.
    cJSON_AddNumberToObject(root, "cycle_ms", data.cycle_time_ms);

    // --- BME680 ---
    cJSON_AddNumberToObject(root, "temp",    data.temperature);
    cJSON_AddNumberToObject(root, "humi",    data.humidity);
    cJSON_AddNumberToObject(root, "pres",    data.pressure);

    // --- PMS5003 ---
    cJSON_AddNumberToObject(root, "pm1",     data.pm1_0);
    cJSON_AddNumberToObject(root, "pm25",    data.pm2_5);
    cJSON_AddNumberToObject(root, "pm10",    data.pm10);

    // --- MQ-135 ---
    cJSON_AddNumberToObject(root, "co2",     data.co2_ppm);

    // --- Chỉ số tính toán (DataFusion) ---
    cJSON_AddNumberToObject(root, "aqi",         data.aqi);
    cJSON_AddNumberToObject(root, "comfort",     data.comfort_index);

    if (data.pms5003_ready && data.aqi_category < 6)
        cJSON_AddStringToObject(root, "aqi_cat",     AQI_LABELS[data.aqi_category]);
    if (data.bme680_ready && data.comfort_category < 6)
        cJSON_AddStringToObject(root, "comfort_cat", COMFORT_LABELS[data.comfort_category]);
    if (data.mq135_ready && data.co2_category < 3)
        cJSON_AddStringToObject(root, "co2_cat",     CO2_LABELS[data.co2_category]);

    // --- Trạng thái sẵn sàng cảm biến ---
    cJSON_AddBoolToObject(root, "bme680_ok",  data.bme680_ready);
    cJSON_AddBoolToObject(root, "pms_ok",     data.pms5003_ready);
    cJSON_AddBoolToObject(root, "mq135_ok",   data.mq135_ready);

    // --- Cảnh báo vượt ngưỡng (DataFusion::computeAlertLevel) ---
    // alert_level: 0=NONE 1=WARNING 2=CRITICAL; alert_reason mô tả lý do CAO
    // NHẤT trong chu kỳ (AQI_HAZARDOUS/COMFORT_DANGER/CO2_CRITICAL/CALIB_DRIFT_*/
    // CALIB_OVERDUE_30D/NONE). Gửi ở MỌI payload (không chỉ alert) để dashboard
    // hiển thị mức độ nguy hiểm real-time.
    // alert_flags: bitmask FLAG_* (DataFusion.hpp) BỔ SUNG cho alert_reason —
    // set ĐỘC LẬP cho từng điều kiện đang active, cho phép dashboard phát
    // hiện NHIỀU điều kiện CRITICAL/WARNING xảy ra ĐỒNG THỜI (ví dụ AQI và
    // Comfort cùng CRITICAL trong 1 chu kỳ) mà alert_reason không thể hiện hết.
    cJSON_AddNumberToObject(root, "alert_level",  data.alert_level);
    cJSON_AddStringToObject(root, "alert_reason", data.alert_reason);
    cJSON_AddNumberToObject(root, "alert_flags",  data.alert_flags);
    cJSON_AddNumberToObject(root, "alert_latency_ms", data.alert_latency_ms);

    // --- Hiệu chuẩn (Drift Self-Check) ---
    // calib_reason ĐỘC LẬP với alert_reason — luôn cụ thể (CALIB_DRIFT_TEMP/
    // PM25/.../CALIB_OVERDUE_30D hoặc NONE) khi calib_needed=true, ngay cả khi
    // alert_reason đang mang lý do AQI/Comfort/CO2 CRITICAL khác.
    cJSON_AddBoolToObject  (root, "calib_alert",  data.calib_needed);
    cJSON_AddStringToObject(root, "calib_reason", data.calib_reason);
    cJSON_AddNumberToObject(root, "calib_ts",     (double)data.last_calib_timestamp);

    // fmt=0: compact JSON (không khoảng trắng) — tiết kiệm buffer và băng thông
    size_t n = 0;
    if (cJSON_PrintPreallocated(root, out, static_cast<int>(out_sz), 0)) {
        n = std::strlen(out);
    }
    cJSON_Delete(root);
    return n;
}

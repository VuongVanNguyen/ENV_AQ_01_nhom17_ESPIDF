#include "NetworkManager.hpp"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include <algorithm>
#include <cstring>
#include <cstdio>

static const char *TAG = "NetworkManager";

static std::atomic<bool> s_sntp_synced{false};

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
    esp_netif_sntp_deinit();

}

esp_err_t NetworkManager::init() {

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

esp_err_t NetworkManager::initWifi() {
    netif_ = esp_netif_create_default_wifi_sta();
    if (!netif_) return ESP_FAIL;

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) return err;

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

    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (err != ESP_OK) return err;
    err = esp_wifi_start();
    if (err != ESP_OK) return err;

    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    return ESP_OK;
}

esp_err_t NetworkManager::initMqtt() {

    uint8_t mac[6] = {};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(mqtt_client_id_, sizeof(mqtt_client_id_),
             "aq01_%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri    = Cfg::MQTT_BROKER_URL;
    cfg.credentials.client_id = mqtt_client_id_;

    if (Cfg::MQTT_ACCESS_TOKEN[0] != '\0') {
        cfg.credentials.username = Cfg::MQTT_ACCESS_TOKEN;
    }

    cfg.network.disable_auto_reconnect = false;
    cfg.network.reconnect_timeout_ms   = 3000;
    cfg.network.timeout_ms             = Cfg::MQTT_NETWORK_TIMEOUT_MS;

    cfg.session.keepalive              = Cfg::MQTT_KEEPALIVE_SEC;

    cfg.network.tcp_keep_alive_cfg.keep_alive_enable   = true;
    cfg.network.tcp_keep_alive_cfg.keep_alive_idle     = 20;
    cfg.network.tcp_keep_alive_cfg.keep_alive_interval = 5;
    cfg.network.tcp_keep_alive_cfg.keep_alive_count    = 3;

    cfg.outbox.limit = Cfg::MQTT_OUTBOX_LIMIT_BYTES;

    mqtt_ = esp_mqtt_client_init(&cfg);
    if (!mqtt_) return ESP_FAIL;

    esp_err_t err = esp_mqtt_client_register_event(
        mqtt_,
        static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
        &NetworkManager::mqttEventHandler,
        this);
    if (err != ESP_OK) return err;

    return esp_mqtt_client_start(mqtt_);
}

esp_err_t NetworkManager::initSntp() {

    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(Cfg::NTP_SERVER_URL);
    cfg.sync_cb = &NetworkManager::sntpSyncCb;

    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {

        ESP_LOGW(TAG, "SNTP đã được khởi tạo trước đó — bỏ qua");
        return ESP_OK;
    }
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "SNTP khởi tạo thành công — server='%s'", Cfg::NTP_SERVER_URL);
    return ESP_OK;
}

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

    int msg_id = esp_mqtt_client_enqueue(mqtt_, Cfg::MQTT_TOPIC_DATA, json,
                                         static_cast<int>(n), 1, 0, true);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

bool NetworkManager::isConnected() const {
    return wifi_connected_.load() && mqtt_connected_.load();
}

bool NetworkManager::isTimeSynced() const {
    return s_sntp_synced.load();
}

void NetworkManager::setCommandCallback(CommandCallback cb) {
    cmd_callback_ = std::move(cb);
}

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

            auto *evt = static_cast<wifi_event_sta_disconnected_t *>(event_data);
            self->wifi_connected_.store(false);
            ESP_LOGW(TAG, "Wi-Fi mất kết nối (lí do=%d) — chờ kết nối lại", (int)evt->reason);
            esp_wifi_connect();
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

        esp_netif_dns_info_t dns_backup = {};
        dns_backup.ip.type = ESP_IPADDR_TYPE_V4;
        dns_backup.ip.u_addr.ip4.addr = esp_ip4addr_aton(Cfg::WIFI_DNS_BACKUP);
        esp_netif_set_dns_info(self->netif_, ESP_NETIF_DNS_BACKUP, &dns_backup);
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

            const char *request_id = evt->topic + cmd_prefix_len + 1;
            size_t id_len = static_cast<size_t>(evt->topic_len) - cmd_prefix_len - 1;
            char request_id_buf[16] = {};
            std::memcpy(request_id_buf, request_id, std::min(id_len, sizeof(request_id_buf) - 1));

            char payload[64];
            size_t copy_len = std::min(static_cast<size_t>(evt->data_len), sizeof(payload) - 1);
            std::memcpy(payload, evt->data, copy_len);
            payload[copy_len] = '\0';

            cJSON *root = cJSON_Parse(payload);
            if (!root) {
                ESP_LOGW(TAG, "Lệnh nhận được không phải JSON hợp lệ: %s", payload);
                break;
            }

            cJSON *cmd_item = cJSON_GetObjectItem(root, "method");
            if (cJSON_IsString(cmd_item)) {
                ESP_LOGI(TAG, "Nhận lệnh RPC từ ThingsBoard: '%s'", cmd_item->valuestring);
                self->dispatchCommand(cmd_item->valuestring);

                char resp_topic[64];
                snprintf(resp_topic, sizeof(resp_topic), "v1/devices/me/rpc/response/%s", request_id_buf);
                esp_mqtt_client_enqueue(self->mqtt_, resp_topic, "{\"result\":\"ok\"}", 0, 1, 0, true);
            }
            cJSON_Delete(root);
            break;
        }

        case MQTT_EVENT_ERROR:
            if (evt && evt->error_handle) {

                ESP_LOGE(TAG,
                         "MQTT loại lỗi=%d sock_errno=%d esp_tls_last_esp_err=0x%x esp_tls_stack_err=%d",
                         (int)evt->error_handle->error_type,
                         evt->error_handle->esp_transport_sock_errno,
                         (unsigned)evt->error_handle->esp_tls_last_esp_err,
                         evt->error_handle->esp_tls_stack_err);
            }
            break;

        default: break;
    }
}

void NetworkManager::sntpSyncCb(struct timeval *) {

    s_sntp_synced.store(true);
    ESP_LOGI(TAG, "SNTP đã đồng bộ — Unix time hợp lệ, time(NULL) sẵn sàng");
}

void NetworkManager::dispatchCommand(const char *cmd) {

    if (cmd_callback_) {
        cmd_callback_(cmd);
    } else {
        ESP_LOGW(TAG, "Lệnh '%s' không có handler — đăng ký setCommandCallback()", cmd);
    }
}

size_t NetworkManager::buildJson(const AirData &data, const char *msg_type, char *out, size_t out_sz) {
    static const char *AQI_LABELS[]     = {"Tốt", "Trung bình", "Kém", "Xấu", "Rất xấu", "Nguy hại"};
    static const char *COMFORT_LABELS[] = {"Dễ chịu", "Hơi nóng", "Nóng khó chịu", "Rất khó chịu", "Stress nhiệt", "Cấp cứu"};
    static const char *CO2_LABELS[]     = {"Tốt", "Trung bình", "Xấu"};

    cJSON *root = cJSON_CreateObject();
    if (!root) return 0;
    constexpr int64_t EPOCH_2020 = 1577836800LL;
    cJSON *vals;
    if (data.timestamp > EPOCH_2020) {

        cJSON_AddNumberToObject(root, "ts", (double)(data.timestamp * 1000LL));
        vals = cJSON_CreateObject();
        if (!vals) { cJSON_Delete(root); return 0; }
        cJSON_AddItemToObject(root, "values", vals);
    } else {
        vals = root;
    }

    cJSON_AddStringToObject(vals, "msg_type", msg_type);

    cJSON_AddBoolToObject  (vals, "valid",    data.data_valid);

    cJSON_AddNumberToObject(vals, "cycle_ms", data.cycle_time_ms);

    cJSON_AddNumberToObject(vals, "temp",    data.temperature);
    cJSON_AddNumberToObject(vals, "humi",    data.humidity);
    cJSON_AddNumberToObject(vals, "pres",    data.pressure);

    cJSON_AddNumberToObject(vals, "pm1",     data.pm1_0);
    cJSON_AddNumberToObject(vals, "pm25",    data.pm2_5);
    cJSON_AddNumberToObject(vals, "pm10",    data.pm10);

    cJSON_AddNumberToObject(vals, "co2",     data.co2_ppm);

    cJSON_AddNumberToObject(vals, "aqi",         data.aqi);
    cJSON_AddNumberToObject(vals, "comfort",     data.comfort_index);

    if (data.pms5003_ready && data.aqi_category < 6)
        cJSON_AddStringToObject(vals, "aqi_cat",     AQI_LABELS[data.aqi_category]);
    if (data.bme680_ready && data.comfort_category < 6)
        cJSON_AddStringToObject(vals, "comfort_cat", COMFORT_LABELS[data.comfort_category]);
    if (data.mq135_ready && data.co2_category < 3)
        cJSON_AddStringToObject(vals, "co2_cat",     CO2_LABELS[data.co2_category]);

    cJSON_AddBoolToObject(vals, "bme680_ok",  data.bme680_ready);
    cJSON_AddBoolToObject(vals, "pms_ok",     data.pms5003_ready);
    cJSON_AddBoolToObject(vals, "mq135_ok",   data.mq135_ready);

    cJSON_AddNumberToObject(vals, "alert_level",      data.alert_level);
    cJSON_AddStringToObject(vals, "alert_reason",     data.alert_reason);
    cJSON_AddNumberToObject(vals, "alert_flags",      data.alert_flags);

    cJSON_AddNumberToObject(vals, "alert_latency_ms", data.alert_latency_ms);

    cJSON_AddBoolToObject  (vals, "calib_alert",  data.calib_needed);
    cJSON_AddStringToObject(vals, "calib_reason", data.calib_reason);

    cJSON_AddNumberToObject(vals, "calib_ts",     (double)data.last_calib_timestamp);

    size_t n = 0;
    if (cJSON_PrintPreallocated(root, out, static_cast<int>(out_sz), 0)) {
        n = std::strlen(out);
    }
    cJSON_Delete(root);
    return n;
}

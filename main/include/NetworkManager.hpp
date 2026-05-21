#pragma once

// ============================================================
// NetworkManager.hpp
//   Quản lý kết nối Wi-Fi STA + MQTT client + SNTP time sync.
//
//   Luồng hoạt động (event-driven, không polling):
//     esp_wifi_start()  → WIFI_EVENT_STA_START  → esp_wifi_connect()
//     Got IP            → IP_EVENT_STA_GOT_IP   → wifi_connected_=true
//     Broker chấp nhận  → MQTT_EVENT_CONNECTED  → mqtt_connected_=true
//     SNTP sync xong    → sntpSyncCb()          → s_sntp_synced=true
//
//   isConnected() → taskNetwork quyết định buffer xuống SD (StorageHelper)
//   isTimeSynced() → DataFusion/main biết khi nào time(NULL) là Unix time thật
//
//   Mọi credential/endpoint lấy từ namespace Cfg (config.hpp),
//   sinh từ Kconfig.projbuild — không hardcode trong source.
// ============================================================

#include "DataStructures.hpp"
#include "config.hpp"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "mqtt_client.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include <atomic>
#include <functional>

// Callback được gọi khi broker gửi lệnh xuống qua MQTT_TOPIC_CMD.
// `cmd`: chuỗi từ field "cmd" trong JSON payload (ví dụ: "confirm_calib").
// Được đăng ký bởi DataFusion/main — không gọi trực tiếp từ NetworkManager.
using CommandCallback = std::function<void(const char *cmd)>;

class NetworkManager {
public:
    NetworkManager();
    ~NetworkManager();

    NetworkManager(const NetworkManager&)            = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;

    // Khởi tạo Wi-Fi STA, MQTT client và SNTP.
    // Không block — toàn bộ kết nối xử lý ngầm qua event handler;
    // hàm trả về ngay sau khi đăng ký handler và start driver.
    esp_err_t init();

    // Serialize AirData → JSON và publish lên topic dữ liệu chính.
    // Trả ESP_ERR_INVALID_STATE nếu MQTT chưa connected — caller
    // (taskNetwork) sẽ buffer xuống SD card qua StorageHelper.
    esp_err_t publishData(const AirData &data);

    // Publish cảnh báo (calib drift / ngưỡng vượt mức) lên topic alert.
    // `reason`: chuỗi mô tả lý do cảnh báo (ví dụ "CALIB_DRIFT_TEMP").
    esp_err_t publishAlert(const AirData &data, const char *reason);

    // Đăng ký callback nhận lệnh từ broker (topic MQTT_TOPIC_CMD).
    // Gọi trước init(). Callback chạy trong context của MQTT event task —
    // nếu cần ghi NVS hay thay đổi shared state, dùng mutex hoặc queue.
    void setCommandCallback(CommandCallback cb);

    // True ⇔ cả Wi-Fi (đã có IP) lẫn MQTT đều đang connected.
    bool isConnected() const;

    // True ⇔ SNTP đã đồng bộ ít nhất một lần — time(NULL) trả Unix time hợp lệ.
    // DataFusion/main dùng để quyết định khi nào ghi đè AirData.timestamp.
    bool isTimeSynced() const;

private:
    esp_netif_t              *netif_;
    esp_mqtt_client_handle_t  mqtt_;
    EventGroupHandle_t        evt_group_;

    // Atomic: được set từ ESP event callback (có thể chạy trên core khác).
    std::atomic<bool> wifi_connected_;
    std::atomic<bool> mqtt_connected_;

    CommandCallback cmd_callback_;

    // Client ID = "aq01_<MAC WiFi STA>" — duy nhất mỗi thiết bị,
    // tránh xung đột broker khi nhiều trạm AQ01 kết nối cùng lúc.
    char mqtt_client_id_[24];

    esp_err_t initWifi();
    esp_err_t initMqtt();
    esp_err_t initSntp();  // khởi tạo SNTP sau khi netif + event loop sẵn sàng

    // Xử lý lệnh nhận từ broker: built-in commands (reboot...) trước,
    // sau đó delegate sang cmd_callback_ cho các lệnh nghiệp vụ (confirm_calib...).
    void dispatchCommand(const char *cmd);

    // Static handlers: esp_event yêu cầu free function signature;
    // `arg` = `this` để truy cập state instance.
    static void wifiEventHandler(void *arg, esp_event_base_t base,
                                 int32_t id, void *event_data);
    static void ipEventHandler  (void *arg, esp_event_base_t base,
                                 int32_t id, void *event_data);
    static void mqttEventHandler(void *arg, esp_event_base_t base,
                                 int32_t id, void *event_data);

    // Callback SNTP: gọi bởi lwip khi lần đầu đồng bộ thành công.
    // esp_sntp_time_cb_t = void(*)(struct timeval*) — compatible với static method.
    static void sntpSyncCb(struct timeval *tv);

    // Serialize AirData → JSON compact (không alloc heap — dùng cJSON_PrintPreallocated).
    // Trả số byte ghi vào `out` (không kèm '\0') hoặc 0 nếu lỗi/buffer nhỏ.
    static size_t buildJson(const AirData &data, char *out, size_t out_sz,
                            const char *alert_reason = nullptr);
};

#pragma once

#include "DataStructures.hpp"
#include "config.hpp"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "mqtt_client.h"

#include <atomic>
#include <functional>

using CommandCallback = std::function<void(const char *cmd)>;

class NetworkManager {
public:
    NetworkManager();
    ~NetworkManager();

    NetworkManager(const NetworkManager&)            = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;

    esp_err_t init();

    esp_err_t publishData(const AirData &data);

    esp_err_t publishAlert(const AirData &data);

    void setCommandCallback(CommandCallback cb);

    bool isConnected() const;

    bool isTimeSynced() const;

private:
    esp_netif_t              *netif_;
    esp_mqtt_client_handle_t  mqtt_;

    std::atomic<bool> wifi_connected_;
    std::atomic<bool> mqtt_connected_;

    char mqtt_client_id_[24];

    CommandCallback cmd_callback_;

    esp_err_t initWifi();
    esp_err_t initMqtt();
    esp_err_t initSntp();

    void dispatchCommand(const char *cmd);

    static void wifiEventHandler(void *arg, esp_event_base_t base,
                                 int32_t id, void *event_data);
    static void ipEventHandler  (void *arg, esp_event_base_t base,
                                 int32_t id, void *event_data);
    static void mqttEventHandler(void *arg, esp_event_base_t base,
                                 int32_t id, void *event_data);

    static void sntpSyncCb(struct timeval *tv);

    static size_t buildJson(const AirData &data, const char *msg_type, char *out, size_t out_sz);

    esp_err_t publishJson(const char *msg_type, const AirData &data);
};

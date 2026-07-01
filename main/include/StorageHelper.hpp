#pragma once

#include "DataStructures.hpp"
#include "config.hpp"

#include "esp_err.h"
#include "sdmmc_cmd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <cstdint>
#include <cstddef>
#include <functional>

using PublishFn = std::function<esp_err_t(const AirData &)>;

class StorageHelper {
public:

    enum class EventType : uint8_t {
        ALERT_LEVEL_CHANGED  = 0,
        ALERT_REASON_CHANGED = 1,
        CALIB_NEEDED_SET     = 2,
        CALIB_CONFIRMED      = 3,
        MQTT_CONNECTED       = 4,
        MQTT_DISCONNECTED    = 5,
        SYSTEM_BOOT          = 6,
        SD_LOG_ROTATED       = 7,
        ALERT_LATENCY_EXCEEDED = 8,

        WARMUP_SKIPPED = 9,

    };

    StorageHelper();
    ~StorageHelper();

    StorageHelper(const StorageHelper&)            = delete;
    StorageHelper& operator=(const StorageHelper&) = delete;

    esp_err_t init();

    esp_err_t logData(const AirData &data);

    esp_err_t logEvent(EventType type, const AirData &snapshot);

    esp_err_t bufferOffline(const AirData &data);

    esp_err_t drainOffline(PublishFn publish_fn);

    bool   isMounted() const;
    size_t offlineCount() const;

private:

    sdmmc_card_t *card_;
    QueueHandle_t storage_queue_;
    TaskHandle_t  storage_task_;
    volatile bool mounted_;
    size_t        offline_count_;
    uint32_t      dropped_data_;
    uint32_t      dropped_event_;

    long      data_file_bytes_;
    long      event_file_bytes_;
    uint32_t  data_rows_pending_fsync_;
    PublishFn drain_publish_fn_;
    int64_t   last_log_timestamp_;

    static void taskStorage(void *arg);
    void taskLoop();

    esp_err_t mountCard();

    esp_err_t ensureHeader(const char *path, const char *header, long &size_out);
    esp_err_t writeDataRow(const AirData &data);
    esp_err_t writeEventRow(EventType type, const AirData &snapshot);
    esp_err_t rotateDataLog(const AirData &data);

    esp_err_t pushOfflineRecord(const AirData &data);
    esp_err_t drainOfflineRecords(const PublishFn &publish_fn);
    esp_err_t restoreOfflineState();
    esp_err_t readOfflineHead(uint32_t &head_index) const;
    esp_err_t writeOfflineHead(uint32_t head_index) const;
    esp_err_t compactOfflineQueue(uint32_t head, long total_records);

    static size_t      csvEscape(const char *in, char *out, size_t out_sz);
    static int         timeValidOf(int64_t timestamp);
    static const char *eventTypeName(EventType type);
};

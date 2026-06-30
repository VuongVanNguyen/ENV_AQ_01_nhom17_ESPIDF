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

// Callback phát lại 1 record offline — trỏ tới NetworkManager::publishData()
// (NetworkManager.hpp §1). Truyền vào drainOffline() để StorageHelper KHÔNG
// #include NetworkManager.hpp (tránh phụ thuộc vòng, §1/§8, giống triết lý
// DataFusion.hpp §9).
using PublishFn = std::function<esp_err_t(const AirData &)>;

// StorageHelper — sink CHỈ-ĐỌC của pipeline (§0): log CSV định kỳ (§3),
// event log rời rạc (§4), offline buffer bền vững trên SD (§5), trên MỘT
// task riêng (§6). Non-copyable như SensorManager/NetworkManager/DataFusion.
class StorageHelper {
public:
    // §4.1 — sự kiện rời rạc (edge-triggered) ghi vào SD_EVENT_LOG_FILE
    // qua logEvent(). Nested public: caller dùng StorageHelper::EventType::*.
    enum class EventType : uint8_t {
        ALERT_LEVEL_CHANGED  = 0, // alert_level đổi (NONE<->WARNING<->CRITICAL)
        ALERT_REASON_CHANGED = 1, // alert_reason / alert_flags đổi
        CALIB_NEEDED_SET     = 2, // calib_needed false->true (kèm calib_reason)
        CALIB_CONFIRMED      = 3, // confirm_calib OK (DataFusion::confirmRecalibration, §4.4)
        MQTT_CONNECTED       = 4, // NetworkManager::isConnected() false->true
        MQTT_DISCONNECTED    = 5, // NetworkManager::isConnected() true->false
        SYSTEM_BOOT          = 6, // mốc khởi động — phục vụ truy vết
        SD_LOG_ROTATED       = 7, // airdata.csv vừa xoay vòng (§3.6)
        ALERT_LATENCY_EXCEEDED = 8, // alert_latency_ms > Cfg::ALERT_MAX_LATENCY_MS
                                    // (taskNetwork/main.cpp đo, snapshot.alert_reason
                                    // cho biết alert nào bị trễ — CLAUDE.md §3)
        WARMUP_SKIPPED = 9,         // lệnh MQTT "skip_warmup" — force *_ready = true
                                    // toàn pipeline (LCD + DataFusion + MQTT dashboard)
                                    // khi sensor vật lý đã warm nhưng timer bị reset.
    };

    StorageHelper();
    ~StorageHelper();

    StorageHelper(const StorageHelper&)            = delete;
    StorageHelper& operator=(const StorageHelper&) = delete;

    // §1 — gọi ĐÚNG 1 LẦN ở app_main(), SAU nvs_flash_init(). Mount SD (§2),
    // tạo storage_queue_ + storage_task_ (§6), ensureHeader cho 2 file CSV
    // (§3.2/§4.2), khôi phục offline_count_ từ SD_OFFLINE_QUEUE (§5.4).
    // Trả lỗi (KHÔNG ESP_ERROR_CHECK ở caller) nếu SD vắng/hỏng — module vào
    // chế độ SD_ABSENT (§13), KHÔNG làm sập pipeline.
    esp_err_t init();

    // §1/§3 — non-blocking: enqueue {kind=DATA, payload=data} (xQueueSend
    // timeout 0). Trả ESP_ERR_TIMEOUT/ESP_FAIL nếu queue đầy (drop + đếm
    // dropped_data_, §6.4/§11) hoặc SD_ABSENT.
    esp_err_t logData(const AirData &data);

    // §1/§4 — non-blocking: enqueue {kind=EVENT, evt=type, payload=snapshot}.
    // CHỦ THỂ gọi (pipeline/taskNetwork/cmd_callback) chịu trách nhiệm phát
    // hiện edge (§4.3) — StorageHelper chỉ ghi.
    esp_err_t logEvent(EventType type, const AirData &snapshot);

    // §1/§5.2 — non-blocking: enqueue {kind=OFFLINE_PUSH, payload=data}.
    // Gọi từ taskNetwork khi NetworkManager::publishData()/publishAlert()
    // trả ESP_ERR_INVALID_STATE (MQTT chưa connected).
    esp_err_t bufferOffline(const AirData &data);

    // §1/§5.3 — gọi từ taskNetwork NGAY khi isConnected() chuyển false->true.
    // publish_fn = wrapper gọi NetworkManager::publishData (bind tại call
    // site trong main.cpp, §8) — enqueue {kind=OFFLINE_DRAIN}; storageTask
    // phát lại tuần tự theo lô (Cfg::OFFLINE_DRAIN_BATCH, §5.3).
    esp_err_t drainOffline(PublishFn publish_fn);

    bool   isMounted() const;     // §1 — thẻ SD đã mount thành công? (SD_ABSENT, §13)
    size_t offlineCount() const;  // §1 — số record tồn trong offline queue (§5.4)

private:
    // ---- Trạng thái SD/queue/task (§1 — truy cập từ nhiều task, volatile/atomic-ish) ----
    sdmmc_card_t *card_;            // handle thẻ (sdspi mount, §2)
    QueueHandle_t storage_queue_;   // hàng lệnh tới storageTask (§6.1)
    TaskHandle_t  storage_task_;    // task ghi SD duy nhất (§6.2)
    volatile bool mounted_;         // trạng thái mount — SD_ABSENT khi false (§13)
    size_t        offline_count_;   // số record offline hiện tại (§5)
    uint32_t      dropped_data_;    // queue-full khi logData/bufferOffline (§6.4/§11)
    uint32_t      dropped_event_;   // queue-full khi logEvent (§6.4/§11)

    // ---- Trạng thái nội bộ storageTask (CHỈ truy cập trong storageTask — §6.3) ----
    long      data_file_bytes_;          // kích thước hiện tại SD_LOG_FILE (§3.6)
    long      event_file_bytes_;         // kích thước hiện tại SD_EVENT_LOG_FILE (§4.5)
    uint32_t  data_rows_pending_fsync_;  // số dòng chưa fsync (§3.5)
    PublishFn drain_publish_fn_;         // publish_fn gần nhất từ drainOffline() (§5.3)
    int64_t   last_log_timestamp_;       // AirData::timestamp của dòng airdata.csv gần nhất
                                          // — gating nhịp ghi theo SD_LOG_INTERVAL_MS (§3.1)

    // ---- Thân task (§6) ----
    static void taskStorage(void *arg);
    void taskLoop();

    // ---- Mount SD qua SPI (§2) ----
    esp_err_t mountCard();

    // ---- CSV: log dữ liệu (§3) + event log (§4) ----
    esp_err_t ensureHeader(const char *path, const char *header, long &size_out);
    esp_err_t writeDataRow(const AirData &data);
    esp_err_t writeEventRow(EventType type, const AirData &snapshot);
    esp_err_t rotateDataLog(const AirData &data);  // §3.6 — vượt SD_LOG_MAX_BYTES

    // ---- Offline buffer (§5) ----
    esp_err_t pushOfflineRecord(const AirData &data);              // §5.2
    esp_err_t drainOfflineRecords(const PublishFn &publish_fn);    // §5.3
    esp_err_t restoreOfflineState();                               // §5.4 (gọi từ init)
    esp_err_t readOfflineHead(uint32_t &head_index) const;         // §5.5
    esp_err_t writeOfflineHead(uint32_t head_index) const;         // §5.5
    esp_err_t compactOfflineQueue(uint32_t head, long total_records); // §5.5 — nén khi head lớn

    // ---- Tiện ích chung (§3.4 CSV-safe, §7 time_valid) ----
    static size_t      csvEscape(const char *in, char *out, size_t out_sz);
    static int         timeValidOf(int64_t timestamp);
    static const char *eventTypeName(EventType type);
};
// ============================================================================

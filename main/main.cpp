// ============================================================
// main.cpp — Entry point dự án ENV-AQ-01
// ============================================================

#include "sdkconfig.h"
#include "config.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "SensorManager.hpp"
#include "DataStructures.hpp"
#include "Filters.hpp"
#include "DataFusion.hpp"
#include "DisplayManager.hpp"
#include "NetworkManager.hpp"
#include "StorageHelper.hpp"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstring>
#include <ctime>

static const char *TAG = "main";

// ============================================================
// Instance toàn cục, hạ tầng đồng bộ, FreeRTOS task
// (CLAUDE.md §2/§3/§4 — xem app_main() phía dưới cho thứ tự khởi tạo)
// ============================================================

static SensorManager  s_sensor;
static Filters        s_filters;
static DataFusion     s_fusion;
static DisplayManager s_display;
static NetworkManager s_network;
static StorageHelper  s_storage;

// shared_data — nguồn ghi/sửa của pipeline + cmd_callback (XM-7), bảo vệ bởi s_shared_mtx.
static AirData            s_shared{};
static SemaphoreHandle_t  s_shared_mtx = nullptr;
// airDataQueue length-1 — snapshot chỉ-đọc cho taskDisplay/taskNetwork (XM-9).
static QueueHandle_t      s_airq = nullptr;

// Notice mất/khôi phục mạng — taskNetwork ghi (xQueueOverwrite), CHỈ taskDisplay
// đọc (xQueueReceive, tiêu thụ) rồi tự gọi s_display.showMessage() (XM-15).
// DisplayManager không có mutex nội bộ nên KHÔNG được gọi trực tiếp từ
// taskNetwork (race với update()/tick() của taskDisplay) — hàng đợi length-1
// này giữ nguyên tắc "chỉ 1 task duy nhất chạm vào s_display".
enum class DisplayNotice : uint8_t { NET_LOST = 0, NET_RESTORED = 1 };
static QueueHandle_t      s_display_notice = nullptr;

// Lệnh MQTT "skip_warmup" — cmd_callback (context MQTT event task) set cờ này,
// taskSensor đọc mỗi chu kỳ và override *_ready = true trong AirData local trước
// khi Filters/DataFusion nhận. Dùng atomic để tránh race không cần mutex/queue.
// Tự tắt khi SensorManager::isFullyReady() trả true (timer thật đã hết).
static std::atomic<bool>  s_force_skip_warmup{false};

// Nguồn thời gian thực dùng chung cho main.cpp (đồng nhất DataFusion.hpp §7 /
// StorageHelper.hpp §7): Unix time nếu SNTP đã sync, ngược lại giây-từ-boot.
static int64_t nowUnixOrUptime() {
    if (s_network.isTimeSynced()) {
        return static_cast<int64_t>(time(nullptr));
    }
    return esp_timer_get_time() / 1000000LL;
}

// [G/XM-12/XM-13] main SỞ HỮU GPIO — DataFusion không chạm GPIO (DataFusion.hpp §8).
// Lái LED/Buzzer trực tiếp từ data.alert_level (0=NONE 1=WARNING 2=CRITICAL).
static void driveAlertOutputs(const AirData &data) {
    const auto level = static_cast<DataFusion::AlertLevel>(data.alert_level);
    gpio_set_level(static_cast<gpio_num_t>(Cfg::LED_GREEN_PIN),
                    static_cast<uint32_t>(level == DataFusion::AlertLevel::NONE));
    gpio_set_level(static_cast<gpio_num_t>(Cfg::LED_YELLOW_PIN),
                    static_cast<uint32_t>(level == DataFusion::AlertLevel::WARNING));
    gpio_set_level(static_cast<gpio_num_t>(Cfg::LED_RED_PIN),
                    static_cast<uint32_t>(level == DataFusion::AlertLevel::CRITICAL));
    gpio_set_level(static_cast<gpio_num_t>(Cfg::BUZZER_PIN),
                    static_cast<uint32_t>(level == DataFusion::AlertLevel::CRITICAL));
}

// Gắn esp_timer định kỳ → đánh thức task qua xTaskNotifyGive (H.1/H.2/H.3 — CLAUDE.md
// §4 Non-blocking: KHÔNG vTaskDelay làm logic nghiệp vụ). Callback chạy trong context
// task dịch vụ của esp_timer (dispatch mặc định ESP_TIMER_TASK) — gọi xTaskNotifyGive
// thẳng, không cần biến thể "FromISR".
static esp_err_t startPeriodicNotifier(esp_timer_handle_t &timer, const char *name,
                                        TaskHandle_t task, uint32_t period_ms) {
    esp_timer_create_args_t args{};
    args.callback = [](void *arg) { xTaskNotifyGive(static_cast<TaskHandle_t>(arg)); };
    args.arg  = task;
    args.name = name;

    esp_err_t err = esp_timer_create(&args, &timer);
    if (err != ESP_OK) return err;
    return esp_timer_start_periodic(timer, static_cast<uint64_t>(period_ms) * 1000ULL);
}

// ============================================================
// taskSensor — PIPELINE THỜI GIAN THỰC (≤ Cfg::MAX_CYCLE_TIME_MS, H.1)
//   SensorManager → Filters → DataFusion → shared_data/airDataQueue → GPIO cảnh báo
// ============================================================
static void taskSensor(void *) {
    esp_task_wdt_add(nullptr);

    bool prev_calib_needed = false;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const int64_t t0 = esp_timer_get_time();

        AirData local{};

        // (1) Đọc thô — readAll() tự zero-init + set cờ readiness/warmup.
        if (s_sensor.readAll(local) != ESP_OK) {
            ESP_LOGW(TAG, "SensorManager::readAll lỗi trong chu kỳ này");
        }

        // (1b) Override skip_warmup: force *_ready = true toàn pipeline khi lệnh MQTT
        // "skip_warmup" được gửi (sensor vật lý đã ổn định, chỉ timer phần mềm bị reset
        // do rút/cắm điện). Cờ tự tắt khi timer thật hết hạn — không cần lệnh tắt riêng.
        if (s_force_skip_warmup.load()) {
            local.bme680_ready  = true;
            local.pms5003_ready = true;
            local.mq135_ready   = true;
            if (s_sensor.isFullyReady()) {
                s_force_skip_warmup.store(false);
            }
        }

        // (2) Lọc nhiễu in-place (sanity range + EMA/SMA).
        s_filters.process(local);

        // (3) Hợp nhất dữ liệu: AQI / Comfort(THI) / CO2 cat, drift self-check,
        //     computeAlertLevel — XM-5: main lấy isTimeSynced() rồi truyền vào.
        const bool synced = s_network.isTimeSynced();
        s_fusion.process(local, synced);

        // (4) Cập nhật shared_data dưới mutex (XM-7).
        xSemaphoreTake(s_shared_mtx, portMAX_DELAY);
        s_shared = local;
        xSemaphoreGive(s_shared_mtx);

        const int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
        local.cycle_time_ms = static_cast<uint16_t>(
            std::min<int64_t>(elapsed_ms, UINT16_MAX));
        if (elapsed_ms > static_cast<int64_t>(Cfg::MAX_CYCLE_TIME_MS)) {
            ESP_LOGW(TAG, "Chu kỳ pipeline vượt ngân sách: %lldms > %ums",
                     (long long)elapsed_ms, (unsigned)Cfg::MAX_CYCLE_TIME_MS);
        }

        // (5) Publish snapshot cho taskDisplay/taskNetwork — SAU khi unlock (XM-9).
        xQueueOverwrite(s_airq, &local);

        // (6) Lái LED/Buzzer ngay tại đây để đạt độ trễ cảnh báo ≤3s (XM-14).
        driveAlertOutputs(local);

        // (7) Edge calib_needed false→true → Event Log (StorageHelper.hpp §4.3).
        if (local.calib_needed && !prev_calib_needed) {
            s_storage.logEvent(StorageHelper::EventType::CALIB_NEEDED_SET, local);
        }
        prev_calib_needed = local.calib_needed;

        // (8) Enqueue log dữ liệu định kỳ — StorageHelper tự gating theo
        //     Cfg::SD_LOG_INTERVAL_MS (StorageHelper.cpp writeDataRow §3.1),
        //     pipeline chỉ enqueue O(1), KHÔNG ghi SD mỗi 300ms.
        s_storage.logData(local);

        // (9) Feed watchdog.
        esp_task_wdt_reset();
    }
}

// ============================================================
// taskDisplay — LCD, cadence [LCD_MIN_INTERVAL_MS, LCD_MAX_INTERVAL_MS] (H.2)
// ============================================================
static void taskDisplay(void *) {
    esp_task_wdt_add(nullptr);

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // XM-15: notice mạng (taskNetwork → đây) ưu tiên trong chu kỳ này —
        // tiêu thụ (Receive, không Peek) để chỉ hiển thị đúng 1 lần mỗi edge.
        // showMessage() tự set overlay Cfg::LCD_OVERLAY_MIN_MS nên update()
        // các chu kỳ kế tiếp tự động bỏ qua cho tới khi overlay hết hạn —
        // không cần thêm cờ gì ở taskDisplay. Non-blocking (timeout 0) nên
        // không vi phạm WDT/non-blocking rule (CLAUDE.md §4).
        DisplayNotice notice;
        if (xQueueReceive(s_display_notice, &notice, 0) == pdTRUE) {
            if (notice == DisplayNotice::NET_LOST) {
                s_display.showMessage("NETWORK LOST", "Saving to SD");
            } else {
                s_display.showMessage("NETWORK OK", "Syncing...");
            }
        }

        // Snapshot chỉ-đọc (XM-9). Trước khi taskSensor có chu kỳ đầu tiên,
        // queue rỗng → snap giữ AirData{} mặc định (mọi *_ready=false) →
        // DisplayManager tự hiển thị WARMING UP, đúng hành vi mong muốn.
        AirData snap{};
        xQueuePeek(s_airq, &snap, 0);

        s_display.update(snap);   // XM-1: vẽ trang hiện tại (no-op nếu overlay đang giữ)
        s_display.tick();         // XM-1: rotate trang / blink CALIB_ALERT

        esp_task_wdt_reset();
    }
}

// ============================================================
// taskNetwork — MQTT publish / alert debounce / offline buffer / event log (H.3)
// ============================================================
static void taskNetwork(void *) {
    esp_task_wdt_add(nullptr);

    static DataFusion::AlertLevel last_published_level = DataFusion::AlertLevel::NONE; // XM-11
    static bool     prev_connected   = false;                       // edge MQTT
    static uint8_t  prev_alert_level = 0;                            // edge Event Log §4.3
    static uint16_t prev_alert_flags = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        AirData snap{};
        xQueuePeek(s_airq, &snap, 0);

        // Tính sớm để publishData mang alert_latency_ms đúng. Khác 0 chỉ khi level
        // đổi mức (= "latency phát hiện → publish"); chu kỳ không đổi để 0.
        const auto level = static_cast<DataFusion::AlertLevel>(snap.alert_level);
        const bool level_changed = (level != last_published_level);
        snap.alert_latency_ms = level_changed
            ? static_cast<uint32_t>(
                  std::max<int64_t>(0, esp_timer_get_time() - snap.alert_level_changed_us) / 1000)
            : 0u;

        // (a) EDGE kết nối MQTT.
        const bool conn = s_network.isConnected();
        if (conn != prev_connected) {
            if (conn) {
                s_storage.logEvent(StorageHelper::EventType::MQTT_CONNECTED, snap);
                s_storage.drainOffline([](const AirData &d) { return s_network.publishData(d); });
                DisplayNotice notice = DisplayNotice::NET_RESTORED;     // XM-15
                xQueueOverwrite(s_display_notice, &notice);
            } else {
                s_storage.logEvent(StorageHelper::EventType::MQTT_DISCONNECTED, snap);
                DisplayNotice notice = DisplayNotice::NET_LOST;         // XM-15
                xQueueOverwrite(s_display_notice, &notice);
            }
            prev_connected = conn;
        }

        // (b) Publish dữ liệu — mất mạng (đã biết trước hoặc rớt giữa chừng)
        //     → Offline Buffer trên SD (CLAUDE.md §4 "Offline Buffer").
        if (conn) {
            esp_err_t pub_err = s_network.publishData(snap);
            if (pub_err == ESP_ERR_INVALID_STATE) {
                s_storage.bufferOffline(snap);
            } else if (pub_err != ESP_OK) {
                ESP_LOGW(TAG, "publishData thất bại (%s) — bỏ qua chu kỳ này",
                         esp_err_to_name(pub_err));
            }
        } else {
            s_storage.bufferOffline(snap);
        }

        // (c) Debounce publishAlert theo alert_level — đáp ứng NFR ≤ ALERT_MAX_LATENCY_MS (XM-11).
        if (level_changed) {
            if (snap.alert_latency_ms > Cfg::ALERT_MAX_LATENCY_MS) {
                ESP_LOGW(TAG, "Vi phạm NFR alert latency: %u ms > %u ms (alert_reason=%s)",
                         (unsigned)snap.alert_latency_ms, (unsigned)Cfg::ALERT_MAX_LATENCY_MS,
                         snap.alert_reason);
                s_storage.logEvent(StorageHelper::EventType::ALERT_LATENCY_EXCEEDED, snap);
            }
            s_network.publishAlert(snap);
            s_storage.logEvent(StorageHelper::EventType::ALERT_LEVEL_CHANGED, snap);
            last_published_level = level;
        }

        // (d) Edge alert_reason/alert_flags đổi — Event Log riêng (StorageHelper.hpp §4.3),
        //     độc lập với debounce publishAlert ở (c).
        if (snap.alert_level != prev_alert_level || snap.alert_flags != prev_alert_flags) {
            s_storage.logEvent(StorageHelper::EventType::ALERT_REASON_CHANGED, snap);
            prev_alert_level = snap.alert_level;
            prev_alert_flags = snap.alert_flags;
        }

        // (H.6) Flush baseline NVS dirty ở task ưu tiên thấp hơn taskSensor —
        // idempotent, an toàn gọi dư nhịp (DataFusion.hpp persistBaselineIfDirty()).
        s_fusion.persistBaselineIfDirty();

        esp_task_wdt_reset();
    }
}

// ============================================================
// app_main
// ============================================================
extern "C" void app_main() {
    // [PM-1] Cân nhắc tối ưu công suất (CHƯA triển khai):
    //   esp_wifi_set_ps(WIFI_PS_MIN_MODEM) (NetworkManager::initWifi) đã xử lý
    //   phần Wi-Fi của NFR <=2W. Phần CPU/peripheral idle (Dynamic Frequency
    //   Scaling + Automatic Light Sleep qua CONFIG_PM_ENABLE + esp_pm_configure())
    //   chưa bật — nếu cần, gọi esp_pm_configure() ở đây, đầu app_main(), trước
    //   khi tạo task/driver. Chỉ làm sau khi đo công suất thực tế cho thấy cần
    //   thiết — tickless idle có thể ảnh hưởng timing UART (PMS5003) và I2C
    //   (BME680/PCF8574), cần test kỹ trước khi bật production.

    // NVS bắt buộc cho SensorManager (calibration baseline)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "=== PRODUCTION MODE ===");

    // ---- I.1: phát hiện reset do Watchdog ngay đầu app_main() — KHÔNG reset
    //      im lặng (CLAUDE.md §4/§6.5). Event Log SYSTEM_BOOT được ghi sau,
    //      khi StorageHelper đã sẵn sàng (H.5).
    const esp_reset_reason_t reset_reason = esp_reset_reason();
    const bool recovered_from_wdt = (reset_reason == ESP_RST_TASK_WDT) ||
                                     (reset_reason == ESP_RST_INT_WDT) ||
                                     (reset_reason == ESP_RST_WDT);
    if (recovered_from_wdt) {
        ESP_LOGE(TAG, "Hồi phục từ WDT reset (lí do=%d)", (int)reset_reason);
    }

    // ---- D.2: hạ tầng đồng bộ TRƯỚC khi tạo task ----
    s_shared_mtx = xSemaphoreCreateMutex();
    assert(s_shared_mtx != nullptr);
    s_airq = xQueueCreate(1, sizeof(AirData));
    assert(s_airq != nullptr);
    s_display_notice = xQueueCreate(1, sizeof(DisplayNotice));  // XM-15
    assert(s_display_notice != nullptr);

    // ---- D.3: SensorManager — gọi i2cdev_init() lần đầu; PHẢI chạy TRƯỚC
    //      DisplayManager::init() (XM-4, cùng Cfg::I2C_PORT). Cốt lõi của hệ
    //      thống quan trắc → lỗi ở đây là không thể tiếp tục.
    ESP_ERROR_CHECK(s_sensor.init());

    // ---- D.4: DisplayManager — hiển thị là phụ trợ, lỗi không làm sập quan trắc ----
    if (s_display.init() != ESP_OK) {
        ESP_LOGE(TAG, "DisplayManager init thất bại — tiếp tục không hiển thị LCD");
    }

    // ---- D.5: DataFusion — nạp baseline/last_calib_ts từ NVS. Thiếu baseline
    //      (NVS trống/lỗi) vẫn an toàn: hasBaseline()=false, driftSelfCheck bỏ qua. ----
    if (s_fusion.init() != ESP_OK) {
        ESP_LOGE(TAG, "DataFusion init thất bại — tiếp tục không có baseline hiệu chuẩn");
    }
    if (!s_fusion.hasBaseline()) {
        ESP_LOGW(TAG, "Chưa có baseline hiệu chuẩn — chờ cảm biến warmup để chốt mẫu đầu tiên");
    }

    // ---- D.6: StorageHelper — SAU nvs_flash_init. KHÔNG ESP_ERROR_CHECK: SD
    //      vắng/hỏng vẫn phải chạy tiếp (StorageHelper.hpp §13 SD_ABSENT). ----
    if (s_storage.init() != ESP_OK) {
        ESP_LOGE(TAG, "StorageHelper init thất bại (SD_ABSENT) — tiếp tục không log SD");
    }

    // ---- D.7/XM-6: setCommandCallback PHẢI gọi TRƯỚC network.init() ----
    // [XM-7/XM-8] Lambda chạy trong context MQTT event task — lock shared_data_mutex
    // quanh đọc/ghi s_shared rồi mới gọi confirmRecalibration(); KHÔNG retry tự động
    // khi lỗi. [StorageHelper.hpp §4.3] CALIB_CONFIRMED chỉ ghi khi err == ESP_OK.
    s_network.setCommandCallback([](const char *cmd) {
        // skip_warmup: đề phòng rút/cắm điện sau khi sensor đã ổn định — timer
        // phần mềm reset về 0 trong khi sensor vật lý vẫn warm. Force *_ready = true
        // toàn pipeline (LCD + DataFusion + MQTT dashboard) cho đến khi timer thật hết.
        if (std::strcmp(cmd, "skip_warmup") == 0) {
            s_force_skip_warmup.store(true);

            AirData snapshot{};
            if (xSemaphoreTake(s_shared_mtx, pdMS_TO_TICKS(100)) == pdTRUE) {
                snapshot = s_shared;
                xSemaphoreGive(s_shared_mtx);
            }
            ESP_LOGI(TAG, "skip_warmup: force *_ready toàn pipeline — sensor vật lý đã warm");
            s_storage.logEvent(StorageHelper::EventType::WARMUP_SKIPPED, snapshot);
            return;
        }

        if (std::strcmp(cmd, "confirm_calib") == 0) {
            if (xSemaphoreTake(s_shared_mtx, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGW(TAG, "confirm_calib: không lấy được shared_data_mutex");
                return;
            }
            const bool synced = s_network.isTimeSynced();
            const esp_err_t calib_err = s_fusion.confirmRecalibration(s_shared, synced);
            const AirData snapshot = s_shared;
            xSemaphoreGive(s_shared_mtx);

            if (calib_err == ESP_OK) {
                ESP_LOGI(TAG, "confirm_calib: hiệu chuẩn đã được xác nhận");
                s_storage.logEvent(StorageHelper::EventType::CALIB_CONFIRMED, snapshot);
            } else {
                ESP_LOGW(TAG, "confirm_calib thất bại: %s", esp_err_to_name(calib_err));
            }
            return;
        }
    });

    // ---- D.8: NetworkManager — non-blocking; WiFi/MQTT/SNTP kết nối ngầm qua
    //      event handler. Lỗi vẫn tiếp tục: hệ thống chạy offline, dữ liệu
    //      được đệm xuống SD (Offline Buffer, CLAUDE.md §4). ----
    if (s_network.init() != ESP_OK) {
        ESP_LOGE(TAG, "NetworkManager init thất bại — tiếp tục offline (buffer xuống SD)");
    }

    // ---- D.9/Mục G/XM-12: GPIO LED/Buzzer — mức LOW (tắt) lúc boot ----
    const gpio_config_t alert_gpio_cfg = {
        .pin_bit_mask = (1ULL << Cfg::LED_RED_PIN) | (1ULL << Cfg::LED_YELLOW_PIN) |
                        (1ULL << Cfg::LED_GREEN_PIN) | (1ULL << Cfg::BUZZER_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&alert_gpio_cfg));
    gpio_set_level(static_cast<gpio_num_t>(Cfg::LED_RED_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(Cfg::LED_YELLOW_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(Cfg::LED_GREEN_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(Cfg::BUZZER_PIN), 0);

    // ---- D.10/Mục H: tạo task rồi gắn nhịp esp_timer (mỗi task tự
    //      esp_task_wdt_add(NULL) ở đầu thân — Mục I) ----
    TaskHandle_t h_sensor = nullptr, h_display = nullptr, h_network = nullptr;
    xTaskCreate(taskSensor, "sensor", Cfg::TASK_STACK_SENSOR_WORDS, nullptr,
                Cfg::TASK_PRIO_SENSOR, &h_sensor);
    xTaskCreate(taskDisplay, "display", Cfg::TASK_STACK_DISPLAY_WORDS, nullptr,
                Cfg::TASK_PRIO_DISPLAY, &h_display);
    xTaskCreate(taskNetwork, "network", Cfg::TASK_STACK_NETWORK_WORDS, nullptr,
                Cfg::TASK_PRIO_NETWORK, &h_network);

    // [XM-1] Cadence LCD kẹp trong [LCD_MIN_INTERVAL_MS, LCD_MAX_INTERVAL_MS] —
    // CONFIG_DISPLAY_UPDATE_INTERVAL_MS có range Kconfig rộng hơn (1000-10000ms).
    const uint32_t display_period_ms = std::clamp(Cfg::DISPLAY_UPDATE_INTERVAL_MS,
                                                    Cfg::LCD_MIN_INTERVAL_MS,
                                                    Cfg::LCD_MAX_INTERVAL_MS);

    const uint32_t sensor_period_ms = std::clamp(Cfg::SENSOR_READ_INTERVAL_MS,
                                                  Cfg::SENSOR_MIN_INTERVAL_MS,
                                                  Cfg::SENSOR_MAX_INTERVAL_MS);

    esp_timer_handle_t sensor_timer = nullptr, display_timer = nullptr, network_timer = nullptr;
    ESP_ERROR_CHECK(startPeriodicNotifier(sensor_timer, "sensor_tick", h_sensor,
                                           sensor_period_ms));
    ESP_ERROR_CHECK(startPeriodicNotifier(display_timer, "display_tick", h_display,
                                           display_period_ms));
    ESP_ERROR_CHECK(startPeriodicNotifier(network_timer, "network_tick", h_network,
                                           sensor_period_ms));

    // ---- H.5: mốc khởi động — phục vụ truy vết (kèm ESP_LOGE phía trên nếu
    //      vừa hồi phục từ WDT reset) ----
    AirData boot_snapshot{};
    boot_snapshot.timestamp = nowUnixOrUptime();
    s_storage.logEvent(StorageHelper::EventType::SYSTEM_BOOT, boot_snapshot);

    ESP_LOGI(TAG,
             "Production mode khởi tạo hoàn tất — chu kỳ pipeline=%ums "
             "LCD=%ums network=%ums",
             (unsigned)sensor_period_ms, (unsigned)display_period_ms,
             (unsigned)sensor_period_ms);
}

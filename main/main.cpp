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
#include "esp_attr.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <ctime>

static const char *TAG = "main";

static SensorManager  s_sensor;
static Filters        s_filters;
static DataFusion     s_fusion;
static DisplayManager s_display;
static NetworkManager s_network;
static StorageHelper  s_storage;

static AirData            s_shared{};
static SemaphoreHandle_t  s_shared_mtx = nullptr;
static QueueHandle_t      s_airq = nullptr;

enum class DisplayNotice : uint8_t { NET_LOST = 0, NET_RESTORED = 1 };
static QueueHandle_t      s_display_notice = nullptr;

static std::atomic<bool>  s_force_skip_warmup{false};
static std::atomic<bool>  s_shutdown_requested{false};

static int64_t nowUnixOrUptime() {
    if (s_network.isTimeSynced()) {
        return static_cast<int64_t>(time(nullptr));
    }
    return esp_timer_get_time() / 1000000LL;
}

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

static void doShutdown() {
    bool expected = false;
    if (!s_shutdown_requested.compare_exchange_strong(expected, true)) {
        return;
    }

    gpio_set_level(static_cast<gpio_num_t>(Cfg::LED_RED_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(Cfg::LED_YELLOW_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(Cfg::BUZZER_PIN), 0);
    s_storage.prepareShutdown();

    for (;;) {
        gpio_set_level(static_cast<gpio_num_t>(Cfg::LED_GREEN_PIN), 1);
        vTaskDelay(pdMS_TO_TICKS(Cfg::SHUTDOWN_LED_BLINK_MS));
        gpio_set_level(static_cast<gpio_num_t>(Cfg::LED_GREEN_PIN), 0);
        vTaskDelay(pdMS_TO_TICKS(Cfg::SHUTDOWN_LED_BLINK_MS));
    }
}

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

static void taskSensor(void *) {
    esp_task_wdt_add(nullptr);

    bool prev_calib_needed = false;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const int64_t t0 = esp_timer_get_time();

        AirData local{};

        if (s_sensor.readAll(local) != ESP_OK) {
            ESP_LOGW(TAG, "SensorManager::readAll lỗi trong chu kỳ này");
        }

        if (s_force_skip_warmup.load()) {
            local.bme680_ready  = true;
            local.pms5003_ready = true;
            local.mq135_ready   = true;
            if (s_sensor.isFullyReady()) {
                s_force_skip_warmup.store(false);
            }
        }

        s_filters.process(local);

        const bool synced = s_network.isTimeSynced();
        s_fusion.process(local, synced);

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

        xQueueOverwrite(s_airq, &local);

        if (!s_shutdown_requested.load()) {
            driveAlertOutputs(local);
        }

        if (local.calib_needed && !prev_calib_needed) {
            s_storage.logEvent(StorageHelper::EventType::CALIB_NEEDED_SET, local);
        }
        prev_calib_needed = local.calib_needed;

        s_storage.logData(local);

        esp_task_wdt_reset();
    }
}

static void taskDisplay(void *) {
    esp_task_wdt_add(nullptr);

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        DisplayNotice notice;
        if (xQueueReceive(s_display_notice, &notice, 0) == pdTRUE) {
            if (notice == DisplayNotice::NET_LOST) {
                s_display.showMessage("NETWORK LOST", "Saving to SD");
            } else {
                s_display.showMessage("NETWORK OK", "Syncing...");
            }
        }

        AirData snap{};
        xQueuePeek(s_airq, &snap, 0);

        s_display.update(snap);
        s_display.tick();

        esp_task_wdt_reset();
    }
}

static void taskNetwork(void *) {
    esp_task_wdt_add(nullptr);

    static DataFusion::AlertLevel last_published_level = DataFusion::AlertLevel::NONE;
    static bool     prev_connected   = false;
    static uint8_t  prev_alert_level = 0;
    static uint16_t prev_alert_flags = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        AirData snap{};
        xQueuePeek(s_airq, &snap, 0);

        const auto level = static_cast<DataFusion::AlertLevel>(snap.alert_level);
        const bool level_changed = (level != last_published_level);
        snap.alert_latency_ms = level_changed
            ? static_cast<uint32_t>(
                  std::max<int64_t>(0, esp_timer_get_time() - snap.alert_level_changed_us) / 1000)
            : 0u;

        const bool conn = s_network.isConnected();
        if (conn != prev_connected) {
            if (conn) {
                s_storage.logEvent(StorageHelper::EventType::MQTT_CONNECTED, snap);
                s_storage.drainOffline([](const AirData &d) { return s_network.publishData(d); });
                DisplayNotice notice = DisplayNotice::NET_RESTORED;
                xQueueOverwrite(s_display_notice, &notice);
            } else {
                s_storage.logEvent(StorageHelper::EventType::MQTT_DISCONNECTED, snap);
                DisplayNotice notice = DisplayNotice::NET_LOST;
                xQueueOverwrite(s_display_notice, &notice);
            }
            prev_connected = conn;
        }

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

        if (snap.alert_level != prev_alert_level || snap.alert_flags != prev_alert_flags) {
            s_storage.logEvent(StorageHelper::EventType::ALERT_REASON_CHANGED, snap);
            prev_alert_level = snap.alert_level;
            prev_alert_flags = snap.alert_flags;
        }

        s_fusion.persistBaselineIfDirty();

        esp_task_wdt_reset();
    }
}

static void IRAM_ATTR taskButtonIsr(void *arg) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(arg), &woken);
    portYIELD_FROM_ISR(woken);
}

static void taskButton(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(Cfg::BUTTON_DEBOUNCE_MS));
        if (gpio_get_level(static_cast<gpio_num_t>(Cfg::SHUTDOWN_BUTTON_PIN)) != 0) {
            continue;
        }

        bool still_pressed = true;
        for (uint32_t held_ms = 0; held_ms < Cfg::BUTTON_HOLD_MS; held_ms += Cfg::BUTTON_POLL_MS) {
            vTaskDelay(pdMS_TO_TICKS(Cfg::BUTTON_POLL_MS));
            if (gpio_get_level(static_cast<gpio_num_t>(Cfg::SHUTDOWN_BUTTON_PIN)) != 0) {
                still_pressed = false;
                break;
            }
        }

        if (still_pressed) {
            ESP_LOGW(TAG, "Nút shutdown giữ đủ %ums — thực thi shutdown cục bộ",
                     (unsigned)Cfg::BUTTON_HOLD_MS);
            doShutdown();
        }
    }
}

extern "C" void app_main() {
    // [PM-1] Cân nhắc tối ưu công suất (CHƯA triển khai):
    //   esp_wifi_set_ps(WIFI_PS_MIN_MODEM) (NetworkManager::initWifi) đã xử lý
    //   phần Wi-Fi của NFR <=2W. Phần CPU/peripheral idle (Dynamic Frequency
    //   Scaling + Automatic Light Sleep qua CONFIG_PM_ENABLE + esp_pm_configure())
    //   chưa bật — nếu cần, gọi esp_pm_configure() ở đây, đầu app_main(), trước
    //   khi tạo task/driver. Chỉ làm sau khi đo công suất thực tế cho thấy cần
    //   thiết — tickless idle có thể ảnh hưởng timing UART (PMS5003) và I2C
    //   (BME680/PCF8574), cần test kỹ trước khi bật production.

    setenv("TZ", "ICT-7", 1);
    tzset();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "=== PRODUCTION MODE ===");

    const esp_reset_reason_t reset_reason = esp_reset_reason();
    const bool recovered_from_wdt = (reset_reason == ESP_RST_TASK_WDT) ||
                                     (reset_reason == ESP_RST_INT_WDT) ||
                                     (reset_reason == ESP_RST_WDT);
    if (recovered_from_wdt) {
        ESP_LOGE(TAG, "Hồi phục từ WDT reset (lí do=%d)", (int)reset_reason);
    }

    s_shared_mtx = xSemaphoreCreateMutex();
    assert(s_shared_mtx != nullptr);
    s_airq = xQueueCreate(1, sizeof(AirData));
    assert(s_airq != nullptr);
    s_display_notice = xQueueCreate(1, sizeof(DisplayNotice));
    assert(s_display_notice != nullptr);

    ESP_ERROR_CHECK(s_sensor.init());

    if (s_display.init() != ESP_OK) {
        ESP_LOGE(TAG, "DisplayManager init thất bại — tiếp tục không hiển thị LCD");
    }

    if (s_fusion.init() != ESP_OK) {
        ESP_LOGE(TAG, "DataFusion init thất bại — tiếp tục không có baseline hiệu chuẩn");
    }
    if (!s_fusion.hasBaseline()) {
        ESP_LOGW(TAG, "Chưa có baseline hiệu chuẩn — chờ cảm biến warmup để chốt mẫu đầu tiên");
    }

    if (s_storage.init() != ESP_OK) {
        ESP_LOGE(TAG, "StorageHelper init thất bại (SD_ABSENT) — tiếp tục không log SD");
    }

    s_network.setCommandCallback([](const char *cmd) {
        if (std::strcmp(cmd, "reboot") == 0) {
            ESP_LOGW(TAG, "Thực thi lệnh reboot...");
            s_storage.prepareShutdown();
            esp_restart();
            return;
        }

        if (std::strcmp(cmd, "shutdown") == 0) {
            ESP_LOGW(TAG, "Thực thi lệnh shutdown...");
            doShutdown();
            return;
        }

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

    if (s_network.init() != ESP_OK) {
        ESP_LOGE(TAG, "NetworkManager init thất bại — tiếp tục offline (buffer xuống SD)");
    }

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

    const gpio_config_t button_gpio_cfg = {
        .pin_bit_mask = (1ULL << Cfg::SHUTDOWN_BUTTON_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&button_gpio_cfg));

    TaskHandle_t h_sensor = nullptr, h_display = nullptr, h_network = nullptr, h_button = nullptr;
    xTaskCreate(taskSensor, "sensor", Cfg::TASK_STACK_SENSOR_WORDS, nullptr,
                Cfg::TASK_PRIO_SENSOR, &h_sensor);
    xTaskCreate(taskDisplay, "display", Cfg::TASK_STACK_DISPLAY_WORDS, nullptr,
                Cfg::TASK_PRIO_DISPLAY, &h_display);
    xTaskCreate(taskNetwork, "network", Cfg::TASK_STACK_NETWORK_WORDS, nullptr,
                Cfg::TASK_PRIO_NETWORK, &h_network);
    xTaskCreate(taskButton, "button", Cfg::TASK_STACK_BUTTON_WORDS, nullptr,
                Cfg::TASK_PRIO_BUTTON, &h_button);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(static_cast<gpio_num_t>(Cfg::SHUTDOWN_BUTTON_PIN),
                                          taskButtonIsr, h_button));

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

    AirData boot_snapshot{};
    boot_snapshot.timestamp = nowUnixOrUptime();
    s_storage.logEvent(StorageHelper::EventType::SYSTEM_BOOT, boot_snapshot);

    ESP_LOGI(TAG,
             "Production mode khởi tạo hoàn tất — chu kỳ pipeline=%ums "
             "LCD=%ums network=%ums",
             (unsigned)sensor_period_ms, (unsigned)display_period_ms,
             (unsigned)sensor_period_ms);
}

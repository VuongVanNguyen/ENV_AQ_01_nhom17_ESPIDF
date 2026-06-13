// ============================================================
// main.cpp — Entry point dự án ENV-AQ-01
//
//   TEST MODE  : idf.py menuconfig → Sensor Test Mode → chọn sensor
//   PRODUCTION : idf.py menuconfig → Sensor Test Mode → Production
//   Không cần sửa file này khi chuyển qua lại giữa test và production.
// ============================================================

#include "sdkconfig.h"
#include "config.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

// ---- Test mode: include task declarations ----
#if defined(CONFIG_TEST_MODE_BME680) || \
    defined(CONFIG_TEST_MODE_PMS5003) || \
    defined(CONFIG_TEST_MODE_MQ135)
#   include "test_sensors.hpp"
#endif

// ---- TEST_MODE_ALL + Production: dùng SensorManager ----
#if defined(CONFIG_TEST_MODE_ALL) || defined(CONFIG_TEST_MODE_NONE)
#   include "SensorManager.hpp"
#   include "DataStructures.hpp"
#endif

// ============================================================
// Task test tất cả cảm biến qua SensorManager (TEST_MODE_ALL)
// ============================================================
#if defined(CONFIG_TEST_MODE_ALL)

static SensorManager s_sensor;

static void test_all_task(void *) {
    ESP_ERROR_CHECK(s_sensor.init());
    ESP_LOGI("TEST_ALL", "SensorManager init OK. Đang chờ warmup...");
    ESP_LOGI("TEST_ALL", "BME680=%lums  PMS5003=%lums  MQ135=%lums",
             (unsigned long)Cfg::BME680_WARMUP_MS,
             (unsigned long)Cfg::PMS5003_WARMUP_MS,
             (unsigned long)Cfg::MQ135_WARMUP_MS);

    AirData data;
    while (true) {
        if (s_sensor.readAll(data) == ESP_OK) {
            ESP_LOGI("TEST_ALL",
                     "T=%5.2f°C  RH=%5.2f%%  P=%7.2fhPa  Gas=%8.0fΩ | "
                     "PM1=%3u  PM2.5=%3u  PM10=%3u | CO2=%7.1fppm | "
                     "bme=%s  pms=%s  mq=%s",
                     data.temperature, data.humidity,
                     data.pressure,    data.gas_resistance,
                     data.pm1_0, data.pm2_5, data.pm10,
                     data.co2_ppm,
                     data.bme680_ready  ? "RDY" : "WARM",
                     data.pms5003_ready ? "RDY" : "WARM",
                     data.mq135_ready   ? "RDY" : "WARM");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

#endif // CONFIG_TEST_MODE_ALL

// ============================================================
// app_main
// ============================================================
extern "C" void app_main() {
    // NVS bắt buộc cho SensorManager (calibration baseline)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

#if defined(CONFIG_TEST_MODE_BME680)
    ESP_LOGI(TAG, "=== TEST MODE: BME680 ONLY ===");
    xTaskCreate(test_bme680_task, "test_bme680",
                Cfg::TASK_STACK_SENSOR_WORDS, nullptr, 5, nullptr);

#elif defined(CONFIG_TEST_MODE_PMS5003)
    ESP_LOGI(TAG, "=== TEST MODE: PMS5003 ONLY ===");
    xTaskCreate(test_pms5003_task, "test_pms5003",
                Cfg::TASK_STACK_SENSOR_WORDS, nullptr, 5, nullptr);

#elif defined(CONFIG_TEST_MODE_MQ135)
    ESP_LOGI(TAG, "=== TEST MODE: MQ-135 ONLY ===");
    xTaskCreate(test_mq135_task, "test_mq135",
                Cfg::TASK_STACK_SENSOR_WORDS, nullptr, 5, nullptr);

#elif defined(CONFIG_TEST_MODE_ALL)
    ESP_LOGI(TAG, "=== TEST MODE: ALL SENSORS ===");
    xTaskCreate(test_all_task, "test_all",
                Cfg::TASK_STACK_SENSOR_WORDS, nullptr, 5, nullptr);

#else
    // Production mode (CONFIG_TEST_MODE_NONE)
    // TODO: Khởi tạo và tạo task cho SensorManager, NetworkManager,
    //       DisplayManager, StorageHelper, DataFusion
    //
    // ---- ĐẶC TẢ LIÊN QUAN ĐẾN DisplayManager (DisplayManager.hpp, chưa triển khai) ----
    // [XM-4] Thứ tự khởi tạo (CLAUDE.md §4): SensorManager::init() (gọi i2cdev_init() lần
    //        đầu) PHẢI chạy TRƯỚC DisplayManager::init() — cả hai dùng chung Cfg::I2C_PORT.
    // [XM-1] Cadence LCD (CLAUDE.md §3): mỗi chu kỳ của task hiển thị, period nằm trong
    //        [Cfg::LCD_MIN_INTERVAL_MS=2000, Cfg::LCD_MAX_INTERVAL_MS=5000] và đồng bộ với
    //        CONFIG_DISPLAY_UPDATE_INTERVAL_MS, phải gọi theo đúng thứ tự:
    //          DisplayManager::update(data);  // vẽ trang hiện tại
    //          DisplayManager::tick();        // chuẩn bị trang/blink kế tiếp
    //
    // ---- ĐẶC TẢ LIÊN QUAN ĐẾN DataFusion (DataFusion.hpp §7, §9) ----
    // [XM-5] time_synced: DataFusion KHÔNG tự gọi NetworkManager::isTimeSynced()
    //        (tránh circular dependency — xem DataFusion.hpp §9). main.cpp PHẢI
    //        lấy bool synced = network.isTimeSynced() mỗi chu kỳ và truyền vào:
    //          dataFusion.process(data, synced);
    //          dataFusion.confirmRecalibration(data, synced);  // trong cmd_callback "confirm_calib"
    //
    // ---- ĐẶC TẢ DÂY NỐI cmd_callback "confirm_calib" (NetworkManager.hpp, DataFusion.hpp §6.4) ----
    // [XM-6] Thứ tự gọi: network.setCommandCallback(lambda) PHẢI được gọi TRƯỚC
    //        network.init() (xem comment NetworkManager.hpp — "Gọi trước init()").
    //        Lambda capture &dataFusion, &shared_data (AirData) và &network.
    // [XM-7] Đồng bộ AirData dùng chung: lambda chạy trong context MQTT event task
    //        (có thể khác core với task xử lý dữ liệu chính ghi liên tục vào
    //        shared_data). DataFusion::mutex_ (nội bộ) CHỈ bảo vệ baseline_/NVS,
    //        KHÔNG bảo vệ struct AirData truyền vào confirmRecalibration(). main.cpp
    //        PHẢI tự tạo SemaphoreHandle_t/mutex riêng (hoặc queue) bọc quanh
    //        shared_data, và lambda PHẢI lock mutex đó trước khi đọc/ghi
    //        data.temperature/humidity/pm2_5/.../sensors_ready, data_valid, calib_needed.
    // [XM-8] Nội dung lambda khi nhận cmd == "confirm_calib":
    //          lock(shared_data_mutex);
    //          bool synced = network.isTimeSynced();             // §XM-5
    //          esp_err_t err = dataFusion.confirmRecalibration(shared_data, synced);
    //          unlock(shared_data_mutex);
    //          if (err != ESP_OK) ESP_LOGW(...);                 // không retry tự động
    ESP_LOGI(TAG, "=== PRODUCTION MODE ===");

#endif
}

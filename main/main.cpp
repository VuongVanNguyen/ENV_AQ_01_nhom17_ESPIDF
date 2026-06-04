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
                     "bme=%s  pms=%s  mq=%s  all=%s",
                     data.temperature, data.humidity,
                     data.pressure,    data.gas_resistance,
                     data.pm1_0, data.pm2_5, data.pm10,
                     data.co2_ppm,
                     data.bme680_ready  ? "RDY" : "WARM",
                     data.pms5003_ready ? "RDY" : "WARM",
                     data.mq135_ready   ? "RDY" : "WARM",
                     data.sensors_ready ? "READY" : "WARMING");
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
    ESP_LOGI(TAG, "=== PRODUCTION MODE ===");

#endif
}

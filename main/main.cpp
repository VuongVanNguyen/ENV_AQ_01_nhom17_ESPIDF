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
    // ================================================================
    // ĐẶC TẢ PRODUCTION MODE — main.cpp (CHƯA hiện thực hoá; chỉ đặc tả)
    //   Toàn bộ phần dưới là COMMENT mô tả mọi thứ main.cpp BẮT BUỘC phải
    //   có khi triển khai. Tuân thủ nghiêm CLAUDE.md §3 (NFR) + §4 (Key
    //   Rules) + §6 (Debug). Mọi hằng số đến từ Cfg (config.hpp); mọi hàm
    //   liên-module gọi đúng chữ ký đã khai báo trong các *.hpp tương ứng.
    //
    // ================================================================
    // A. VAI TRÒ main.cpp (CLAUDE.md §2 — "Khởi tạo + điều phối")
    // ----------------------------------------------------------------
    //   main.cpp KHÔNG chứa nghiệp vụ tính toán (đã nằm ở SensorManager/
    //   Filters/DataFusion). main.cpp chỉ:
    //     (1) Khởi tạo NVS + 5 module theo ĐÚNG thứ tự (mục D).
    //     (2) Tạo hạ tầng đồng bộ: shared_data + mutex + airDataQueue (mục E).
    //     (3) Nối callback "confirm_calib" giữa NetworkManager↔DataFusion (mục F).
    //     (4) gpio_config() + lái LED/Buzzer theo AlertLevel (mục G — DataFusion
    //         KHÔNG chạm GPIO, DataFusion.hpp §8).
    //     (5) Tạo & điều phối các FreeRTOS task (mục H), bằng esp_timer/queue/
    //         Event Group — KHÔNG dùng vTaskDelay làm logic nghiệp vụ (CLAUDE.md §4).
    //     (6) Đăng ký & feed Task Watchdog Timer cho mọi task vòng-lặp-vô-hạn (mục I).
    //
    // ================================================================
    // B. #include & INSTANCE TOÀN CỤC (file-scope static, RAII) — NGUỒN TỪ FILE NÀO
    // ----------------------------------------------------------------
    //   #include cần thêm cho Production (ngoài các include sẵn có ở đầu file):
    //     "SensorManager.hpp", "Filters.hpp", "DataFusion.hpp",
    //     "DisplayManager.hpp", "NetworkManager.hpp", "StorageHelper.hpp",
    //     "DataStructures.hpp" (AirData), "esp_timer.h" (nhịp chu kỳ + nguồn
    //     thời gian fallback), "esp_task_wdt.h" (TWDT — mục I), "esp_system.h"
    //     + "esp_reset_reason()" (phát hiện WDT reset — mục I), "driver/gpio.h"
    //     (gpio_config LED/Buzzer — mục G), "freertos/semphr.h" (mutex shared_data),
    //     "freertos/queue.h" (airDataQueue), "freertos/event_groups.h" (tuỳ chọn,
    //     đồng bộ task), <time.h> ("now" khi isTimeSynced — mục H.3).
    //   INSTANCE (đặt static file-scope để sống suốt vòng đời, tránh heap; giống
    //   s_sensor của TEST_MODE_ALL ở trên):
    //     static SensorManager  s_sensor;     // SensorManager.hpp
    //     static Filters        s_filters;    // Filters.hpp
    //     static DataFusion     s_fusion;     // DataFusion.hpp
    //     static DisplayManager s_display;    // DisplayManager.hpp
    //     static NetworkManager s_network;    // NetworkManager.hpp
    //     static StorageHelper  s_storage;    // StorageHelper.hpp
    //     static AirData        s_shared{};   // shared_data — nguồn ghi của pipeline (mục E)
    //     static SemaphoreHandle_t s_shared_mtx;  // bảo vệ s_shared (XM-7)
    //     static QueueHandle_t     s_airq;        // airDataQueue length-1 (XM-9)
    //   (Tất cả non-copyable — đã xoá copy ctor trong từng module.)
    //
    // ================================================================
    // C. HẰNG SỐ — KHAI BÁO Ở ĐÂU (CLAUDE.md §4 "Cấu hình dự án" — KHÔNG hardcode)
    // ----------------------------------------------------------------
    //   main.cpp KHÔNG khai báo hằng số mới; TÁI DÙNG từ Cfg (config.hpp):
    //     - Nhịp pipeline ............ Cfg::SENSOR_READ_INTERVAL_MS (§6, Kconfig).
    //     - Ngân sách chu kỳ ......... Cfg::MAX_CYCLE_TIME_MS = 300 (NFR §3, đo & assert).
    //     - Nhịp LCD ................. Cfg::DISPLAY_UPDATE_INTERVAL_MS, kẹp trong
    //                                  [Cfg::LCD_MIN_INTERVAL_MS, LCD_MAX_INTERVAL_MS] (XM-1).
    //     - Độ trễ cảnh báo .......... Cfg::ALERT_MAX_LATENCY_MS = 3000 (NFR §3, mục G/H.3).
    //     - Pin LED/Buzzer ........... Cfg::LED_RED_PIN/LED_YELLOW_PIN/LED_GREEN_PIN/
    //                                  BUZZER_PIN (config.hpp §5; CLAUDE.md §5).
    //     - Stack/Prio task .......... Cfg::TASK_STACK_*_WORDS / Cfg::TASK_PRIO_* (§15).
    //     - Topic/nhịp log SD ........ Cfg::MQTT_TOPIC_* / Cfg::SD_LOG_INTERVAL_MS (qua module).
    //   Mọi tham số chỉnh được (WiFi/MQTT/ngưỡng/nhịp) đã ở Kconfig.projbuild →
    //   CONFIG_* → Cfg::* (config.hpp). Nếu cần hằng MỚI cho main → thêm vào
    //   config.hpp (Cfg), KHÔNG literal rời trong main.cpp.
    //
    // ================================================================
    // D. THỨ TỰ KHỞI TẠO trong app_main() (BẮT BUỘC — phụ thuộc lẫn nhau)
    // ----------------------------------------------------------------
    //   D.0 nvs_flash_init() — ĐÃ làm ở đầu app_main() (phía trên), trước MỌI
    //       module dùng NVS (SensorManager calib baseline, DataFusion baseline).
    //   D.1 (Tuỳ chọn) esp_pm_configure() — xem [PM-1] phía trên, CHƯA bật.
    //   D.2 Tạo hạ tầng đồng bộ TRƯỚC khi tạo task:
    //         s_shared_mtx = xSemaphoreCreateMutex();   // != nullptr (assert)
    //         s_airq       = xQueueCreate(1, sizeof(AirData));  // XM-9
    //   D.3 s_sensor.init() — ESP_ERROR_CHECK. NỘI BỘ gọi i2cdev_init() LẦN ĐẦU
    //       (idempotent). PHẢI chạy TRƯỚC s_display.init() (XM-4, CLAUDE.md §4 —
    //       cùng Cfg::I2C_PORT; mutex per-port i2cdev mới serialize đúng).
    //   D.4 s_display.init() — sau D.3. Lỗi → ESP_LOGE + tiếp tục (hiển thị là
    //       phụ trợ, không làm sập quan trắc); hoặc ESP_ERROR_CHECK tuỳ chính sách.
    //   D.5 s_fusion.init() — nạp baseline/last_calib_ts từ NVS (DataFusion.hpp §6.5).
    //   D.6 s_storage.init() — SAU nvs_flash_init. KHÔNG ESP_ERROR_CHECK
    //       (StorageHelper.hpp §13: SD vắng/hỏng vẫn phải chạy tiếp — chỉ ESP_LOGE,
    //       module vào SD_ABSENT). init() tự tạo storage_queue_ + storageTask (prio
    //       thấp nhất) và khôi phục offline_count_ (§5.4).
    //   D.7 s_network.setCommandCallback(lambda) — PHẢI gọi TRƯỚC s_network.init()
    //       (XM-6, NetworkManager.hpp "Gọi trước init()").
    //   D.8 s_network.init() — non-blocking; WiFi/MQTT/SNTP kết nối ngầm qua event
    //       handler (NetworkManager.hpp). esp_wifi_set_ps(WIFI_PS_MODEM) đã xử lý
    //       trong initWifi() (phần Wi-Fi của NFR ≤2W).
    //   D.9 gpio_config() LED/Buzzer (mục G) — có thể làm sớm (trước tạo task) để
    //       trạng thái output xác định ngay từ boot (mặc định tắt).
    //   D.10 Đăng ký TWDT cho app_main hoặc bỏ qua; tạo các task (mục H) — mỗi
    //        task tự esp_task_wdt_add(NULL) ở đầu thân (mục I).
    //   GHI CHÚ readiness: ngay sau init, các cảm biến CÒN trong warmup
    //   (Cfg::*_WARMUP_MS — BME680 5', PMS5003 30s, MQ135 20'); cờ *_ready=false →
    //   DataFusion giữ SENTINEL/NAN, Display in "WARMING UP". main KHÔNG chờ warmup
    //   (không block) — pipeline cứ chạy, mỗi cảm biến ready lúc nào dùng lúc đó.
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
    //
    // ---- ĐẶC TẢ CHIA SẺ AirData GIỮA PIPELINE (<=300ms) VÀ taskDisplay/taskNetwork ----
    // [XM-9] Cadence pipeline (SensorManager→Filters→DataFusion, <=300ms, NFR §3) nhanh
    //        hơn taskDisplay (2-5s) và taskNetwork. Dùng QueueHandle_t length-1 +
    //        xQueueOverwrite()/xQueuePeek() để publish "AirData mới nhất" cho 2 task
    //        đó — không polling, không block producer:
    //          QueueHandle_t airDataQueue = xQueueCreate(1, sizeof(AirData));
    //          // Producer (pipeline task, cuối mỗi cycle, SAU khi unlock shared_data_mutex):
    //          xQueueOverwrite(airDataQueue, &shared_data);
    //          // Consumer (taskDisplay/taskNetwork, đầu mỗi cycle riêng của chúng):
    //          AirData snapshot; xQueuePeek(airDataQueue, &snapshot, 0);
    //        airDataQueue KHÔNG thay thế shared_data_mutex ở [XM-7]: nhánh confirm_calib
    //        cần read-modify-write TRỰC TIẾP lên shared_data (ghi baseline_/calib_needed/
    //        calib_alert), bắt buộc mutex riêng. Hai cơ chế song song, vai trò khác nhau:
    //          - shared_data (+ mutex)  : nguồn ghi/sửa của pipeline + cmd_callback.
    //          - airDataQueue (overwrite): bản snapshot chỉ-đọc cho display/publish.
    //
    // ---- ĐẶC TẢ CẢNH BÁO VƯỢT NGƯỠNG (AirData.alert_level/alert_reason — DataFusion.hpp §8/§11) ----
    // [XM-10] publishData() (gọi mỗi chu kỳ taskNetwork) đã tự chèn "alert_level"/
    //         "alert_reason"/"alert_flags"/"calib_reason" vào JSON
    //         (NetworkManager::buildJson) — dashboard có "live gauge" mức nguy
    //         hiểm hiện tại (kể cả khi nhiều điều kiện CRITICAL/WARNING xảy ra
    //         đồng thời, qua alert_flags) mà không cần thêm gì ở main.cpp.
    // [XM-11] Sự kiện publishAlert() (đáp ứng NFR ≤ Cfg::ALERT_MAX_LATENCY_MS = 3s, §8):
    //           taskNetwork giữ static/local AlertLevel last_published_level = NONE
    //           (snapshot trước đó, đọc từ airDataQueue).
    //           Mỗi chu kỳ, so snapshot.alert_level với last_published_level:
    //             - Nếu KHÁC nhau (tăng lên WARNING/CRITICAL hoặc giảm về NONE) →
    //               network.publishAlert(snapshot) ngay, rồi cập nhật
    //               last_published_level = snapshot.alert_level. data.alert_reason
    //               (lý do CAO NHẤT trong chu kỳ) và data.calib_reason (lý do
    //               hiệu chuẩn CỤ THỂ, ĐỘC LẬP — "CALIB_DRIFT_TEMP"/"CALIB_DRIFT_PM25"/
    //               .../"CALIB_OVERDUE_30D"/"NONE") đã có sẵn trong JSON (§8/§11),
    //               không cần truyền thêm tham số.
    //             - Nếu GIỐNG nhau → không publish lại (debounce, tránh spam
    //               MQTT_TOPIC_ALERT khi tình trạng vượt ngưỡng kéo dài nhiều chu kỳ).
    //
    // ================================================================
    // G. LÁI LED / BUZZER THEO ALERTLEVEL (DataFusion.hpp §8 — main SỞ HỮU GPIO)
    // ----------------------------------------------------------------
    // [XM-12] DataFusion KHÔNG chạm GPIO (DataFusion.hpp §8). main.cpp sở hữu:
    //         gpio_config() mode=GPIO_MODE_OUTPUT cho Cfg::LED_RED_PIN(25),
    //         LED_YELLOW_PIN(26), LED_GREEN_PIN(27), BUZZER_PIN(32) (CLAUDE.md §5).
    //         pin_bit_mask = (1ULL<<25)|(1ULL<<26)|(1ULL<<27)|(1ULL<<32);
    //         pull-up/down disable, intr disable. Khởi tạo mức LOW (tắt) lúc boot.
    // [XM-13] Lái output từ snapshot.alert_level (đọc qua airDataQueue, mục E) —
    //         ánh xạ theo enum DataFusion::AlertLevel (DataFusion.hpp):
    //           NONE(0)    → GREEN on,  YELLOW/RED off, Buzzer off.
    //           WARNING(1) → YELLOW on, GREEN/RED off,  Buzzer off (hoặc beep ngắn 1 lần).
    //           CRITICAL(2)→ RED on,    GREEN/YELLOW off, Buzzer on (liên tục/ngắt quãng).
    //         Khi MỌI cảm biến chưa ready (data_valid=false / alert_level=NONE do
    //         warmup) → coi như NONE (GREEN), KHÔNG bật buzzer giả (DataFusion.hpp §11).
    // [XM-14] NFR ĐỘ TRỄ CẢNH BÁO ≤ Cfg::ALERT_MAX_LATENCY_MS (3s, §3): việc lái
    //         GPIO PHẢI nằm trên đường tới hạn NHANH — đặt trong taskSensor (cuối
    //         mỗi chu kỳ ≤300ms, ngay sau DataFusion::process()) HOẶC trong một
    //         task cảnh báo riêng đọc airDataQueue với chu kỳ ≪3s. KHÔNG đặt phụ
    //         thuộc taskNetwork/taskStorage (có thể chậm do I/O). Buzzer nhấp nháy
    //         dùng esp_timer, KHÔNG vTaskDelay (CLAUDE.md §4).
    //
    // ================================================================
    // H. CÁC FREERTOS TASK — CHI TIẾT (CLAUDE.md §3 §4 §15)
    // ----------------------------------------------------------------
    // H.1 taskSensor — PIPELINE THỜI GIAN THỰC (≤ Cfg::MAX_CYCLE_TIME_MS = 300ms)
    //     · Tạo: xTaskCreate(taskSensor,"sensor",Cfg::TASK_STACK_SENSOR_WORDS,
    //       nullptr, Cfg::TASK_PRIO_SENSOR(5 — CAO NHẤT), &h). (Có thể ghim core
    //       qua xTaskCreatePinnedToCore nếu cần tách khỏi WiFi core — tuỳ đo đạc.)
    //     · NHỊP: KHÔNG vTaskDelay làm logic (CLAUDE.md §4). Dùng esp_timer_create()
    //       + esp_timer_start_periodic(Cfg::SENSOR_READ_INTERVAL_MS*1000) bắn
    //       semaphore/notify; task chờ bằng xSemaphoreTake/ulTaskNotifyTake rồi
    //       chạy 1 chu kỳ. (Cho phép chờ-trên-tín-hiệu; KHÁC với delay timing.)
    //     · MỘT CHU KỲ (đo bằng esp_timer_get_time() đầu/cuối, assert ≤300ms — NFR §3):
    //         (1) AirData local; s_sensor.readAll(local) — SensorManager.hpp.
    //             readAll() tự chờ TPHG cycle BME680 nội tại (lib bme680) — đó là
    //             measurement duration của cảm biến, KHÔNG phải timing nghiệp vụ.
    //         (2) s_filters.process(local) — Filters.hpp (sanity + EMA/SMA, in-place).
    //         (3) bool synced = s_network.isTimeSynced();  // XM-5, NetworkManager.hpp
    //             s_fusion.process(local, synced) — DataFusion.hpp (AQI/THI/CO2 cat,
    //             drift self-check, computeAlertLevel → ghi alert_level/reason/
    //             flags/calib_*). process() thuần O(1), không I/O (DataFusion.hpp §12).
    //         (4) Cập nhật s_shared dưới mutex (XM-7):
    //               xSemaphoreTake(s_shared_mtx, portMAX_DELAY);
    //               s_shared = local;
    //               xSemaphoreGive(s_shared_mtx);
    //         (5) Publish snapshot cho consumer (XM-9, SAU khi unlock):
    //               xQueueOverwrite(s_airq, &local);
    //         (6) LÁI GPIO cảnh báo từ local.alert_level (mục G/XM-13/14) — ngay
    //             tại đây để đạt độ trễ ≤3s.
    //         (7) EDGE detect calib_needed false→true (so với biến static chu-kỳ-
    //             trước) → s_storage.logEvent(CALIB_NEEDED_SET, local) (StorageHelper.hpp
    //             §4.3; non-blocking enqueue).
    //         (8) (Tuỳ chọn) s_storage.logData(local) — nhưng NÊN để taskStorage tự
    //             gating theo Cfg::SD_LOG_INTERVAL_MS (StorageHelper.hpp §3.1) để
    //             KHÔNG ghi mỗi 300ms (mòn thẻ). Pipeline chỉ enqueue O(1).
    //         (9) esp_task_wdt_reset() — feed WDT (mục I).
    //
    // H.2 taskDisplay — LCD (cadence 2–5s; XM-1/XM-4)
    //     · Tạo: xTaskCreate(taskDisplay,"display",Cfg::TASK_STACK_DISPLAY_WORDS,
    //       nullptr, Cfg::TASK_PRIO_DISPLAY(3), &h).
    //     · NHỊP: chu kỳ trong [Cfg::LCD_MIN_INTERVAL_MS=2000, LCD_MAX_INTERVAL_MS
    //       =5000], đồng bộ Cfg::DISPLAY_UPDATE_INTERVAL_MS (Kconfig, đã nằm trong
    //       dải). Dùng esp_timer/queue-timeout (KHÔNG vTaskDelay nghiệp vụ —
    //       vTaskDelay ở taskDisplay chỉ là cách chờ nhịp hiển thị PHỤ, ưu tiên
    //       esp_timer để nhất quán CLAUDE.md §4).
    //     · MỖI CHU KỲ:
    //         AirData snap; xQueuePeek(s_airq,&snap,0);  // XM-9 (read-only snapshot)
    //         s_display.update(snap);   // vẽ trang hiện tại — XM-1
    //         s_display.tick();         // rotate trang / blink CALIB_ALERT — XM-1
    //         esp_task_wdt_reset();
    //     · Truy cập I2C của Display tự được i2cdev mutex per-port serialize với
    //       readAll() của taskSensor (CLAUDE.md §4) — KHÔNG cần mutex riêng cho bus.
    //     · showMessage() có thể gọi từ taskNetwork (vd "Mat mang") — overlay tự
    //       giữ ≥ Cfg::LCD_OVERLAY_MIN_MS (DisplayManager.hpp).
    //
    // H.3 taskNetwork — MQTT publish / alert / offline buffer / event log
    //     · Tạo: xTaskCreate(taskNetwork,"network",Cfg::TASK_STACK_NETWORK_WORDS,
    //       nullptr, Cfg::TASK_PRIO_NETWORK(4), &h).
    //     · GIỮ static: AlertLevel last_published_level=NONE (XM-11);
    //       bool prev_connected=false (edge MQTT); uint8_t prev_alert_level,
    //       prev_alert_flags (edge cho Event Log §4.3).
    //     · MỖI CHU KỲ (nhịp ~ Cfg::SENSOR_READ_INTERVAL_MS hoặc riêng; esp_timer):
    //         AirData snap; xQueuePeek(s_airq,&snap,0);
    //         bool conn = s_network.isConnected();   // NetworkManager.hpp
    //         (a) EDGE kết nối: conn != prev_connected →
    //               conn ? logEvent(MQTT_CONNECTED) : logEvent(MQTT_DISCONNECTED)
    //               (StorageHelper.hpp §4.3); nếu false→true → s_storage.drainOffline(
    //               publish_fn) với publish_fn = [](const AirData&d){return
    //               s_network.publishData(d);} (StorageHelper.hpp §5.3); prev_connected=conn.
    //         (b) PUBLISH DỮ LIỆU: nếu conn → err = s_network.publishData(snap)
    //               (buildJson đã chèn alert_level/reason/flags/calib_reason/
    //               calib_alert — XM-10, NetworkManager.hpp). Nếu trả
    //               ESP_ERR_INVALID_STATE (MQTT rớt) → s_storage.bufferOffline(snap)
    //               (Offline Buffer, StorageHelper.hpp §5.2; CLAUDE.md §4).
    //         (c) ALERT debounce (XM-11): snap.alert_level != last_published_level →
    //               s_network.publishAlert(snap) (≤ ALERT_MAX_LATENCY_MS) +
    //               logEvent(ALERT_LEVEL_CHANGED, snap); last_published_level=...
    //         (d) EDGE alert_reason/flags đổi (≠ prev) → logEvent(ALERT_REASON_CHANGED,
    //               snap); cập nhật prev_*.
    //         (e) esp_task_wdt_reset();
    //     · publishData/publishAlert đọc CHỈ snapshot (không chạm s_shared) → KHÔNG
    //       cần mutex; airDataQueue (overwrite/peek) là kênh đọc an toàn (XM-9).
    //
    // H.4 taskStorage — KHÔNG tạo ở main.cpp: StorageHelper::init() TỰ tạo task
    //     ghi SD duy nhất (prio Cfg::TASK_PRIO_STORAGE=2, thấp nhất) + queue
    //     (StorageHelper.hpp §6). main.cpp chỉ gọi API enqueue non-blocking
    //     (logData/logEvent/bufferOffline/drainOffline). I/O SD chậm KHÔNG cản
    //     pipeline ≤300ms hay cảnh báo ≤3s (StorageHelper.hpp §6.2/§11). storageTask
    //     tự feed WDT trong vòng lặp của nó (nếu được add vào TWDT — mục I).
    //
    // H.5 (Khuyến nghị) logEvent(SYSTEM_BOOT, snap) một lần sau khi tạo task xong —
    //     mốc khởi động phục vụ truy vết + ghi nhận lý do reset (mục I) vào Event Log.
    //
    // H.6 FLUSH BASELINE NVS (deferred — DataFusion.hpp §6.5/§12)
    //     · s_fusion.process() KHÔNG ghi NVS trong pipeline (≤300ms): baseline-init
    //       (3 lần lúc boot khi từng cảm biến qua warmup) chỉ chốt RAM + bật cờ
    //       dirty. main PHẢI gọi s_fusion.persistBaselineIfDirty() ĐỊNH KỲ ở task
    //       ƯU TIÊN THẤP (KHÔNG taskSensor) để thực hiện nvs_commit chậm ngoài
    //       đường tới hạn — ví dụ cuối mỗi chu kỳ taskNetwork (prio 4) hoặc
    //       taskDisplay (prio 3), hoặc một timer chậm. Hàm idempotent (không dirty
    //       → trả ngay), nên gọi dư nhịp vô hại. Block flash chỉ xảy ra ~3 lần lúc
    //       boot rồi thôi (confirmRecalibration ghi NVS đồng bộ riêng, không qua cờ).
    //     · KHÔNG đặt trong taskSensor: sẽ kéo nvs_commit (~hàng chục–trăm ms) trở
    //       lại pipeline ≤300ms — đúng thứ vừa tách ra để tránh WDT/NFR (mục I.3).
    //
    // ================================================================
    // I. WATCHDOG TIMER — XỬ LÝ & PHÒNG WDT RESET (yêu cầu kỹ thuật #6)
    // ----------------------------------------------------------------
    //   I.1 PHÁT HIỆN reset do WDT ở đầu app_main(): esp_reset_reason()
    //       ("esp_system.h"). Nếu == ESP_RST_TASK_WDT (Task WDT) hoặc
    //       ESP_RST_INT_WDT (Interrupt WDT) hoặc ESP_RST_WDT → ESP_LOGE("main",
    //       "Recovered from WDT reset (reason=%d)") + (sau khi StorageHelper sẵn
    //       sàng) logEvent(SYSTEM_BOOT, snap) kèm ghi chú để truy vết (CLAUDE.md
    //       §6.5 "mọi panic/exception phải investigate"). KHÔNG chỉ reset im lặng.
    //   I.2 ĐĂNG KÝ Task WDT (TWDT) cho MỌI task có vòng lặp vô hạn dài:
    //         esp_task_wdt_add(NULL) ở ĐẦU thân task (taskSensor/taskDisplay/
    //         taskNetwork; storageTask trong StorageHelper nếu I/O bị bao bởi
    //         timeout an toàn). Mỗi vòng lặp gọi esp_task_wdt_reset() (xem mục H).
    //       · Nếu CONFIG_ESP_TASK_WDT_INIT bật mặc định (watch IDLE0/IDLE1): các
    //         task prio cao phải nhường CPU đủ để IDLE chạy — pipeline ≤300ms +
    //         chờ-trên-tín-hiệu (semaphore/notify) đảm bảo điều này; TUYỆT ĐỐI
    //         không busy-spin giữ CPU.
    //   I.3 NGƯỠNG TIMEOUT: cấu hình qua menuconfig (CONFIG_ESP_TASK_WDT_TIMEOUT_S).
    //       Mọi task subscribe PHẢI reset trong chu kỳ NGẮN HƠN timeout. taskSensor
    //       (≤300ms) + taskDisplay/taskNetwork (vài giây) → đặt timeout > nhịp task
    //       chậm nhất subscribe (vd ≥ LCD_MAX_INTERVAL_MS). storageTask có thể
    //       block fsync/rename hàng trăm ms: HOẶC không add vào TWDT, HOẶC timeout
    //       đủ rộng — tránh false-positive WDT khi ghi SD chậm (StorageHelper.hpp §6).
    //   I.4 INTERRUPT WDT (INT_WDT): không tắt ngắt / không giữ spinlock quá lâu.
    //       Truy cập I2C (BME680/PCF8574) ĐI QUA i2cdev (mutex per-port, KHÔNG
    //       critical section dài — CLAUDE.md §4); KHÔNG ôm I2C_DEV_TAKE_MUTEX rồi
    //       làm việc nặng. ISR (nếu có) phải ngắn.
    //   I.5 (Tuỳ chọn) Core Dump khi panic (CLAUDE.md §6.4): bật partition coredump
    //       qua menuconfig để phân tích sau bằng idf.py coredump-info/-debug —
    //       không thay thế việc feed WDT đúng cách.
    //
    // ================================================================
    // J. TỐI ƯU CÔNG SUẤT (CHƯA triển khai — yêu cầu #9: GIỮ NGUYÊN cho tương lai)
    // ----------------------------------------------------------------
    //   Xem khối [PM-1] ở đầu app_main(). Production mode HIỆN TẠI KHÔNG bật
    //   esp_pm_configure()/light-sleep; chỉ giữ esp_wifi_set_ps(WIFI_PS_MODEM)
    //   (đã có trong NetworkManager::initWifi). Phần CPU/peripheral idle để dành
    //   thêm sau khi đo công suất thực tế (tickless idle có thể ảnh hưởng timing
    //   UART PMS5003 / I2C — cần test trước khi bật).
    //
    // ================================================================
    // K. THIẾT KẾ CHO DEBUG MCU QUA ESP-PROG/JTAG (CLAUDE.md §6 — yêu cầu #10)
    // ----------------------------------------------------------------
    //   K.1 KHÔNG dùng GPIO12/13/14/15 (TDI/TCK/TMS/TDO — CLAUDE.md §6.2) cho bất
    //       kỳ ngoại vi nào: pin map dự án (I2C 21/22, UART 16/17/4, ADC 34, SPI
    //       18/19/23/5, LED 25/26/27, Buzzer 32) ĐÃ né hoàn toàn — main.cpp KHÔNG
    //       gpio_config() các chân JTAG. (Kiểm tra lại khi thêm ngoại vi mới.)
    //   K.2 GIỮ NGUYÊN mọi ESP_LOGI/W/E (TAG="main") — KHÔNG xoá/comment khi commit
    //       (CLAUDE.md §6.5): log là nguồn debug song song JTAG.
    //   K.3 Biến nên đặt breakpoint/watch qua JTAG: s_shared (AirData), snapshot
    //       trong từng task, alert_level/alert_reason, calib_needed/calib_reason,
    //       trạng thái s_network.isConnected()/isTimeSynced(), s_storage.isMounted()/
    //       offlineCount(). Đặt instance ở file-scope (mục B) giúp GDB truy cập ổn định.
    //   K.4 STACK CHECK trước release (CLAUDE.md §6.5): trong build debug, in
    //       uxTaskGetStackHighWaterMark() của từng task (sensor/display/network/
    //       storage) để chỉnh Cfg::TASK_STACK_*_WORDS, tránh stack overflow.
    //   K.5 Khởi tạo gpio/driver tuần tự, có ESP_LOGI mốc từng bước (mục D) để khi
    //       dừng tại breakpoint trong app_main() (ESP-IDF dừng ở app_main mặc định,
    //       CLAUDE.md §6.3) có thể bước qua từng init và quan sát giá trị trả về.
    //
    // ================================================================
    // L. THAM CHIẾU NGUỒN HÀM/CALLBACK LIÊN-MODULE (tóm tắt — yêu cầu #3/#7)
    // ----------------------------------------------------------------
    //   SensorManager.hpp : init(), readAll(AirData&), isFullyReady().
    //   Filters.hpp       : process(AirData&), reset().
    //   DataFusion.hpp    : init(), process(AirData&,bool synced),
    //                       confirmRecalibration(AirData&,bool synced),
    //                       persistBaselineIfDirty() [gọi ĐỊNH KỲ ở task ưu tiên
    //                       THẤP — flush baseline-init xuống NVS, KHÔNG trong
    //                       taskSensor; xem H.6],
    //                       getAlertLevel()/calibReason() (debug), enum AlertLevel.
    //   DisplayManager.hpp: init(), update(const AirData&), tick(),
    //                       showMessage(l1,l2), setBacklight(bool) (chưa dùng).
    //   NetworkManager.hpp: setCommandCallback(cb) [TRƯỚC init], init(),
    //                       publishData(const AirData&), publishAlert(const AirData&),
    //                       isConnected(), isTimeSynced(); CommandCallback =
    //                       std::function<void(const char*)>.
    //   StorageHelper.hpp : init(), logData(), logEvent(EventType,snap),
    //                       bufferOffline(), drainOffline(PublishFn), isMounted(),
    //                       offlineCount(); EventType::*, PublishFn =
    //                       std::function<esp_err_t(const AirData&)>.
    //   config.hpp (Cfg)  : toàn bộ hằng (mục C). DataStructures.hpp : AirData.
    //   ESP-IDF           : nvs_flash_* (đã gọi), esp_timer_*, esp_task_wdt_*,
    //                       gpio_config(), esp_reset_reason(), time()/esp_timer_get_time().
    ESP_LOGI(TAG, "=== PRODUCTION MODE ===");

#endif
}

#pragma once

// ============================================================
// config.hpp — Xương sống tham số toàn dự án ENV-AQ-01
//
// Quy tắc sử dụng:
//  - Tất cả module #include "config.hpp" để lấy hằng số.
//  - Tham số có thể tuỳ chỉnh qua menuconfig → Kconfig.projbuild
//    (CONFIG_*). Phần còn lại là hằng vật lý/phần cứng cố định.
//  - Không hardcode bất kỳ giá trị nào trong file .cpp/.hpp khác.
// ============================================================

#include "sdkconfig.h"
#include <cstdint>
#include <cstddef>

namespace Cfg {

// ============================================================
// 1. I2C BUS
// ============================================================

inline constexpr int      I2C_SDA_PIN       = 21;
inline constexpr int      I2C_SCL_PIN       = 22;
inline constexpr uint32_t I2C_FREQ_HZ       = 400'000;   // Fast Mode 400 kHz

// ---- Địa chỉ thiết bị trên bus I2C dùng chung ----
inline constexpr uint8_t  BME680_I2C_ADDR   = 0x76;      // SDO = GND
inline constexpr uint8_t  PCF8574_I2C_ADDR  = 0x20;      // A0/A1/A2 = GND

// ============================================================
// 2. PMS5003 — UART
// ============================================================

inline constexpr int      PMS_UART_PORT     = 2;          // UART_NUM_2 — GPIO16/17 là default pins của UART2, không remap
inline constexpr int      PMS_UART_RX_PIN   = 16;
inline constexpr int      PMS_UART_TX_PIN   = 17;
inline constexpr int      PMS_SET_PIN       = 4;          // LOW = sleep, HIGH = active
inline constexpr int      PMS_UART_BAUD     = 9600;
inline constexpr int      PMS_FRAME_LEN     = 32;         // Độ dài frame chuẩn PMS5003
inline constexpr int      PMS_VALID_STREAK_OK = 3;        // Số frame OK liên tiếp cần có trước khi cờ ready=true

// ============================================================
// 2b. BME680 — Cấu hình đo (Bosch ref + thư viện esp-idf-lib/bme680)
// ============================================================

inline constexpr uint16_t BME680_HEATER_TEMP_C  = 320;    // °C — vùng gas-sense tối ưu
inline constexpr uint16_t BME680_HEATER_DUR_MS  = 150;    // Thời gian heat trước khi đo gas
inline constexpr int16_t  BME680_AMBIENT_TEMP_C = 25;     // Ambient mặc định, được cập nhật runtime

// ============================================================
// 3. MQ-135 — ADC
// ============================================================

// GPIO34 = ADC1 Channel 6 trên ESP32
// Dùng ADC1 (không dùng ADC2) để tránh xung đột khi WiFi hoạt động
inline constexpr int      MQ135_ADC_UNIT    = 0;          // ADC_UNIT_1
inline constexpr int      MQ135_ADC_CHANNEL = 6;          // ADC1_CHANNEL_6 → GPIO34
inline constexpr int      MQ135_ADC_ATTEN   = 3;          // ADC_ATTEN_DB_12 (0–3.3V)
inline constexpr int      MQ135_ADC_BITWIDTH = 12;        // ADC_BITWIDTH_12

// Hệ số chuyển đổi điện áp → nồng độ (cần hiệu chuẩn thực tế)
inline constexpr float    MQ135_VREF        = 3.3f;       // V
inline constexpr float    MQ135_RL_KOHM     = 10.0f;      // Điện trở tải (kΩ)
inline constexpr float    MQ135_RO_KOHM     = 10.0f;      // Điện trở cơ sở trong không khí sạch

// ============================================================
// 4. SD CARD — SPI
// ============================================================

inline constexpr int      SD_SPI_SCK_PIN    = 18;
inline constexpr int      SD_SPI_MISO_PIN   = 19;
inline constexpr int      SD_SPI_MOSI_PIN   = 23;
inline constexpr int      SD_SPI_CS_PIN     = 5;
inline constexpr int      SD_SPI_HOST       = 1;          // SPI2_HOST

inline constexpr const char *SD_MOUNT_POINT    = "/sdcard";
inline constexpr const char *SD_LOG_FILE       = "/sdcard/airdata.csv";
inline constexpr const char *SD_OFFLINE_QUEUE  = "/sdcard/offline_queue.bin";

// ============================================================
// 5. OUTPUT CẢNH BÁO — GPIO
// ============================================================

inline constexpr int      LED_RED_PIN       = 25;
inline constexpr int      LED_YELLOW_PIN    = 26;
inline constexpr int      LED_GREEN_PIN     = 27;
inline constexpr int      BUZZER_PIN        = 32;

// ============================================================
// 6. TIMING (đến từ Kconfig — có thể chỉnh qua menuconfig)
// ============================================================

inline constexpr uint32_t SENSOR_READ_INTERVAL_MS    = CONFIG_SENSOR_READ_INTERVAL_MS;
inline constexpr uint32_t DISPLAY_UPDATE_INTERVAL_MS = CONFIG_DISPLAY_UPDATE_INTERVAL_MS;

// NFR cứng — không thay đổi qua menuconfig
inline constexpr uint32_t MAX_CYCLE_TIME_MS          = 300;    // Tổng chu kỳ đọc+xử lý
inline constexpr uint32_t ALERT_MAX_LATENCY_MS       = 3'000;  // Cảnh báo phải phát trong 3s
inline constexpr uint32_t LCD_MIN_INTERVAL_MS        = 2'000;  // Giới hạn dưới cập nhật LCD
inline constexpr uint32_t LCD_MAX_INTERVAL_MS        = 5'000;  // Giới hạn trên cập nhật LCD

// ---- Warmup / Stabilization các cảm biến ----
// Tính từ thời điểm SensorManager::init() trả ESP_OK. Trước khi hết
// warmup, các giá trị tương ứng vẫn được đọc nhưng cờ *_ready=false
// để module hạ nguồn coi là RÁC, không feed vào AQI / cảnh báo / log.
inline constexpr uint32_t BME680_WARMUP_MS           = 300'000;  // MOx đạt cân bằng hóa học (~5 phút)
inline constexpr uint32_t PMS5003_WARMUP_MS          = 30'000;   // Fan đạt tốc quay danh định
inline constexpr uint32_t MQ135_WARMUP_MS            = 1'200'000; // MOX preheat theo datasheet (20 phút)

// ============================================================
// 7. AQI — NGƯỠNG PHÂN LOẠI (PM2.5, µg/m³, tiêu chuẩn VN)
// ============================================================

// Giá trị đến từ Kconfig để có thể điều chỉnh theo quy định mới
inline constexpr float    AQI_GOOD_MAX     = static_cast<float>(CONFIG_AQI_GOOD_MAX);
inline constexpr float    AQI_MODERATE_MAX = static_cast<float>(CONFIG_AQI_MODERATE_MAX);
inline constexpr float    AQI_POOR_MAX     = static_cast<float>(CONFIG_AQI_POOR_MAX);
inline constexpr float    AQI_BAD_MAX      = static_cast<float>(CONFIG_AQI_BAD_MAX);
inline constexpr float    AQI_VERY_BAD_MAX = static_cast<float>(CONFIG_AQI_VERY_BAD_MAX);
// > AQI_VERY_BAD_MAX → category 5 (Nguy hại)

// ============================================================
// 8. NGƯỠNG CẢNH BÁO BUZZER / LED
// ============================================================

inline constexpr float    ALERT_CO2_PPM   = static_cast<float>(CONFIG_ALERT_CO2_PPM);
inline constexpr float    ALERT_PM25_UGM3 = static_cast<float>(CONFIG_ALERT_PM25_UGM3);

// ============================================================
// 9. MẠNG — WiFi & MQTT
// ============================================================

inline constexpr const char *WIFI_SSID         = CONFIG_WIFI_SSID;
inline constexpr const char *WIFI_PASSWORD     = CONFIG_WIFI_PASSWORD;
inline constexpr const char *MQTT_BROKER_URL   = CONFIG_MQTT_BROKER_URL;
inline constexpr const char *MQTT_TOPIC_DATA   = CONFIG_MQTT_TOPIC;
inline constexpr const char *MQTT_TOPIC_ALERT  = CONFIG_MQTT_TOPIC_ALERT;
inline constexpr const char *MQTT_TOPIC_CMD    = CONFIG_MQTT_TOPIC_CMD;  // subscribe — nhận lệnh từ broker
inline constexpr const char *NTP_SERVER_URL    = CONFIG_NTP_SERVER_URL;  // SNTP server
inline constexpr int         MQTT_JSON_BUF_LEN = 512;   // Byte — đủ cho toàn bộ AirData

// ============================================================
// 10. BỘ LỌC NHIỄU (Filters)
// ============================================================

inline constexpr size_t   FILTER_WINDOW_SIZE   = 5;     // Cửa sổ moving average

// Kalman — giá trị mặc định cho tất cả tín hiệu
inline constexpr float    KALMAN_PROCESS_NOISE = 0.01f; // Q: dao động mô hình
inline constexpr float    KALMAN_MEAS_NOISE    = 0.1f;  // R: nhiễu đo lường

// Outlier rejection — loại bỏ nếu lệch quá N lần độ lệch chuẩn
inline constexpr float    OUTLIER_SIGMA        = 3.0f;

// ============================================================
// 11. HIỆU CHUẨN & DRIFT SELF-CHECK
// ============================================================

// NFR: lệch > 10% so với baseline → set calib_needed = true + MQTT alert
inline constexpr float    DRIFT_THRESHOLD_PCT  = 10.0f;

// NFR: chu kỳ tái hiệu chuẩn tối đa 30 ngày
inline constexpr int32_t  CALIB_INTERVAL_DAYS  = 30;
inline constexpr int64_t  CALIB_INTERVAL_SEC   =
    static_cast<int64_t>(CALIB_INTERVAL_DAYS) * 86'400LL;

// Độ chính xác sau hiệu chuẩn (dùng trong kiểm thử nghiệm thu)
inline constexpr float    ACCURACY_TEMP_C      = 0.5f;  // ±°C
inline constexpr float    ACCURACY_HUMI_RH     = 3.0f;  // ±%RH
inline constexpr float    ACCURACY_INDEX_PCT   = 10.0f; // AQI/TVOC/Comfort ±%

// ============================================================
// 12. NVS — KEYS LƯU HIỆU CHUẨN
// ============================================================

inline constexpr const char *NVS_NAMESPACE         = "aq01_calib";
inline constexpr const char *NVS_KEY_BL_TEMP       = "bl_temp";
inline constexpr const char *NVS_KEY_BL_HUMI       = "bl_humi";
inline constexpr const char *NVS_KEY_BL_PM25       = "bl_pm25";
inline constexpr const char *NVS_KEY_BL_CO2        = "bl_co2";
inline constexpr const char *NVS_KEY_LAST_CALIB_TS = "last_calib_ts";

// ============================================================
// 13. FREERTOS TASK CONFIG
// ============================================================

inline constexpr uint32_t TASK_STACK_SENSOR_WORDS   = 4096;
inline constexpr uint32_t TASK_STACK_NETWORK_WORDS  = 6144;
inline constexpr uint32_t TASK_STACK_DISPLAY_WORDS  = 3072;
inline constexpr uint32_t TASK_STACK_STORAGE_WORDS  = 4096;

inline constexpr uint32_t TASK_PRIO_SENSOR          = 5;
inline constexpr uint32_t TASK_PRIO_NETWORK         = 4;
inline constexpr uint32_t TASK_PRIO_DISPLAY         = 3;
inline constexpr uint32_t TASK_PRIO_STORAGE         = 2;

} // namespace Cfg

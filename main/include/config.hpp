#pragma once

#include "sdkconfig.h"
#include <cstdint>
#include <cstddef>

namespace Cfg {

// ============================================================
// 1. I2C BUS
// ============================================================

inline constexpr int      I2C_SDA_PIN       = 21;
inline constexpr int      I2C_SCL_PIN       = 22;
inline constexpr uint32_t I2C_FREQ_HZ       = 400'000;

inline constexpr int      I2C_PORT          = 0;

inline constexpr uint8_t  BME680_I2C_ADDR   = 0x76;
inline constexpr uint8_t  PCF8574_I2C_ADDR  = 0x27;

// ============================================================
// 2. PMS5003 — UART
// ============================================================

inline constexpr int      PMS_UART_PORT     = 2;
inline constexpr int      PMS_UART_RX_PIN   = 16;
inline constexpr int      PMS_UART_TX_PIN   = 17;
inline constexpr int      PMS_SET_PIN       = 4;
inline constexpr int      PMS_UART_BAUD     = 9600;
inline constexpr int      PMS_FRAME_LEN     = 32;
inline constexpr int      PMS_VALID_STREAK_OK = 3;

// ============================================================
// 3. MQ-135 — ADC
// ============================================================

inline constexpr int      MQ135_ADC_UNIT    = 0;
inline constexpr int      MQ135_ADC_CHANNEL = 6;
inline constexpr int      MQ135_ADC_ATTEN   = 3;
inline constexpr int      MQ135_ADC_BITWIDTH = 12;
inline constexpr uint32_t MQ135_ADC_DEFAULT_VREF_MV = 1100;

inline constexpr float    MQ135_VREF        = 3.3f;
inline constexpr float    MQ135_RL_KOHM     = 1.0f;
inline constexpr float    MQ135_RO_KOHM     = 37.10f;  

inline constexpr float    MQ135_SENSOR_VCC      = 5.0f;
inline constexpr float    MQ135_DIVIDER_RA_KOHM = 2.0f;
inline constexpr float    MQ135_DIVIDER_RB_KOHM = 3.0f;
inline constexpr float    MQ135_DIVIDER_RATIO   =
    MQ135_DIVIDER_RB_KOHM / (MQ135_DIVIDER_RA_KOHM + MQ135_DIVIDER_RB_KOHM);

inline constexpr float    MQ135_CURVE_A     = 110.47f;
inline constexpr float    MQ135_CURVE_B     = -2.8612f;

inline constexpr float    MQ135_TREF_C      = 20.0f;
inline constexpr float    MQ135_CORA        =  0.00035f;
inline constexpr float    MQ135_CORB        =  0.02718f;
inline constexpr float    MQ135_CORC        =  1.39538f;
inline constexpr float    MQ135_CORD        =  0.0018f;
inline constexpr float    MQ135_CORE        = -0.003333333f;
inline constexpr float    MQ135_CORF        = -0.001923077f;
inline constexpr float    MQ135_CORG        =  1.130128205f;

// ============================================================
// 4. SD CARD — SPI
// ============================================================

inline constexpr int      SD_SPI_SCK_PIN    = 18;
inline constexpr int      SD_SPI_MISO_PIN   = 19;
inline constexpr int      SD_SPI_MOSI_PIN   = 23;
inline constexpr int      SD_SPI_CS_PIN     = 5;
inline constexpr int      SD_SPI_HOST       = 1;
inline constexpr uint32_t SD_SHUTDOWN_TIMEOUT_MS = 3'000;
inline constexpr uint32_t SHUTDOWN_LED_BLINK_MS  = 300;

inline constexpr const char *SD_MOUNT_POINT    = "/sdcard";
inline constexpr const char *SD_LOG_FILE       = "/sdcard/airdata.csv";
inline constexpr const char *SD_OFFLINE_QUEUE  = "/sdcard/offline_queue.bin";

inline constexpr const char *SD_EVENT_LOG_FILE = "/sdcard/events.csv";
inline constexpr const char *SD_OFFLINE_HEAD   = "/sdcard/offline_queue.hdr";

inline constexpr uint32_t SD_LOG_INTERVAL_MS   = CONFIG_SD_LOG_INTERVAL_MS;

inline constexpr uint32_t SD_FSYNC_EVERY_N_ROWS = 1;

inline constexpr int      SD_MAX_OPEN_FILES     = 4;

inline constexpr long      SD_LOG_MAX_BYTES     = 5L * 1024 * 1024;
inline constexpr int       SD_LOG_MAX_FILES     = 5;

inline constexpr long      SD_EVENT_MAX_BYTES   = 1L * 1024 * 1024;

inline constexpr size_t    STORAGE_QUEUE_LEN    = 16;

inline constexpr size_t    OFFLINE_QUEUE_MAX_RECORDS = 2000;

inline constexpr size_t    OFFLINE_DRAIN_BATCH  = 20;

inline constexpr uint32_t  OFFLINE_DRAIN_PACE_MS = 1000;

inline constexpr uint32_t  OFFLINE_MAGIC          = 0x31305141;
inline constexpr uint16_t  OFFLINE_FORMAT_VERSION = 1;

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

inline constexpr uint32_t MAX_CYCLE_TIME_MS          = 300;
inline constexpr uint32_t ALERT_MAX_LATENCY_MS       = 3'000;
inline constexpr uint32_t LCD_MIN_INTERVAL_MS        = 2'000;
inline constexpr uint32_t LCD_MAX_INTERVAL_MS        = 5'000;

inline constexpr uint32_t SENSOR_MIN_INTERVAL_MS     = 500;
inline constexpr uint32_t SENSOR_MAX_INTERVAL_MS     = ALERT_MAX_LATENCY_MS - MAX_CYCLE_TIME_MS;

inline constexpr uint32_t LCD_OVERLAY_MIN_MS         = 5'000;

inline constexpr uint32_t BME680_WARMUP_MS           = 2'000;
inline constexpr uint32_t PMS5003_WARMUP_MS          = 30'000;
inline constexpr uint32_t MQ135_WARMUP_MS            = 1'200'000;

// ============================================================
// 7. AQI — ĐIỂM GÃY NỘI SUY & NGƯỠNG PHÂN LOẠI (tiêu chuẩn VN)
// ============================================================

inline constexpr float    AQI_INDEX_BP[]   = {0.0f, 50.0f, 100.0f, 150.0f, 200.0f, 300.0f, 500.0f};
inline constexpr float    AQI_PM25_BP[]    = {0.0f, 25.0f, 50.0f, 80.0f, 150.0f, 250.0f, 500.0f};
inline constexpr float    AQI_PM10_BP[]    = {0.0f, 50.0f, 150.0f, 250.0f, 350.0f, 420.0f, 600.0f};
inline constexpr float    AQI_MAX_INDEX    = 500.0f;

inline constexpr float    AQI_GOOD_MAX     = static_cast<float>(CONFIG_AQI_GOOD_MAX);
inline constexpr float    AQI_MODERATE_MAX = static_cast<float>(CONFIG_AQI_MODERATE_MAX);
inline constexpr float    AQI_POOR_MAX     = static_cast<float>(CONFIG_AQI_POOR_MAX);
inline constexpr float    AQI_BAD_MAX      = static_cast<float>(CONFIG_AQI_BAD_MAX);
inline constexpr float    AQI_VERY_BAD_MAX = static_cast<float>(CONFIG_AQI_VERY_BAD_MAX);

// ============================================================
// 8. COMFORT INDEX (THI) — DataFusion / DisplayManager
// ============================================================

inline constexpr float    COMFORT_DI_K1       = 0.55f;
inline constexpr float    COMFORT_DI_K2       = 14.5f;
inline constexpr float    COMFORT_DI_RH_SCALE = 0.01f;

inline constexpr float    COMFORT_DI_OK       = 21.0f;
inline constexpr float    COMFORT_DI_WARM     = 24.0f;
inline constexpr float    COMFORT_DI_HOT      = 27.0f;
inline constexpr float    COMFORT_DI_SEVERE   = 29.0f;
inline constexpr float    COMFORT_DI_DANGER   = 32.0f;

// ============================================================
// 9. CO2 — DẢI PHÂN LOẠI ĐỊNH TÍNH (DisplayManager)
// ============================================================

inline constexpr float    CO2_GOOD_MAX     = 1000.0f;
inline constexpr float    CO2_MODERATE_MAX = 2000.0f;

// ============================================================
// 10. NGƯỠNG CẢNH BÁO BUZZER / LED
// ============================================================

inline constexpr float    ALERT_CO2_PPM          = static_cast<float>(CONFIG_ALERT_CO2_PPM);
inline constexpr float    ALERT_CO2_CRITICAL_PPM = static_cast<float>(CONFIG_ALERT_CO2_CRITICAL_PPM);

// ============================================================
// 11. MẠNG — WiFi & MQTT
// ============================================================

inline constexpr const char *WIFI_SSID         = CONFIG_WIFI_SSID;
inline constexpr const char *WIFI_PASSWORD     = CONFIG_WIFI_PASSWORD;
inline constexpr const char *WIFI_DNS_BACKUP   = CONFIG_WIFI_DNS_BACKUP;
inline constexpr const char *MQTT_BROKER_URL   = CONFIG_MQTT_BROKER_URL;
inline constexpr const char *MQTT_TOPIC_DATA   = CONFIG_MQTT_TOPIC;
inline constexpr const char *MQTT_TOPIC_CMD    = CONFIG_MQTT_TOPIC_CMD;
inline constexpr const char *MQTT_ACCESS_TOKEN = CONFIG_MQTT_ACCESS_TOKEN;
inline constexpr const char *NTP_SERVER_URL    = CONFIG_NTP_SERVER_URL;
inline constexpr int         MQTT_JSON_BUF_LEN = 640;

inline constexpr uint32_t    MQTT_OUTBOX_LIMIT_BYTES = 16 * 1024;

// ============================================================
// 12. BỘ LỌC NHIỄU (Filters) — EMA/SMA hai tầng
// ============================================================

inline constexpr size_t   FILTER_WINDOW_SIZE   = 5;

inline constexpr float    EMA_ALPHA_TEMP       = 0.10f;
inline constexpr float    EMA_ALPHA_PRESS      = 0.10f;
inline constexpr float    EMA_ALPHA_CO2        = 0.20f;
inline constexpr float    EMA_ALPHA_PM1        = 0.50f;
inline constexpr float    EMA_ALPHA_PM25       = 0.50f;
inline constexpr float    EMA_ALPHA_PM10       = 0.30f;

inline constexpr float    SANITY_TEMP_MIN      =   -40.0f;
inline constexpr float    SANITY_TEMP_MAX      =    85.0f;
inline constexpr float    SANITY_HUMI_MIN      =     0.0f;
inline constexpr float    SANITY_HUMI_MAX      =   100.0f;
inline constexpr float    SANITY_PRESS_MIN     =   300.0f;
inline constexpr float    SANITY_PRESS_MAX     =  1100.0f;
inline constexpr float    SANITY_CO2_MIN       =   100.0f;
inline constexpr float    SANITY_CO2_MAX       =  5000.0f;
inline constexpr float    SANITY_PM_MIN        =     0.0f;
inline constexpr float    SANITY_PM_MAX        =   500.0f;

// ============================================================
// 13. HIỆU CHUẨN & DRIFT SELF-CHECK
// ============================================================

inline constexpr float    DRIFT_THRESHOLD_PCT  = 10.0f;

inline constexpr int32_t  CALIB_INTERVAL_DAYS  = 30;
inline constexpr int64_t  CALIB_INTERVAL_SEC   =
    static_cast<int64_t>(CALIB_INTERVAL_DAYS) * 86'400LL;

inline constexpr float    ACCURACY_TEMP_C      = 0.5f;
inline constexpr float    ACCURACY_HUMI_RH     = 3.0f;
inline constexpr float    ACCURACY_INDEX_PCT   = 10.0f;

inline constexpr float    DRIFT_TEMP_ABS_C     = ACCURACY_TEMP_C;

inline constexpr float    DRIFT_HUMI_ABS_PCT   = ACCURACY_HUMI_RH;

// ============================================================
// 14. NVS — KEYS LƯU HIỆU CHUẨN
// ============================================================

inline constexpr const char *NVS_NAMESPACE         = "aq01_calib";
inline constexpr const char *NVS_KEY_BL_TEMP       = "bl_temp";
inline constexpr const char *NVS_KEY_BL_HUMI       = "bl_humi";
inline constexpr const char *NVS_KEY_BL_PM25       = "bl_pm25";
inline constexpr const char *NVS_KEY_BL_PM10       = "bl_pm10";
inline constexpr const char *NVS_KEY_BL_CO2        = "bl_co2";
inline constexpr const char *NVS_KEY_BL_PRESSURE   = "bl_press";
inline constexpr const char *NVS_KEY_BL_AQI        = "bl_aqi";
inline constexpr const char *NVS_KEY_BL_COMFORT    = "bl_comfort";
inline constexpr const char *NVS_KEY_LAST_CALIB_TS = "last_calib_ts";

inline constexpr const char *NVS_KEY_BL_MASK       = "bl_mask";

// ============================================================
// 15. FREERTOS TASK CONFIG
// ============================================================

inline constexpr uint32_t TASK_STACK_SENSOR_WORDS   = 4096;
inline constexpr uint32_t TASK_STACK_NETWORK_WORDS  = 6144;
inline constexpr uint32_t TASK_STACK_DISPLAY_WORDS  = 3072;
inline constexpr uint32_t TASK_STACK_STORAGE_WORDS  = 4096;

inline constexpr uint32_t TASK_PRIO_SENSOR          = 5;
inline constexpr uint32_t TASK_PRIO_NETWORK         = 4;
inline constexpr uint32_t TASK_PRIO_DISPLAY         = 3;
inline constexpr uint32_t TASK_PRIO_STORAGE         = 2;

}

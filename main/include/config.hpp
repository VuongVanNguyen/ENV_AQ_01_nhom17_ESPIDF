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

// Port I2C dùng chung cho TOÀN BỘ thiết bị trên bus (BME680 + PCF8574).
// SensorManager::bme680Setup() và DisplayManager::init() PHẢI cùng tham
// chiếu hằng số này — không tự khai báo số port rời rạc ở module khác,
// nếu không mutex per-port của i2cdev sẽ không serialize đúng truy cập.
inline constexpr int      I2C_PORT          = 0;         // I2C_NUM_0

// ---- Địa chỉ thiết bị trên bus I2C dùng chung ----
inline constexpr uint8_t  BME680_I2C_ADDR   = 0x76;      // SDO = GND
// [DEBUG] 0x20 (A0/A1/A2=GND) không phản hồi (ESP_ERR_INVALID_RESPONSE) trên
// board thực tế — đang thử 0x27 (PCF8574 phổ biến trên module LCD backpack
// giá rẻ). Nếu vẫn fail, thử 0x3F (PCF8574A, base address khác hẳn PCF8574).
inline constexpr uint8_t  PCF8574_I2C_ADDR  = 0x27;

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
inline constexpr uint32_t MQ135_ADC_DEFAULT_VREF_MV = 1100; // mV — điện áp tham chiếu nội ESP32, dùng khi eFuse chưa burn

// Hệ số chuyển đổi điện áp → nồng độ (cần hiệu chuẩn thực tế)
inline constexpr float    MQ135_VREF        = 3.3f;       // V
inline constexpr float    MQ135_RL_KOHM     = 1.0f;       // Điện trở tải (kΩ) — ký hiệu "102" trên module = 1kΩ
inline constexpr float    MQ135_RO_KOHM     = 7.93f;      // kΩ — R0_corrected = Rs_clean(4.35)/RATIO_CLEAN_AIR(0.624) / CF(32°C,75%RH=0.8792); đã tính lại sau khi bật T/RH compensation

// Đường cong CO2 — hệ số từ MQUnifiedsensor, tham chiếu CO2 khí quyển 2026 (~426 ppm)
// Công thức: ppm = MQ135_CURVE_A * (Rs_comp/R0) ^ MQ135_CURVE_B
// Rs_comp = Rs / CF(T, RH)  ← chuẩn hóa Rs về 20°C/65%RH trước khi áp dụng power-law
inline constexpr float    MQ135_CURVE_A     = 110.47f;
inline constexpr float    MQ135_CURVE_B     = -2.8612f;

// Bù nhiệt/ẩm — piecewise linear fit từ đường cong datasheet MQ-135 (GeorgK 2015)
// Điều kiện tham chiếu: 20°C / 65%RH.
// CF (T < 20°C):  CORA·T² − CORB·T + CORC − (RH−33)·CORD
// CF (T ≥ 20°C):  CORE·T  + CORF·RH + CORG
// ⚠️  R0 (MQ135_RO_KOHM) nên được đo lại sau khi bật correction để sai số tuyệt đối là nhỏ nhất.
inline constexpr float    MQ135_TREF_C      = 20.0f;
inline constexpr float    MQ135_RHREF_PCT   = 65.0f;
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
inline constexpr int      SD_SPI_HOST       = 1;          // SPI2_HOST

inline constexpr const char *SD_MOUNT_POINT    = "/sdcard";
inline constexpr const char *SD_LOG_FILE       = "/sdcard/airdata.csv";
inline constexpr const char *SD_OFFLINE_QUEUE  = "/sdcard/offline_queue.bin";

// ---- StorageHelper — Event Log + Offline Buffer (StorageHelper.hpp §4/§5/§9) ----
inline constexpr const char *SD_EVENT_LOG_FILE = "/sdcard/events.csv";
inline constexpr const char *SD_OFFLINE_HEAD   = "/sdcard/offline_queue.hdr";

// Nhịp ghi log dữ liệu định kỳ vào SD_LOG_FILE (StorageHelper.hpp §3.1) —
// nên là bội số của SENSOR_READ_INTERVAL_MS. Từ Kconfig (mặc định 30s).
inline constexpr uint32_t SD_LOG_INTERVAL_MS   = CONFIG_SD_LOG_INTERVAL_MS;

// fsync sau mỗi N dòng ghi CSV (StorageHelper.hpp §3.5) — ghi thưa (>=30s/dòng)
// nên ưu tiên an toàn dữ liệu, fsync mỗi dòng.
inline constexpr uint32_t SD_FSYNC_EVERY_N_ROWS = 1;

// max_files khi mount (StorageHelper.hpp §2.3) — airdata.csv + events.csv +
// offline_queue.bin + offline_queue.hdr mở đồng thời.
inline constexpr int      SD_MAX_OPEN_FILES     = 4;

// Xoay vòng airdata.csv (StorageHelper.hpp §3.6): vượt SD_LOG_MAX_BYTES →
// rename airdata.csv -> airdata.1.csv (dồn thứ tự), mở file mới với header;
// giữ tối đa SD_LOG_MAX_FILES bản (airdata.csv + airdata.1..N-1.csv).
inline constexpr long      SD_LOG_MAX_BYTES     = 5L * 1024 * 1024; // 5 MB
inline constexpr int       SD_LOG_MAX_FILES     = 5;

// Cap dung lượng events.csv (StorageHelper.hpp §4.5) — mật độ thấp, không
// xoay vòng theo §3.6 nhưng vẫn cần trần để tránh đầy thẻ trong vận hành dài hạn.
inline constexpr long      SD_EVENT_MAX_BYTES   = 1L * 1024 * 1024; // 1 MB

// Độ dài hàng đợi lệnh tới storageTask (StorageHelper.hpp §6.1).
inline constexpr size_t    STORAGE_QUEUE_LEN    = 16;

// Trần số record trong offline queue (StorageHelper.hpp §5.2) — vượt mức này
// thì drop record CŨ NHẤT (FIFO ring) để không ăn hết thẻ.
inline constexpr size_t    OFFLINE_QUEUE_MAX_RECORDS = 2000;

// Số record phát lại tối đa mỗi lượt drain (StorageHelper.hpp §5.3) — tránh
// độc chiếm storageTask / flood broker.
inline constexpr size_t    OFFLINE_DRAIN_BATCH  = 20;

// Khoảng nghỉ giữa 2 lần publish khi drain (StorageHelper §5.3) — broker
// (ThingsBoard Cloud) áp rate-limit per-device; phát hết batch liên tiếp
// không nghỉ khiến broker đóng kết nối (transport EOF) ngay sau khi drain.
// Chạy trong storageTask (không phải pipeline cảm biến) nên vTaskDelay ở
// đây không vi phạm NFR ≤300ms / quy tắc non-blocking nghiệp vụ (CLAUDE.md §4).
inline constexpr uint32_t  OFFLINE_DRAIN_PACE_MS = 300;

// MAGIC + VERSION ở đầu offline_queue.bin (StorageHelper.hpp §5.1) — phát
// hiện format cũ không tương thích (struct AirData đổi layout) để bỏ file cũ
// thay vì phát lại dữ liệu rác.
inline constexpr uint32_t  OFFLINE_MAGIC          = 0x31305141; // "AQ01" little-endian
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

// NFR cứng — không thay đổi qua menuconfig
inline constexpr uint32_t MAX_CYCLE_TIME_MS          = 300;    // Tổng chu kỳ đọc+xử lý
inline constexpr uint32_t ALERT_MAX_LATENCY_MS       = 3'000;  // Cảnh báo phải phát trong 3s
inline constexpr uint32_t LCD_MIN_INTERVAL_MS        = 2'000;  // Giới hạn dưới cập nhật LCD
inline constexpr uint32_t LCD_MAX_INTERVAL_MS        = 5'000;  // Giới hạn trên cập nhật LCD

// Cận trên an toàn cho SENSOR_READ_INTERVAL_MS — taskSensor/taskNetwork chỉ
// được notify mỗi interval này, nên alert/MQTT publish tối đa trễ
// (interval + 1 chu kỳ xử lý) sau khi điều kiện xảy ra. Đảm bảo tổng đó
// không vượt ALERT_MAX_LATENCY_MS dù sdkconfig bị cấu hình giá trị lớn
// (Kconfig range chỉ validate ở menuconfig UI, không enforce lúc build).
inline constexpr uint32_t SENSOR_MIN_INTERVAL_MS     = 500;
inline constexpr uint32_t SENSOR_MAX_INTERVAL_MS     = ALERT_MAX_LATENCY_MS - MAX_CYCLE_TIME_MS;

// Thời lượng tối thiểu giữ overlay message (DisplayManager::showMessage)
// trước khi update() được phép vẽ lại frame trạng thái bình thường —
// đảm bảo người dùng kịp đọc thông báo tức thời (vd "Mất mạng").
inline constexpr uint32_t LCD_OVERLAY_MIN_MS         = 5'000;

// ---- Warmup / Stabilization các cảm biến ----
// Tính từ thời điểm SensorManager::init() trả ESP_OK. Trước khi hết
// warmup, các giá trị tương ứng vẫn được đọc nhưng cờ *_ready=false
// để module hạ nguồn coi là RÁC, không feed vào AQI / cảnh báo / log.
inline constexpr uint32_t BME680_WARMUP_MS           = 300'000;  // MOx đạt cân bằng hóa học (~5 phút)
inline constexpr uint32_t PMS5003_WARMUP_MS          = 30'000;   // Fan đạt tốc quay danh định
inline constexpr uint32_t MQ135_WARMUP_MS            = 1'200'000; // MOX preheat theo datasheet (20 phút)

// ============================================================
// 7. AQI — ĐIỂM GÃY NỘI SUY & NGƯỠNG PHÂN LOẠI (tiêu chuẩn VN)
// ============================================================

// AQI breakpoints theo tiêu chuẩn VN
inline constexpr float    AQI_INDEX_BP[]   = {0.0f, 50.0f, 100.0f, 150.0f, 200.0f, 300.0f, 500.0f};
inline constexpr float    AQI_PM25_BP[]    = {0.0f, 25.0f, 50.0f, 80.0f, 150.0f, 250.0f, 500.0f};
inline constexpr float    AQI_PM10_BP[]    = {0.0f, 50.0f, 150.0f, 250.0f, 350.0f, 420.0f, 600.0f};
inline constexpr float    AQI_MAX_INDEX    = 500.0f; // Trần clamp AQI (§3.5)

// Dải CHỈ SỐ AQI (0–500) dùng để phân loại aqi_category (§3.4). Giá trị đến
// từ Kconfig để có thể điều chỉnh theo quy định mới (đơn vị: chỉ số AQI,
// KHÔNG phải nồng độ PM2.5):
//   ≤GOOD_MAX→GOOD, ≤MODERATE_MAX→MODERATE, ≤POOR_MAX→POOR,
//   ≤BAD_MAX→BAD, ≤VERY_BAD_MAX→VERY_BAD, else HAZARDOUS.
inline constexpr float    AQI_GOOD_MAX     = static_cast<float>(CONFIG_AQI_GOOD_MAX);
inline constexpr float    AQI_MODERATE_MAX = static_cast<float>(CONFIG_AQI_MODERATE_MAX);
inline constexpr float    AQI_POOR_MAX     = static_cast<float>(CONFIG_AQI_POOR_MAX);
inline constexpr float    AQI_BAD_MAX      = static_cast<float>(CONFIG_AQI_BAD_MAX);
inline constexpr float    AQI_VERY_BAD_MAX = static_cast<float>(CONFIG_AQI_VERY_BAD_MAX);

// ============================================================
// 8. COMFORT INDEX (THI) — DataFusion / DisplayManager
// ============================================================

// THI = T - K1*(1 - RH_SCALE*RH)*(T - K2)  (DataFusion.hpp §5.1)
inline constexpr float    COMFORT_DI_K1       = 0.55f;
inline constexpr float    COMFORT_DI_K2       = 14.5f;
inline constexpr float    COMFORT_DI_RH_SCALE = 0.01f;

// Ngưỡng phân loại comfort_category (DataFusion.hpp §5.2), theo thang Thom
// Discomfort Index (DI) ngoài đời thực — 6 mức, đối xứng AqiCategory (§7):
//   DI < OK               : 0 COMFORTABLE  (de chiu)
//   OK     <= DI < WARM   : 1 SLIGHTLY_HOT  (hoi nong)
//   WARM   <= DI < HOT    : 2 HOT           (nong kho chiu)
//   HOT    <= DI < SEVERE : 3 VERY_HOT      (rat kho chiu)
//   SEVERE <= DI < DANGER : 4 HEAT_STRESS   (nguy co stress nhiet)
//   DI >= DANGER          : 5 DANGER        (cap cuu y te / nguy hiem tinh mang)
inline constexpr float    COMFORT_DI_OK       = 21.0f;
inline constexpr float    COMFORT_DI_WARM     = 24.0f;
inline constexpr float    COMFORT_DI_HOT      = 27.0f;
inline constexpr float    COMFORT_DI_SEVERE   = 29.0f;
inline constexpr float    COMFORT_DI_DANGER   = 32.0f;

// ============================================================
// 9. CO2 — DẢI PHÂN LOẠI ĐỊNH TÍNH (DisplayManager)
// ============================================================
// Tham chiếu ASHRAE: <1000ppm thông gió tốt, 1000-2000ppm gây buồn ngủ/giảm
// tập trung, >2000ppm kém. Dùng để ánh xạ data.co2_ppm -> nhãn "Tot/TB/Xau".
//   CO2 <= CO2_GOOD_MAX            : Tot
//   CO2_GOOD_MAX < CO2 <= MODERATE : TB
//   CO2 > CO2_MODERATE_MAX         : Xau
inline constexpr float    CO2_GOOD_MAX     = 1000.0f; // ppm
inline constexpr float    CO2_MODERATE_MAX = 2000.0f; // ppm

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
inline constexpr const char *MQTT_BROKER_URL   = CONFIG_MQTT_BROKER_URL;
inline constexpr const char *MQTT_TOPIC_DATA   = CONFIG_MQTT_TOPIC;
inline constexpr const char *MQTT_TOPIC_CMD    = CONFIG_MQTT_TOPIC_CMD;  // subscribe — nhận lệnh từ broker
inline constexpr const char *MQTT_ACCESS_TOKEN = CONFIG_MQTT_ACCESS_TOKEN;  // username MQTT (rỗng nếu broker không cần auth)
inline constexpr const char *NTP_SERVER_URL    = CONFIG_NTP_SERVER_URL;  // SNTP server
inline constexpr int         MQTT_JSON_BUF_LEN = 512;   // Byte — đủ cho toàn bộ AirData

// ============================================================
// 12. BỘ LỌC NHIỄU (Filters) — EMA/SMA hai tầng (CLAUDE.md Mục 7)
// ============================================================

inline constexpr size_t   FILTER_WINDOW_SIZE   = 5;     // Cửa sổ SMA humidity (hợp lệ 3..5)

// EMA alpha — hệ số làm trơn theo từng kênh (Mục 7.2 / 7.3 / 7.4)
inline constexpr float    EMA_ALPHA_TEMP       = 0.10f; // BME680 temperature
inline constexpr float    EMA_ALPHA_PRESS      = 0.10f; // BME680 pressure
inline constexpr float    EMA_ALPHA_CO2        = 0.20f; // MQ-135 CO2 ppm (sau bù T/RH)
inline constexpr float    EMA_ALPHA_PM1        = 0.50f; // PMS5003 PM1.0
inline constexpr float    EMA_ALPHA_PM25       = 0.50f; // PMS5003 PM2.5
inline constexpr float    EMA_ALPHA_PM10       = 0.30f; // PMS5003 PM10

// Sanity check range — Tầng 1 (range vật lý khả dĩ của từng sensor)
inline constexpr float    SANITY_TEMP_MIN      =   -40.0f;   // °C
inline constexpr float    SANITY_TEMP_MAX      =    85.0f;   // °C
inline constexpr float    SANITY_HUMI_MIN      =     0.0f;   // %RH
inline constexpr float    SANITY_HUMI_MAX      =   100.0f;   // %RH
inline constexpr float    SANITY_PRESS_MIN     =   300.0f;   // hPa
inline constexpr float    SANITY_PRESS_MAX     =  1100.0f;   // hPa
inline constexpr float    SANITY_CO2_MIN       =   100.0f;   // ppm
inline constexpr float    SANITY_CO2_MAX       =  5000.0f;   // ppm
inline constexpr float    SANITY_PM_MIN        =     0.0f;   // µg/m³
inline constexpr float    SANITY_PM_MAX        =   500.0f;   // µg/m³

// ============================================================
// 13. HIỆU CHUẨN & DRIFT SELF-CHECK
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
inline constexpr float    ACCURACY_INDEX_PCT   = 10.0f; // AQI/Comfort(THI)/CO2 ±% (TVOC đã bỏ)

// Drift nhiệt độ T dùng SAI SỐ TUYỆT ĐỐI (°C) thay vì dev% — dev% trên thang
// Celsius quá nhạy khi baseline gần 0 (vd 0.5°C lệch ở baseline=5°C = 10%).
inline constexpr float    DRIFT_TEMP_ABS_C     = ACCURACY_TEMP_C; // = 0.5°C

// Drift độ ẩm RH dùng SAI SỐ TUYỆT ĐỐI (%RH) theo đúng chỉ tiêu hiệu chuẩn
// ACCURACY_HUMI_RH (±3%RH) — chặt hơn và đúng đơn vị so với dev% 10%.
inline constexpr float    DRIFT_HUMI_ABS_PCT   = ACCURACY_HUMI_RH; // = 3.0 %RH

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

// Bitmask 3 nhóm baseline (BME680/PMS5003/MQ135) — xem DataFusion.hpp §6.5.
// Thiếu key này (NVS cũ trước khi có per-group baseline) → DataFusion coi
// như cả 3 nhóm đã hợp lệ (0b111), vì code cũ chỉ ghi baseline khi cả 3
// cảm biến đều ready.
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

} // namespace Cfg

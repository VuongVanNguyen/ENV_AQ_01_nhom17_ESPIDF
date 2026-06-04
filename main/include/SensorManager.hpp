#pragma once

// ============================================================
// SensorManager.hpp
//   Driver tổng hợp cho 3 cảm biến:
//     - BME680  : I2C  (T, RH, P, R_gas) — qua esp-idf-lib/bme680
//     - PMS5003 : UART (PM1.0 / PM2.5 / PM10)
//     - MQ-135  : ADC1 (CO2 ppm ước lượng)
//
//   Toàn bộ tham số phần cứng lấy từ namespace Cfg trong "config.hpp"
//   — không hardcode bất kỳ giá trị nào trong file .cpp.
//
//   I2C BUS:
//     Bus dùng chung (BME680 + PCF8574/LCD) được quản lý bởi thư viện
//     i2cdev (esp-idf-lib). SensorManager::init() tự gọi i2cdev_init()
//     (hàm này idempotent — có cờ static guard nội bộ). DisplayManager
//     nên cũng dùng i2cdev khai báo PCF8574 trên cùng port → mutex
//     per-port của i2cdev tự bảo vệ truy cập I2C giữa các task
//     (đúng yêu cầu của CLAUDE.md).
//
//   READINESS:
//     Mỗi cảm biến có thời gian warmup riêng (xem Cfg::*_WARMUP_MS).
//     readAll() vẫn trả về dữ liệu nhưng đánh cờ *_ready=false trong
//     AirData khi cảm biến chưa đủ ổn định — module hạ nguồn phải tôn
//     trọng cờ này để loại bỏ giá trị rác.
// ============================================================

#include "DataStructures.hpp"
#include "config.hpp"

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#include "bme680.h"   // esp-idf-lib — đã có guard extern "C" sẵn

class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    SensorManager(const SensorManager&)            = delete;
    SensorManager& operator=(const SensorManager&) = delete;

    // Khởi tạo cả 3 cảm biến. Gọi đúng 1 lần sau khi nvs_flash_init()
    // ở app_main() đã thành công (cần cho NVS calibration baseline).
    // i2cdev_init() được hàm này tự gọi nội bộ.
    esp_err_t init();

    // Đọc đầy đủ 1 chu kỳ. Hàm này tự chờ BME680 hoàn tất TPHG cycle
    // (thông qua bme680 lib — sử dụng vTaskDelay đúng bằng measurement
    // duration nội tại của cảm biến, KHÔNG phải logic timing nghiệp vụ).
    // Cờ *_ready trong out được set theo trạng thái warmup + heater/fan.
    esp_err_t readAll(AirData &out);

    // True khi cả 3 cảm biến đều đã hết warmup. Tiện cho DisplayManager
    // hiển thị "WARMING UP" thay vì giá trị 0 lúc khởi động.
    bool isFullyReady() const;

private:
    // ---- BME680 (esp-idf-lib/bme680 + i2cdev) ----
    bme680_t  bme680_dev_;
    bool      bme680_inited_;

    // ---- PMS5003 ----
    bool      uart_installed_;
    int       pms_valid_streak_;  // số frame OK (checksum hợp lệ) liên tiếp gần nhất

    // ---- MQ-135 ----
    adc_oneshot_unit_handle_t adc_handle_;
    adc_cali_handle_t         adc_cali_;
    bool                      adc_cali_enabled_;

    // ---- Thời điểm init() trả ESP_OK (us, đơn vị esp_timer) ----
    // Dùng để tính warmup. Một mốc DUY NHẤT cho cả 3 cảm biến — vì cả 3
    // được cấp điện chung khi MCU boot.
    int64_t   boot_time_us_;

    // ---- BME680 helpers ----
    esp_err_t bme680Setup();      // bme680_init_desc + bme680_init_sensor + heater profile
    esp_err_t bme680ReadOnce(float &t_c, float &rh, float &p_hpa,
                             float &gas_ohm, bool &heater_ok);

    // ---- PMS5003 helpers ----
    esp_err_t pmsInit();
    esp_err_t pmsReadFrame(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10);
    static esp_err_t pmsReadOneFrame(uart_port_t port, uint8_t *buf, TickType_t scan_ticks);

    // ---- MQ-135 helpers ----
    esp_err_t mq135Init();
    // t_c và rh_pct từ BME680 — dùng để chuẩn hóa Rs về điều kiện tham chiếu trước khi áp power-law.
    esp_err_t mq135ReadPpm(float &co2_ppm, float t_c, float rh_pct);
    // Hệ số bù nhiệt/ẩm GeorgK 2015 — hằng số từ Cfg::MQ135_COR*.
    static float mq135CorrectionFactor(float t_c, float rh_pct);

    // ---- Utility ----
    uint32_t  elapsedMs() const;  // ms trôi qua từ boot_time_us_
};

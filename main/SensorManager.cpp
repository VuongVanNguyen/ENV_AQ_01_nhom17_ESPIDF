// ============================================================
// SensorManager.cpp
//   Driver thuần ESP-IDF + esp-idf-lib/bme680 cho 3 cảm biến:
//     BME680  (I2C  - addr 0x76) — qua thư viện bme680 + i2cdev
//     PMS5003 (UART1)            — đọc frame thô theo datasheet
//     MQ-135  (ADC1 channel 6 → GPIO34, dùng adc_oneshot_*)
//
//   Toàn bộ hằng phần cứng đọc từ namespace Cfg (config.hpp).
//   Cờ readiness/warmup được cập nhật vào AirData mỗi chu kỳ đọc
//   — module hạ nguồn phải tôn trọng cờ này để loại bỏ giá trị rác.
// ============================================================

#include "SensorManager.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2cdev.h"           // i2cdev_init() — bus mgmt cho bme680 lib

#include <cstring>
#include <cmath>

static const char *TAG = "SensorManager";

// ============================================================
// Constructor / Destructor
// ============================================================
SensorManager::SensorManager()
    : bme680_dev_{},                 // i2c_dev_t bên trong cần zero-init
      bme680_inited_(false),
      uart_installed_(false),
      pms_valid_streak_(0),
      adc_handle_(nullptr),
      adc_cali_(nullptr),
      adc_cali_enabled_(false),
      boot_time_us_(0) {}

SensorManager::~SensorManager() {
    if (bme680_inited_) {
        bme680_free_desc(&bme680_dev_);          // gỡ device + mutex của i2cdev
    }
    if (adc_cali_enabled_ && adc_cali_) {
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_delete_scheme_line_fitting(adc_cali_);
#elif ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(adc_cali_);
#endif
    }
    if (adc_handle_) {
        adc_oneshot_del_unit(adc_handle_);
    }
    if (uart_installed_) {
        uart_driver_delete(static_cast<uart_port_t>(Cfg::PMS_UART_PORT));
    }
}

// ============================================================
// Public init() — gọi đúng 1 lần từ app_main()
//   Tiền điều kiện: nvs_flash_init() đã chạy xong (cho NVS calib).
//   i2cdev_init() được gọi nội bộ — idempotent, có thể gọi lại từ
//   DisplayManager mà không lỗi.
// ============================================================
esp_err_t SensorManager::init() {
    ESP_LOGI(TAG, "Initializing 3 sensors (BME680 / PMS5003 / MQ-135)...");

    // Khởi tạo subsystem i2cdev — bắt buộc trước mọi i2c_dev_t.
    // Hàm này có cờ static guard nên gọi nhiều lần không leak.
    esp_err_t err = i2cdev_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2cdev_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = bme680Setup();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME680 init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = pmsInit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PMS5003 init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = mq135Init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQ-135 init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Mốc thời gian dùng để tính warmup cho cả 3 cảm biến.
    boot_time_us_ = esp_timer_get_time();
    ESP_LOGI(TAG, "SensorManager ready — warmup: BME680=%lums PMS=%lums MQ135=%lums",
             (unsigned long)Cfg::BME680_WARMUP_MS,
             (unsigned long)Cfg::PMS5003_WARMUP_MS,
             (unsigned long)Cfg::MQ135_WARMUP_MS);
    return ESP_OK;
}

// ============================================================
// Public readAll() — đọc 1 chu kỳ đầy đủ + cập nhật cờ readiness
// ============================================================
esp_err_t SensorManager::readAll(AirData &out) {
    out = AirData{};                              // zero-init tất cả field

    const uint32_t up_ms = elapsedMs();

    // ---------- BME680 ----------
    float t = 0, rh = 0, p = 0, gas = 0;
    bool  heater_ok = false;
    esp_err_t err = bme680ReadOnce(t, rh, p, gas, heater_ok);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BME680 read failed: %s", esp_err_to_name(err));
        return err;                               // lỗi cứng → bỏ chu kỳ
    }
    out.temperature    = t;
    out.humidity       = rh;
    out.pressure       = p;
    out.gas_resistance = gas;
    // BME680 ready khi: đã qua warmup + heater stable + gas hợp lệ (>0)
    out.bme680_ready = (up_ms >= Cfg::BME680_WARMUP_MS)
                       && heater_ok
                       && (gas > 0.0f);

    // Cập nhật ambient temperature cho lần đo gas tiếp theo — cải thiện
    // độ chính xác của res_heat (thư viện dùng giá trị này tính heat profile).
    bme680_set_ambient_temperature(&bme680_dev_, static_cast<int16_t>(t));

    // ---------- PMS5003 ----------
    uint16_t pm1 = 0, pm25 = 0, pm10 = 0;
    err = pmsReadFrame(pm1, pm25, pm10);
    if (err == ESP_OK) {
        out.pm1_0 = pm1; out.pm2_5 = pm25; out.pm10 = pm10;
        // Frame checksum OK → tăng streak (cap để tránh tràn). Lưu ý: KHÔNG
        // reset khi PM=0 — trong môi trường thực sự sạch, PM có thể =0 hợp lệ.
        if (pms_valid_streak_ < Cfg::PMS_VALID_STREAK_OK) ++pms_valid_streak_;
    } else {
        ESP_LOGW(TAG, "PMS5003 frame fail (%s) — giữ giá trị 0",
                 esp_err_to_name(err));
        pms_valid_streak_ = 0;                    // chỉ reset khi đọc frame thất bại
    }
    // PMS5003 ready khi: hết warmup + đã có đủ N frame OK liên tiếp gần nhất
    out.pms5003_ready = (up_ms >= Cfg::PMS5003_WARMUP_MS)
                        && (pms_valid_streak_ >= Cfg::PMS_VALID_STREAK_OK);

    // ---------- MQ-135 ----------
    float co2 = 0;
    err = mq135ReadPpm(co2);
    if (err == ESP_OK) {
        out.co2_ppm = co2;
    }
    out.mq135_ready = (up_ms >= Cfg::MQ135_WARMUP_MS) && (co2 > 0.0f);
    // tvoc_ppm sẽ do DataFusion suy ra từ gas_resistance (chưa tính ở đây)

    // ---------- Tổng hợp ----------
    out.sensors_ready = out.bme680_ready && out.pms5003_ready && out.mq135_ready;

    // Timestamp tương đối (giây) — NetworkManager/DataFusion có thể ghi đè
    // bằng Unix-time chính xác sau khi SNTP đồng bộ.
    out.timestamp  = esp_timer_get_time() / 1000000LL;
    out.data_valid = true;
    return ESP_OK;
}

bool SensorManager::isFullyReady() const {
    const uint32_t up_ms = elapsedMs();
    return up_ms >= Cfg::BME680_WARMUP_MS
        && up_ms >= Cfg::PMS5003_WARMUP_MS
        && up_ms >= Cfg::MQ135_WARMUP_MS;
}

uint32_t SensorManager::elapsedMs() const {
    if (boot_time_us_ == 0) return 0;             // init() chưa chạy xong
    return static_cast<uint32_t>((esp_timer_get_time() - boot_time_us_) / 1000LL);
}

// ============================================================
// BME680 — Setup qua thư viện esp-idf-lib/bme680
// ============================================================
esp_err_t SensorManager::bme680Setup() {
    // bme680_init_desc nội bộ gọi i2c_dev_create_mutex (i2cdev) — tạo
    // device + mutex riêng. Bus I2C dùng chung được i2cdev quản lý theo
    // port; lần đầu device được tạo → port bus được khởi tạo lazy.
    esp_err_t err = bme680_init_desc(&bme680_dev_,
                                     Cfg::BME680_I2C_ADDR,
                                     static_cast<i2c_port_t>(0),         // I2C_NUM_0
                                     static_cast<gpio_num_t>(Cfg::I2C_SDA_PIN),
                                     static_cast<gpio_num_t>(Cfg::I2C_SCL_PIN));
    if (err != ESP_OK) return err;
    bme680_inited_ = true;

    // probe chip + reset + load calibration + cấu hình mặc định
    err = bme680_init_sensor(&bme680_dev_);
    if (err != ESP_OK) return err;

    // Oversampling: T x2, P x16, H x1 — cân bằng độ chính xác & duration.
    // Total measurement ≈ 5ms TPH + 150ms heater ≈ 160ms (vẫn < NFR 300ms).
    err = bme680_set_oversampling_rates(&bme680_dev_,
                                        BME680_OSR_2X,
                                        BME680_OSR_16X,
                                        BME680_OSR_1X);
    if (err != ESP_OK) return err;

    // IIR filter cho T & P giảm nhiễu ngắn hạn — coef 3 là cân bằng tốt.
    err = bme680_set_filter_size(&bme680_dev_, BME680_IIR_SIZE_3);
    if (err != ESP_OK) return err;

    // Heater profile 0: 320°C trong 150ms (theo Cfg) — vùng tối ưu cho VOC.
    err = bme680_set_heater_profile(&bme680_dev_, 0,
                                    Cfg::BME680_HEATER_TEMP_C,
                                    Cfg::BME680_HEATER_DUR_MS);
    if (err != ESP_OK) return err;
    err = bme680_use_heater_profile(&bme680_dev_, 0);
    if (err != ESP_OK) return err;

    // Ambient ban đầu 25°C — sẽ được readAll() cập nhật theo lần đo trước.
    err = bme680_set_ambient_temperature(&bme680_dev_, Cfg::BME680_AMBIENT_TEMP_C);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "BME680 ready (addr 0x%02X, heater %u°C/%ums)",
             Cfg::BME680_I2C_ADDR,
             (unsigned)Cfg::BME680_HEATER_TEMP_C,
             (unsigned)Cfg::BME680_HEATER_DUR_MS);
    return ESP_OK;
}

// ============================================================
// BME680 — Trigger forced mode + đọc compensation qua thư viện
//   heater_ok = true ⇔ thư viện trả gas_resistance > 0 (nội bộ check
//   gas_valid + heater_stable trước khi convert).
// ============================================================
esp_err_t SensorManager::bme680ReadOnce(float &t_c, float &rh, float &p_hpa,
                                        float &gas_ohm, bool &heater_ok) {
    // Đo duration phụ thuộc OSR/heater profile — yêu cầu thư viện tính ra
    // chính xác thay vì hardcode (NFR 300ms cycle).
    uint32_t dur_ticks = 0;
    esp_err_t err = bme680_get_measurement_duration(&bme680_dev_, &dur_ticks);
    if (err != ESP_OK) return err;

    err = bme680_force_measurement(&bme680_dev_);
    if (err != ESP_OK) return err;

    // Chờ đúng bằng measurement duration của cảm biến — đây là I/O wait
    // (giống đợi ADC convert), KHÔNG phải logic timing nghiệp vụ.
    vTaskDelay(dur_ticks);

    bme680_values_float_t r{};
    err = bme680_get_results_float(&bme680_dev_, &r);
    if (err != ESP_OK) return err;

    t_c       = r.temperature;
    rh        = r.humidity;
    p_hpa     = r.pressure;                       // thư viện đã trả hPa
    gas_ohm   = r.gas_resistance;
    heater_ok = (r.gas_resistance > 0.0f);        // 0 = gas_invalid hoặc heater chưa stable
    return ESP_OK;
}

// ============================================================
// PMS5003 — UART
// ============================================================
esp_err_t SensorManager::pmsInit() {
    // SET pin: HIGH = active, LOW = sleep. Để fan luôn chạy khi cảm biến
    // đang trong chu kỳ đo — NFR Modem-sleep chỉ áp dụng cho Wi-Fi.
    gpio_config_t set_cfg = {
        .pin_bit_mask = 1ULL << Cfg::PMS_SET_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&set_cfg);
    if (err != ESP_OK) return err;
    gpio_set_level(static_cast<gpio_num_t>(Cfg::PMS_SET_PIN), 1);

    uart_config_t cfg = {
        .baud_rate           = Cfg::PMS_UART_BAUD,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_DEFAULT,
        .flags               = {},
    };
    const uart_port_t port = static_cast<uart_port_t>(Cfg::PMS_UART_PORT);

    // RX buffer đủ chứa ~2 frame để chống miss khi task bận
    err = uart_driver_install(port, Cfg::PMS_RX_BUF_SIZE * 2, 0, 0, nullptr, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(port, &cfg);
    if (err != ESP_OK) { uart_driver_delete(port); return err; }
    err = uart_set_pin(port, Cfg::PMS_UART_TX_PIN, Cfg::PMS_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) { uart_driver_delete(port); return err; }

    uart_installed_ = true;
    ESP_LOGI(TAG, "PMS5003 UART%d ready (RX=%d TX=%d SET=%d)",
             Cfg::PMS_UART_PORT, Cfg::PMS_UART_RX_PIN,
             Cfg::PMS_UART_TX_PIN, Cfg::PMS_SET_PIN);
    return ESP_OK;
}

esp_err_t SensorManager::pmsReadFrame(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10) {
    const uart_port_t port = static_cast<uart_port_t>(Cfg::PMS_UART_PORT);
    uint8_t buf[Cfg::PMS_FRAME_LEN];

    // Quét tìm magic 0x42 0x4D — tối đa 64 byte để giới hạn thời gian
    uint8_t b = 0;
    int scanned = 0;
    while (scanned < 64) {
        int n = uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(150));
        if (n != 1) return ESP_ERR_TIMEOUT;
        scanned++;
        if (b == 0x42) {
            n = uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(20));
            if (n == 1 && b == 0x4D) break;
        }
    }
    if (scanned >= 64) return ESP_ERR_NOT_FOUND;

    buf[0] = 0x42; buf[1] = 0x4D;
    int n = uart_read_bytes(port, buf + 2, Cfg::PMS_FRAME_LEN - 2, pdMS_TO_TICKS(100));
    if (n != Cfg::PMS_FRAME_LEN - 2) return ESP_ERR_TIMEOUT;

    // Verify checksum: tổng 30 byte đầu == 2 byte cuối (big-endian)
    uint16_t sum = 0;
    for (int i = 0; i < Cfg::PMS_FRAME_LEN - 2; ++i) sum += buf[i];
    uint16_t expect = (static_cast<uint16_t>(buf[Cfg::PMS_FRAME_LEN - 2]) << 8)
                    | buf[Cfg::PMS_FRAME_LEN - 1];
    if (sum != expect) return ESP_ERR_INVALID_CRC;

    // Dùng giá trị "atmospheric" (offset 10/12/14) — phù hợp môi trường thực
    pm1  = (static_cast<uint16_t>(buf[10]) << 8) | buf[11];
    pm25 = (static_cast<uint16_t>(buf[12]) << 8) | buf[13];
    pm10 = (static_cast<uint16_t>(buf[14]) << 8) | buf[15];
    return ESP_OK;
}

// ============================================================
// MQ-135 — ADC1 oneshot + calibration
// ============================================================
esp_err_t SensorManager::mq135Init() {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id  = static_cast<adc_unit_t>(Cfg::MQ135_ADC_UNIT),   // ADC_UNIT_1
        .clk_src  = static_cast<adc_oneshot_clk_src_t>(0),          // default
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &adc_handle_);
    if (err != ESP_OK) return err;

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = static_cast<adc_atten_t>(Cfg::MQ135_ADC_ATTEN),     // 12 dB → 0..3.3V
        .bitwidth = static_cast<adc_bitwidth_t>(Cfg::MQ135_ADC_BITWIDTH),
    };
    err = adc_oneshot_config_channel(adc_handle_,
                                     static_cast<adc_channel_t>(Cfg::MQ135_ADC_CHANNEL),
                                     &chan_cfg);
    if (err != ESP_OK) return err;

    // ESP32 (chip gốc) dùng line-fitting; chip mới hơn dùng curve-fitting
    adc_cali_enabled_ = false;
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id  = static_cast<adc_unit_t>(Cfg::MQ135_ADC_UNIT),
        .atten    = static_cast<adc_atten_t>(Cfg::MQ135_ADC_ATTEN),
        .bitwidth = static_cast<adc_bitwidth_t>(Cfg::MQ135_ADC_BITWIDTH),
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_) == ESP_OK) {
        adc_cali_enabled_ = true;
    }
#elif ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = static_cast<adc_unit_t>(Cfg::MQ135_ADC_UNIT),
        .chan     = static_cast<adc_channel_t>(Cfg::MQ135_ADC_CHANNEL),
        .atten    = static_cast<adc_atten_t>(Cfg::MQ135_ADC_ATTEN),
        .bitwidth = static_cast<adc_bitwidth_t>(Cfg::MQ135_ADC_BITWIDTH),
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_) == ESP_OK) {
        adc_cali_enabled_ = true;
    }
#endif

    ESP_LOGI(TAG, "MQ-135 ADC1 ch%d ready (calib=%s)",
             Cfg::MQ135_ADC_CHANNEL, adc_cali_enabled_ ? "yes" : "no");
    return ESP_OK;
}

esp_err_t SensorManager::mq135ReadPpm(float &co2_ppm) {
    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc_handle_,
                                     static_cast<adc_channel_t>(Cfg::MQ135_ADC_CHANNEL),
                                     &raw);
    if (err != ESP_OK) return err;

    int mv = 0;
    if (adc_cali_enabled_) {
        adc_cali_raw_to_voltage(adc_cali_, raw, &mv);
    } else {
        // Fallback nếu calibration không có (eFuse trống) — tuyến tính 12-bit/3.3V
        mv = static_cast<int>((raw * Cfg::MQ135_VREF * 1000.0f) /
                              ((1 << Cfg::MQ135_ADC_BITWIDTH) - 1));
    }

    float vrl = mv / 1000.0f;
    if (vrl <= 0.01f) {                          // tránh chia 0 khi mạch hở
        co2_ppm = 0.0f;
        return ESP_OK;
    }
    // RS = (Vc - VRL) * RL / VRL; ratio = RS / R0
    float rs    = (Cfg::MQ135_VREF - vrl) * Cfg::MQ135_RL_KOHM / vrl;
    float ratio = rs / Cfg::MQ135_RO_KOHM;

    // Đường cong CO2 đặc trưng MQ-135 (datasheet): ppm = 116.6020682 * ratio^-2.769
    co2_ppm = 116.6020682f * powf(ratio, -2.769034857f);
    return ESP_OK;
}

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

#include <algorithm>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2cdev.h"           

#include <cstring>
#include <cmath>
#include <ctime>

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
    ESP_LOGI(TAG, "Khởi tạo 3 cảm biến (BME680 / PMS5003 / MQ-135)...");

    // Khởi tạo subsystem i2cdev — bắt buộc trước mọi i2c_dev_t.
    // Hàm này có cờ static guard nên gọi nhiều lần không leak.
    esp_err_t err = i2cdev_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Khởi tạo I2C thất bại: %s", esp_err_to_name(err));
        return err;
    }

    err = bme680Setup();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME680 khởi tạo thất bại: %s", esp_err_to_name(err));
        return err;
    }

    err = pmsInit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PMS5003 khởi tạo thất bại: %s", esp_err_to_name(err));
        return err;
    }

    err = mq135Init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQ-135 khởi tạo thất bại: %s", esp_err_to_name(err));
        return err;
    }

    // Mốc thời gian dùng để tính warmup cho cả 3 cảm biến.
    boot_time_us_ = esp_timer_get_time();
    ESP_LOGI(TAG, "Cảm biến sẵn sàng - chờ ổn định: BME680=%lums PMS=%lums MQ135=%lums",
             (unsigned long)Cfg::BME680_WARMUP_MS,
             (unsigned long)Cfg::PMS5003_WARMUP_MS,
             (unsigned long)Cfg::MQ135_WARMUP_MS);
    return ESP_OK;
}

// ============================================================
// Public readAll() — đọc 1 chu kỳ đầy đủ + cập nhật cờ readiness
// ============================================================
esp_err_t SensorManager::readAll(AirData &data) {
    data = AirData{};                              // zero-init tất cả field

    const uint32_t up_ms = elapsedMs();

    // ---------- BME680 ----------
    float t = 0, rh = 0, p = 0, gas = 0;
    bool  heater_ok = false;
    esp_err_t err = bme680ReadOnce(t, rh, p, gas, heater_ok);
    const bool bme_ok = (err == ESP_OK);
    if (bme_ok) {
        data.temperature    = t;
        data.humidity       = rh;
        data.pressure       = p;
        data.gas_resistance = gas;
        // BME680 ready khi: đã qua warmup + heater stable + gas hợp lệ (>0)
        data.bme680_ready = (up_ms >= Cfg::BME680_WARMUP_MS)
                           && heater_ok
                           && (gas > 0.0f);

        // Cập nhật ambient temperature cho lần đo gas tiếp theo — cải thiện
        // độ chính xác của res_heat (thư viện dùng giá trị này tính heat profile).
        bme680_set_ambient_temperature(&bme680_dev_, static_cast<int16_t>(t));
    } else {
        ESP_LOGW(TAG, "Đọc BME680 thất bại (%s) — giữ giá trị 0", esp_err_to_name(err));
    }

    // ---------- PMS5003 ----------
    uint16_t pm1 = 0, pm25 = 0, pm10 = 0;
    err = pmsReadFrame(pm1, pm25, pm10);
    const bool pms_ok = (err == ESP_OK);
    if (pms_ok) {
        data.pm1_0 = pm1; data.pm2_5 = pm25; data.pm10 = pm10;
        // Frame checksum OK → tăng streak (cap để tránh tràn). Lưu ý: KHÔNG
        // reset khi PM=0 — trong môi trường thực sự sạch, PM có thể =0 hợp lệ.
        if (pms_valid_streak_ < Cfg::PMS_VALID_STREAK_OK) ++pms_valid_streak_;
    } else {
        ESP_LOGW(TAG, "PMS5003 đọc frame thất bại (%s) — giữ giá trị 0",
                 esp_err_to_name(err));
        pms_valid_streak_ = 0;                    // chỉ reset khi đọc frame thất bại
    }
    // PMS5003 ready khi: hết warmup + đã có đủ N frame OK liên tiếp gần nhất
    data.pms5003_ready = (up_ms >= Cfg::PMS5003_WARMUP_MS)
                        && (pms_valid_streak_ >= Cfg::PMS_VALID_STREAK_OK);

    // ---------- MQ-135 ----------
    // t và rh đã được đọc từ BME680 phía trên — dùng để chuẩn hóa Rs về 20°C/65%RH.
    float co2 = 0;
    err = mq135ReadPpm(co2, t, rh);
    const bool mq_ok = (err == ESP_OK);
    if (mq_ok) {
        data.co2_ppm = co2;
    } else {
        ESP_LOGW(TAG, "Đọc MQ-135 thất bại (%s) — giữ giá trị 0",
                 esp_err_to_name(err));
    }
    data.mq135_ready = (up_ms >= Cfg::MQ135_WARMUP_MS) && (co2 > 0.0f);

    // Dùng Unix time nếu SNTP đã sync (heuristic: time(NULL) > 1577836800 = 1/1/2020).
    // Trước khi SNTP sẵn sàng, fallback về seconds-from-boot — monotonic nhưng không lịch,
    // reset về 0 sau reboot. NetworkManager::isTimeSynced() là nguồn chính xác hơn nhưng
    // tránh coupling SensorManager → NetworkManager.
    time_t now = time(NULL);
    data.timestamp = (now > 1577836800LL) ? static_cast<int64_t>(now)
                                         : (esp_timer_get_time() / 1000000LL);
    // data_valid = true nếu ít nhất 1 cảm biến đọc thành công trong chu kỳ này.
    data.data_valid = bme_ok || pms_ok || mq_ok;
    if (!data.data_valid) {
        ESP_LOGE(TAG, "Cả 3 cảm biến đều đọc thất bại trong chu kỳ này");
        return ESP_FAIL;
    }
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
                                     static_cast<i2c_port_t>(Cfg::I2C_PORT),
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

    ESP_LOGI(TAG, "BME680 sẵn sàng (addr 0x%02X, heater %u°C/%ums)",
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

    // RX buffer chứa 8 frame (> ESP32 FIFO 128 bytes), đủ dự phòng khi task bị preempt
    err = uart_driver_install(port, Cfg::PMS_FRAME_LEN * 8, 0, 0, nullptr, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(port, &cfg);
    if (err != ESP_OK) { uart_driver_delete(port); return err; }
    err = uart_set_pin(port, Cfg::PMS_UART_TX_PIN, Cfg::PMS_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) { uart_driver_delete(port); return err; }

    uart_installed_ = true;
    ESP_LOGI(TAG, "PMS5003 UART%d sẵn sàng (RX=%d TX=%d SET=%d)",
             Cfg::PMS_UART_PORT, Cfg::PMS_UART_RX_PIN,
             Cfg::PMS_UART_TX_PIN, Cfg::PMS_SET_PIN);
    return ESP_OK;
}

// Quét tìm 0x42 0x4D, đọc đủ PMS_FRAME_LEN byte vào buf, xác minh checksum.
// scan_ticks: timeout mỗi byte khi tìm start byte — dùng 150ms lần đầu (chờ frame
// chưa có), dùng 2ms khi drain (dữ liệu đã có sẵn trong ring buffer).
esp_err_t SensorManager::pmsReadOneFrame(uart_port_t port, uint8_t *buf, TickType_t scan_ticks) {
    uint8_t b = 0;
    int scanned = 0;
    bool got_start = false;
    bool found = false;
    while (scanned < Cfg::PMS_FRAME_LEN * 2) {
        TickType_t ticks = got_start ? std::min(scan_ticks, pdMS_TO_TICKS(20)) : scan_ticks;
        if (uart_read_bytes(port, &b, 1, ticks) != 1) {
            if (got_start) { got_start = false; continue; }
            return ESP_ERR_TIMEOUT;
        }
        scanned++;
        if (got_start) {
            if (b == 0x4D) { buf[1] = b; found = true; break; }
            got_start = (b == 0x42);
            if (got_start) buf[0] = b;
        } else if (b == 0x42) {
            buf[0] = b;
            got_start = true;
        }
    }
    if (!found) return ESP_ERR_NOT_FOUND;
    if (uart_read_bytes(port, buf + 2, Cfg::PMS_FRAME_LEN - 2, pdMS_TO_TICKS(100)) != Cfg::PMS_FRAME_LEN - 2)
        return ESP_ERR_TIMEOUT;
    uint16_t sum = 0;
    for (int i = 0; i < Cfg::PMS_FRAME_LEN - 2; ++i) sum += buf[i];
    uint16_t expect = (static_cast<uint16_t>(buf[Cfg::PMS_FRAME_LEN - 2]) << 8)
                    | buf[Cfg::PMS_FRAME_LEN - 1];
    return (sum == expect) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

esp_err_t SensorManager::pmsReadFrame(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10) {
    const uart_port_t port = static_cast<uart_port_t>(Cfg::PMS_UART_PORT);
    uint8_t buf[Cfg::PMS_FRAME_LEN];

    // Chờ frame đầu tiên — scan_ticks 150ms để bắt frame kể cả khi buffer trống
    esp_err_t err = pmsReadOneFrame(port, buf, pdMS_TO_TICKS(150));
    if (err != ESP_OK) return err;

    // Drain các frame tích đống: đọc tiếp khi còn đủ 1 frame trong ring buffer.
    // uart_read_bytes trả về ngay (dữ liệu đã trong RAM) → không tốn thời gian chờ.
    // Giữ frame cuối cùng hợp lệ để luôn trả về dữ liệu mới nhất.
    // Guard: nếu avail không giảm sau 1 iteration (pmsReadOneFrame tiêu thụ 0 byte
    // do corrupt/partial data) → break tránh vòng lặp vô hạn.
    size_t avail = 0;
    uart_get_buffered_data_len(port, &avail);
    while (avail >= static_cast<size_t>(Cfg::PMS_FRAME_LEN)) {
        uint8_t tmp[Cfg::PMS_FRAME_LEN];
        if (pmsReadOneFrame(port, tmp, pdMS_TO_TICKS(2)) == ESP_OK)
            memcpy(buf, tmp, Cfg::PMS_FRAME_LEN);
        size_t tmp_avail = 0;
        uart_get_buffered_data_len(port, &tmp_avail);
        if (tmp_avail >= avail) break;
        avail = tmp_avail;
    }

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
        .unit_id      = static_cast<adc_unit_t>(Cfg::MQ135_ADC_UNIT),
        .atten        = static_cast<adc_atten_t>(Cfg::MQ135_ADC_ATTEN),
        .bitwidth     = static_cast<adc_bitwidth_t>(Cfg::MQ135_ADC_BITWIDTH),
        .default_vref = Cfg::MQ135_ADC_DEFAULT_VREF_MV,
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

    ESP_LOGI(TAG, "MQ-135 ADC1 kênh %d sẵn sàng (hiệu chuẩn=%s)",
             Cfg::MQ135_ADC_CHANNEL, adc_cali_enabled_ ? "có" : "không");
    return ESP_OK;
}

// Hệ số bù nhiệt/ẩm GeorgK 2015 — piecewise linear fit từ đường cong datasheet MQ-135.
// Trả về CF để caller tính Rs_comp = Rs / CF, qua đó chuẩn hóa Rs về 20°C/65%RH.
// Dùng hằng số từ Cfg::MQ135_COR* — không hardcode giá trị nào.
float SensorManager::mq135CorrectionFactor(float t_c, float rh_pct) {
    if (t_c < Cfg::MQ135_TREF_C) {
        return Cfg::MQ135_CORA * t_c * t_c
             - Cfg::MQ135_CORB * t_c
             + Cfg::MQ135_CORC
             - (rh_pct - 33.0f) * Cfg::MQ135_CORD;
    }
    return Cfg::MQ135_CORE * t_c
         + Cfg::MQ135_CORF * rh_pct
         + Cfg::MQ135_CORG;
}

esp_err_t SensorManager::mq135ReadPpm(float &co2_ppm, float t_c, float rh_pct) {
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

    // Rs = (Vc − VRL) × RL / VRL
    float rs = (Cfg::MQ135_VREF - vrl) * Cfg::MQ135_RL_KOHM / vrl;

    // Chuẩn hóa Rs về điều kiện tham chiếu (20°C/65%RH) trước khi áp dụng power-law
    float cf      = mq135CorrectionFactor(t_c, rh_pct);
    float rs_comp = (cf > 0.001f) ? rs / cf : rs;
    float ratio   = rs_comp / Cfg::MQ135_RO_KOHM;

    // ppm = A × (Rs_comp/R0)^B — hệ số MQUnifiedsensor, R0 cần đo lại sau khi bật correction
    co2_ppm = Cfg::MQ135_CURVE_A * powf(ratio, Cfg::MQ135_CURVE_B);
    return ESP_OK;
}

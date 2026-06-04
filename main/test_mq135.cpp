// ============================================================
// test_mq135.cpp — MQ-135 Filter Selection Tool
//
// Mục đích: xác định loại bộ lọc DỰA TRÊN ĐO LƯỜNG.
//   Giá trị CO2 đã được bù T/RH từ BME680 (mirror SensorManager::mq135CorrectionFactor).
//
// Tự động 4 phase:
//   COLLECT    — 60 mẫu nền, ngồi yên không tác động
//   ANALYZE    — tính thống kê, quyết định loại bộ lọc
//   PERTURB    — thổi hơi thở ~30s, tự phát hiện onset/peak
//   RECOMMEND  — tính α (EMA) hoặc window (SMA)
//
// Toàn bộ output đều qua Teleplot — KHÔNG cần IDF Monitor.
//
// Kênh Teleplot quan trọng:
//   MQ135_phase    : 0=WARMUP 1=COLLECT 3=WAIT_BREATH 4=PERTURBING 6=DONE
//   MQ135_action   : 1=chờ_warmup 2=ngồi_yên 3=THỔ_HƠI_THỞ_30s 4=đang_đo 5=xem_kết_quả
//   MQ135_filter   : 0=chưa_biết 1=Median 2=EMA 3=SMA
//   MQ135_CO2      : CO2 ppm đã bù T/RH từ BME680
//   MQ135_CO2_raw  : CO2 ppm chưa bù (tham chiếu so sánh)
//   MQ135_T        : Nhiệt độ BME680 (°C)
//   MQ135_RH       : Độ ẩm BME680 (%RH)
//   MQ135_CF       : Hệ số bù (CF) = Rs / Rs_comp
//   MQ135_lag1r    : lag-1 autocorrelation (>0.5 → EMA, ≤0.5 → SMA)
//   MQ135_outlier  : % outlier (>5% → cần Median)
//   MQ135_alpha75  : α EMA để capture 75% biên độ step
//   MQ135_window   : window SMA (nếu SMA được chọn)
//
// Bật: idf.py menuconfig → Sensor Test Mode → MQ-135 only
// ============================================================

#include "sdkconfig.h"
#ifdef CONFIG_TEST_MODE_MQ135

#include "test_sensors.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bme680.h"
#include "i2cdev.h"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <array>

static const char *TAG = "TEST_MQ135";

static constexpr float RATIO_CLEAN_AIR = 0.624f;  // Rs_clean/R0 trong không khí sạch
static constexpr int   COLLECT_N       = 60;       // ~2 phút tại 2s/sample

// ============================================================
// Hệ số bù T/RH — mirror chính xác SensorManager::mq135CorrectionFactor()
// Dùng hằng số Cfg::MQ135_COR* từ config.hpp, không hardcode.
// ============================================================

static float mq135_correction_factor(float t_c, float rh_pct) {
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

// ============================================================
// State machine
// ============================================================

enum class Phase : uint8_t {
    WARMUP        = 0,
    COLLECT       = 1,
    ANALYZE       = 2,  // one-shot
    WAIT_PERTURB  = 3,
    PERTURB       = 4,
    RECOMMEND     = 5,  // one-shot
    DONE          = 6,
};

// Action code — xuất qua MQ135_action để Teleplot hiển thị
// User nhìn vào số này để biết cần làm gì, không cần đọc log
enum class Action : int {
    WAIT_WARMUP   = 1,  // Chờ warmup, không làm gì
    SIT_STILL     = 2,  // Ngồi yên, không thở gần sensor
    BREATHE_NOW   = 3,  // THỔ HƠI THỞ GẦN SENSOR ~30s
    PERTURBING    = 4,  // Đang ghi dữ liệu, giữ nguyên tư thế
    SEE_RESULTS   = 5,  // Xem MQ135_filter / MQ135_alpha75 / MQ135_window
};

// ============================================================
// Thống kê nền
// ============================================================

struct NoiseStats {
    float mean;
    float stddev;
    float min_val;
    float max_val;
    float lag1_autocorr;
    float outlier_rate;
};

static NoiseStats compute_noise_stats(const float *data, int n) {
    NoiseStats s{};
    s.min_val =  FLT_MAX;
    s.max_val = -FLT_MAX;

    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum      += data[i];
        s.min_val = std::min(s.min_val, data[i]);
        s.max_val = std::max(s.max_val, data[i]);
    }
    s.mean = static_cast<float>(sum / n);

    double sq = 0.0;
    for (int i = 0; i < n; i++) {
        double d = data[i] - s.mean;
        sq += d * d;
    }
    s.stddev = static_cast<float>(std::sqrt(sq / (n - 1)));

    // r1 > 0.5: sample liên tiếp tương quan → EMA hiệu quả hơn SMA
    // r1 ≤ 0.5: sample gần độc lập → SMA tương đương EMA, đơn giản hơn
    double cov = 0.0, var = 0.0;
    for (int i = 1; i < n; i++) {
        double d0 = data[i - 1] - s.mean;
        double d1 = data[i]     - s.mean;
        cov += d0 * d1;
        var += d0 * d0;
    }
    s.lag1_autocorr = (var > 1e-10) ? static_cast<float>(cov / var) : 0.0f;

    int outliers = 0;
    for (int i = 0; i < n; i++) {
        if (std::abs(data[i] - s.mean) > 3.0f * s.stddev) {
            outliers++;
        }
    }
    s.outlier_rate = static_cast<float>(outliers) / n;

    return s;
}

// Quyết định loại bộ lọc: 1=Median, 2=EMA, 3=SMA
static int decide_filter_type(const NoiseStats &st) {
    if (st.outlier_rate > 0.05f)   return 1; // Median
    if (st.lag1_autocorr > 0.5f)   return 2; // EMA
    return 3;                                 // SMA
}

// α để EMA đạt X% biên độ step sau n mẫu: (1-α)^n = 1-X → α = 1-(1-X)^(1/n)
static float ema_alpha(float capture_fraction, int n) {
    if (n <= 0) return 0.0f;
    return 1.0f - powf(1.0f - capture_fraction, 1.0f / n);
}

// ============================================================
// Kết quả — lưu persistent để emit mỗi tick sau khi tính xong
// ============================================================

struct TestOutput {
    // Phase 1
    float mean          = 0;
    float stddev        = 0;
    float lag1r         = 0;
    float outlier_pct   = 0;
    float onset_thresh  = 0;
    int   filter_type   = 0;   // 1=Median 2=EMA 3=SMA 0=unknown
    bool  phase1_done   = false;

    // Phase 2
    float peak          = 0;
    int   samples_peak  = 0;
    float amplitude_pct = 0;
    float alpha90       = 0;
    float alpha75       = 0;
    float alpha50       = 0;
    int   sma_window    = 0;
    bool  phase2_done   = false;
};

static void emit_output(const TestOutput &o) {
    if (!o.phase1_done) return;
    printf(">MQ135_mean:%.2f\n",        o.mean);
    printf(">MQ135_stddev:%.2f\n",      o.stddev);
    printf(">MQ135_lag1r:%.3f\n",       o.lag1r);
    printf(">MQ135_outlier:%.1f\n",     o.outlier_pct);
    printf(">MQ135_onset_thr:%.1f\n",   o.onset_thresh);
    printf(">MQ135_filter:%d\n",        o.filter_type);

    if (!o.phase2_done) return;
    printf(">MQ135_peak:%.1f\n",        o.peak);
    printf(">MQ135_amplitude:%.1f\n",   o.amplitude_pct);
    printf(">MQ135_alpha90:%.3f\n",     o.alpha90);
    printf(">MQ135_alpha75:%.3f\n",     o.alpha75);
    printf(">MQ135_alpha50:%.3f\n",     o.alpha50);
    printf(">MQ135_window:%d\n",        o.sma_window);
}

// ============================================================
// Task chính
// ============================================================

void test_mq135_task(void *) {
    // Chú thích Teleplot — hiển thị trong Messages pane
    printf("# === MQ135 Filter Selection Tool (T/RH compensation via BME680) ===\n");
    printf("# MQ135_action: 1=wait 2=sit_still 3=BREATHE_30s 4=perturbing 5=see_results\n");
    printf("# MQ135_filter: 1=Median 2=EMA 3=SMA\n");
    printf("# MQ135_CO2: gia tri da bu T/RH  |  MQ135_CO2_raw: chua bu (tham khao)\n");
    printf("# Theo doi MQ135_phase va MQ135_action de biet can lam gi\n");

    // ============================================================
    // Init BME680 — dùng để lấy T/RH cho compensation
    // Mirror SensorManager::bme680Setup() (không dùng class wrapper).
    // Nếu BME680 không khả dụng: fallback về điều kiện tham chiếu 20°C/65%RH.
    // ============================================================

    bool      bme680_avail = false;
    bme680_t  bme_dev{};
    float     last_t  = Cfg::MQ135_TREF_C;    // fallback: 20°C
    float     last_rh = Cfg::MQ135_RHREF_PCT;  // fallback: 65%RH

    if (i2cdev_init() == ESP_OK) {
        esp_err_t berr = bme680_init_desc(
            &bme_dev,
            Cfg::BME680_I2C_ADDR,
            static_cast<i2c_port_t>(0),
            static_cast<gpio_num_t>(Cfg::I2C_SDA_PIN),
            static_cast<gpio_num_t>(Cfg::I2C_SCL_PIN));
        if (berr == ESP_OK) berr = bme680_init_sensor(&bme_dev);
        if (berr == ESP_OK) berr = bme680_set_oversampling_rates(
            &bme_dev, BME680_OSR_2X, BME680_OSR_16X, BME680_OSR_1X);
        if (berr == ESP_OK) berr = bme680_set_filter_size(&bme_dev, BME680_IIR_SIZE_3);
        if (berr == ESP_OK) berr = bme680_set_heater_profile(
            &bme_dev, 0, Cfg::BME680_HEATER_TEMP_C, Cfg::BME680_HEATER_DUR_MS);
        if (berr == ESP_OK) berr = bme680_use_heater_profile(&bme_dev, 0);
        if (berr == ESP_OK) berr = bme680_set_ambient_temperature(
            &bme_dev, Cfg::BME680_AMBIENT_TEMP_C);
        bme680_avail = (berr == ESP_OK);
    }

    if (bme680_avail) {
        ESP_LOGI(TAG, "BME680 sẵn sàng (addr=0x%02X SDA=%d SCL=%d) — T/RH compensation active",
                 Cfg::BME680_I2C_ADDR, Cfg::I2C_SDA_PIN, Cfg::I2C_SCL_PIN);
    } else {
        ESP_LOGW(TAG, "BME680 không khả dụng — fallback T=%.0f°C RH=%.0f%% (ref conditions)",
                 Cfg::MQ135_TREF_C, Cfg::MQ135_RHREF_PCT);
    }

    // ============================================================
    // Init ADC1 — mirror SensorManager::mq135Init()
    // ============================================================

    adc_oneshot_unit_handle_t handle = nullptr;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id  = static_cast<adc_unit_t>(Cfg::MQ135_ADC_UNIT),
        .clk_src  = static_cast<adc_oneshot_clk_src_t>(0),
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = static_cast<adc_atten_t>(Cfg::MQ135_ADC_ATTEN),
        .bitwidth = static_cast<adc_bitwidth_t>(Cfg::MQ135_ADC_BITWIDTH),
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        handle, static_cast<adc_channel_t>(Cfg::MQ135_ADC_CHANNEL), &chan_cfg));

    // --- Calibration ---
    adc_cali_handle_t cali = nullptr;
    bool cali_ok = false;
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id      = static_cast<adc_unit_t>(Cfg::MQ135_ADC_UNIT),
        .atten        = static_cast<adc_atten_t>(Cfg::MQ135_ADC_ATTEN),
        .bitwidth     = static_cast<adc_bitwidth_t>(Cfg::MQ135_ADC_BITWIDTH),
        .default_vref = Cfg::MQ135_ADC_DEFAULT_VREF_MV,
    };
    cali_ok = (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali) == ESP_OK);
#elif ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = static_cast<adc_unit_t>(Cfg::MQ135_ADC_UNIT),
        .chan     = static_cast<adc_channel_t>(Cfg::MQ135_ADC_CHANNEL),
        .atten    = static_cast<adc_atten_t>(Cfg::MQ135_ADC_ATTEN),
        .bitwidth = static_cast<adc_bitwidth_t>(Cfg::MQ135_ADC_BITWIDTH),
    };
    cali_ok = (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali) == ESP_OK);
#endif

    ESP_LOGI(TAG, "init OK  cali=%s  R0=%.2fkΩ  RL=%.1fkΩ  RATIO_CLEAN=%.3f",
             cali_ok ? "yes" : "no", Cfg::MQ135_RO_KOHM, Cfg::MQ135_RL_KOHM, RATIO_CLEAN_AIR);

    // --- State ---
    Phase phase = Phase::WARMUP;
    constexpr int64_t WARMUP_US = static_cast<int64_t>(Cfg::MQ135_WARMUP_MS) * 1000LL;
    const int64_t t_start = esp_timer_get_time();

    std::array<float, COLLECT_N> buf{};
    int collect_idx = 0;

    NoiseStats baseline{};
    TestOutput out{};
    int perturb_count = 0;

    while (true) {
        // ============================================================
        // Đọc BME680 (T/RH) — mirror SensorManager::bme680ReadOnce()
        // Giữ last_t / last_rh từ lần đọc trước nếu lần này thất bại.
        // ============================================================

        if (bme680_avail) {
            uint32_t dur_ticks = 0;
            bme680_values_float_t r{};
            if (bme680_get_measurement_duration(&bme_dev, &dur_ticks) == ESP_OK
                && bme680_force_measurement(&bme_dev) == ESP_OK) {
                vTaskDelay(dur_ticks);   // I/O wait — đợi chip hoàn tất, không phải logic
                if (bme680_get_results_float(&bme_dev, &r) == ESP_OK) {
                    last_t  = r.temperature;
                    last_rh = r.humidity;
                    // Cập nhật ambient → cải thiện heat profile lần đo tiếp (mirror SensorManager)
                    bme680_set_ambient_temperature(&bme_dev, static_cast<int16_t>(last_t));
                }
            }
        }

        // ============================================================
        // Đọc ADC MQ-135
        // ============================================================

        int raw = 0;
        if (adc_oneshot_read(handle,
                static_cast<adc_channel_t>(Cfg::MQ135_ADC_CHANNEL), &raw) != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        int mv = 0;
        if (cali_ok) {
            adc_cali_raw_to_voltage(cali, raw, &mv);
        } else {
            mv = static_cast<int>((raw * Cfg::MQ135_VREF * 1000.0f)
                                  / ((1 << Cfg::MQ135_ADC_BITWIDTH) - 1));
        }

        float vrl = mv / 1000.0f;
        int64_t elapsed_us  = esp_timer_get_time() - t_start;
        bool    warmup_done = (elapsed_us >= WARMUP_US);
        float   warmup_pct  = std::min(100.0f, elapsed_us * 100.0f / (float)WARMUP_US);

        // Tín hiệu mạch hở / chưa đủ nhiệt
        if (vrl <= 0.01f) {
            printf(">MQ135_raw:%d\n",          raw);
            printf(">MQ135_warmup_pct:%.1f\n", warmup_pct);
            printf(">MQ135_phase:%d\n",        static_cast<int>(phase));
            printf(">MQ135_action:%d\n",       static_cast<int>(Action::WAIT_WARMUP));
            ESP_LOGW(TAG, "Vrl=%.3fV — mạch hở hoặc chưa đủ nhiệt", vrl);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // ============================================================
        // Tính Rs → ppm_raw (không bù) và ppm (đã bù T/RH)
        // Logic mirror SensorManager::mq135ReadPpm() + mq135CorrectionFactor()
        // ============================================================

        float rs = (Cfg::MQ135_VREF - vrl) * Cfg::MQ135_RL_KOHM / vrl;

        // Không bù — dùng để quan sát tác động của correction trên Teleplot
        float ratio_raw = rs / Cfg::MQ135_RO_KOHM;
        float ppm_raw   = Cfg::MQ135_CURVE_A * powf(ratio_raw, Cfg::MQ135_CURVE_B);

        // Bù T/RH: chuẩn hóa Rs về 20°C/65%RH — dùng cho toàn bộ state machine
        float cf      = mq135_correction_factor(last_t, last_rh);
        float rs_comp = (cf > 0.001f) ? rs / cf : rs;
        float ratio   = rs_comp / Cfg::MQ135_RO_KOHM;
        float ppm     = Cfg::MQ135_CURVE_A * powf(ratio, Cfg::MQ135_CURVE_B);

        // ============================================================
        // State machine (chạy trên ppm đã bù — đúng với giá trị production)
        // ============================================================

        Action action = Action::WAIT_WARMUP;

        switch (phase) {

        case Phase::WARMUP:
            action = Action::WAIT_WARMUP;
            ESP_LOGW(TAG, "[WARMUP %.1f%%] CO2=%.1fppm (raw=%.1f) T=%.1f°C RH=%.1f%%",
                     warmup_pct, ppm, ppm_raw, last_t, last_rh);
            if (warmup_done) {
                phase       = Phase::COLLECT;
                collect_idx = 0;
                ESP_LOGI(TAG, "Warmup done — COLLECT started, sit still");
            }
            break;

        case Phase::COLLECT:
            action = Action::SIT_STILL;
            buf[collect_idx++] = ppm;
            ESP_LOGI(TAG, "[COLLECT %2d/%d] CO2=%.1fppm (raw=%.1f) T=%.1f°C RH=%.1f%%",
                     collect_idx, COLLECT_N, ppm, ppm_raw, last_t, last_rh);
            if (collect_idx >= COLLECT_N) {
                phase = Phase::ANALYZE;
            }
            break;

        case Phase::ANALYZE:
            // one-shot: tính stats, chuyển sang WAIT_PERTURB ngay trong tick này
            baseline            = compute_noise_stats(buf.data(), COLLECT_N);
            out.mean            = baseline.mean;
            out.stddev          = baseline.stddev;
            out.lag1r           = baseline.lag1_autocorr;
            out.outlier_pct     = baseline.outlier_rate * 100.0f;
            out.onset_thresh    = baseline.mean + 4.0f * baseline.stddev;
            out.filter_type     = decide_filter_type(baseline);
            out.phase1_done     = true;

            ESP_LOGI(TAG, "ANALYZE: mean=%.1f std=%.1f lag1r=%.3f outlier=%.1f%% → filter=%d",
                     out.mean, out.stddev, out.lag1r, out.outlier_pct, out.filter_type);

            phase  = Phase::WAIT_PERTURB;
            action = Action::BREATHE_NOW;
            [[fallthrough]];

        case Phase::WAIT_PERTURB:
            action = Action::BREATHE_NOW;
            ESP_LOGI(TAG, "[WAIT] CO2=%.1fppm | onset>%.1fppm khi san sang tho hoi tho",
                     ppm, out.onset_thresh);
            if (ppm > out.onset_thresh) {
                phase            = Phase::PERTURB;
                out.peak         = ppm;
                out.samples_peak = 1;
                perturb_count    = 1;
                ESP_LOGI(TAG, "Onset detected! CO2=%.1fppm", ppm);
            }
            break;

        case Phase::PERTURB:
            action = Action::PERTURBING;
            perturb_count++;
            if (ppm > out.peak) {
                out.peak         = ppm;
                out.samples_peak = perturb_count;
            }
            ESP_LOGI(TAG, "[PERTURB %2d] CO2=%.1fppm  peak=%.1fppm @s%d",
                     perturb_count, ppm, out.peak, out.samples_peak);
            printf(">MQ135_perturb:%.1f\n", ppm);
            {
                bool past_peak = (perturb_count >= 5) && (ppm < out.peak * 0.90f);
                bool timeout   = (perturb_count >= 60);
                if (past_peak || timeout) {
                    phase = Phase::RECOMMEND;
                }
            }
            break;

        case Phase::RECOMMEND:
            // one-shot: tính params bộ lọc
            out.amplitude_pct = 100.0f * (out.peak - baseline.mean) / baseline.mean;
            out.alpha90       = ema_alpha(0.90f, out.samples_peak);
            out.alpha75       = ema_alpha(0.75f, out.samples_peak);
            out.alpha50       = ema_alpha(0.50f, out.samples_peak);
            out.sma_window    = std::max(2, out.samples_peak / 2);
            out.phase2_done   = true;

            ESP_LOGI(TAG, "RECOMMEND: filter=%d  amp=%.1f%%  peak_s=%d  α75=%.3f  win=%d",
                     out.filter_type, out.amplitude_pct, out.samples_peak,
                     out.alpha75, out.sma_window);

            phase  = Phase::DONE;
            action = Action::SEE_RESULTS;
            [[fallthrough]];

        case Phase::DONE:
            action = Action::SEE_RESULTS;
            ESP_LOGI(TAG, "[DONE] CO2=%.1fppm (raw=%.1f)", ppm, ppm_raw);
            break;
        }

        // ============================================================
        // Teleplot output — tất cả kênh mỗi tick
        // ============================================================

        printf(">MQ135_raw:%d\n",          raw);
        printf(">MQ135_V:%.4f\n",          vrl);
        printf(">MQ135_RS:%.4f\n",         rs);
        printf(">MQ135_T:%.2f\n",          last_t);
        printf(">MQ135_RH:%.2f\n",         last_rh);
        printf(">MQ135_CF:%.4f\n",         cf);
        printf(">MQ135_ratio:%.5f\n",      ratio);
        printf(">MQ135_CO2_raw:%.1f\n",    ppm_raw);
        printf(">MQ135_CO2:%.1f\n",        ppm);
        printf(">MQ135_warmup_pct:%.1f\n", warmup_pct);
        printf(">MQ135_phase:%d\n",        static_cast<int>(phase));
        printf(">MQ135_action:%d\n",       static_cast<int>(action));

        emit_output(out);  // các kênh stats — chỉ emit khi đã có dữ liệu

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

#endif // CONFIG_TEST_MODE_MQ135

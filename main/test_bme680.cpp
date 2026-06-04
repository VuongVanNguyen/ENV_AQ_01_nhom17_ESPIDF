// ============================================================
// test_bme680.cpp — Standalone test BME680
//   Logic mirror chính xác SensorManager::bme680Setup() và
//   SensorManager::bme680ReadOnce() — không dùng class wrapper.
//
//   Bật: idf.py menuconfig → Sensor Test Mode → BME680 only
// ============================================================

#include "sdkconfig.h"
#ifdef CONFIG_TEST_MODE_BME680

#include "test_sensors.hpp"
#include "config.hpp"

#include "bme680.h"
#include "i2cdev.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

static const char *TAG = "TEST_BME680";

namespace {
    constexpr int   STAT_WINDOW       = 10;        // rolling window (mẫu)
    constexpr float OUTLIER_T_DELTA   = 0.10f;    // °C: |ΔT| > này → outlier (~20× stddev đo)
    constexpr float OUTLIER_RH_DELTA  = 0.50f;    // %RH              (~13× stddev đo)
    constexpr float OUTLIER_GAS_DELTA = 10000.0f; // Ω                (~7× stddev đo)
}

struct RollingStat {
    float buf[STAT_WINDOW]{};
    int   n   = 0;
    int   idx = 0;

    void push(float v) {
        buf[idx] = v;
        idx = (idx + 1) % STAT_WINDOW;
        if (n < STAT_WINDOW) ++n;
    }
    float mean() const {
        float s = 0;
        for (int i = 0; i < n; ++i) s += buf[i];
        return n ? s / n : 0.0f;
    }
    float stddev() const {
        if (n < 2) return 0.0f;
        float m = mean(), var = 0;
        for (int i = 0; i < n; ++i) var += (buf[i] - m) * (buf[i] - m);
        return sqrtf(var / n);
    }
    void reset() { n = 0; idx = 0; }
};

void test_bme680_task(void *) {
    // --- i2cdev_init() — bắt buộc trước bme680_init_desc (giống SensorManager::init()) ---
    ESP_ERROR_CHECK(i2cdev_init());

    // --- Setup BME680 (giống SensorManager::bme680Setup()) ---
    bme680_t dev{};
    ESP_ERROR_CHECK(bme680_init_desc(
        &dev,
        Cfg::BME680_I2C_ADDR,
        static_cast<i2c_port_t>(0),
        static_cast<gpio_num_t>(Cfg::I2C_SDA_PIN),
        static_cast<gpio_num_t>(Cfg::I2C_SCL_PIN)));

    ESP_ERROR_CHECK(bme680_init_sensor(&dev));

    // Oversampling: T x2, P x16, H x1
    ESP_ERROR_CHECK(bme680_set_oversampling_rates(
        &dev, BME680_OSR_2X, BME680_OSR_16X, BME680_OSR_1X));

    // IIR filter coef 3 cho T & P
    ESP_ERROR_CHECK(bme680_set_filter_size(&dev, BME680_IIR_SIZE_3));

    // Heater profile 0: 320°C / 150ms
    ESP_ERROR_CHECK(bme680_set_heater_profile(
        &dev, 0, Cfg::BME680_HEATER_TEMP_C, Cfg::BME680_HEATER_DUR_MS));
    ESP_ERROR_CHECK(bme680_use_heater_profile(&dev, 0));

    // Ambient ban đầu 25°C — được cập nhật sau mỗi lần đo
    ESP_ERROR_CHECK(bme680_set_ambient_temperature(&dev, Cfg::BME680_AMBIENT_TEMP_C));

    ESP_LOGI(TAG, "BME680 init OK  addr=0x%02X  SDA=%d  SCL=%d  heater=%u°C/%ums",
             Cfg::BME680_I2C_ADDR, Cfg::I2C_SDA_PIN, Cfg::I2C_SCL_PIN,
             (unsigned)Cfg::BME680_HEATER_TEMP_C, (unsigned)Cfg::BME680_HEATER_DUR_MS);
    ESP_LOGI(TAG, "Bắt đầu đọc. gas_resistance > 0 → heater ổn định.");
    ESP_LOGI(TAG, "Warmup BME680 = %lu ms (~%.0f phút) — theo Cfg::BME680_WARMUP_MS",
             (unsigned long)Cfg::BME680_WARMUP_MS, Cfg::BME680_WARMUP_MS / 60000.0);
    ESP_LOGI(TAG, "Outlier threshold: T=±%.1f°C  RH=±%.1f%%  Gas=±%.0fΩ (local test constant)",
             OUTLIER_T_DELTA, OUTLIER_RH_DELTA, OUTLIER_GAS_DELTA);

    // --- Warmup: mirror SensorManager::readAll() ---
    // ready = (elapsedMs() >= BME680_WARMUP_MS) && heater_ok && gas > 0
    const int64_t t_start = esp_timer_get_time();

    // ── Trạng thái rolling — chỉ tích lũy sau khi warmup xong ───────────
    float prev_T = 0, prev_RH = 0, prev_Gas = 0;
    bool  first_stable = true;  // true lại sau mỗi lần reset stats
    bool  was_ready    = false; // phát hiện edge warmup → ready
    RollingStat stat_T, stat_RH, stat_Gas;
    uint32_t total_samples   = 0;
    uint32_t outlier_T_cnt   = 0;
    uint32_t outlier_RH_cnt  = 0;
    uint32_t outlier_Gas_cnt = 0;

    // --- Vòng lặp đọc (giống SensorManager::bme680ReadOnce()) ---
    while (true) {
        uint32_t dur_ticks = 0;
        if (bme680_get_measurement_duration(&dev, &dur_ticks) != ESP_OK) {
            ESP_LOGE(TAG, "Không lấy được measurement duration");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        if (bme680_force_measurement(&dev) != ESP_OK) {
            ESP_LOGE(TAG, "force_measurement thất bại");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // I/O wait — chờ đúng bằng thời gian đo nội tại của chip
        vTaskDelay(dur_ticks);

        bme680_values_float_t r{};
        if (bme680_get_results_float(&dev, &r) != ESP_OK) {
            ESP_LOGW(TAG, "Đọc kết quả thất bại");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Mirror SensorManager::readAll(): ready = time + heater_ok + gas > 0
        uint32_t elapsed_ms = static_cast<uint32_t>((esp_timer_get_time() - t_start) / 1000LL);
        bool heater_ok  = (r.gas_resistance > 0.0f);
        bool ready      = (elapsed_ms >= Cfg::BME680_WARMUP_MS) && heater_ok;
        float warmup_pct = (elapsed_ms >= Cfg::BME680_WARMUP_MS)
                           ? 100.0f
                           : elapsed_ms * 100.0f / static_cast<float>(Cfg::BME680_WARMUP_MS);

        // ── Teleplot raw — xuất cả trong warmup để quan sát đường cong khởi động
        printf(">BME680_t_ms:%lu\n",       elapsed_ms);
        printf(">BME680_Temp:%.2f\n",      r.temperature);
        printf(">BME680_RH:%.2f\n",        r.humidity);
        printf(">BME680_Pres:%.2f\n",      r.pressure);
        printf(">BME680_Gas:%.0f\n",       r.gas_resistance);
        printf(">BME680_warmup_pct:%.1f\n", warmup_pct);
        printf(">BME680_ready:%d\n",       ready ? 1 : 0);

        if (ready) {
            // ── Edge warmup → ready: reset stats một lần duy nhất ────────
            if (!was_ready) {
                ESP_LOGI(TAG, ">>> Warmup hoàn tất — reset stats, bắt đầu đo nhiễu ổn định <<<");
                stat_T.reset(); stat_RH.reset(); stat_Gas.reset();
                total_samples = outlier_T_cnt = outlier_RH_cnt = outlier_Gas_cnt = 0;
                first_stable = was_ready = true;
            }

            // ── Delta & outlier detection ─────────────────────────────────
            float dT = 0, dRH = 0, dGas = 0;
            bool  out_T = false, out_RH = false, out_Gas = false;
            if (!first_stable) {
                dT   = r.temperature    - prev_T;
                dRH  = r.humidity       - prev_RH;
                dGas = r.gas_resistance - prev_Gas;

                out_T   = (fabsf(dT)   > OUTLIER_T_DELTA);
                out_RH  = (fabsf(dRH)  > OUTLIER_RH_DELTA);
                out_Gas = (fabsf(dGas) > OUTLIER_GAS_DELTA);

                if (out_T)   ++outlier_T_cnt;
                if (out_RH)  ++outlier_RH_cnt;
                if (out_Gas) ++outlier_Gas_cnt;

                if (out_T || out_RH || out_Gas) {
                    ESP_LOGW(TAG, "[OUTLIER] dT=%+.3f°C  dRH=%+.3f%%  dGas=%+.0fΩ%s%s%s",
                             dT, dRH, dGas,
                             out_T   ? "  ← T!" : "",
                             out_RH  ? "  ← RH!" : "",
                             out_Gas ? "  ← Gas!" : "");
                }
            }

            // ── Rolling stat ──────────────────────────────────────────────
            stat_T.push(r.temperature);
            stat_RH.push(r.humidity);
            stat_Gas.push(r.gas_resistance);
            ++total_samples;

            prev_T = r.temperature; prev_RH = r.humidity; prev_Gas = r.gas_resistance;
            first_stable = false;

            // ── Log + Teleplot noise metrics ──────────────────────────────
            ESP_LOGI(TAG, "[READY] T=%6.2f°C  RH=%5.2f%%  P=%7.2fhPa  Gas=%8.0fΩ",
                     r.temperature, r.humidity, r.pressure, r.gas_resistance);

            printf(">BME680_dTemp:%.4f\n",  dT);
            printf(">BME680_dRH:%.4f\n",    dRH);
            printf(">BME680_dGas:%.0f\n",   dGas);
            printf(">BME680_stdT:%.4f\n",   stat_T.stddev());
            printf(">BME680_stdRH:%.4f\n",  stat_RH.stddev());
            printf(">BME680_stdGas:%.0f\n", stat_Gas.stddev());
            if (total_samples > 1) {
                float denom = static_cast<float>(total_samples - 1);
                printf(">BME680_out_T_rate:%.3f\n",   outlier_T_cnt   / denom);
                printf(">BME680_out_RH_rate:%.3f\n",  outlier_RH_cnt  / denom);
                printf(">BME680_out_Gas_rate:%.3f\n", outlier_Gas_cnt / denom);
            }
        } else {
            ESP_LOGW(TAG, "[WARMUP %5.1f%%] T=%6.2f°C  RH=%5.2f%%  P=%7.2fhPa  Gas=%s",
                     warmup_pct, r.temperature, r.humidity, r.pressure,
                     heater_ok ? "đang ổn định..." : "NOT STABLE");
        }

        // Cập nhật ambient temp → cải thiện heat profile cho lần đo tiếp
        bme680_set_ambient_temperature(&dev, static_cast<int16_t>(r.temperature));

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

#endif // CONFIG_TEST_MODE_BME680

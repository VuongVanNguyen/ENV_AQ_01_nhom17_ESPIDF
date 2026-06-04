// ============================================================
// test_pms5003.cpp — Standalone test PMS5003
//   Logic mirror chính xác SensorManager::pmsInit(),
//   SensorManager::pmsReadOneFrame() và SensorManager::pmsReadFrame()
//   — không dùng class wrapper.
//
//   Bật: idf.py menuconfig → Sensor Test Mode → PMS5003 only
//
//   Mục tiêu thu thập dữ liệu để chọn bộ lọc:
//     ─ Đo stddev baseline → noise floor của từng kênh PM
//     ─ Đo delta per sample (|ΔPM|) → phân tích phân phối: Gaussian hay impulse?
//     ─ Đếm spike tại nhiều ngưỡng δ → ước lượng heavy-tail noise
//     ─ Theo dõi max delta → biên độ spike cực đại
//     ─ mean(|delta|) / stddev → nếu tỉ lệ cao → impulse-dominated → favor Median
//                                 nếu thấp     → Gaussian-like    → favor EMA/SMA
//   Từ các số liệu trên mới quyết định bộ lọc nào phù hợp.
// ============================================================

#include "sdkconfig.h"  // phải include trước #ifdef để ninja track dependency đúng
#ifdef CONFIG_TEST_MODE_PMS5003

#include "test_sensors.hpp"
#include "config.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <cmath>
#include <cstring>

static const char *TAG = "TEST_PMS5003";

namespace {
    constexpr int STAT_WINDOW = 10;  // rolling window (mẫu)

    // Các ngưỡng delta để đếm spike rate — dải rộng để phân tích phân phối.
    // Sau khi có stddev thực tế sẽ biết ngưỡng nào ứng với bao nhiêu σ.
    constexpr float SPIKE_THR_LO  =  5.0f;   // µg/m³ — ~1–2 lsb tại giá trị thấp
    constexpr float SPIKE_THR_MID = 15.0f;   // µg/m³ — spike trung bình
    constexpr float SPIKE_THR_HI  = 30.0f;   // µg/m³ — spike lớn
}

// ── Rolling statistics ─────────────────────────────────────────────────────
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

// ── Rolling mean of |delta| — đo mức noise tuyệt đối không phụ thuộc sign ──
struct RollingAbsDelta {
    float buf[STAT_WINDOW]{};
    int   n   = 0;
    int   idx = 0;

    void push(float abs_delta) {
        buf[idx] = abs_delta;
        idx = (idx + 1) % STAT_WINDOW;
        if (n < STAT_WINDOW) ++n;
    }
    float mean() const {
        float s = 0;
        for (int i = 0; i < n; ++i) s += buf[i];
        return n ? s / n : 0.0f;
    }
    void reset() { n = 0; idx = 0; }
};

// Mirror SensorManager::pmsReadOneFrame()
static esp_err_t read_one_frame(uart_port_t port, uint8_t *buf, TickType_t scan_ticks) {
    uint8_t b = 0;
    int scanned = 0;
    bool got_start = false, found = false;

    while (scanned < Cfg::PMS_FRAME_LEN * 2) {
        TickType_t ticks = got_start
            ? std::min(scan_ticks, pdMS_TO_TICKS(20))
            : scan_ticks;
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

    if (uart_read_bytes(port, buf + 2, Cfg::PMS_FRAME_LEN - 2, pdMS_TO_TICKS(100))
            != Cfg::PMS_FRAME_LEN - 2)
        return ESP_ERR_TIMEOUT;

    uint16_t sum = 0;
    for (int i = 0; i < Cfg::PMS_FRAME_LEN - 2; ++i) sum += buf[i];
    uint16_t expect = (static_cast<uint16_t>(buf[Cfg::PMS_FRAME_LEN - 2]) << 8)
                    | buf[Cfg::PMS_FRAME_LEN - 1];
    return (sum == expect) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

void test_pms5003_task(void *) {
    // --- Init SET pin HIGH = active (giống SensorManager::pmsInit()) ---
    gpio_config_t set_cfg = {
        .pin_bit_mask = 1ULL << Cfg::PMS_SET_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&set_cfg));
    gpio_set_level(static_cast<gpio_num_t>(Cfg::PMS_SET_PIN), 1);

    // --- Init UART ---
    const uart_port_t port = static_cast<uart_port_t>(Cfg::PMS_UART_PORT);
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
    ESP_ERROR_CHECK(uart_driver_install(port, Cfg::PMS_FRAME_LEN * 8, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port,
        Cfg::PMS_UART_TX_PIN, Cfg::PMS_UART_RX_PIN,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "PMS5003 init OK  UART%d  RX=%d  TX=%d  SET=%d",
             Cfg::PMS_UART_PORT, Cfg::PMS_UART_RX_PIN,
             Cfg::PMS_UART_TX_PIN, Cfg::PMS_SET_PIN);
    ESP_LOGI(TAG, "Cần %d frame OK liên tiếp → pms5003_ready=true",
             Cfg::PMS_VALID_STREAK_OK);
    ESP_LOGI(TAG, "Warmup PMS5003 = %lu ms (~%.0f giây) — theo Cfg::PMS5003_WARMUP_MS",
             (unsigned long)Cfg::PMS5003_WARMUP_MS, Cfg::PMS5003_WARMUP_MS / 1000.0);
    ESP_LOGI(TAG, "Spike thresholds: LO=%.0f  MID=%.0f  HI=%.0f µg/m³/sample",
             SPIKE_THR_LO, SPIKE_THR_MID, SPIKE_THR_HI);
    ESP_LOGI(TAG, "Quan sat Teleplot: stdPM25, meanAbsDelta, spike_rate_* de quyet dinh bo loc");

    const int64_t t_start = esp_timer_get_time();

    // ── Trạng thái rolling — chỉ tích lũy sau khi warmup xong ─────────────
    float prev_pm1 = 0, prev_pm25 = 0, prev_pm10 = 0;
    bool  first_stable = true;
    bool  was_ready    = false;

    RollingStat    stat_pm1,  stat_pm25,  stat_pm10;
    RollingAbsDelta mad_pm1,  mad_pm25,   mad_pm10;  // mean absolute delta

    uint32_t total_samples = 0;
    float    max_delta_pm1 = 0, max_delta_pm25 = 0, max_delta_pm10 = 0;

    // Spike counters tại 3 ngưỡng cho kênh PM2.5 (kênh quan trọng nhất cho AQI)
    uint32_t spike_lo_cnt = 0, spike_mid_cnt = 0, spike_hi_cnt = 0;

    // --- Vòng lặp đọc ---
    int streak = 0;
    int total  = 0;
    while (true) {
        uint8_t buf[Cfg::PMS_FRAME_LEN];

        esp_err_t err = read_one_frame(port, buf, pdMS_TO_TICKS(2000));

        // Drain frame tích đống: giữ frame mới nhất hợp lệ
        if (err == ESP_OK) {
            size_t avail = 0;
            uart_get_buffered_data_len(port, &avail);
            while (avail >= static_cast<size_t>(Cfg::PMS_FRAME_LEN)) {
                uint8_t tmp[Cfg::PMS_FRAME_LEN];
                if (read_one_frame(port, tmp, pdMS_TO_TICKS(2)) == ESP_OK)
                    memcpy(buf, tmp, Cfg::PMS_FRAME_LEN);
                size_t tmp_avail = 0;
                uart_get_buffered_data_len(port, &tmp_avail);
                if (tmp_avail >= avail) break;
                avail = tmp_avail;
            }
        }

        total++;
        if (err == ESP_OK) {
            if (streak < Cfg::PMS_VALID_STREAK_OK) ++streak;

            // Dùng giá trị atmospheric (offset 10/12/14)
            uint16_t pm1  = (static_cast<uint16_t>(buf[10]) << 8) | buf[11];
            uint16_t pm25 = (static_cast<uint16_t>(buf[12]) << 8) | buf[13];
            uint16_t pm10 = (static_cast<uint16_t>(buf[14]) << 8) | buf[15];

            uint32_t elapsed_ms = static_cast<uint32_t>((esp_timer_get_time() - t_start) / 1000LL);
            bool     ready      = (elapsed_ms >= Cfg::PMS5003_WARMUP_MS)
                                  && (streak >= Cfg::PMS_VALID_STREAK_OK);
            float    warmup_pct = (elapsed_ms >= Cfg::PMS5003_WARMUP_MS)
                                  ? 100.0f
                                  : elapsed_ms * 100.0f / static_cast<float>(Cfg::PMS5003_WARMUP_MS);

            // Teleplot raw — xuất cả trong warmup
            printf(">PMS_PM1:%u\n",          pm1);
            printf(">PMS_PM25:%u\n",         pm25);
            printf(">PMS_PM10:%u\n",         pm10);
            printf(">PMS_streak:%d\n",       streak);
            printf(">PMS_warmup_pct:%.1f\n", warmup_pct);
            printf(">PMS_ready:%d\n",        ready ? 1 : 0);

            if (ready) {
                // ── Edge warmup → ready: reset stats ────────────────────────
                if (!was_ready) {
                    ESP_LOGI(TAG, ">>> Warmup hoàn tất — reset stats, bắt đầu thu thập so lieu chon bo loc <<<");
                    stat_pm1.reset(); stat_pm25.reset(); stat_pm10.reset();
                    mad_pm1.reset();  mad_pm25.reset();  mad_pm10.reset();
                    total_samples   = 0;
                    max_delta_pm1   = max_delta_pm25 = max_delta_pm10 = 0;
                    spike_lo_cnt    = spike_mid_cnt   = spike_hi_cnt  = 0;
                    first_stable    = was_ready = true;
                }

                // ── Delta per sample ─────────────────────────────────────────
                float dpm1 = 0, dpm25 = 0, dpm10 = 0;
                float abs_d1 = 0, abs_d25 = 0, abs_d10 = 0;
                if (!first_stable) {
                    dpm1  = static_cast<float>(pm1)  - prev_pm1;
                    dpm25 = static_cast<float>(pm25) - prev_pm25;
                    dpm10 = static_cast<float>(pm10) - prev_pm10;
                    abs_d1  = fabsf(dpm1);
                    abs_d25 = fabsf(dpm25);
                    abs_d10 = fabsf(dpm10);

                    if (abs_d1  > max_delta_pm1)  max_delta_pm1  = abs_d1;
                    if (abs_d25 > max_delta_pm25) max_delta_pm25 = abs_d25;
                    if (abs_d10 > max_delta_pm10) max_delta_pm10 = abs_d10;

                    mad_pm1.push(abs_d1);
                    mad_pm25.push(abs_d25);
                    mad_pm10.push(abs_d10);

                    // Spike rate tại 3 ngưỡng cho PM2.5 — kênh AQI quan trọng nhất
                    if (abs_d25 > SPIKE_THR_LO)  ++spike_lo_cnt;
                    if (abs_d25 > SPIKE_THR_MID) ++spike_mid_cnt;
                    if (abs_d25 > SPIKE_THR_HI)  ++spike_hi_cnt;

                    // Log spike bất thường để quan sát pattern
                    if (abs_d25 > SPIKE_THR_MID) {
                        ESP_LOGW(TAG, "[SPIKE PM25 %+.0f µg/m³] PM1=%u  PM25=%u  PM10=%u",
                                 dpm25, pm1, pm25, pm10);
                    }
                }

                // ── Rolling stat ─────────────────────────────────────────────
                stat_pm1.push(static_cast<float>(pm1));
                stat_pm25.push(static_cast<float>(pm25));
                stat_pm10.push(static_cast<float>(pm10));
                ++total_samples;

                prev_pm1  = static_cast<float>(pm1);
                prev_pm25 = static_cast<float>(pm25);
                prev_pm10 = static_cast<float>(pm10);
                first_stable = false;

                float std25  = stat_pm25.stddev();
                float mad25  = mad_pm25.mean();

                ESP_LOGI(TAG, "[READY #%3d] PM1=%3u  PM25=%3u  PM10=%3u  "
                         "std25=%.2f  mad25=%.2f  maxD25=%.0f",
                         total, pm1, pm25, pm10, std25, mad25, max_delta_pm25);

                // ── Teleplot noise metrics ────────────────────────────────────
                printf(">PMS_dPM1:%.1f\n",          dpm1);
                printf(">PMS_dPM25:%.1f\n",          dpm25);
                printf(">PMS_dPM10:%.1f\n",          dpm10);
                printf(">PMS_stdPM1:%.2f\n",         stat_pm1.stddev());
                printf(">PMS_stdPM25:%.2f\n",        std25);
                printf(">PMS_stdPM10:%.2f\n",        stat_pm10.stddev());
                printf(">PMS_meanAbsDelta_PM25:%.2f\n", mad25);
                printf(">PMS_maxDelta_PM25:%.0f\n",  max_delta_pm25);

                // Spike rate PM2.5 tại 3 ngưỡng — dùng để phân tích phân phối delta
                if (total_samples > 1) {
                    float denom = static_cast<float>(total_samples - 1);
                    printf(">PMS_spike_lo_rate:%.3f\n",  spike_lo_cnt  / denom);
                    printf(">PMS_spike_mid_rate:%.3f\n", spike_mid_cnt / denom);
                    printf(">PMS_spike_hi_rate:%.3f\n",  spike_hi_cnt  / denom);
                }

                // meanAbsDelta / stddev: ~ sqrt(2/pi) ≈ 0.80 với Gaussian,
                // cao hơn đáng kể → heavy-tail / impulse → favor Median filter
                if (std25 > 0.0f) {
                    printf(">PMS_mad_std_ratio_PM25:%.3f\n", mad25 / std25);
                }
            } else {
                ESP_LOGW(TAG, "[WARMUP %5.1f%%  #%3d] PM1=%3u  PM25=%3u  PM10=%3u  streak=%d/%d",
                         warmup_pct, total, pm1, pm25, pm10, streak, Cfg::PMS_VALID_STREAK_OK);
            }
        } else {
            streak = 0;
            uint32_t elapsed_ms = static_cast<uint32_t>((esp_timer_get_time() - t_start) / 1000LL);
            float warmup_pct = (elapsed_ms >= Cfg::PMS5003_WARMUP_MS)
                               ? 100.0f
                               : elapsed_ms * 100.0f / static_cast<float>(Cfg::PMS5003_WARMUP_MS);
            ESP_LOGW(TAG, "[WARMUP %5.1f%%  #%3d] Frame thất bại: %s  streak=reset",
                     warmup_pct, total, esp_err_to_name(err));
            printf(">PMS_streak:%d\n",       streak);
            printf(">PMS_warmup_pct:%.1f\n", warmup_pct);
            printf(">PMS_ready:0\n");
        }
        // PMS5003 tự gửi 1 frame/giây — không cần delay thêm
    }
}

#endif // CONFIG_TEST_MODE_PMS5003

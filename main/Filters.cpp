// ============================================================
// Filters.cpp — Hiện thực module lọc nhiễu EMA/SMA hai tầng.
//
//   Xem Filters.hpp và CLAUDE.md Mục 7 để biết thiết kế đầy đủ.
//   Toàn bộ tham số lọc và sanity range lấy từ namespace Cfg.
// ============================================================

#include "Filters.hpp"
#include "esp_log.h"
#include <cmath>
#include <algorithm>

static const char *TAG = "Filters";

// ============================================================
// Constructor
// ============================================================
Filters::Filters() {
    reset();
}

// ============================================================
// reset() — xoá toàn bộ trạng thái filter
// ============================================================
void Filters::reset() {
    ema_temp_  = {0.0f, false};
    ema_press_ = {0.0f, false};
    ema_co2_   = {0.0f, false};
    ema_pm1_   = {0.0f, false};
    ema_pm25_  = {0.0f, false};
    ema_pm10_  = {0.0f, false};

    sma_humi_.buf.fill(0.0f);
    sma_humi_.write_idx   = 0;
    sma_humi_.count       = 0;
    sma_humi_.last_output = 0.0f;

    prev_bme680_ready_  = false;
    prev_mq135_ready_   = false;
    prev_pms5003_ready_ = false;
}

// ============================================================
// process() — điểm vào chính, lọc in-place toàn bộ AirData
// ============================================================
void Filters::process(AirData &data) {
    // §5.3: data_valid == false → bỏ qua toàn bộ chu kỳ, không cập nhật state
    if (!data.data_valid) {
        return;
    }

    // §5.2: phát hiện chuyển trạng thái false→true → tái khởi tạo filter nhóm đó
    if (data.bme680_ready  && !prev_bme680_ready_)  { resetBme680State();  }
    if (data.mq135_ready   && !prev_mq135_ready_)   { resetMq135State();   }
    if (data.pms5003_ready && !prev_pms5003_ready_) { resetPms5003State(); }

    prev_bme680_ready_  = data.bme680_ready;
    prev_mq135_ready_   = data.mq135_ready;
    prev_pms5003_ready_ = data.pms5003_ready;

    processBme680(data);
    processMq135(data);
    processPms5003(data);
}

// ============================================================
// BME680: temperature (EMA), humidity (SMA), pressure (EMA)
// gas_resistance: không lọc — chỉ giữ giá trị thô (SensorManager) để xem qua JTAG,
// không feed vào AQI/Comfort/CO2/alert nào (CLAUDE.md: đã bỏ tính TVOC)
// ============================================================
void Filters::processBme680(AirData &data) {
    if (!data.bme680_ready) {
        // Hold-last-good: ghi lại giá trị đã lọc cuối cùng nếu đã khởi tạo
        if (ema_temp_.initialized)  data.temperature    = ema_temp_.prev;
        if (sma_humi_.count > 0)    data.humidity       = sma_humi_.last_output;
        if (ema_press_.initialized) data.pressure       = ema_press_.prev;
        return;
    }

    // ---- Temperature — EMA α = Cfg::EMA_ALPHA_TEMP ----
    if (inRange(data.temperature, Cfg::SANITY_TEMP_MIN, Cfg::SANITY_TEMP_MAX)) {
        data.temperature = ema_temp_.initialized
            ? emaUpdate(ema_temp_, data.temperature, Cfg::EMA_ALPHA_TEMP)
            : emaWarmStart(ema_temp_, data.temperature);
    } else {
        ESP_LOGW(TAG, "Nhiệt độ ngoài mức cho phép: %.2f°C [%.0f, %.0f]",
                 data.temperature, Cfg::SANITY_TEMP_MIN, Cfg::SANITY_TEMP_MAX);
        if (ema_temp_.initialized) { data.temperature = ema_temp_.prev; }
    }

    // ---- Humidity — SMA W = Cfg::FILTER_WINDOW_SIZE ----
    if (inRange(data.humidity, Cfg::SANITY_HUMI_MIN, Cfg::SANITY_HUMI_MAX)) {
        data.humidity = smaUpdate(sma_humi_, data.humidity);
    } else {
        ESP_LOGW(TAG, "Độ ẩm ngoài mức cho phép: %.2f%% [%.0f, %.0f]",
                 data.humidity, Cfg::SANITY_HUMI_MIN, Cfg::SANITY_HUMI_MAX);
        if (sma_humi_.count > 0) { data.humidity = sma_humi_.last_output; }
    }

    // ---- Pressure — EMA α = Cfg::EMA_ALPHA_PRESS ----
    if (inRange(data.pressure, Cfg::SANITY_PRESS_MIN, Cfg::SANITY_PRESS_MAX)) {
        data.pressure = ema_press_.initialized
            ? emaUpdate(ema_press_, data.pressure, Cfg::EMA_ALPHA_PRESS)
            : emaWarmStart(ema_press_, data.pressure);
    } else {
        ESP_LOGW(TAG, "Áp suất ngoài mức cho phép: %.2f hPa [%.0f, %.0f]",
                 data.pressure, Cfg::SANITY_PRESS_MIN, Cfg::SANITY_PRESS_MAX);
        if (ema_press_.initialized) { data.pressure = ema_press_.prev; }
    }
}

// ============================================================
// MQ-135: co2_ppm (giá trị đã bù T/RH bởi SensorManager trước khi đến đây)
// ============================================================
void Filters::processMq135(AirData &data) {
    if (!data.mq135_ready) {
        if (ema_co2_.initialized) { data.co2_ppm = ema_co2_.prev; }
        return;
    }

    if (inRange(data.co2_ppm, Cfg::SANITY_CO2_MIN, Cfg::SANITY_CO2_MAX)) {
        data.co2_ppm = ema_co2_.initialized
            ? emaUpdate(ema_co2_, data.co2_ppm, Cfg::EMA_ALPHA_CO2)
            : emaWarmStart(ema_co2_, data.co2_ppm);
    } else {
        ESP_LOGW(TAG, "CO2 ngoài mức cho phép: %.1f ppm [%.0f, %.0f]",
                 data.co2_ppm, Cfg::SANITY_CO2_MIN, Cfg::SANITY_CO2_MAX);
        if (ema_co2_.initialized) { data.co2_ppm = ema_co2_.prev; }
    }
}

// ============================================================
// PMS5003: pm1_0, pm2_5, pm10
//   EMA tính nội bộ bằng float; ghi ngược về uint16_t: lroundf + clamp [0, 500]
// ============================================================
void Filters::processPms5003(AirData &data) {
    const long pm_max = static_cast<long>(Cfg::SANITY_PM_MAX);

    if (!data.pms5003_ready) {
        if (ema_pm1_.initialized) {
            data.pm1_0 = static_cast<uint16_t>(
                std::clamp(lroundf(ema_pm1_.prev),  0L, pm_max));
        }
        if (ema_pm25_.initialized) {
            data.pm2_5 = static_cast<uint16_t>(
                std::clamp(lroundf(ema_pm25_.prev), 0L, pm_max));
        }
        if (ema_pm10_.initialized) {
            data.pm10  = static_cast<uint16_t>(
                std::clamp(lroundf(ema_pm10_.prev), 0L, pm_max));
        }
        return;
    }

    // ---- PM1.0 — EMA α = Cfg::EMA_ALPHA_PM1 ----
    {
        float raw = static_cast<float>(data.pm1_0);
        if (inRange(raw, Cfg::SANITY_PM_MIN, Cfg::SANITY_PM_MAX)) {
            float f = ema_pm1_.initialized
                ? emaUpdate(ema_pm1_, raw, Cfg::EMA_ALPHA_PM1)
                : emaWarmStart(ema_pm1_, raw);
            data.pm1_0 = static_cast<uint16_t>(
                std::clamp(lroundf(f), 0L, pm_max));
        } else {
            ESP_LOGW(TAG, "PM1.0 ngoài mức cho phép: %u ug/m3 [%.0f, %.0f]",
                     data.pm1_0, Cfg::SANITY_PM_MIN, Cfg::SANITY_PM_MAX);
            if (ema_pm1_.initialized) {
                data.pm1_0 = static_cast<uint16_t>(
                    std::clamp(lroundf(ema_pm1_.prev), 0L, pm_max));
            }
        }
    }

    // ---- PM2.5 — EMA α = Cfg::EMA_ALPHA_PM25 ----
    {
        float raw = static_cast<float>(data.pm2_5);
        if (inRange(raw, Cfg::SANITY_PM_MIN, Cfg::SANITY_PM_MAX)) {
            float f = ema_pm25_.initialized
                ? emaUpdate(ema_pm25_, raw, Cfg::EMA_ALPHA_PM25)
                : emaWarmStart(ema_pm25_, raw);
            data.pm2_5 = static_cast<uint16_t>(
                std::clamp(lroundf(f), 0L, pm_max));
        } else {
            ESP_LOGW(TAG, "PM2.5 ngoài mức cho phép: %u ug/m3 [%.0f, %.0f]",
                     data.pm2_5, Cfg::SANITY_PM_MIN, Cfg::SANITY_PM_MAX);
            if (ema_pm25_.initialized) {
                data.pm2_5 = static_cast<uint16_t>(
                    std::clamp(lroundf(ema_pm25_.prev), 0L, pm_max));
            }
        }
    }

    // ---- PM10 — EMA α = Cfg::EMA_ALPHA_PM10 ----
    {
        float raw = static_cast<float>(data.pm10);
        if (inRange(raw, Cfg::SANITY_PM_MIN, Cfg::SANITY_PM_MAX)) {
            float f = ema_pm10_.initialized
                ? emaUpdate(ema_pm10_, raw, Cfg::EMA_ALPHA_PM10)
                : emaWarmStart(ema_pm10_, raw);
            data.pm10 = static_cast<uint16_t>(
                std::clamp(lroundf(f), 0L, pm_max));
        } else {
            ESP_LOGW(TAG, "PM10 ngoài mức cho phép: %u ug/m3 [%.0f, %.0f]",
                     data.pm10, Cfg::SANITY_PM_MIN, Cfg::SANITY_PM_MAX);
            if (ema_pm10_.initialized) {
                data.pm10 = static_cast<uint16_t>(
                    std::clamp(lroundf(ema_pm10_.prev), 0L, pm_max));
            }
        }
    }
}

// ============================================================
// Static helpers
// ============================================================

float Filters::emaWarmStart(EmaState &s, float x) {
    s.prev        = x;
    s.initialized = true;
    return x;
}

float Filters::emaUpdate(EmaState &s, float x, float alpha) {
    s.prev = alpha * x + (1.0f - alpha) * s.prev;
    return s.prev;
}

// SMA với ring buffer — trung bình trên số mẫu hiện có (tránh kéo về 0 khi chưa đủ W)
float Filters::smaUpdate(SmaState &s, float x) {
    s.buf[s.write_idx] = x;
    s.write_idx = (s.write_idx + 1) % Cfg::FILTER_WINDOW_SIZE;
    if (s.count < Cfg::FILTER_WINDOW_SIZE) {
        ++s.count;
    }
    float sum = 0.0f;
    for (size_t i = 0; i < s.count; ++i) {
        sum += s.buf[i];
    }
    s.last_output = sum / static_cast<float>(s.count);
    return s.last_output;
}

bool Filters::inRange(float x, float lo, float hi) {
    return x >= lo && x <= hi;
}

// ============================================================
// Reset per sensor group (gọi khi phát hiện cờ false→true)
// ============================================================
void Filters::resetBme680State() {
    ema_temp_  = {0.0f, false};
    ema_press_ = {0.0f, false};
    sma_humi_.buf.fill(0.0f);
    sma_humi_.write_idx   = 0;
    sma_humi_.count       = 0;
    sma_humi_.last_output = 0.0f;
    ESP_LOGD(TAG, "BME680 filter state reset (bme680_ready false->true)");
}

void Filters::resetMq135State() {
    ema_co2_ = {0.0f, false};
    ESP_LOGD(TAG, "MQ-135 filter state reset (mq135_ready false->true)");
}

void Filters::resetPms5003State() {
    ema_pm1_  = {0.0f, false};
    ema_pm25_ = {0.0f, false};
    ema_pm10_ = {0.0f, false};
    ESP_LOGD(TAG, "PMS5003 filter state reset (pms5003_ready false->true)");
}

// ============================================================
// getFilterState() — truy vấn trạng thái hiện tại (debug/JTAG)
// ============================================================
bool Filters::getFilterState(float &temp, float &humi, float &press,
                              float &co2,  float &pm1,  float &pm25,  float &pm10) const {
    temp  = ema_temp_.initialized  ? ema_temp_.prev  : 0.0f;
    humi  = (sma_humi_.count > 0)  ? sma_humi_.last_output : 0.0f;
    press = ema_press_.initialized ? ema_press_.prev : 0.0f;
    co2   = ema_co2_.initialized   ? ema_co2_.prev   : 0.0f;
    pm1   = ema_pm1_.initialized   ? ema_pm1_.prev   : 0.0f;
    pm25  = ema_pm25_.initialized  ? ema_pm25_.prev  : 0.0f;
    pm10  = ema_pm10_.initialized  ? ema_pm10_.prev  : 0.0f;

    return ema_temp_.initialized  && (sma_humi_.count > 0) &&
           ema_press_.initialized &&
           ema_co2_.initialized   && ema_pm1_.initialized  &&
           ema_pm25_.initialized  && ema_pm10_.initialized;
}

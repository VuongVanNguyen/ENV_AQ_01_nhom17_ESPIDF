#include "DataFusion.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#include <cmath>
#include <ctime>
#include <iterator>

static const char *TAG = "DataFusion";

DataFusion::DataFusion()
    : baseline_{},
      last_calib_ts_(0),
      pending_calib_ts_(0),
      baseline_mask_(0),
      baseline_dirty_(false),
      last_alert_level_(AlertLevel::NONE),
      alert_level_changed_us_(0),
      last_category_(AqiCategory::GOOD),
      last_comfort_category_(ComfortCategory::COMFORTABLE),
      last_co2_category_(Co2Category::GOOD),
      nvs_handle_(0), mutex_(nullptr)
{
    setReason(kNoCalibReason);
}

DataFusion::~DataFusion() {
    if (nvs_handle_) {
        nvs_close(nvs_handle_);
    }
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t DataFusion::init() {
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "Lỗi khởi tạo mutex");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = nvs_open(Cfg::NVS_NAMESPACE, NVS_READWRITE, &nvs_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS mở thất bại: %s", esp_err_to_name(err));
        return err;
    }

    err = loadBaseline();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Tải baseline hiệu chuẩn từ NVS (mask=0x%02X)", baseline_mask_);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        baseline_mask_ = 0;
        err = ESP_OK;
        ESP_LOGI(TAG, "Không thể truy cập NVS/Không tìm thấy baseline");
    } else {
        ESP_LOGE(TAG, "Khởi tạo baseline thất bại: %s", esp_err_to_name(err));
    }

    return err;
}

void DataFusion::process(AirData &data, bool time_synced) {
    if (!data.data_valid) {
        ESP_LOGW(TAG, "Dữ liệu không hợp lệ, không kết hợp dữ liệu");
        if (last_alert_level_ != AlertLevel::NONE) {
            alert_level_changed_us_ = esp_timer_get_time();
        }
        last_alert_level_ = AlertLevel::NONE;
        setSafeSentinel(data);
        return;
    }

    if (data.pms5003_ready) {
        computeAqi(data);
    } else {
        data.aqi = NAN;
        data.aqi_category = static_cast<uint8_t>(AqiCategory::GOOD);
        last_category_ = AqiCategory::GOOD;
    }

    if (data.bme680_ready) {
        computeComfort(data);
    } else {
        data.comfort_index = NAN;
        data.comfort_category = static_cast<uint8_t>(ComfortCategory::COMFORTABLE);
        last_comfort_category_ = ComfortCategory::COMFORTABLE;
    }

    if (data.mq135_ready) {
        computeCo2Category(data);
    } else {
        data.co2_category = static_cast<uint8_t>(Co2Category::GOOD);
        last_co2_category_ = Co2Category::GOOD;
    }

    if (data.bme680_ready && !(baseline_mask_ & BASELINE_BME680_BIT)) {
        initializeBaselineGroup(BASELINE_BME680_BIT, data, time_synced);
    }
    if (data.pms5003_ready && !(baseline_mask_ & BASELINE_PMS5003_BIT)) {
        initializeBaselineGroup(BASELINE_PMS5003_BIT, data, time_synced);
    }
    if (data.mq135_ready && !(baseline_mask_ & BASELINE_MQ135_BIT)) {
        initializeBaselineGroup(BASELINE_MQ135_BIT, data, time_synced);
    }

    driftSelfCheck(data, time_synced);
    computeAlertLevel(data);

    data.last_calib_timestamp = last_calib_ts_;
}

esp_err_t DataFusion::confirmRecalibration(AirData &data, bool time_synced) {
    if (!data.data_valid) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!data.bme680_ready && !data.pms5003_ready && !data.mq135_ready) {
        ESP_LOGW(TAG, "Không có cảm biến nào sẵn sàng để xác nhận tái hiệu chuẩn");
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t now = getNow(time_synced);
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (data.bme680_ready) {
        baseline_.temperature = data.temperature;
        baseline_.humidity = data.humidity;
        baseline_.pressure = data.pressure;
        baseline_.comfort_index = data.comfort_index;
        baseline_mask_ |= BASELINE_BME680_BIT;
    }
    if (data.pms5003_ready) {
        baseline_.aqi = data.aqi;
        baseline_mask_ |= BASELINE_PMS5003_BIT;
    }
    if (data.mq135_ready) {
        baseline_.co2 = data.co2_ppm;
        baseline_mask_ |= BASELINE_MQ135_BIT;
    }
    setReason(kNoCalibReason);

    esp_err_t err = saveBaseline(now);
    if (err == ESP_OK) {
        baseline_dirty_ = false;
    }
    xSemaphoreGive(mutex_);

    if (err == ESP_OK) {
        data.calib_needed = false;
        data.last_calib_timestamp = now;
        computeAlertLevel(data);
        ESP_LOGI(TAG, "Hiệu chuẩn xác nhận (mask=0x%02X) tại %lld", baseline_mask_, (long long)now);
    }
    return err;
}

bool DataFusion::hasBaseline() const {
    return baseline_mask_ != 0;
}

void DataFusion::setSafeSentinel(AirData &data) const {
    data.aqi = NAN;
    data.aqi_category = static_cast<uint8_t>(AqiCategory::GOOD);
    data.comfort_index = NAN;
    data.comfort_category = static_cast<uint8_t>(ComfortCategory::COMFORTABLE);
    data.co2_category = static_cast<uint8_t>(Co2Category::GOOD);
    data.calib_needed = false;
    data.last_calib_timestamp = last_calib_ts_;
    data.alert_level = static_cast<uint8_t>(AlertLevel::NONE);
    data.alert_flags = 0;
    data.alert_level_changed_us = alert_level_changed_us_;
    std::strncpy(data.alert_reason, kNoAlertReason, sizeof(data.alert_reason) - 1);
    data.alert_reason[sizeof(data.alert_reason) - 1] = '\0';
    std::strncpy(data.calib_reason, kNoCalibReason, sizeof(data.calib_reason) - 1);
    data.calib_reason[sizeof(data.calib_reason) - 1] = '\0';
}

void DataFusion::setReason(const char *reason) {
    std::strncpy(last_calib_reason_, reason, sizeof(last_calib_reason_) - 1);
    last_calib_reason_[sizeof(last_calib_reason_) - 1] = '\0';
}

int64_t DataFusion::getNow(bool time_synced) const {
    if (time_synced) {
        return static_cast<int64_t>(time(nullptr));
    }
    return static_cast<int64_t>(esp_timer_get_time() / 1000000LL);
}

esp_err_t DataFusion::loadBaseline() {
    if (!nvs_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    float temp = 0.0f;
    float humi = 0.0f;
    float co2 = 0.0f;
    float press = 0.0f;
    float aqi = 0.0f;
    float comfort = 0.0f;
    int64_t ts = 0;
    size_t sz = sizeof(float);

    esp_err_t err = nvs_get_blob(nvs_handle_, Cfg::NVS_KEY_BL_TEMP, &temp, &sz);
    if (err != ESP_OK || sz != sizeof(float)) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    sz = sizeof(float);
    err = nvs_get_blob(nvs_handle_, Cfg::NVS_KEY_BL_HUMI, &humi, &sz);
    if (err != ESP_OK || sz != sizeof(float)) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    sz = sizeof(float);
    err = nvs_get_blob(nvs_handle_, Cfg::NVS_KEY_BL_CO2, &co2, &sz);
    if (err != ESP_OK || sz != sizeof(float)) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    sz = sizeof(float);
    err = nvs_get_blob(nvs_handle_, Cfg::NVS_KEY_BL_PRESSURE, &press, &sz);
    if (err != ESP_OK || sz != sizeof(float)) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    sz = sizeof(float);
    err = nvs_get_blob(nvs_handle_, Cfg::NVS_KEY_BL_AQI, &aqi, &sz);
    if (err != ESP_OK || sz != sizeof(float)) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    sz = sizeof(float);
    err = nvs_get_blob(nvs_handle_, Cfg::NVS_KEY_BL_COMFORT, &comfort, &sz);
    if (err != ESP_OK || sz != sizeof(float)) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    err = nvs_get_i64(nvs_handle_, Cfg::NVS_KEY_LAST_CALIB_TS, &ts);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t mask = 0;
    err = nvs_get_u8(nvs_handle_, Cfg::NVS_KEY_BL_MASK, &mask);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        mask = 0b111;
    } else if (err != ESP_OK) {
        return err;
    }

    baseline_.temperature = temp;
    baseline_.humidity = humi;
    baseline_.co2 = co2;
    baseline_.pressure = press;
    baseline_.aqi = aqi;
    baseline_.comfort_index = comfort;
    last_calib_ts_ = ts;
    baseline_mask_ = mask;
    return ESP_OK;
}

esp_err_t DataFusion::saveBaseline(int64_t timestamp) {
    esp_err_t err = writeBaselineToNvs(baseline_, baseline_mask_, timestamp);
    if (err == ESP_OK) {
        last_calib_ts_ = timestamp;
    }
    return err;
}

esp_err_t DataFusion::writeBaselineToNvs(const Baseline &bl, uint8_t mask, int64_t timestamp) {
    if (!nvs_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_TEMP, &bl.temperature, sizeof(bl.temperature));
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_HUMI, &bl.humidity, sizeof(bl.humidity));
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_CO2, &bl.co2, sizeof(bl.co2));
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_PRESSURE, &bl.pressure, sizeof(bl.pressure));
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_AQI, &bl.aqi, sizeof(bl.aqi));
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_COMFORT, &bl.comfort_index, sizeof(bl.comfort_index));
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_i64(nvs_handle_, Cfg::NVS_KEY_LAST_CALIB_TS, timestamp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Không thể lưu timestamp hiệu chuẩn: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u8(nvs_handle_, Cfg::NVS_KEY_BL_MASK, mask);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Không thể lưu baseline_mask: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_commit(nvs_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Không thể commit baseline hiệu chuẩn: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t DataFusion::persistBaselineIfDirty() {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (!baseline_dirty_) {
        xSemaphoreGive(mutex_);
        return ESP_OK;
    }

    const uint8_t mask = baseline_mask_;
    const int64_t ts   = pending_calib_ts_;
    esp_err_t err = writeBaselineToNvs(baseline_, mask, ts);
    if (err == ESP_OK) {
        baseline_dirty_ = false;
        ESP_LOGI(TAG, "Đã flush baseline xuống NVS (mask=0x%02X) tại %lld",
                 mask, (long long)ts);
    } else {
        ESP_LOGW(TAG, "Ghi baseline NVS thất bại (%s) — giữ dirty, thử lại lượt sau",
                 esp_err_to_name(err));
    }
    xSemaphoreGive(mutex_);
    return err;
}

void DataFusion::initializeBaselineGroup(uint8_t group_bit, const AirData &data, bool time_synced) {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Không thể lấy mutex để khởi tạo baseline nhóm 0x%02X", group_bit);
        return;
    }

    const char *group_name = "?";
    switch (group_bit) {
        case BASELINE_BME680_BIT:
            baseline_.temperature = data.temperature;
            baseline_.humidity = data.humidity;
            baseline_.pressure = data.pressure;
            baseline_.comfort_index = data.comfort_index;
            group_name = "BME680";
            break;
        case BASELINE_PMS5003_BIT:
            baseline_.aqi = data.aqi;
            group_name = "PMS5003";
            break;
        case BASELINE_MQ135_BIT:
            baseline_.co2 = data.co2_ppm;
            group_name = "MQ135";
            break;
        default:
            break;
    }

    const int64_t ts = (baseline_mask_ == 0) ? getNow(time_synced) : last_calib_ts_;
    baseline_mask_ |= group_bit;
    setReason(kNoCalibReason);

    last_calib_ts_    = ts;
    pending_calib_ts_ = ts;
    baseline_dirty_   = true;
    ESP_LOGI(TAG, "Baseline nhóm %s chốt vào RAM (mask=0x%02X) tại %lld — chờ flush NVS",
             group_name, baseline_mask_, (long long)ts);

    xSemaphoreGive(mutex_);
}

void DataFusion::computeAqi(AirData &data) {
    const float aqi_pm25 = computeAqiSubindex(data.pm2_5, Cfg::AQI_PM25_BP);
    const float aqi_pm10 = computeAqiSubindex(data.pm10, Cfg::AQI_PM10_BP);
    data.aqi = fmaxf(aqi_pm25, aqi_pm10);

    if (!hasFiniteValue(data.aqi)) {
        data.aqi_category = static_cast<uint8_t>(AqiCategory::GOOD);
        last_category_ = AqiCategory::GOOD;
        return;
    }

    if (data.aqi <= Cfg::AQI_GOOD_MAX) {
        last_category_ = AqiCategory::GOOD;
    } else if (data.aqi <= Cfg::AQI_MODERATE_MAX) {
        last_category_ = AqiCategory::MODERATE;
    } else if (data.aqi <= Cfg::AQI_POOR_MAX) {
        last_category_ = AqiCategory::POOR;
    } else if (data.aqi <= Cfg::AQI_BAD_MAX) {
        last_category_ = AqiCategory::BAD;
    } else if (data.aqi <= Cfg::AQI_VERY_BAD_MAX) {
        last_category_ = AqiCategory::VERY_BAD;
    } else {
        last_category_ = AqiCategory::HAZARDOUS;
    }

    data.aqi_category = static_cast<uint8_t>(last_category_);
}

void DataFusion::computeComfort(AirData &data) {
    if (!hasFiniteValue(data.temperature) || !hasFiniteValue(data.humidity)) {
        data.comfort_index = NAN;
        data.comfort_category = static_cast<uint8_t>(ComfortCategory::COMFORTABLE);
        last_comfort_category_ = ComfortCategory::COMFORTABLE;
        return;
    }

    const float t = data.temperature;
    const float rh = data.humidity;
    data.comfort_index = t - Cfg::COMFORT_DI_K1 * (1.0f - Cfg::COMFORT_DI_RH_SCALE * rh) * (t - Cfg::COMFORT_DI_K2);

    const float di = data.comfort_index;
    if (di < Cfg::COMFORT_DI_OK) {
        last_comfort_category_ = ComfortCategory::COMFORTABLE;
    } else if (di < Cfg::COMFORT_DI_WARM) {
        last_comfort_category_ = ComfortCategory::SLIGHTLY_HOT;
    } else if (di < Cfg::COMFORT_DI_HOT) {
        last_comfort_category_ = ComfortCategory::HOT;
    } else if (di < Cfg::COMFORT_DI_SEVERE) {
        last_comfort_category_ = ComfortCategory::VERY_HOT;
    } else if (di < Cfg::COMFORT_DI_DANGER) {
        last_comfort_category_ = ComfortCategory::HEAT_STRESS;
    } else {
        last_comfort_category_ = ComfortCategory::DANGER;
    }

    data.comfort_category = static_cast<uint8_t>(last_comfort_category_);
}

void DataFusion::computeCo2Category(AirData &data) {
    if (!hasFiniteValue(data.co2_ppm) || data.co2_ppm < 0.0f) {
        data.co2_category = static_cast<uint8_t>(Co2Category::GOOD);
        last_co2_category_ = Co2Category::GOOD;
        return;
    }

    if (data.co2_ppm <= Cfg::CO2_GOOD_MAX) {
        last_co2_category_ = Co2Category::GOOD;
    } else if (data.co2_ppm <= Cfg::CO2_MODERATE_MAX) {
        last_co2_category_ = Co2Category::MODERATE;
    } else {
        last_co2_category_ = Co2Category::BAD;
    }

    data.co2_category = static_cast<uint8_t>(last_co2_category_);
}

void DataFusion::driftSelfCheck(AirData &data, bool time_synced) {
    data.calib_needed = false;
    setReason(kNoCalibReason);

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "driftSelfCheck: không lấy được mutex_, bỏ qua chu kỳ này");
        return;
    }
    const Baseline local_baseline   = baseline_;
    const uint8_t  local_mask       = baseline_mask_;
    const int64_t  local_last_calib = last_calib_ts_;
    xSemaphoreGive(mutex_);

    if (local_mask == 0) {
        return;
    }

    bool drift = false;
    const float threshold = Cfg::DRIFT_THRESHOLD_PCT / 100.0f;
    const bool bme680_base  = (local_mask & BASELINE_BME680_BIT)  != 0;
    const bool pms5003_base = (local_mask & BASELINE_PMS5003_BIT) != 0;
    const bool mq135_base   = (local_mask & BASELINE_MQ135_BIT)   != 0;

    if (data.bme680_ready && bme680_base && hasFiniteValue(data.temperature)) {
        const float dev_abs = fabsf(data.temperature - local_baseline.temperature);
        if (dev_abs > Cfg::DRIFT_TEMP_ABS_C) {
            drift = true;
            setReason("CALIB_DRIFT_TEMP");
        }
    }

    if (!drift && data.bme680_ready && bme680_base && hasFiniteValue(data.humidity)) {
        const float dev_abs = fabsf(data.humidity - local_baseline.humidity);
        if (dev_abs > Cfg::DRIFT_HUMI_ABS_PCT) {
            drift = true;
            setReason("CALIB_DRIFT_HUMI");
        }
    }

    if (!drift && data.mq135_ready && mq135_base && hasFiniteValue(data.co2_ppm) && local_baseline.co2 != 0.0f) {
        const float dev = fabsf(data.co2_ppm - local_baseline.co2) / fabsf(local_baseline.co2);
        if (dev > threshold) {
            drift = true;
            setReason("CALIB_DRIFT_CO2");
        }
    }

    const float index_threshold = Cfg::ACCURACY_INDEX_PCT / 100.0f;

    if (!drift && data.pms5003_ready && pms5003_base && hasFiniteValue(data.aqi) && local_baseline.aqi != 0.0f) {
        const float dev = fabsf(data.aqi - local_baseline.aqi) / fabsf(local_baseline.aqi);
        if (dev > index_threshold) {
            drift = true;
            setReason("CALIB_DRIFT_AQI");
        }
    }

    if (!drift && data.bme680_ready && bme680_base && hasFiniteValue(data.comfort_index) && local_baseline.comfort_index != 0.0f) {
        const float dev = fabsf(data.comfort_index - local_baseline.comfort_index) / fabsf(local_baseline.comfort_index);
        if (dev > index_threshold) {
            drift = true;
            setReason("CALIB_DRIFT_COMFORT");
        }
    }

    const int64_t now = getNow(time_synced);
    if (!drift && local_last_calib > 0 && (now - local_last_calib) > Cfg::CALIB_INTERVAL_SEC) {
        drift = true;
        setReason("CALIB_OVERDUE_30D");
    }

    if (drift) {
        data.calib_needed = true;
        ESP_LOGW(TAG, "Cần hiệu chuẩn: %s", last_calib_reason_);
    }
}

void DataFusion::computeAlertLevel(AirData &data) {

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "computeAlertLevel: không lấy được mutex_, bỏ qua chu kỳ này");
        return;
    }

    AlertLevel level = AlertLevel::NONE;
    const char *reason = kNoAlertReason;
    uint16_t flags = 0;

    auto consider = [&](AlertLevel candidate, const char *candidate_reason) {
        if (candidate > level) {
            level = candidate;
            reason = candidate_reason;
        }
    };

    if (data.data_valid) {
        if (data.aqi_category >= static_cast<uint8_t>(AqiCategory::VERY_BAD)) {
            consider(AlertLevel::CRITICAL, "AQI_HAZARDOUS");
            flags |= FLAG_AQI_HAZARDOUS;
        } else if (data.aqi_category >= static_cast<uint8_t>(AqiCategory::BAD)) {
            consider(AlertLevel::WARNING, "AQI_BAD");
            flags |= FLAG_AQI_BAD;
        }

        if (data.comfort_category >= static_cast<uint8_t>(ComfortCategory::DANGER)) {
            consider(AlertLevel::CRITICAL, "COMFORT_DANGER");
            flags |= FLAG_COMFORT_DANGER;
        } else if (data.comfort_category >= static_cast<uint8_t>(ComfortCategory::VERY_HOT)) {
            consider(AlertLevel::WARNING, "COMFORT_VERY_HOT");
            flags |= FLAG_COMFORT_VERY_HOT;
        }

        if (data.mq135_ready && data.co2_ppm > Cfg::ALERT_CO2_CRITICAL_PPM) {
            consider(AlertLevel::CRITICAL, "CO2_CRITICAL");
            flags |= FLAG_CO2_CRITICAL;
        } else if (data.mq135_ready && data.co2_ppm > Cfg::ALERT_CO2_PPM) {
            consider(AlertLevel::WARNING, "CO2_WARNING");
            flags |= FLAG_CO2_WARNING;
        }

        if (data.calib_needed) {
            consider(AlertLevel::WARNING, last_calib_reason_);
            flags |= FLAG_CALIB_NEEDED;
        }
    }

    if (level != last_alert_level_) {
        alert_level_changed_us_ = esp_timer_get_time();
    }
    data.alert_level_changed_us = alert_level_changed_us_;

    last_alert_level_ = level;
    data.alert_level = static_cast<uint8_t>(level);
    data.alert_flags = flags;
    std::strncpy(data.alert_reason, reason, sizeof(data.alert_reason) - 1);
    data.alert_reason[sizeof(data.alert_reason) - 1] = '\0';

    const char *calib_reason = data.calib_needed ? last_calib_reason_ : kNoCalibReason;
    std::strncpy(data.calib_reason, calib_reason, sizeof(data.calib_reason) - 1);
    data.calib_reason[sizeof(data.calib_reason) - 1] = '\0';

    xSemaphoreGive(mutex_);
}

float DataFusion::computeAqiSubindex(float concentration, const float *breakpoints) const {
    if (!hasFiniteValue(concentration) || concentration < 0.0f) {
        return NAN;
    }

    constexpr size_t kCount = std::size(Cfg::AQI_INDEX_BP);
    if (concentration <= breakpoints[0]) {
        return Cfg::AQI_INDEX_BP[0];
    }

    for (size_t i = 1; i < kCount; ++i) {
        if (concentration <= breakpoints[i]) {
            const float bp_lo = breakpoints[i - 1];
            const float bp_hi = breakpoints[i];
            const float i_lo = Cfg::AQI_INDEX_BP[i - 1];
            const float i_hi = Cfg::AQI_INDEX_BP[i];
            return ((i_hi - i_lo) / (bp_hi - bp_lo)) * (concentration - bp_lo) + i_lo;
        }
    }

    return Cfg::AQI_MAX_INDEX;
}

bool DataFusion::hasFiniteValue(float value) const {
    return std::isfinite(value);
}

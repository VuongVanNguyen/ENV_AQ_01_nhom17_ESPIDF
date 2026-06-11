#include "DataFusion.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#include <cmath>
#include <ctime>
#include <iterator>

static const char *TAG = "DataFusion";

DataFusion::DataFusion()
    : last_calib_ts_(0), has_baseline_(false), last_alert_level_(AlertLevel::NONE),
      last_category_(AqiCategory::GOOD), nvs_handle_(0), mutex_(nullptr) {
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
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = nvs_open(Cfg::NVS_NAMESPACE, NVS_READWRITE, &nvs_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = loadBaseline();
    if (err == ESP_OK) {
        has_baseline_ = true;
        ESP_LOGI(TAG, "Loaded calibration baseline from NVS");
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        has_baseline_ = false;
        err = ESP_OK;
        ESP_LOGI(TAG, "No calibration baseline found in NVS; waiting for stable sample");
    } else {
        ESP_LOGE(TAG, "Failed to load baseline: %s", esp_err_to_name(err));
    }

    return err;
}

void DataFusion::process(AirData &data, bool time_synced) {
    if (!data.data_valid) {
        ESP_LOGW(TAG, "Data invalid, skipping fusion");
        last_alert_level_ = AlertLevel::NONE;
        setSafeSentinel(data);
        return;
    }

    if (!has_baseline_ && data.sensors_ready) {
        initializeBaselineFromData(data, time_synced);
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
    }

    driftSelfCheck(data, time_synced);
    computeAlertLevel(data);

    data.last_calib_timestamp = last_calib_ts_;
}

esp_err_t DataFusion::confirmRecalibration(AirData &data, bool time_synced) {
    if (!data.data_valid) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!data.sensors_ready) {
        ESP_LOGW(TAG, "Cannot confirm recalibration before all sensors are ready");
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t now = getNow(time_synced);
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    baseline_.temperature = data.temperature;
    baseline_.humidity = data.humidity;
    baseline_.pm25 = static_cast<float>(data.pm2_5);
    baseline_.co2 = data.co2_ppm;
    baseline_.gas_resistance = data.gas_resistance;
    last_calib_ts_ = now;
    has_baseline_ = true;
    setReason(kNoCalibReason);

    esp_err_t err = saveBaseline(now);
    xSemaphoreGive(mutex_);

    if (err == ESP_OK) {
        data.calib_needed = false;
        data.last_calib_timestamp = now;
        ESP_LOGI(TAG, "Calibration confirmed and baseline saved at %lld", (long long)now);
    }
    return err;
}

DataFusion::AlertLevel DataFusion::getAlertLevel() const {
    return last_alert_level_;
}

DataFusion::AqiCategory DataFusion::lastCategory() const {
    return last_category_;
}

bool DataFusion::hasBaseline() const {
    return has_baseline_;
}

int64_t DataFusion::lastCalibrationTimestamp() const {
    return last_calib_ts_;
}

const char *DataFusion::calibReason() const {
    return last_calib_reason_[0] ? last_calib_reason_ : kNoCalibReason;
}

void DataFusion::setSafeSentinel(AirData &data) const {
    data.aqi = NAN;
    data.aqi_category = static_cast<uint8_t>(AqiCategory::GOOD);
    data.comfort_index = NAN;
    data.calib_needed = false;
    data.last_calib_timestamp = last_calib_ts_;
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
    float pm25 = 0.0f;
    float co2 = 0.0f;
    float gas = 0.0f;
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
    err = nvs_get_blob(nvs_handle_, Cfg::NVS_KEY_BL_PM25, &pm25, &sz);
    if (err != ESP_OK || sz != sizeof(float)) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    sz = sizeof(float);
    err = nvs_get_blob(nvs_handle_, Cfg::NVS_KEY_BL_CO2, &co2, &sz);
    if (err != ESP_OK || sz != sizeof(float)) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    sz = sizeof(float);
    err = nvs_get_blob(nvs_handle_, Cfg::NVS_KEY_BL_GAS, &gas, &sz);
    if (err != ESP_OK || sz != sizeof(float)) {
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }

    err = nvs_get_i64(nvs_handle_, Cfg::NVS_KEY_LAST_CALIB_TS, &ts);
    if (err != ESP_OK) {
        return err;
    }

    baseline_.temperature = temp;
    baseline_.humidity = humi;
    baseline_.pm25 = pm25;
    baseline_.co2 = co2;
    baseline_.gas_resistance = gas;
    last_calib_ts_ = ts;
    return ESP_OK;
}

esp_err_t DataFusion::saveBaseline(int64_t timestamp) {
    if (!nvs_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_TEMP, &baseline_.temperature, sizeof(baseline_.temperature));
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_HUMI, &baseline_.humidity, sizeof(baseline_.humidity));
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_PM25, &baseline_.pm25, sizeof(baseline_.pm25));
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_CO2, &baseline_.co2, sizeof(baseline_.co2));
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(nvs_handle_, Cfg::NVS_KEY_BL_GAS, &baseline_.gas_resistance, sizeof(baseline_.gas_resistance));
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_i64(nvs_handle_, Cfg::NVS_KEY_LAST_CALIB_TS, timestamp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write calibration timestamp: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_commit(nvs_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit calibration baseline: %s", esp_err_to_name(err));
        return err;
    }

    last_calib_ts_ = timestamp;
    has_baseline_ = true;
    return ESP_OK;
}

void DataFusion::initializeBaselineFromData(const AirData &data, bool time_synced) {
    if (!data.sensors_ready) {
        return;
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Unable to lock mutex to initialize baseline");
        return;
    }

    baseline_.temperature = data.temperature;
    baseline_.humidity = data.humidity;
    baseline_.pm25 = static_cast<float>(data.pm2_5);
    baseline_.co2 = data.co2_ppm;
    baseline_.gas_resistance = data.gas_resistance;
    last_calib_ts_ = getNow(time_synced);
    has_baseline_ = true;
    setReason(kNoCalibReason);

    const esp_err_t err = saveBaseline(last_calib_ts_);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "First calibration baseline saved at %lld", (long long)last_calib_ts_);
    } else {
        ESP_LOGW(TAG, "Failed to save first calibration baseline: %s", esp_err_to_name(err));
    }

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

    if (data.aqi <= Cfg::AQI_CAT_BP[0]) {
        last_category_ = AqiCategory::GOOD;
    } else if (data.aqi <= Cfg::AQI_CAT_BP[1]) {
        last_category_ = AqiCategory::MODERATE;
    } else if (data.aqi <= Cfg::AQI_CAT_BP[2]) {
        last_category_ = AqiCategory::POOR;
    } else if (data.aqi <= Cfg::AQI_CAT_BP[3]) {
        last_category_ = AqiCategory::BAD;
    } else if (data.aqi <= Cfg::AQI_CAT_BP[4]) {
        last_category_ = AqiCategory::VERY_BAD;
    } else {
        last_category_ = AqiCategory::HAZARDOUS;
    }

    data.aqi_category = static_cast<uint8_t>(last_category_);
}

void DataFusion::computeComfort(AirData &data) const {
    if (!hasFiniteValue(data.temperature) || !hasFiniteValue(data.humidity)) {
        data.comfort_index = NAN;
        return;
    }

    const float t = data.temperature;
    const float rh = data.humidity;
    data.comfort_index = t - Cfg::COMFORT_DI_K1 * (1.0f - Cfg::COMFORT_DI_RH_SCALE * rh) * (t - Cfg::COMFORT_DI_K2);
}

void DataFusion::driftSelfCheck(AirData &data, bool time_synced) {
    data.calib_needed = false;
    setReason(kNoCalibReason);

    if (!has_baseline_) {
        return;
    }

    bool drift = false;
    const float threshold = Cfg::DRIFT_THRESHOLD_PCT / 100.0f;

    if (data.bme680_ready && hasFiniteValue(data.temperature)) {
        const float dev_abs = fabsf(data.temperature - baseline_.temperature);
        if (dev_abs > Cfg::DRIFT_TEMP_ABS_C) {
            drift = true;
            setReason("CALIB_DRIFT_TEMP");
        }
    }

    if (!drift && data.bme680_ready && hasFiniteValue(data.humidity) && baseline_.humidity != 0.0f) {
        const float dev = fabsf(data.humidity - baseline_.humidity) / fabsf(baseline_.humidity);
        if (dev > threshold) {
            drift = true;
            setReason("CALIB_DRIFT_HUMI");
        }
    }

    if (!drift && data.pms5003_ready && hasFiniteValue(data.pm2_5) && baseline_.pm25 != 0.0f) {
        const float dev = fabsf(static_cast<float>(data.pm2_5) - baseline_.pm25) / fabsf(baseline_.pm25);
        if (dev > threshold) {
            drift = true;
            setReason("CALIB_DRIFT_PM25");
        }
    }

    if (!drift && data.mq135_ready && hasFiniteValue(data.co2_ppm) && baseline_.co2 != 0.0f) {
        const float dev = fabsf(data.co2_ppm - baseline_.co2) / fabsf(baseline_.co2);
        if (dev > threshold) {
            drift = true;
            setReason("CALIB_DRIFT_CO2");
        }
    }

    const int64_t now = getNow(time_synced);
    if (!drift && last_calib_ts_ > 0 && (now - last_calib_ts_) > Cfg::CALIB_INTERVAL_SEC) {
        drift = true;
        setReason("CALIB_OVERDUE_30D");
    }

    if (drift) {
        data.calib_needed = true;
        ESP_LOGW(TAG, "Calibration needed: %s", last_calib_reason_);
    }
}

void DataFusion::computeAlertLevel(AirData &data) {
    last_alert_level_ = AlertLevel::NONE;

    if (!data.data_valid) {
        return;
    }

    if (data.sensors_ready && data.aqi_category >= static_cast<uint8_t>(AqiCategory::VERY_BAD)) {
        last_alert_level_ = AlertLevel::CRITICAL;
    }

    if (data.mq135_ready && data.co2_ppm > Cfg::ALERT_CO2_PPM) {
        if (last_alert_level_ != AlertLevel::CRITICAL) {
            last_alert_level_ = AlertLevel::WARNING;
        }
    }

    if (data.pms5003_ready && static_cast<float>(data.pm2_5) > Cfg::ALERT_PM25_UGM3) {
        if (last_alert_level_ != AlertLevel::CRITICAL) {
            last_alert_level_ = AlertLevel::WARNING;
        }
    }

    if (data.calib_needed) {
        if (last_alert_level_ == AlertLevel::NONE) {
            last_alert_level_ = AlertLevel::WARNING;
        }
    }
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

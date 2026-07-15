#pragma once

#include "DataStructures.hpp"
#include "config.hpp"

#include "esp_err.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <cstdint>
#include <cstddef>
#include <cstring>

class DataFusion {
public:
    enum class AqiCategory : uint8_t {
        GOOD       = 0,
        MODERATE   = 1,
        POOR       = 2,
        BAD        = 3,
        VERY_BAD   = 4,
        HAZARDOUS  = 5,
    };

    enum class ComfortCategory : uint8_t {
        COMFORTABLE  = 0,
        SLIGHTLY_HOT = 1,
        HOT          = 2,
        VERY_HOT     = 3,
        HEAT_STRESS  = 4,
        DANGER       = 5,
    };

    enum class Co2Category : uint8_t {
        GOOD     = 0,
        MODERATE = 1,
        BAD      = 2,
    };

    enum class AlertLevel : uint8_t {
        NONE     = 0,
        WARNING  = 1,
        CRITICAL = 2,
    };

    static constexpr uint16_t FLAG_AQI_BAD          = 1u << 0;
    static constexpr uint16_t FLAG_AQI_HAZARDOUS    = 1u << 1;
    static constexpr uint16_t FLAG_COMFORT_VERY_HOT = 1u << 2;
    static constexpr uint16_t FLAG_COMFORT_DANGER   = 1u << 3;
    static constexpr uint16_t FLAG_CO2_WARNING      = 1u << 4;
    static constexpr uint16_t FLAG_CO2_CRITICAL     = 1u << 5;
    static constexpr uint16_t FLAG_CALIB_NEEDED     = 1u << 6;

    DataFusion();
    ~DataFusion();

    DataFusion(const DataFusion&)            = delete;
    DataFusion& operator=(const DataFusion&) = delete;

    esp_err_t init();
    void process(AirData &data, bool time_synced = false);
    esp_err_t confirmRecalibration(AirData &data, bool time_synced = false);

    esp_err_t persistBaselineIfDirty();

    bool hasBaseline() const;

private:
    struct Baseline {
        float temperature;
        float humidity;
        float co2;
        float pressure;
        float aqi;
        float comfort_index;
    } baseline_;

    int64_t last_calib_ts_;

    int64_t pending_calib_ts_;

    uint8_t baseline_mask_;

    bool baseline_dirty_;
    static constexpr uint8_t BASELINE_BME680_BIT  = 1u << 0;
    static constexpr uint8_t BASELINE_PMS5003_BIT = 1u << 1;
    static constexpr uint8_t BASELINE_MQ135_BIT   = 1u << 2;

    AlertLevel last_alert_level_;

    int64_t alert_level_changed_us_;

    AqiCategory last_category_;
    ComfortCategory last_comfort_category_;
    Co2Category last_co2_category_;
    char last_calib_reason_[32];

    nvs_handle_t nvs_handle_;
    mutable SemaphoreHandle_t mutex_;

    static constexpr const char *kNoCalibReason = "NONE";
    static constexpr const char *kNoAlertReason = "NONE";

    void setSafeSentinel(AirData &data) const;
    void setReason(const char *reason);
    int64_t getNow(bool time_synced) const;

    esp_err_t loadBaseline();
    esp_err_t saveBaseline(int64_t timestamp);

    esp_err_t writeBaselineToNvs(const Baseline &bl, uint8_t mask, int64_t timestamp);

    void initializeBaselineGroup(uint8_t group_bit, const AirData &data, bool time_synced);
    void computeAqi(AirData &data);
    void computeComfort(AirData &data);
    void computeCo2Category(AirData &data);
    void driftSelfCheck(AirData &data, bool time_synced);
    void computeAlertLevel(AirData &data);
    float computeAqiSubindex(float concentration, const float *breakpoints) const;
    bool hasFiniteValue(float value) const;
};

#pragma once

#include <cstdint>

struct AirData {

    float temperature;
    float humidity;
    float pressure;
    float gas_resistance;

    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10;

    float co2_ppm;

    float aqi;
    uint8_t aqi_category;
    float comfort_index;
    uint8_t comfort_category;
    uint8_t co2_category;

    uint8_t alert_level;
    char alert_reason[24];

    int64_t alert_level_changed_us;

    uint32_t alert_latency_ms;

    uint16_t alert_flags;

    bool calib_needed;
    char calib_reason[24];
    int64_t last_calib_timestamp;

    bool bme680_ready;
    bool pms5003_ready;
    bool mq135_ready;

    int64_t timestamp;
    bool data_valid;
    uint16_t cycle_time_ms;

};

#pragma once

#include "DataStructures.hpp"
#include "config.hpp"

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#include "bme680.h"

class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    SensorManager(const SensorManager&)            = delete;
    SensorManager& operator=(const SensorManager&) = delete;

    esp_err_t init();

    esp_err_t readAll(AirData &data);

    bool isFullyReady() const;

private:

    bme680_t  bme680_dev_;
    bool      bme680_inited_;

    bool      uart_installed_;
    int       pms_valid_streak_;

    adc_oneshot_unit_handle_t adc_handle_;
    adc_cali_handle_t         adc_cali_;
    bool                      adc_cali_enabled_;

    int64_t   boot_time_us_;

    esp_err_t bme680Setup();
    esp_err_t bme680ReadOnce(float &t_c, float &rh, float &p_hpa);

    esp_err_t pmsInit();
    esp_err_t pmsReadFrame(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10);
    static esp_err_t pmsReadOneFrame(uart_port_t port, uint8_t *buf, TickType_t scan_ticks);

    esp_err_t mq135Init();

    esp_err_t mq135ReadPpm(float &co2_ppm, float t_c, float rh_pct);

    static float mq135CorrectionFactor(float t_c, float rh_pct);

    uint32_t  elapsedMs() const;
};

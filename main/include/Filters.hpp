#pragma once

#include "DataStructures.hpp"
#include "config.hpp"

#include <array>
#include <cstddef>

class Filters {
public:
    Filters();
    ~Filters() = default;

    void process(AirData &data);

    void reset();

    bool getFilterState(float &temp, float &humi, float &press,
                        float &co2,  float &pm1,  float &pm25,  float &pm10) const;

private:

    struct EmaState {
        float prev;
        bool  initialized;
    };

    struct SmaState {
        std::array<float, Cfg::FILTER_WINDOW_SIZE> buf;
        size_t write_idx;
        size_t count;
        float  last_output;
    };

    EmaState ema_temp_;
    SmaState sma_humi_;
    EmaState ema_press_;
    EmaState ema_co2_;
    EmaState ema_pm1_;
    EmaState ema_pm25_;
    EmaState ema_pm10_;

    bool prev_bme680_ready_;
    bool prev_mq135_ready_;
    bool prev_pms5003_ready_;

    static float emaWarmStart(EmaState &s, float x);

    static float emaUpdate(EmaState &s, float x, float alpha);

    static float smaUpdate(SmaState &s, float x);

    static bool inRange(float x, float lo, float hi);

    void resetBme680State();
    void resetMq135State();
    void resetPms5003State();

    void processBme680(AirData &data);
    void processMq135(AirData &data);
    void processPms5003(AirData &data);
};

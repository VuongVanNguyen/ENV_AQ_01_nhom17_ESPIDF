#pragma once

#include "DataStructures.hpp"
#include "config.hpp"
#include "esp_err.h"
#include <hd44780.h>
#include <pcf8574.h>

class DisplayManager {
public:

    enum class ScreenPage : uint8_t {
        MAIN_AQI_CI = 0,
        MAIN_CO2_PM = 1,
        DETAIL = 2
    };

    enum class DisplayState : uint8_t {
        WARMING_UP = 0,
        NORMAL = 2
    };

    DisplayManager();
    ~DisplayManager() = default;
    DisplayManager(const DisplayManager&) = delete;
    DisplayManager& operator=(const DisplayManager&) = delete;

    esp_err_t init();

    void update(const AirData &data);

    void tick();

    void showMessage(const char *line1, const char *line2 = nullptr);

    void setBacklight(bool on);

    ScreenPage currentPage() const { return current_page_; }
    DisplayState currentState() const { return current_state_; }
    bool isInitialized() const { return initialized_; }

private:
    i2c_dev_t pcf_;
    hd44780_t lcd_;

    bool initialized_;
    ScreenPage current_page_;
    DisplayState current_state_;

    char shadow_[2][17];
    char current_display_[2][17];

    bool calib_alert_active_;

    bool overlay_active_;
    int64_t overlay_expire_us_;

    static esp_err_t write_cb(const hd44780_t *lcd, uint8_t data);

    void evaluateState(const AirData &data);
    void renderToShadow(const AirData &data);
    void commitDirtyCheck();

    int calibAlertRow(const char *reason) const;
};

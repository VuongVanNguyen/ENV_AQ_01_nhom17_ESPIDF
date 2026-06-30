
#include "DisplayManager.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DisplayManager";

// Con trỏ tĩnh phục vụ cho write_cb (vì callback của hd44780 là C-style function pointer)
static i2c_dev_t* s_pcf_ptr = nullptr;

// Ký tự tự định nghĩa (CGRAM)
static const uint8_t char_degree[] = { 0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00 }; // Ký tự '°'

// ============================================================================
// Constructor
// ============================================================================
DisplayManager::DisplayManager()
    : pcf_{},
      lcd_{},
      initialized_(false),
      current_page_(ScreenPage::MAIN_AQI_CI),
      current_state_(DisplayState::WARMING_UP),
      shadow_{},
      current_display_{},
      calib_alert_active_(false),
      overlay_active_(false),
      overlay_expire_us_(0) {}

// ============================================================================
// HÀM CALLBACK PHẦN CỨNG
// ============================================================================
esp_err_t DisplayManager::write_cb(const hd44780_t *lcd, uint8_t data) {
    if (!s_pcf_ptr) return ESP_FAIL;
    // Ghi 1 byte xuống PCF8574 qua I2C (đã được bảo vệ bằng mutex nội bộ của i2cdev)
    return pcf8574_port_write(s_pcf_ptr, data);
}

// ============================================================================
// KHỞI TẠO MODULE
// ============================================================================
esp_err_t DisplayManager::init() {
    ESP_LOGI(TAG, "Khởi tạo LCD qua PCF8574...");

  // 1. Cấu hình PCF8574 descriptor
    // Lưu ý: Ép kiểu (gpio_num_t) để chiều lòng C++
    // Dùng chung Cfg::I2C_PORT với SensorManager::bme680Setup() (CLAUDE.md §4)
    // — bắt buộc để mutex per-port của i2cdev serialize đúng BME680 + PCF8574.
    esp_err_t err = pcf8574_init_desc(&pcf_,
                                       Cfg::PCF8574_I2C_ADDR,
                                       static_cast<i2c_port_t>(Cfg::I2C_PORT),
                                       (gpio_num_t)Cfg::I2C_SDA_PIN,
                                       (gpio_num_t)Cfg::I2C_SCL_PIN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCF8574 khởi tạo thất bại: %s", esp_err_to_name(err));
        return err;
    }
    s_pcf_ptr = &pcf_;

    // 2. Cấu hình driver HD44780
    // 2. Cấu hình driver HD44780
    // Xóa sạch vùng nhớ của struct để tránh rác và qua mặt cảnh báo của compiler
    memset(&lcd_, 0, sizeof(lcd_)); 
    
    lcd_.write_cb = write_cb;
    lcd_.font = HD44780_FONT_5X8;
    lcd_.lines = 2;
    
    // Khai báo chân (Pinout) — khớp CLAUDE.md §5.
    // [GHI CHÚ] hd44780_t KHÔNG có field cho RW (P1): driver không bao giờ set
    //   bit 1 → P1 luôn = 0 = LOW = write-mode (đúng yêu cầu). Đây là điều kiện
    //   PHẦN CỨNG bắt buộc: RW phải nối P1 và không bị kéo HIGH ở ngoài.
    lcd_.pins.rs = 0;
    lcd_.pins.e  = 2;
    lcd_.pins.d4 = 4;
    lcd_.pins.d5 = 5;
    lcd_.pins.d6 = 6;
    lcd_.pins.d7 = 7;
    lcd_.pins.bl = 3;
    
    // 3. Khởi tạo chuỗi lệnh phần cứng
    err = hd44780_init(&lcd_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD khởi tạo thất bại! Kiểm tra dây nối I2C hoặc địa chỉ.");
        return err;
    }

    // Nạp ký tự đặc biệt vào CGRAM (Slot 0)
    hd44780_upload_character(&lcd_, 0, char_degree);

    hd44780_switch_backlight(&lcd_, true);
    
    // Xóa màn hình ảo và vật lý
    memset(shadow_, ' ', sizeof(shadow_));
    memset(current_display_, ' ', sizeof(current_display_));
    hd44780_clear(&lcd_);

    initialized_ = true;
    
    // Màn hình chào Booting
    showMessage("ENV-AQ-01", "Starting...");
    
    ESP_LOGI(TAG, "LCD khởi tạo thành công.");
    return ESP_OK;
}

// ============================================================================
// CẬP NHẬT LOGIC
// ============================================================================
void DisplayManager::update(const AirData &data) {
    if (!initialized_) return;

    // Overlay message (showMessage) đang trong thời gian giữ tối thiểu —
    // bỏ qua render trạng thái thường để người dùng kịp đọc thông báo.
    if (overlay_active_) {
        if (esp_timer_get_time() < overlay_expire_us_) {
            return;
        }
        overlay_active_ = false;
    }

    evaluateState(data);

    // Xóa sạch buffer ảo bằng phím Space trước khi vẽ frame mới
    memset(shadow_, ' ', sizeof(shadow_));

    if (current_state_ == DisplayState::WARMING_UP) {
        snprintf(shadow_[0], 17, "   WARMING UP   ");
        snprintf(shadow_[1], 17, "  Please wait.. ");
    } else {
        renderToShadow(data);
    }

    // Thay thế ký tự NULL (\0) bằng phím Space để ghi đè sạch các ký tự cũ trên LCD
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 16; j++) {
            if (shadow_[i][j] == '\0') shadow_[i][j] = ' ';
        }
    }

    if (calib_alert_active_ && current_state_ == DisplayState::NORMAL) {
        int row = calibAlertRow(data.calib_reason);
        if (row >= 0) {
            int last = 15;
            while (last > 0 && shadow_[row][last] == ' ') last--;
            int mark_col = (last < 15) ? last + 1 : 15;
            shadow_[row][mark_col] = '!';
        }
    }

    commitDirtyCheck();
}

void DisplayManager::evaluateState(const AirData &data) {
    // Cờ skip_warmup_display tự vô hiệu ngay khi cả 3 cảm biến đã ready thật —
    // không cần lệnh tắt riêng (xem setForceSkipWarmupDisplay()).
    if (force_skip_warmup_display_ &&
        data.bme680_ready && data.pms5003_ready && data.mq135_ready) {
        force_skip_warmup_display_ = false;
    }

    if (!data.bme680_ready && !data.pms5003_ready && !data.mq135_ready &&
        !force_skip_warmup_display_) {
        // Chưa cảm biến nào ready (mới boot) — chưa có gì để hiển thị.
        current_state_ = DisplayState::WARMING_UP;
    } else {
        current_state_ = DisplayState::NORMAL;
    }

    calib_alert_active_ = data.calib_needed;
}

// ============================================================================
// ÁNH XẠ calib_reason → DÒNG CẦN VẼ "!" TRÊN TRANG ĐANG HIỂN THỊ
// ============================================================================
int DisplayManager::calibAlertRow(const char *reason) const {
    if (!reason) return -1;
    auto match = [&](const char *s) { return strcmp(reason, s) == 0; };

    switch (current_page_) {
        case ScreenPage::MAIN_AQI_CI:
            if (match("CALIB_DRIFT_AQI")) return 0;       // dòng AQI
            if (match("CALIB_DRIFT_COMFORT")) return 1;   // dòng CI
            if (match("CALIB_OVERDUE_30D")) return 0;     // quá hạn chung — báo ở dòng đầu
            return -1;

        case ScreenPage::MAIN_CO2_PM:
            if (match("CALIB_DRIFT_CO2")) return 0;       // dòng CO2
            if (match("CALIB_DRIFT_PM25") || match("CALIB_DRIFT_PM10")) return 1; // dòng PM
            if (match("CALIB_OVERDUE_30D")) return 0;
            return -1;

        case ScreenPage::DETAIL:
            if (match("CALIB_DRIFT_TEMP") || match("CALIB_DRIFT_HUMI")) return 0; // dòng T/RH
            if (match("CALIB_OVERDUE_30D")) return 0;
            return -1;
    }
    return -1;
}

// ============================================================================
// RENDER NỘI DUNG RA BUFFER ẢO (O(1) String Formatting)
// ============================================================================
void DisplayManager::renderToShadow(const AirData &data) {
    // 2/3 trang (MAIN_AQI_CI, MAIN_CO2_PM) ưu tiên các chỉ số chính AQI/CI/CO2 (kèm nhãn
    // định tính); 1/3 trang (DETAIL) là nhóm phụ T/RH/P — đúng tỉ lệ ưu tiên CLAUDE.md §3.
    switch (current_page_) {
        case ScreenPage::MAIN_AQI_CI:
            {
                if (data.pms5003_ready || force_skip_warmup_display_) {
                    const char* aqi_labels[] = {"Good", "Fair", "Poor", "Bad", "VeryBad", "Hazard"};
                    uint8_t aqi_cat = (data.aqi_category <= 5) ? data.aqi_category : 5;
                    snprintf(shadow_[0], 17, "AQI:%03d %s", (int)lroundf(data.aqi), aqi_labels[aqi_cat]);
                } else {
                    snprintf(shadow_[0], 17, "AQI: WARMING UP");
                }

                if (data.bme680_ready || force_skip_warmup_display_) {
                    // Nhãn CI: data.comfort_category đã được DataFusion phân loại
                    // theo thang Thom DI 6 mức (config.hpp §14) — Display chỉ map số→nhãn.
                    const char* ci_labels[] = {"Good", "S.Hot", "Hot", "V.Hot", "Stress", "Danger"};
                    uint8_t ci_cat = (data.comfort_category <= 5) ? data.comfort_category : 5;
                    snprintf(shadow_[1], 17, "CI:%4.1f %s", data.comfort_index, ci_labels[ci_cat]);
                } else {
                    snprintf(shadow_[1], 17, "CI: WARMING UP");
                }
            }
            break;

        case ScreenPage::MAIN_CO2_PM:
            {
                if (data.mq135_ready || force_skip_warmup_display_) {
                    // co2_category đã được DataFusion phân loại (config.hpp §9) — Display chỉ map số→nhãn.
                    const char* co2_labels[] = {"OK", "Mod", "Bad"};
                    uint8_t co2_cat = (data.co2_category <= 2) ? data.co2_category : 2;
                    snprintf(shadow_[0], 17, "CO2:%04d ppm %s", (int)lroundf(data.co2_ppm), co2_labels[co2_cat]);
                } else {
                    snprintf(shadow_[0], 17, "CO2: WARMING UP");
                }

                if (data.pms5003_ready || force_skip_warmup_display_) {
                    snprintf(shadow_[1], 17, "P25:%03d P10:%03d", (unsigned int)data.pm2_5 % 1000, (unsigned int)data.pm10 % 1000);
                } else {
                    snprintf(shadow_[1], 17, "PM: WARMING UP");
                }
            }
            break;

        case ScreenPage::DETAIL:
            if (data.bme680_ready || force_skip_warmup_display_) {
                // Độ rộng cố định: %5.1f cho temperature (dải sanity -40.0..85.0
                // → tối đa "-40.0" = 5 ký tự) và %3d cho humidity (dải sanity
                // 0..100 → tối đa "100" = 3 ký tự). Tổng = 16 ký tự cho mọi giá
                // trị hợp lệ — không bao giờ tràn hay lệch cột.
                snprintf(shadow_[0], 17, "T:%5.1f\x08" "C H:%3d%%", data.temperature, (int)lroundf(data.humidity)); // \x08 là slot 0 (độ)
                // %6.1f cho pressure (dải sanity 300.0..1100.0 hPa → tối đa
                // "1100.0" = 6 ký tự), còn dư chỗ so với 16 ký tự dòng LCD.
                snprintf(shadow_[1], 17, "P:%6.1f hPa", data.pressure);
            } else {
                snprintf(shadow_[0], 17, "T/RH: WARMING UP");
                snprintf(shadow_[1], 17, "P: WARMING UP");
            }
            break;
    }
}

// ============================================================================
// ĐẨY DỮ LIỆU TỐI ƯU (DIRTY-CHECK)
// ============================================================================
void DisplayManager::commitDirtyCheck() {
    // Thuật toán quét điểm khác biệt giữa buffer ảo và LCD thật
    // Nhằm giảm tối đa số lần gọi I2C Transaction
    for (int row = 0; row < 2; ++row) {
        int col = 0;
        while (col < 16) {
            if (shadow_[row][col] != current_display_[row][col]) {
                hd44780_gotoxy(&lcd_, col, row);
                // Ghi một dải ký tự khác biệt liên tiếp
                while (col < 16 && shadow_[row][col] != current_display_[row][col]) {
                    hd44780_putc(&lcd_, shadow_[row][col]);
                    current_display_[row][col] = shadow_[row][col];
                    col++;
                }
            } else {
                col++;
            }
        }
    }
}

// ============================================================================
// TIỆN ÍCH
// ============================================================================
void DisplayManager::tick() {
    // [XM-1] Cadence 2-5s (Cfg::LCD_MIN_INTERVAL_MS..LCD_MAX_INTERVAL_MS): DisplayManager
    // không tự ép nhịp — main.cpp phải gọi update(data) rồi tick() mỗi chu kỳ (xem main.cpp).
    if (current_state_ == DisplayState::NORMAL) {
        // Luân phiên trang hiển thị: MAIN_AQI_CI -> MAIN_CO2_PM -> DETAIL -> MAIN_AQI_CI ...
        uint8_t next_page = static_cast<uint8_t>(current_page_) + 1;
        if (next_page > static_cast<uint8_t>(ScreenPage::DETAIL)) {
            next_page = 0;
        }
        current_page_ = static_cast<ScreenPage>(next_page);
    }
}

void DisplayManager::showMessage(const char *line1, const char *line2) {
    if (!initialized_) return;

    memset(shadow_, ' ', sizeof(shadow_));
    if (line1) snprintf(shadow_[0], 17, "%-16s", line1); // Căn trái, bù space
    if (line2) snprintf(shadow_[1], 17, "%-16s", line2);

    // Giữ overlay tối thiểu Cfg::LCD_OVERLAY_MIN_MS — update() sẽ bỏ qua
    // render trạng thái thường cho tới khi mốc thời gian này trôi qua,
    // tránh thông báo bị ghi đè ngay ở chu kỳ kế tiếp.
    overlay_active_ = true;
    overlay_expire_us_ = esp_timer_get_time() + static_cast<int64_t>(Cfg::LCD_OVERLAY_MIN_MS) * 1000;

    commitDirtyCheck();
}

void DisplayManager::setBacklight(bool on) {
    if (!initialized_) return;
    hd44780_switch_backlight(&lcd_, on);
} //Có thể bỏ nếu ko dùng đến, hiện tại chưa có caller nào (dead code) — xem [XM-6] trong header.

void DisplayManager::setForceSkipWarmupDisplay(bool on) {
    force_skip_warmup_display_ = on;
}
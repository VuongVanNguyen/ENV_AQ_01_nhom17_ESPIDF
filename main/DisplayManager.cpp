
#include "DisplayManager.hpp"
#include "esp_log.h"
#include <cstdio>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DisplayManager";

// Con trỏ tĩnh phục vụ cho write_cb (vì callback của hd44780 là C-style function pointer)
static i2c_dev_t* s_pcf_ptr = nullptr;

// Ký tự tự định nghĩa (CGRAM)
static const uint8_t char_degree[] = { 0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00 }; // Ký tự '°'
static const uint8_t char_micro[]  = { 0x00, 0x00, 0x11, 0x11, 0x11, 0x13, 0x1D, 0x10 }; // Ký tự 'µ'

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
    ESP_LOGI(TAG, "Initializing LCD via PCF8574...");

  // 1. Cấu hình PCF8574 descriptor
    // Lưu ý: Ép kiểu (gpio_num_t) để chiều lòng C++
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf_, Cfg::PCF8574_I2C_ADDR, (i2c_port_t)0, (gpio_num_t)Cfg::I2C_SDA_PIN, (gpio_num_t)Cfg::I2C_SCL_PIN));
    s_pcf_ptr = &pcf_;

    // 2. Cấu hình driver HD44780
    // 2. Cấu hình driver HD44780
    // Xóa sạch vùng nhớ của struct để tránh rác và qua mặt cảnh báo của compiler
    memset(&lcd_, 0, sizeof(lcd_)); 
    
    lcd_.write_cb = write_cb;
    lcd_.font = HD44780_FONT_5X8;
    lcd_.lines = 2;
    
    // Khai báo chân (Pinout)
    lcd_.pins.rs = 0;
    lcd_.pins.e  = 2;
    lcd_.pins.d4 = 4;
    lcd_.pins.d5 = 5;
    lcd_.pins.d6 = 6;
    lcd_.pins.d7 = 7;
    lcd_.pins.bl = 3;
    
    // 3. Khởi tạo chuỗi lệnh phần cứng
    esp_err_t err = hd44780_init(&lcd_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD Init failed! Check I2C wiring or Address.");
        return err;
    }

    // Nạp ký tự đặc biệt vào CGRAM (Slot 0 và 1)
    hd44780_upload_character(&lcd_, 0, char_degree);
    hd44780_upload_character(&lcd_, 1, char_micro);

    hd44780_switch_backlight(&lcd_, true);
    
    // Xóa màn hình ảo và vật lý
    memset(shadow_, ' ', sizeof(shadow_));
    memset(current_display_, ' ', sizeof(current_display_));
    hd44780_clear(&lcd_);

    initialized_ = true;
    
    // Màn hình chào Booting
    showMessage("   ENV-AQ-01   ", "  Khoi dong...  ");
    
    ESP_LOGI(TAG, "LCD Initialized Successfully");
    return ESP_OK;
}

// ============================================================================
// CẬP NHẬT LOGIC
// ============================================================================
void DisplayManager::update(const AirData &data) {
    if (!initialized_) return;

    evaluateState(data);

    // Xóa sạch buffer ảo bằng phím Space trước khi vẽ frame mới
    memset(shadow_, ' ', sizeof(shadow_));

    if (current_state_ == DisplayState::CALIB_ALERT) {
        snprintf(shadow_[0], 17, "!-CALIB NEEDED-!");
        snprintf(shadow_[1], 17, "  Check Sensor  ");
    } 
    else if (current_state_ == DisplayState::WARMING_UP) {
        snprintf(shadow_[0], 17, "   WARMING UP   ");
        snprintf(shadow_[1], 17, "  Please wait.. ");
    } 
    else if (current_state_ == DisplayState::NORMAL) {
        renderToShadow(data);
    }

    // Thay thế ký tự NULL (\0) bằng phím Space để ghi đè sạch các ký tự cũ trên LCD
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 16; j++) {
            if (shadow_[i][j] == '\0') shadow_[i][j] = ' ';
        }
    }

    commitDirtyCheck();
}

void DisplayManager::evaluateState(const AirData &data) {
    if (data.calib_needed) {
        current_state_ = DisplayState::CALIB_ALERT;
    } else if (!data.sensors_ready) {
        current_state_ = DisplayState::WARMING_UP;
    } else {
        current_state_ = DisplayState::NORMAL;
    }
}

// ============================================================================
// RENDER NỘI DUNG RA BUFFER ẢO (O(1) String Formatting)
// ============================================================================
void DisplayManager::renderToShadow(const AirData &data) {
    switch (current_page_) {
        case ScreenPage::TEMP_HUMI:
            snprintf(shadow_[0], 17, "T:%04.1f\x08" "C H:%02d%%", data.temperature, (int)data.humidity); // \x08 là slot 0 (độ)
            snprintf(shadow_[1], 17, "P:%04d hPa", (int)data.pressure);
            break;

        case ScreenPage::AIR_QUALITY:
            {
                const char* aqi_labels[] = {"Tot", "TB", "Kem", "Xau", "R.Xau", "Nguy"};
                uint8_t cat = (data.aqi_category <= 5) ? data.aqi_category : 5;
                snprintf(shadow_[0], 17, "AQI: %03d  %s", (int)data.aqi, aqi_labels[cat]);
                // Dùng % 1000 để cam kết với compiler số này chỉ có tối đa 3 chữ số
                snprintf(shadow_[1], 17, "PM2.5: %03d \x09g/m3", (unsigned int)data.pm2_5 % 1000); 
            }
            break;

        case ScreenPage::PARTICULATE:
            // Rút gọn PM2.5 thành P2.5 để tổng số ký tự vừa đúng 16 ô của màn hình
            snprintf(shadow_[0], 17, "PM25:%03d P10:%03d", (unsigned int)data.pm2_5 % 1000, (unsigned int)data.pm10 % 1000);
            snprintf(shadow_[1], 17, "CO2: %04d ppm", (int)data.co2_ppm);
            break;

        case ScreenPage::DERIVED:
            snprintf(shadow_[0], 17, "TVOC: %04.2f ppm", data.tvoc_ppm);
            // Comfort index logic có thể ánh xạ sang chuỗi tại đây
            snprintf(shadow_[1], 17, "DI: %.1f", data.comfort_index); 
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
    if (current_state_ == DisplayState::NORMAL) {
        // Luân phiên trang hiển thị
        uint8_t next_page = static_cast<uint8_t>(current_page_) + 1;
        if (next_page > static_cast<uint8_t>(ScreenPage::DERIVED)) {
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
    
    commitDirtyCheck();
}

void DisplayManager::setBacklight(bool on) {
    if (!initialized_) return;
    hd44780_switch_backlight(&lcd_, on);
}
#pragma once

// ============================================================================
// DisplayManager.hpp — ĐẶC TẢ MODULE HIỂN THỊ LCD 16x2 (qua PCF8574 / I2C)
// Kiến trúc dựa trên esp-idf-lib (hd44780 + pcf8574 trên nền i2cdev)
// ============================================================================

#include "DataStructures.hpp" 
#include "config.hpp"         
#include "esp_err.h"
#include <hd44780.h>          
#include <pcf8574.h>          

// §2. API CÔNG KHAI
class DisplayManager {
public:
    // §4. ENUM CẦN KHAI BÁO
    // Mô hình 3 trang theo CLAUDE.md §3: 2/3 số trang ưu tiên 3 chỉ số chính
    // (AQI, CI, CO2 — luôn kèm nhãn định tính), 1/3 còn lại là nhóm phụ T/RH/P,
    // luân phiên (rotate) theo tick().
    enum class ScreenPage : uint8_t {
        MAIN_AQI_CI = 0,  // AQI (kèm nhãn) + CI/Comfort Index (kèm nhãn) — 2 chỉ số chính
        MAIN_CO2_PM = 1,  // CO2 (kèm nhãn) — chỉ số chính + PM2.5/PM10 — chỉ số phụ
        DETAIL = 2        // Nhiệt độ + Độ ẩm + Áp suất — nhóm phụ
    };

    enum class DisplayState : uint8_t {
        WARMING_UP = 0,     // Chưa CẢM BIẾN NÀO ready (lúc mới khởi động)
        NORMAL = 2         
    };

    // Đảm bảo chỉ có 1 instance duy nhất điều khiển phần cứng
    DisplayManager();
    ~DisplayManager() = default;
    DisplayManager(const DisplayManager&) = delete;
    DisplayManager& operator=(const DisplayManager&) = delete;

    // Khởi tạo LCD. Chạy sau khi SensorManager đã gọi i2cdev_init().
    esp_err_t init();

    // Cập nhật nội dung hiển thị dựa trên dữ liệu mới nhất.
    // Cam kết READ-ONLY, không chỉnh sửa AirData.
    void update(const AirData &data);

    // Xử lý luân chuyển trang (Rotate) hoặc nhấp nháy cảnh báo (Blink).
    void tick();

    // Hiển thị thông báo tức thời (ví dụ: mất mạng, lỗi thẻ nhớ SD).
    void showMessage(const char *line1, const char *line2 = nullptr);

    // Tiết kiệm năng lượng: Bật/tắt đèn nền LCD.
    void setBacklight(bool on);

    // Các hàm getter phục vụ debug qua JTAG
    ScreenPage currentPage() const { return current_page_; }
    DisplayState currentState() const { return current_state_; }
    bool isInitialized() const { return initialized_; }

private:
    i2c_dev_t pcf_;     // Descriptor cho thiết bị PCF8574 (kết nối với i2cdev)
    hd44780_t lcd_;     // Descriptor cấu hình chân cho driver hd44780
    
    bool initialized_;
    ScreenPage current_page_;
    DisplayState current_state_;

    // §6.2 Framebuffer phần mềm (16 ký tự + 1 ký tự null terminator cho mỗi dòng)
    char shadow_[2][17];         // Buffer chứa nội dung MUỐN hiển thị
    char current_display_[2][17]; // Buffer chứa nội dung ĐANG hiển thị thực tế

    // true khi data.calib_needed của chu kỳ gần nhất — đặt bởi evaluateState(),
    // đọc bởi update() để vẽ dấu "!" trên đúng dòng liên quan tới calib_reason.
    // Tách khỏi current_state_ để cảnh báo calib không còn che mất AQI/CI/CO2.
    // KHÔNG còn nhấp nháy backlight (gây khó đọc) — xem [XM-CALIB] trong header.
    bool calib_alert_active_;

    // Overlay cho showMessage(): giữ thông báo tối thiểu Cfg::LCD_OVERLAY_MIN_MS
    // trước khi update() được phép vẽ lại frame trạng thái bình thường.
    bool overlay_active_;
    int64_t overlay_expire_us_;

    // Callback ghi I2C bắt buộc phải có để truyền cho hd44780_init
    static esp_err_t write_cb(const hd44780_t *lcd, uint8_t data);

    // Các hàm nội bộ phục vụ render và thuật toán Dirty-check
    void evaluateState(const AirData &data);
    void renderToShadow(const AirData &data);
    void commitDirtyCheck(); // Chỉ gọi I2C transaction ở những vị trí thay đổi text

    // Trả về dòng (0/1) liên quan tới calib_reason trên trang đang hiển thị,
    // hoặc -1 nếu calib_reason không liên quan tới trang hiện tại (không vẽ "!").
    int calibAlertRow(const char *reason) const;
};
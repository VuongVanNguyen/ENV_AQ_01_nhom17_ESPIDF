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

// §4. ENUM CẦN KHAI BÁO
enum class ScreenPage : uint8_t {
    TEMP_HUMI = 0,      // Nhiệt độ + Độ ẩm + Áp suất
    AIR_QUALITY = 1,    // AQI (số) + nhãn category (Tốt/TB/Kém...)
    PARTICULATE = 2,    // PM2.5 + PM10 + CO2
    DERIVED = 3         // TVOC + Comfort Index
};

enum class DisplayState : uint8_t {
    BOOTING = 0,        // Đang khởi động, chờ mẫu dữ liệu đầu tiên
    WARMING_UP = 1,     // Ít nhất 1 cảm biến chưa ready
    CALIB_ALERT = 2,    // Cảnh báo Drift / Cần hiệu chuẩn (Ưu tiên cao nhất)
    NORMAL = 3          // Hoạt động bình thường, luân phiên trang
};

// §2. API CÔNG KHAI
class DisplayManager {
public:
    // Đảm bảo chỉ có 1 instance duy nhất điều khiển phần cứng
    DisplayManager() = default;
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
    
    bool initialized_ = false;
    ScreenPage current_page_ = ScreenPage::TEMP_HUMI;
    DisplayState current_state_ = DisplayState::BOOTING;

    // §6.2 Framebuffer phần mềm (16 ký tự + 1 ký tự null terminator cho mỗi dòng)
    char shadow_[2][17] = {0};         // Buffer chứa nội dung MUỐN hiển thị
    char current_display_[2][17] = {0}; // Buffer chứa nội dung ĐANG hiển thị thực tế

    // Trạng thái nhấp nháy (Blink) backlight cho CALIB_ALERT — đảo mỗi lần
    // tick() được gọi trong khi current_state_ == CALIB_ALERT.
    bool blink_state_ = true;

    // Overlay cho showMessage(): giữ thông báo tối thiểu Cfg::LCD_OVERLAY_MIN_MS
    // trước khi update() được phép vẽ lại frame trạng thái bình thường.
    bool overlay_active_ = false;
    int64_t overlay_expire_us_ = 0;

    // Callback ghi I2C bắt buộc phải có để truyền cho hd44780_init
    static esp_err_t write_cb(const hd44780_t *lcd, uint8_t data);

    // Các hàm nội bộ phục vụ render và thuật toán Dirty-check
    void evaluateState(const AirData &data);
    void renderToShadow(const AirData &data); 
    void commitDirtyCheck(); // Chỉ gọi I2C transaction ở những vị trí thay đổi text
};

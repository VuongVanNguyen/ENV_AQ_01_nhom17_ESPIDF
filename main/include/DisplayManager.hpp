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
        CALIB_ALERT = 1,    // Cảnh báo Drift / Cần hiệu chuẩn (Ưu tiên cao nhất)
        NORMAL = 2          // ≥1 cảm biến ready — luân phiên trang, dòng chưa ready in placeholder riêng
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

    // Trạng thái nhấp nháy (Blink) backlight cho CALIB_ALERT — đảo mỗi lần
    // tick() được gọi trong khi current_state_ == CALIB_ALERT.
    bool blink_state_;

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
};

// ============================================================================
// CHECKLIST KIỂM TRA DISPLAYMANAGER (đối chiếu CLAUDE.md §3 / §4 / §5)
// Đánh dấu [x] khi đã fix & verify. Cập nhật liên tục mỗi lần đụng tới module.
// ============================================================================
//
// --- ĐÃ FIX (đợt refactor hiện tại) ---
// [x] FIX #1  Bỏ hẳn data.tvoc_ppm (field không tồn tại trong AirData) — CLAUDE.md §1.
// [x] FIX #2  Tái cấu trúc ScreenPage còn 3 trang: MAIN_AQI_CI (AQI+CI, kèm nhãn),
//             MAIN_CO2_PM (CO2 kèm nhãn + PM2.5/PM10), DETAIL (T/RH/P) — 2/3 số trang
//             ưu tiên các chỉ số chính, 1/3 là nhóm phụ T/RH/P (CLAUDE.md §3).
// [x] FIX #3  CI (Comfort Index) in kèm nhãn định tính (Tot/Am/Nong/Kho/Nguy/C.Cuu)
//             ánh xạ trực tiếp từ data.comfort_category (6 mức, đã phân loại sẵn
//             trong DataFusion::computeComfort() theo thang Thom DI — config.hpp §14).
// [x] FIX #4  Bỏ DisplayState::BOOTING (trạng thái chết, không bao giờ được
//             evaluateState() set) — mặc định current_state_ = WARMING_UP, đúng
//             với thực tế "chưa có cảm biến nào ready" lúc mới khởi động.
// [x] FIX #5  Wrap-around rotate cập nhật theo enum mới (mod ScreenPage::DETAIL).
// [x] FIX #6  CO2 in kèm nhãn định tính (Tot/TB/Xau) ánh xạ từ config.hpp §15
//             (CO2_GOOD_MAX/CO2_MODERATE_MAX) — xem [XM-5].
//
// >>> NGUYÊN TẮC §3: cả 3 chỉ số chính AQI / CI / CO2 đều in kèm nhãn định tính. ĐẠT.
//
// --- ĐÃ ĐẠT (xác nhận giữ nguyên khi refactor) ---
// [x] Dùng driver esp-idf-lib hd44780 + pcf8574, KHÔNG dùng LiquidCrystal_I2C (§4).
// [x] Dùng chung Cfg::I2C_PORT với SensorManager → mutex per-port i2cdev serialize đúng (§4).
// [x] Pinout PCF8574→LCD khớp §5 (RS=P0, E=P2, D4–D7=P4–P7, BL=P3; RW=P1 ngầm LOW).
// [x] Dirty-check (shadow_ vs current_display_) chỉ ghi I2C ô thay đổi → giảm tải bus (§4).
// [x] Nhãn ASCII không dấu ("Tot/TB/Kem/Xau/R.Xau/Nguy") khớp aqi_category 0..5 & HD44780.
// [x] Ký tự °/µ qua CGRAM slot 0/1, gọi bằng \x08/\x09 (né \x00 = null terminator).
// [x] Độ rộng định dạng đã canh để mỗi dòng ≤ 16 ô, không tràn cột.
//
// ============================================================================
// LỖI / PHỤ THUỘC LIÊN-MODULE (cần module khác xử lý — KHÔNG sửa trong DisplayManager)
// ============================================================================
//
// [XM-1] main.cpp — CADENCE 2–5s (CLAUDE.md §3): DisplayManager không tự ép nhịp. Phải
//        verify main.cpp gọi update()→tick() theo chu kỳ nằm trong
//        [Cfg::LCD_MIN_INTERVAL_MS=2000, Cfg::LCD_MAX_INTERVAL_MS=5000], và
//        CONFIG_DISPLAY_UPDATE_INTERVAL_MS (Kconfig) cũng nằm trong dải này.
//        Thứ tự gọi khuyến nghị mỗi chu kỳ: update(data) rồi tick().
//        TRẠNG THÁI: chưa triển khai (main.cpp Production mode còn là TODO) — đặc tả
//        đã được sao chép vào main.cpp, CHƯA hiện thực hoá theo yêu cầu.
//
// [XM-2] DataFusion.cpp — phải GHI data.comfort_index (THI), data.comfort_category
//        và data.calib_needed.
//        TRẠNG THÁI: ĐÃ XÁC MINH — DataFusion::computeComfort()/driftSelfCheck() ghi
//        đúng 3 trường này (DataFusion.cpp). Không cần sửa thêm.
//
// [XM-3] SensorManager.cpp — phải set đúng 3 cờ độc lập data.bme680_ready/
//        pms5003_ready/mq135_ready (KHÔNG có field tổng hợp sensors_ready —
//        đã bị xoá khỏi AirData).
//        TRẠNG THÁI: ĐÃ XÁC MINH — evaluateState() dùng OR của 3 cờ này để
//        quyết định WARMING_UP; renderToShadow() tự kiểm tra từng cờ để in
//        placeholder "... WARMING UP" cho dòng tương ứng khi cảm biến nguồn
//        chưa ready (tránh (int)NAN khi data.aqi/comfort_index = NAN). Không
//        cần sửa thêm.
//
// [XM-4] main.cpp — i2cdev_init() phải do SensorManager::init() gọi TRƯỚC, rồi mới tới
//        DisplayManager::init() (CLAUDE.md §4).
//        TRẠNG THÁI: chưa triển khai (main.cpp Production mode còn là TODO) — đặc tả
//        đã được sao chép vào main.cpp, CHƯA hiện thực hoá theo yêu cầu.
//
// [XM-5] config.hpp — ĐÃ BỔ SUNG Cfg::CO2_GOOD_MAX / CO2_MODERATE_MAX (§15) để
//        DisplayManager ánh xạ CO2 → "Tot/TB/Xau" (FIX #6). ĐÃ XONG.
//
// [XM-15] main.cpp — showMessage() báo "mất mạng"/"đã có mạng" KHÔNG được gọi
//        trực tiếp từ taskNetwork (DisplayManager không có mutex nội bộ →
//        race với update()/tick() của taskDisplay). Giải pháp: hàng đợi
//        length-1 s_display_notice (taskNetwork ghi xQueueOverwrite, CHỈ
//        taskDisplay đọc xQueueReceive rồi tự gọi showMessage()) — giữ đúng
//        nguyên tắc "chỉ 1 task chạm vào s_display".
//        TRẠNG THÁI: ĐÃ TRIỂN KHAI (main.cpp taskNetwork edge MQTT_CONNECTED/
//        MQTT_DISCONNECTED + taskDisplay đầu mỗi chu kỳ).
//
// [XM-6] (OPTIONAL / LOW PRIORITY — backlog, không phục vụ NFR bắt buộc nào)
//        setBacklight(bool) — public API "tiết kiệm năng lượng" hiện CHƯA có
//        caller (dead code). Ý tưởng nếu sau này cần: main.cpp tắt backlight
//        sau một khoảng idle (NORMAL, không CALIB_ALERT/showMessage mới) để
//        giảm thêm vài chục mA. Đánh giá: lợi ích nhỏ so với ngân sách 2W
//        (chủ yếu do ESP32+WiFi), và có thể gây hiểu nhầm "máy hỏng" trên
//        thiết bị quan trắc luôn hiển thị — chỉ nên làm sau khi main.cpp
//        Production mode đã chạy ổn và đo công suất thực tế cho thấy cần.
//        TRẠNG THÁI: KHÔNG triển khai trừ khi có yêu cầu cụ thể.
// ============================================================================

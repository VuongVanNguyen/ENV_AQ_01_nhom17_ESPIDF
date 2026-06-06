#pragma once

// ============================================================================
// DisplayManager.hpp — ĐẶC TẢ MODULE HIỂN THỊ LCD 16x2 (qua PCF8574 / I2C)
// Dự án ENV-AQ-01 (CLAUDE.md §2, §3, §4, §5)
//
// LƯU Ý: File này hiện chỉ chứa ĐẶC TẢ (comment) — CHƯA hiện thực hoá code.
//        Khi triển khai, header này sẽ chứa: enum ScreenPage, class DisplayManager
//        với các API mô tả ở §2, và #include tương ứng (§9). DisplayManager.cpp
//        hiện đang trống — toàn bộ implementation sẽ đặt ở đó.
//
// QUYẾT ĐỊNH THƯ VIỆN (đọc kỹ — thay cho phương án "tự code giao thức" ban đầu):
//   Module DÙNG driver **hd44780 + pcf8574 của esp-idf-lib** (native ESP-IDF) để
//   điều khiển LCD — KHÔNG tự hiện thực hoá giao thức HD44780 4-bit bằng tay.
//   Cơ sở: CLAUDE.md §4 chỉ CẤM thư viện Arduino `LiquidCrystal_I2C`, không cấm
//   mọi thư viện. esp-idf-lib KHÔNG phải Arduino và ĐÃ là nền tảng của dự án
//   (SensorManager dùng bme680 + i2cdev cùng bộ — SensorManager.hpp). Dùng lại
//   nó giữ tính nhất quán, dùng chung mutex per-port i2cdev (§3), và loại bỏ độ
//   phức tạp/timing dễ sai của việc tự code chuỗi init + xung Enable.
//   ⇒ DisplayManager chỉ còn lo: cấu hình mapping chân, NỘI DUNG hiển thị, trạng
//     thái (warmup/calib), dirty-check mức ứng dụng, và điều phối task (§6,§7,§12).
//
// ============================================================================
// 0. VỊ TRÍ TRONG LUỒNG DỮ LIỆU (CLAUDE.md §2)
// ----------------------------------------------------------------------------
//   SensorManager::readAll()  (đọc thô + set *_ready)
//        → Filters::process()       (làm sạch in-place)
//        → DataFusion::process()    (suy diễn AQI/TVOC/Comfort + Drift + calib)
//        → DisplayManager (LCD)  ←  MODULE NÀY  + NetworkManager + StorageHelper
//
//   DisplayManager là tầng "trình bày" CUỐI luồng: nhận AirData ĐÃ LỌC + ĐÃ SUY
//   DIỄN và CHỈ ĐỌC nó để vẽ lên LCD. Đây là module READ-ONLY trên AirData —
//   TUYỆT ĐỐI không ghi ngược struct (khác Filters/DataFusion ghi in-place).
//
//   TRÁCH NHIỆM (CLAUDE.md §2 bảng module):
//     - Điều khiển LCD 16x2 qua IC mở rộng chân PCF8574 trên bus I2C dùng chung.
//     - Trình bày các thông số môi trường + chỉ số suy diễn theo trang luân phiên.
//     - Hiển thị trạng thái hệ thống: warmup, mất mạng, yêu cầu tái hiệu chuẩn.
//
//   RANH GIỚI (KHÔNG làm — tránh chồng lấn module khác):
//     - KHÔNG đọc cảm biến, KHÔNG tính AQI/TVOC/Comfort (Sensor/Filters/DataFusion).
//     - KHÔNG quyết định mức cảnh báo (AlertLevel do DataFusion tính — §8 của
//       DataFusion.hpp). DisplayManager chỉ ĐỌC kết quả và hiển thị.
//     - KHÔNG điều khiển LED/Buzzer (đó là main.cpp lái GPIO — CLAUDE.md §5).
//     - KHÔNG publish MQTT, KHÔNG ghi SD, KHÔNG chạm NVS.
//     - KHÔNG khởi tạo bus I2C (i2c_master_bus_create / i2cdev_init do
//       SensorManager::init() đã gọi — xem §3). Chỉ ĐĂNG KÝ device PCF8574.
//
// ============================================================================
// 1. PHẦN CỨNG — LCD 16x2 QUA PCF8574 (CLAUDE.md §5)
// ----------------------------------------------------------------------------
//   Bus  : I2C dùng chung BME680 + PCF8574 — SDA=GPIO21, SCL=GPIO22 (Cfg §1).
//   Addr : PCF8574 = 0x20 (A0/A1/A2 nối GND) — Cfg::PCF8574_I2C_ADDR.
//   LCD  : HD44780-compatible 16 cột × 2 hàng, chạy CHẾ ĐỘ 4-BIT.
//
//   ÁNH XẠ CHÂN PCF8574 → LCD (CLAUDE.md §5 "Kết nối PCF8574 → LCD 16x2"):
//        P0 → RS   (0 = lệnh/command, 1 = dữ liệu/data)
//        P1 → RW   (LUÔN = 0 — chỉ ghi; không đọc busy-flag để giảm tải bus)
//        P2 → E    (enable strobe — chốt nibble bằng xung cao→thấp)
//        P3 → Backlight (1 = bật đèn nền)   ← không nêu rõ trong CLAUDE.md §5
//                                              nhưng là chân còn lại theo chuẩn
//                                              module PCF8574-LCD; khai báo ở §10.
//        P4 → D4, P5 → D5, P6 → D6, P7 → D7  (nibble dữ liệu 4-bit)
//
//   ⇒ Một byte gửi xuống PCF8574 có dạng bit:
//        [ D7 D6 D5 D4 | BL  E  RW  RS ]
//        bit7..4 = nibble dữ liệu, bit3 = backlight, bit2 = E, bit1 = RW(=0), bit0 = RS.
//      Vị trí bit này KHÔNG còn được tự ghép tay; nó được khai báo cho driver
//      hd44780 qua trường `hd44780_t.pins` (số THỨ TỰ BIT trên PCF8574, không phải
//      GPIO — vì dùng write-callback, §5). Các vị trí bit (RS=0,RW=1,E=2,BL=3,
//      D4..D7=4..7) khai báo hằng trong config.hpp (§10) — KHÔNG hardcode (CLAUDE.md §4).
//      RW nối bit1: vì chỉ ghi nên callback luôn để RW=0 (driver không đọc busy-flag).
//
//   ĐIỀU KHIỂN GIAO THỨC (CLAUDE.md §4 "LCD Control"):
//     - Giao thức HD44780 4-bit (chuỗi init, tách nibble, xung Enable, timing)
//       do driver **hd44780 (esp-idf-lib)** lo — KHÔNG tự hiện thực hoá bằng tay.
//     - KHÔNG dùng thư viện Arduino `LiquidCrystal_I2C` hay Arduino wrapper nào
//       (CLAUDE.md §1, §4). hd44780/pcf8574 là driver ESP-IDF native, hợp lệ.
//     - Lớp vật lý: driver hd44780 gọi 1 write-callback do DisplayManager cung cấp;
//       callback ghi 1 byte xuống PCF8574 qua pcf8574_port_write() (= 1 transaction
//       I2C, đi qua i2cdev — §3).
//
// ============================================================================
// 2. API CÔNG KHAI CẦN CÓ (mô tả — sẽ khai báo thành method của class DisplayManager)
// ----------------------------------------------------------------------------
//   Tạo đúng 1 instance DisplayManager trong main.cpp, dùng suốt vòng đời.
//   Copy/assign = delete (sở hữu duy nhất handle device PCF8574 + framebuffer).
//
//   esp_err_t init();
//     - Gọi 1 lần trong app_main() SAU khi SensorManager::init() đã chạy (bus I2C
//       + i2cdev_init() đã sẵn sàng — §3). KHÔNG tự khởi tạo bus.
//     - Đăng ký device PCF8574 @ Cfg::PCF8574_I2C_ADDR (pcf8574_init_desc) trên
//       cùng port với BME680; cấu hình + gọi hd44780_init() với write-callback (§5).
//     - Xoá màn hình, nạp custom char (ký tự °, µ — §6.4), bật backlight.
//     - Vẽ màn hình chào (ví dụ "ENV-AQ-01" / "Khoi dong...").
//     - Trả ESP_OK / mã lỗi I2C; KHÔNG block dài, KHÔNG chờ mạng.
//
//   void update(const AirData &data);
//     - GỌI ĐỊNH KỲ bởi task hiển thị (xem §12) — KHÔNG mỗi chu kỳ cảm biến.
//     - Chọn nội dung theo trạng thái (§7): warmup → AQI/thông số → cảnh báo calib.
//     - DIRTY-CHECK (§6.3): chỉ đẩy ký tự/dòng THAY ĐỔI xuống LCD (CLAUDE.md §4
//       "Chỉ cập nhật màn hình khi dữ liệu thay đổi để giảm tải bus I2C").
//     - READ-ONLY trên `data` (tham chiếu const). Tôn trọng cờ *_ready (§7, §11).
//
//   void tick();   (HOẶC để task tự đếm thời gian — chọn 1 cơ chế, mô tả §12)
//     - Luân chuyển trang hiển thị (ScreenPage) sau mỗi Cfg::LCD_PAGE_ROTATE_MS;
//       hoặc nhấp nháy cảnh báo calib. Gọi từ cùng task với update().
//
//   void showMessage(const char *line1, const char *line2 = nullptr);
//     - Tiện ích hiển thị thông báo tức thời (ví dụ "NO WIFI", "SD FULL") do
//       main.cpp / task mạng yêu cầu. Vẫn qua dirty-check + mutex bus (§3).
//
//   void setBacklight(bool on);
//     - Bật/tắt đèn nền qua hd44780_switch_backlight() — phục vụ tiết kiệm năng
//       lượng (CLAUDE.md §3 ≤ 2.0W) khi nhàn rỗi/ban đêm. Không vẽ lại nội dung.
//
//   (Tuỳ chọn, hữu ích cho JTAG/debug — không tác dụng phụ:)
//   ScreenPage currentPage() const;
//   bool isInitialized() const;
//
// ============================================================================
// 3. I2C BUS DÙNG CHUNG — KHỞI TẠO & ĐỒNG BỘ (CLAUDE.md §4 "I2C Shared Bus")
// ----------------------------------------------------------------------------
//   ⚠️ ĐIỂM KIẾN TRÚC QUAN TRỌNG — nhất quán với SensorManager:
//     SensorManager.hpp (§I2C BUS) đã chuẩn hoá bus chung trên thư viện
//     **i2cdev (esp-idf-lib)** — legacy I2C driver — và SensorManager::init()
//     tự gọi i2cdev_init() (idempotent, có static guard).
//
//     ⇒ DisplayManager khai báo PCF8574 bằng component **pcf8574 (esp-idf-lib)**,
//       vốn xây trên CÙNG i2cdev (i2c_dev_t). KHÔNG dùng API i2c_master_* mới
//       (i2c_master_bus_create / i2c_master_transmit). Lý do: ESP-IDF KHÔNG cho
//       phép trộn legacy i2cdev và new i2c_master driver trên CÙNG một port —
//       sẽ xung đột driver. Mutex per-port của i2cdev TỰ bảo vệ truy cập I2C
//       giữa task cảm biến (BME680) và task hiển thị (PCF8574), đúng yêu cầu
//       "mọi truy cập I2C từ nhiều task phải bảo vệ bằng mutex" (CLAUDE.md §4).
//
//     GHI CHÚ về CLAUDE.md §4: mục "LCD Control" nêu i2c_master_transmit() như
//     API KHÁI NIỆM để mô tả "ghi byte xuống PCF8574". Ở mức triển khai, byte đó
//     đi qua pcf8574_port_write() (→ i2cdev) để dùng chung mutex per-port với
//     BME680. Tinh thần ràng buộc (không LiquidCrystal_I2C, bảo vệ bằng mutex)
//     được giữ nguyên; chỉ thay "tự code 4-bit" bằng driver hd44780 native.
//
//   - init() gọi pcf8574_init_desc() tạo descriptor cho PCF8574 (port, addr,
//     SDA/SCL — Cfg §1). KHÔNG gọi i2cdev_init() lại (đã được SensorManager gọi);
//     i2cdev_init() idempotent nên an toàn dù thứ tự nào — thứ tự khuyến nghị:
//     SensorManager trước. Sau đó hd44780_init() với write-callback (§5).
//   - MỌI ghi xuống PCF8574 (do driver hd44780 phát ra) đi qua pcf8574 → i2cdev
//     → mutex per-port.
//
// ============================================================================
// 4. ENUM CẦN KHAI BÁO (trong header này khi triển khai)
// ----------------------------------------------------------------------------
//   enum class ScreenPage : uint8_t  — các trang nội dung luân phiên (§7.2):
//       0 = TEMP_HUMI   : Nhiệt độ + Độ ẩm + Áp suất
//       1 = AIR_QUALITY : AQI (số) + nhãn category (Tốt/TB/Kém/Xấu/Rất xấu/Nguy hại)
//       2 = PARTICULATE : PM2.5 + PM10 + CO2
//       3 = DERIVED     : TVOC + Comfort Index (diễn giải dải DI)
//     Số trang & thứ tự có thể điều chỉnh; tổng số trang khai báo hằng ở §10.
//
//   enum class DisplayState : uint8_t  — trạng thái tổng (§7.1, ưu tiên giảm dần):
//       BOOTING       : ngay sau init(), chờ mẫu hợp lệ đầu tiên.
//       WARMING_UP    : ít nhất 1 cảm biến chưa *_ready (warmup) → §7.3.
//       CALIB_ALERT   : data.calib_needed == true → §7.4 (nhấp nháy, ưu tiên cao).
//       NORMAL        : sensors_ready & data hợp lệ → luân phiên ScreenPage.
//     (DisplayState do DisplayManager TỰ suy ra từ cờ AirData; KHÔNG lưu ở struct.)
//
// ============================================================================
// 5. KHỞI TẠO LCD QUA DRIVER hd44780 (esp-idf-lib) — trong init()
// ----------------------------------------------------------------------------
//   Toàn bộ chuỗi init HD44780 (ép 4-bit, Function/Display/Clear/Entry), tách
//   nibble, xung Enable và timing phần cứng do DRIVER hd44780 lo — DisplayManager
//   KHÔNG tự code các bước này. Nhiệm vụ của init() chỉ là CẤU HÌNH + nối dây:
//
//   5.1 Khai báo descriptor PCF8574 (thành viên i2c_dev_t của lớp):
//         pcf8574_init_desc(&pcf_, Cfg::PCF8574_I2C_ADDR, port, SDA, SCL).
//       (port/SDA/SCL từ Cfg §1; i2cdev_init() đã do SensorManager gọi — §3.)
//
//   5.2 Khai báo cấu hình hd44780_t:
//         - write_cb = write-callback của DisplayManager (§5.4) — KHÁC NULL ⇒
//           driver giao tiếp qua PCF8574 thay vì GPIO trực tiếp.
//         - pins = VỊ TRÍ BIT trên PCF8574 (không phải GPIO): rs, rw(nếu driver
//           dùng), e, bl, d4..d7 — lấy từ hằng Cfg::LCD_BIT_* (§10/§B).
//         - lines = 2, font = 5×8 (LCD 16x2).
//
//   5.3 Gọi hd44780_init(&lcd_): driver tự chạy toàn bộ chuỗi khởi tạo chuẩn.
//       Sau đó: hd44780_switch_backlight(on), nạp custom char (§6.4) bằng
//       hd44780_upload_character(), vẽ màn hình chào.
//
//   5.4 Write-callback (chữ ký hd44780_write_cb_t): nhận giá trị 8-bit driver muốn
//       đặt lên các chân ⇒ DisplayManager chuyển thẳng tới pcf8574_port_write(&pcf_, val).
//       (Đây là CHỖ DUY NHẤT chạm I2C — mọi byte đi qua i2cdev/mutex per-port, §3.)
//
//   API hd44780 dùng để VẼ (ở §6/§7): hd44780_clear(), hd44780_gotoxy(col,line),
//   hd44780_puts()/hd44780_putc(), hd44780_control(on,cursor,blink),
//   hd44780_upload_character(slot,bitmap), hd44780_switch_backlight(on).
//   ⇒ KHÔNG cần khai báo mã lệnh HD44780 hay logic xung Enable trong config.hpp.
//
// ============================================================================
// 6. THUẬT TOÁN — ĐỊNH VỊ KÝ TỰ, FRAMEBUFFER & DIRTY-CHECK
// ----------------------------------------------------------------------------
//   6.1 Định vị con trỏ: dùng hd44780_gotoxy(&lcd_, col, line) — driver tự quy đổi
//       sang địa chỉ DDRAM (hàng0=0x00, hàng1=0x40). DisplayManager KHÔNG tự ghép
//       lệnh Set-DDRAM; chỉ làm việc theo toạ độ (col,line).
//
//   6.2 Framebuffer phần mềm: giữ bản sao nội dung mong muốn 2 hàng × 16 ký tự
//       (ví dụ char shadow_[2][16]). render*() ghi vào shadow trước, KHÔNG ghi LCD.
//
//   6.3 DIRTY-CHECK (BẮT BUỘC — CLAUDE.md §4): so shadow MỚI với shadow ĐANG hiển
//       thị; chỉ hd44780_gotoxy() + hd44780_puts() cho các ĐOẠN ký tự THAY ĐỔI
//       trên mỗi hàng (gộp ô liền kề để giảm số lần gotoxy/ghi). Nếu không có ô
//       nào đổi → KHÔNG phát bất kỳ transaction I2C nào. Mục tiêu: giảm tải bus
//       I2C, tránh trễ bus chung với BME680 (CLAUDE.md §3). LƯU Ý: tránh
//       hd44780_clear() mỗi chu kỳ (lệnh chậm + xoá cả màn hình) — chỉ clear ở
//       init hoặc khi đổi hẳn DisplayState (§7); cập nhật thường dùng ghi đè ký tự.
//
//   6.4 Custom characters (CGRAM): nạp ký tự đặc biệt khi init() bằng
//       hd44780_upload_character(&lcd_, slot, bitmap):
//         - '°' (độ) cho nhiệt độ.
//         - 'µ' (micro) cho µg/m³  (tuỳ chọn — có thể thay bằng "ug/m3" nếu thiếu slot).
//       Tối đa 8 glyph CGRAM (slot 0–7). Bitmap 5×8 đặt static const trong .cpp.
//
// ============================================================================
// 7. THUẬT TOÁN — CHỌN NỘI DUNG THEO TRẠNG THÁI (tôn trọng cờ *_ready)
// ----------------------------------------------------------------------------
//   7.1 Ưu tiên trạng thái (cao → thấp) trong update():
//         (a) !data.data_valid  → giữ màn hình trước HOẶC "DATA ERROR" (không vẽ rác).
//         (b) data.calib_needed → DisplayState::CALIB_ALERT (§7.4) — đè mọi trang.
//         (c) !data.sensors_ready → DisplayState::WARMING_UP (§7.3).
//         (d) còn lại → DisplayState::NORMAL → luân phiên ScreenPage (§7.2).
//
//   7.2 NORMAL — 4 trang luân phiên mỗi Cfg::LCD_PAGE_ROTATE_MS (§10):
//       Mỗi trang CHỈ vẽ kênh có cờ ready tương ứng; kênh chưa ready → "--"
//       (KHÔNG hiển thị 0/NAN như số thật — tránh hiểu nhầm, §11).
//         TEMP_HUMI   : "T:25.3{°}C H:60%"        / "P:1013 hPa"     (bme680_ready)
//         AIR_QUALITY : "AQI:042  Tot"            / "PM2.5:12 ug/m3" (pms5003_ready)
//         PARTICULATE : "PM2.5:012 PM10:020"      / "CO2: 650 ppm"   (pms/mq135_ready)
//         DERIVED     : "TVOC:0.42 ppm"           / "Comfort: De chiu"(bme680_ready)
//       (Bố cục trên là GỢI Ý — phải vừa 16 ký tự/hàng; chốt khi triển khai.)
//       Nhãn category AQI (Tốt/Trung binh/Kém/Xấu/Rất xấu/Nguy hại) lấy theo
//       data.aqi_category (0..5) — bảng nhãn (không dấu, ≤ chiều rộng) ở §10.
//       Diễn giải Comfort lấy theo dải DI (DataFusion.hpp §5.2): dùng các ngưỡng
//       Cfg::COMFORT_DI_* để in "De chiu / Hoi nong / Nong / Rat nong / Stress nhiet".
//
//   7.3 WARMING_UP: hiển thị "WARMING UP" + cảm biến nào chưa sẵn sàng, ví dụ
//         "WARMING UP..."  /  "BME:- PMS:OK MQ:-"
//       dựa trên bme680_ready / pms5003_ready / mq135_ready (DataStructures.hpp).
//       (Đồng nhất gợi ý SensorManager::isFullyReady() — SensorManager.hpp dòng 59.)
//
//   7.4 CALIB_ALERT (CLAUDE.md §3, §4 "Drift Self-Check"): khi data.calib_needed,
//       hiển thị cảnh báo NHẤP NHÁY (ví dụ "CALIB NEEDED" / "Tai hieu chuan!"),
//       đảo nội dung/backlight theo nhịp để gây chú ý. Đây là KÊNH HIỂN THỊ của
//       cờ calib (song song MQTT "calib_alert" do NetworkManager publish — không
//       phải Display). DisplayManager KHÔNG xác nhận/ghi đè baseline (chỉ người
//       dùng qua lệnh MQTT "confirm_calib" → DataFusion::confirmRecalibration).
//
// ============================================================================
// 8. ĐỊNH DẠNG SỐ LIỆU (rendering helpers — thuần chuỗi, O(1))
// ----------------------------------------------------------------------------
//   - Dùng snprintf vào buffer cố định ≤ 17 byte/hàng (16 ký tự + '\0'); KHÔNG
//     cấp phát heap trong vòng cập nhật (giảm phân mảnh, giữ non-blocking).
//   - Số chữ số thập phân hợp lý cho LCD: T/RH 1 chữ số, áp suất số nguyên,
//     AQI số nguyên (zero-pad 3), PM số nguyên, CO2 số nguyên, TVOC 2 chữ số.
//   - Ký tự không thuộc ASCII (°, µ) → dùng glyph CGRAM (§6.4) hoặc thay an toàn.
//   - Chuỗi nhãn tiếng Việt KHÔNG DẤU (HD44780 không có font dấu) để hiển thị đúng.
//
// ============================================================================
// 9. LIÊN KẾT MODULE — HÀM/DỮ LIỆU DÙNG TỪ FILE NÀO
// ----------------------------------------------------------------------------
//   ĐỌC struct AirData ............... DataStructures.hpp (READ-ONLY; trường §11).
//   Hằng số cấu hình ................. config.hpp (namespace Cfg; bổ sung §10).
//   Driver LCD ....................... hd44780 (esp-idf-lib) — chuỗi init + giao
//                                       thức 4-bit + timing. Lớp I/O expander:
//                                       pcf8574 (esp-idf-lib). Cả hai xây trên i2cdev.
//   Bus I2C + mutex per-port ......... i2cdev (esp-idf-lib) — KHỞI TẠO bởi
//                                       SensorManager::init() (SensorManager.hpp,
//                                       khối "I2C BUS"). DisplayManager chỉ đăng ký
//                                       device PCF8574 (pcf8574_init_desc) trên cùng
//                                       port (§3) — KHÔNG khởi tạo lại bus/i2cdev.
//   Cờ readiness (warmup) ............ do SensorManager::readAll() set —
//                                       SensorManager.hpp; tham khảo isFullyReady().
//   AQI/category/TVOC/Comfort/calib ... do DataFusion::process() ghi —
//                                       DataFusion.hpp (§3,§4,§5,§6). Display chỉ ĐỌC.
//   Thông báo mạng/SD (showMessage) .. do main.cpp / task mạng gọi khi
//                                       NetworkManager::isConnected()==false hoặc
//                                       StorageHelper báo lỗi (NetworkManager.hpp).
//   Logging .......................... ESP_LOGI/W/E ("esp_log.h") — KHÔNG Serial.print,
//                                       KHÔNG xoá log khi commit (CLAUDE.md §4, §6.5).
//
//   #include dự kiến khi triển khai header:
//       "DataStructures.hpp", "config.hpp",
//       "hd44780.h", "pcf8574.h"  (esp-idf-lib — driver LCD + I/O expander),
//       "i2cdev.h"  (i2c_dev_t — cùng họ với SensorManager),
//       "esp_err.h", "freertos/FreeRTOS.h", "freertos/task.h" (nếu sở hữu task).
//   (CMakeLists của main phải thêm "hd44780", "pcf8574" vào REQUIRES/PRIV_REQUIRES,
//    cùng "i2cdev" như SensorManager đã yêu cầu.)
//   (KHÔNG kéo NetworkManager.hpp/DataFusion.hpp vào để tránh phụ thuộc vòng —
//    mọi tương tác đi qua struct AirData + dây nối ở main.cpp.)
//
// ============================================================================
// 10. HẰNG SỐ CẦN BỔ SUNG VÀO config.hpp (namespace Cfg) — KHÔNG hardcode .cpp
// ----------------------------------------------------------------------------
//   ĐÃ CÓ (tái dùng): PCF8574_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ,
//     DISPLAY_UPDATE_INTERVAL_MS, LCD_MIN_INTERVAL_MS, LCD_MAX_INTERVAL_MS,
//     TASK_STACK_DISPLAY_WORDS, TASK_PRIO_DISPLAY,
//     COMFORT_DI_* (sẽ thêm bởi DataFusion — dùng để diễn giải dải Comfort §7.2).
//
//   CẦN THÊM (đề xuất — đặt trong mục riêng "LCD / DISPLAY" của config.hpp):
//     §A Hình học LCD:
//       - LCD_COLS = 16, LCD_ROWS = 2
//       (Quy đổi toạ độ→DDRAM do driver hd44780 lo; KHÔNG cần LCD_ROW_OFFSET.)
//     §B Vị trí BIT của LCD trên PCF8574 — nạp vào hd44780_t.pins (§1, §5.2):
//       - LCD_BIT_RS = 0, LCD_BIT_RW = 1, LCD_BIT_E = 2, LCD_BIT_BL = 3,
//         LCD_BIT_D4 = 4, LCD_BIT_D5 = 5, LCD_BIT_D6 = 6, LCD_BIT_D7 = 7
//       (Đây là SỐ THỨ TỰ BIT, không phải mặt nạ — khớp ánh xạ chân CLAUDE.md §5.)
//       (KHÔNG còn LCD_CMD_* hay LCD_E_PULSE_US: mã lệnh/timing 4-bit do driver lo.)
//     §C Hành vi hiển thị:
//       - LCD_PAGE_ROTATE_MS  (chu kỳ luân phiên trang NORMAL — nằm trong
//         [LCD_MIN_INTERVAL_MS, LCD_MAX_INTERVAL_MS] = 2..5 s, CLAUDE.md §3)
//       - LCD_CALIB_BLINK_MS  (nhịp nhấp nháy cảnh báo calib — §7.4)
//       - LCD_PAGE_COUNT      (= số phần tử enum ScreenPage)
//     §D Nhãn hiển thị (không dấu, ≤ chiều rộng cột):
//       - Bảng nhãn AQI category[6] = {"Tot","Trung binh","Kem","Xau","Rat xau","Nguy hai"}
//       - Bảng nhãn Comfort theo dải DI (tham chiếu DataFusion.hpp §5.2)
//   (Các bitmap CGRAM cho '°','µ' đặt static const trong DisplayManager.cpp —
//    dữ liệu glyph nội bộ module, không phải tham số cấu hình.)
//
// ============================================================================
// 11. TRƯỜNG AirData ĐỌC & QUY TẮC READINESS (DataStructures.hpp) — CHỈ ĐỌC
// ----------------------------------------------------------------------------
//   ĐỌC : temperature, humidity, pressure, gas_resistance(*), pm1_0, pm2_5, pm10,
//          co2_ppm, tvoc_ppm, aqi, aqi_category, comfort_index,
//          calib_needed, bme680_ready, pms5003_ready, mq135_ready, sensors_ready,
//          data_valid, timestamp.
//   GHI : (KHÔNG GHI — DisplayManager là module read-only trên AirData).
//
//   QUY TẮC: với mỗi kênh, CHỈ hiển thị giá trị số khi cờ *_ready tương ứng = true
//   (bme680_ready cho T/RH/P/TVOC/Comfort; pms5003_ready cho PM/AQI; mq135_ready
//   cho CO2). Khi chưa ready → in "--" thay vì 0/NAN (§7.2). Khi data_valid=false
//   → KHÔNG vẽ số mới (giữ màn hình cũ hoặc báo lỗi). Đồng nhất triết lý "không
//   feed giá trị warmup" của Filters/DataFusion/SensorManager.
//
// ============================================================================
// 12. RÀNG BUỘC PHI CHỨC NĂNG & THREADING (CLAUDE.md §3, §4)
// ----------------------------------------------------------------------------
//   - TASK riêng: main.cpp tạo taskDisplay (stack Cfg::TASK_STACK_DISPLAY_WORDS,
//     prio Cfg::TASK_PRIO_DISPLAY — thấp hơn Sensor/Network). Task nhận AirData
//     mới nhất qua hàng đợi/shared+mutex từ task xử lý, KHÔNG tự đọc cảm biến.
//   - NON-BLOCKING (CLAUDE.md §4): nhịp cập nhật/luân phiên trang điều khiển bằng
//     xQueueReceive(timeout) HOẶC esp_timer — KHÔNG dùng vTaskDelay() làm logic
//     timing nghiệp vụ. Cập nhật LCD mỗi 2–5 s (Cfg::LCD_MIN/MAX_INTERVAL_MS).
//   - 300 ms (CLAUDE.md §3): nhờ dirty-check (§6.3), một lần update() điển hình chỉ
//     phát vài transaction I2C, không chiếm bus lâu gây trễ chu kỳ đọc BME680.
//   - Năng lượng ≤ 2.0W (CLAUDE.md §3): hỗ trợ tắt backlight khi nhàn rỗi
//     (setBacklight(false)); không busy-loop khi không có dữ liệu mới.
//   - ĐỒNG BỘ I2C: mọi transaction qua mutex per-port của i2cdev (§3) — an toàn
//     khi task Sensor (BME680) và task Display (PCF8574) chạy song song/đa core.
//   - Stack: kiểm tra uxTaskGetStackHighWaterMark() cho taskDisplay trước release
//     (CLAUDE.md §6.5) — đặc biệt do snprintf dùng stack.
//
// ============================================================================
// 13. TIMING PHẦN CỨNG LCD vs. QUY TẮC NON-BLOCKING (làm rõ)
// ----------------------------------------------------------------------------
//   Độ trễ phần cứng HD44780 (xung E sub-µs, lệnh thường ~37–50µs, Clear/Home
//   ~1.5ms) do DRIVER hd44780 quản lý NỘI BỘ — DisplayManager không tự xử lý.
//   Các trễ này là timing phần cứng của thiết bị, KHÔNG phải logic timing nghiệp
//   vụ (giống ngoại lệ BME680 measurement duration trong SensorManager.hpp).
//   Lưu ý ở mức ứng dụng:
//     - TRÁNH hd44780_clear() mỗi chu kỳ (kéo theo trễ ~1.5ms + xoá toàn màn hình);
//       chỉ clear ở init hoặc khi đổi hẳn DisplayState (§6.3, §7).
//     - Tuyệt đối KHÔNG dùng vTaskDelay() để định nhịp luân phiên trang hay chu kỳ
//       cập nhật (đó là việc của queue/esp_timer — §12).
//
// ============================================================================
// 14. LOGGING / DEBUG (CLAUDE.md §4, §6)
// ----------------------------------------------------------------------------
//   - Dùng TAG riêng (ví dụ "DisplayManager") với ESP_LOGI/W/E.
//   - ESP_LOGE khi hd44780_init / pcf8574_port_write trả lỗi (PCF8574 không ACK →
//     kiểm tra dây/địa chỉ 0x20/mapping bit).
//   - ESP_LOGI khi init() (pcf8574_init_desc + hd44780_init) OK; ESP_LOGW khi data_valid=false.
//   - KHÔNG xoá/comment ESP_LOG* khi commit (nguồn debug song song JTAG — §6.5).
//   - Biến nên theo dõi qua JTAG: ScreenPage hiện tại, DisplayState, nội dung
//     shadow framebuffer, mã lỗi i2c gần nhất.
// ============================================================================

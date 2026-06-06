#pragma once

// ============================================================================
// DataFusion.hpp — ĐẶC TẢ MODULE HỢP NHẤT DỮ LIỆU & SUY DIỄN CHỈ SỐ
// Dự án ENV-AQ-01 (CLAUDE.md §2, §3, §4)
//
// LƯU Ý: File này hiện chỉ chứa ĐẶC TẢ (comment) — chưa hiện thực hoá code.
//        DataFusion.cpp đã có sẵn phần đặc tả ở đầu file (Drift/NVS) — GIỮ NGUYÊN.
//        Khi triển khai, header này sẽ chứa: enum AqiCategory/AlertLevel,
//        class DataFusion với các API mô tả ở §1, và #include tương ứng (§9).
//
// ============================================================================
// 0. VỊ TRÍ TRONG LUỒNG DỮ LIỆU (CLAUDE.md §2)
// ----------------------------------------------------------------------------
//   SensorManager::readAll()  (đọc thô + set *_ready)
//        → Filters::process()  (làm sạch in-place: sanity + EMA/SMA)
//        → DataFusion::process()  ← MODULE NÀY (suy diễn AQI/TVOC/Comfort + Drift)
//        → DisplayManager (LCD)  +  NetworkManager (MQTT)  +  StorageHelper (SD)
//
//   DataFusion là tầng "quyết định": nhận AirData ĐÃ LỌC, ghi các trường suy diễn
//   trở lại CHÍNH AirData đó (in-place, không cấp phát struct mới — giống Filters).
//
//   TRÁCH NHIỆM (CLAUDE.md §2 bảng module):
//     - Hợp nhất dữ liệu 3 cảm biến thành bộ chỉ số nhất quán.
//     - Tính AQI theo tiêu chuẩn Việt Nam (+ aqi_category).
//     - Tính TVOC (ppm) từ điện trở gas BME680.
//     - Tính Comfort Index từ nhiệt độ/độ ẩm.
//     - Drift Self-Check so với baseline NVS (CLAUDE.md §3, §4).
//     - Quyết định mức cảnh báo (AlertLevel) cho LED/Buzzer (xem §8).
//
//   RANH GIỚI (KHÔNG làm — tránh chồng lấn module khác):
//     - KHÔNG đọc cảm biến, KHÔNG I2C/UART/ADC (đó là SensorManager).
//     - KHÔNG lọc nhiễu EMA/SMA/sanity (đó là Filters — đã chạy trước).
//     - KHÔNG bù T/RH cho MQ-135 (SensorManager đã làm trong mq135ReadPpm()).
//     - KHÔNG tự publish MQTT, KHÔNG tự điều khiển GPIO LED/Buzzer, KHÔNG ghi SD.
//       → DataFusion chỉ TÍNH và GHI CỜ vào AirData; NetworkManager/main/Display
//         đọc cờ và thực thi I/O (giữ DataFusion thuần tính toán, non-blocking).
//
// ============================================================================
// 1. API CÔNG KHAI CẦN CÓ (mô tả — sẽ khai báo thành method của class DataFusion)
// ----------------------------------------------------------------------------
//   Tạo đúng 1 instance DataFusion trong main.cpp, dùng suốt vòng đời (như Filters).
//   Copy/assign = delete (giữ trạng thái baseline/NVS là duy nhất).
//
//   esp_err_t init();
//     - Gọi 1 lần trong app_main() SAU nvs_flash_init() (main.cpp đã gọi sẵn — §0).
//     - Mở NVS namespace Cfg::NVS_NAMESPACE (nvs_open, READWRITE).
//     - Nạp baseline hiệu chuẩn từ NVS: Cfg::NVS_KEY_BL_TEMP / BL_HUMI / BL_PM25 /
//       BL_CO2 và Cfg::NVS_KEY_LAST_CALIB_TS (xem §6 — vòng đời baseline).
//     - Nếu CHƯA có baseline (lần đầu nạp firmware) → đánh dấu "chưa có baseline",
//       chờ mẫu ổn định đầu tiên để chốt baseline (KHÔNG ghi rác warmup).
//     - Trả ESP_OK / mã lỗi NVS; KHÔNG block, KHÔNG chờ mạng.
//
//   void process(AirData &data);
//     - GỌI MỖI CHU KỲ, ngay sau Filters::process(data), trong cùng task xử lý.
//     - Trình tự bắt buộc (đọc kỹ §11 — tôn trọng cờ *_ready):
//         (1) Kiểm tra data.data_valid; nếu false → bỏ qua suy diễn, để output ở
//             giá trị sentinel an toàn (xem §10) và return sớm.
//         (2) computeTvoc(data)     — chỉ khi data.bme680_ready (cần gas_resistance).
//         (3) computeAqi(data)      — chỉ khi data.pms5003_ready (cần pm2_5/pm10).
//         (4) computeComfort(data)  — chỉ khi data.bme680_ready (cần T/RH).
//         (5) driftSelfCheck(data)  — chỉ trên các kênh có cờ ready tương ứng.
//         (6) computeAlertLevel(data) — tổng hợp ngưỡng + category (§8).
//     - Toàn bộ O(1), thuần số học, KHÔNG vòng lặp dài, KHÔNG I/O → đảm bảo NFR
//       tổng chu kỳ ≤ Cfg::MAX_CYCLE_TIME_MS (300 ms, CLAUDE.md §3).
//
//   void confirmRecalibration(AirData &data);
//     - Người dùng XÁC NHẬN tái hiệu chuẩn (qua lệnh MQTT "confirm_calib").
//     - Được gọi từ command-callback đăng ký trong main.cpp (xem §6.4, §9) —
//       KHÔNG gọi từ NetworkManager (CLAUDE.md §2 ghi chú dòng "setCommandCallback").
//     - Ghi baseline MỚI = giá trị ổn định hiện tại; cập nhật last_calib_timestamp
//       (cách lấy thời gian: §7); nvs_set_* + nvs_commit() (CLAUDE.md §4 Data
//       Persistence); xoá cờ data.calib_needed.
//     - Đây là ĐƯỜNG DUY NHẤT được ghi đè baseline (KHÔNG bao giờ tự động — §6.3).
//
//   AlertLevel getAlertLevel() const;   // mức cảnh báo của chu kỳ gần nhất (§8)
//   AqiCategory lastCategory() const;   // tiện cho Display/JTAG; không tác dụng phụ
//   bool getCalibState(...) const;      // soi baseline + tuổi calib (debug/JTAG)
//
// ============================================================================
// 2. ENUM CẦN KHAI BÁO (trong header này khi triển khai)
// ----------------------------------------------------------------------------
//   enum class AqiCategory : uint8_t  — ÁNH XẠ TRỰC TIẾP sang AirData::aqi_category:
//       0 = Tốt        (Good)        → LED XANH
//       1 = Trung bình (Moderate)    → LED XANH
//       2 = Kém        (Poor)        → LED VÀNG
//       3 = Xấu        (Bad)         → LED VÀNG/ĐỎ
//       4 = Rất xấu    (Very bad)    → LED ĐỎ
//       5 = Nguy hại   (Hazardous)   → LED ĐỎ + Buzzer
//     (Khớp đúng chú thích AirData::aqi_category trong DataStructures.hpp.)
//
//   enum class AlertLevel : uint8_t { NONE, WARNING, CRITICAL };
//     - NONE     : trong ngưỡng an toàn.
//     - WARNING  : vượt 1 ngưỡng (LED vàng, chưa buzzer).
//     - CRITICAL : AQI category ≥ 4 HOẶC CO2/PM2.5 vượt ngưỡng cứng (LED đỏ + buzzer).
//     - Dùng cho main.cpp lái GPIO trong ≤ Cfg::ALERT_MAX_LATENCY_MS (3 s, §8/§3).
//
// ============================================================================
// 3. THUẬT TOÁN — AQI THEO TIÊU CHUẨN VIỆT NAM (QĐ 1459/QĐ-TCMT 2019)
// ----------------------------------------------------------------------------
//   ĐẦU VÀO: data.pm2_5, data.pm10 (µg/m³, ĐÃ qua Filters), chỉ khi pms5003_ready.
//
//   3.1 Sub-index theo phép NỘI SUY TUYẾN TÍNH giữa các điểm gãy (breakpoint):
//         AQI_x = (I_hi − I_lo)/(BP_hi − BP_lo) · (C_x − BP_lo) + I_lo
//       với (BP_lo, BP_hi) là cặp điểm gãy NỒNG ĐỘ bao quanh C_x, và
//           (I_lo, I_hi)   là cặp điểm gãy CHỈ SỐ AQI tương ứng.
//
//   3.2 Bảng điểm gãy CHỈ SỐ (chung mọi chất):  {0, 50, 100, 150, 200, 300, 500}.
//       Bảng điểm gãy NỒNG ĐỘ (µg/m³) theo VN:
//         PM2.5 (trung bình): {0, 25, 50, 80, 150, 250, 500}
//         PM10  (trung bình): {0, 50, 150, 250, 350, 420, 600}
//       → CÁC MẢNG NÀY PHẢI ĐƯỢC KHAI BÁO TRONG config.hpp (xem §10); KHÔNG
//         hardcode trong DataFusion.cpp (CLAUDE.md §4 "Cấu hình dự án").
//
//   3.3 AQI tổng = MAX của các sub-index khả dụng (AQI_pm25, AQI_pm10).
//       → ghi vào data.aqi (float).
//
//   3.4 aqi_category suy ra TỪ GIÁ TRỊ data.aqi theo dải chỉ số:
//         0–50→0, 51–100→1, 101–150→2, 151–200→3, 201–300→4, 301–500→5.
//       → ghi vào data.aqi_category (uint8_t, theo enum AqiCategory §2).
//
//   3.5 Biên/clamp: C_x vượt điểm gãy cao nhất → kẹp AQI = 500 (Nguy hại).
//       C_x âm/không hợp lệ đã bị Filters loại; nếu vẫn gặp → coi như chưa ready.
//
//   GHI CHÚ: Các hằng Cfg::AQI_GOOD_MAX/MODERATE_MAX/... hiện có (config.hpp §7)
//   là ngưỡng PHÂN LOẠI NHANH theo nồng độ PM2.5 — có thể dùng cho đường tắt
//   category, nhưng AQI số (data.aqi) BẮT BUỘC tính bằng nội suy §3.1 để đạt
//   sai số lặp ≤ 10% (CLAUDE.md §3 — chỉ tiêu nghiệm thu AQI).
//
// ============================================================================
// 4. THUẬT TOÁN — TVOC (ppm) TỪ ĐIỆN TRỞ GAS BME680
// ----------------------------------------------------------------------------
//   ĐẦU VÀO: data.gas_resistance (Ω, ĐÃ qua EMA của Filters), data.humidity;
//            chỉ tính khi data.bme680_ready (heater ổn định + gas hợp lệ).
//
//   4.1 Baseline điện trở khí sạch R0_gas:
//       - BME680 KHÔNG cho TVOC tuyệt đối — phải quy chiếu về điện trở khí sạch.
//       - R0_gas = điện trở gas trong môi trường sạch, ước lượng bằng cận-trên
//         trượt (running max/percentile cao) của gas_resistance khi không khí
//         tốt. Lưu/đồng bộ cùng cơ chế baseline NVS (Cfg::NVS_KEY_BL_* — có thể
//         CẦN THÊM key cho gas, xem §10) để sống sót qua reboot.
//
//   4.2 Tỉ số khí: ratio = R0_gas / Rs_gas (ratio > 1 ⇒ có VOC, Rs giảm khi VOC↑).
//
//   4.3 Quy đổi sang TVOC ppm theo luật mũ/log (giống họ MQ): chọn 1 trong:
//         tvoc_ppm = A_tvoc · ratio ^ B_tvoc           (power-law), HOẶC
//         tvoc_ppm = scale · ln(ratio) (+ offset)      (log-linear).
//       → Hằng A_tvoc/B_tvoc (hoặc scale/offset) KHAI BÁO TRONG config.hpp (§10).
//
//   4.4 Bù độ ẩm: độ ẩm cao làm Rs giảm giả → đưa Rs về RH tham chiếu trước khi
//       lấy ratio (tái dùng tinh thần hệ số bù GeorgK đã có cho MQ-135). Hệ số bù
//       (nếu khác MQ-135) khai báo ở config.hpp.
//
//   4.5 Clamp tvoc_ppm ≥ 0; ghi vào data.tvoc_ppm.
//       NFR: sai số lặp TVOC ≤ 10% (CLAUDE.md §3) — dùng gas đã EMA + R0 ổn định.
//
// ============================================================================
// 5. THUẬT TOÁN — COMFORT INDEX (từ Nhiệt độ / Độ ẩm)
// ----------------------------------------------------------------------------
//   ĐẦU VÀO: data.temperature (°C), data.humidity (%RH); khi data.bme680_ready.
//
//   5.1 Khuyến nghị chỉ số bất tiện nhiệt-ẩm Thom (Discomfort Index, °C):
//         DI = T − 0.55·(1 − 0.01·RH)·(T − 14.5)
//       → ghi vào data.comfort_index (float). (Có thể chuẩn hoá về thang 0–100
//         nếu Display cần — quyết định ở DisplayManager, KHÔNG ở đây.)
//
//   5.2 Diễn giải dải DI (để Display/Alert dùng, hằng ngưỡng ở config.hpp §10):
//         DI < 21  : dễ chịu | 21–24 : hơi nóng | 24–27 : nóng khó chịu
//         27–29 : rất khó chịu | ≥ 29 : nguy cơ stress nhiệt.
//
//   5.3 Hệ số 0.55 / 14.5 (và các ngưỡng dải) KHAI BÁO TRONG config.hpp (§10) —
//       không hardcode (CLAUDE.md §4). NFR: sai số lặp Comfort ≤ 10% (CLAUDE.md §3).
//
// ============================================================================
// 6. DRIFT SELF-CHECK + VÒNG ĐỜI BASELINE NVS (CLAUDE.md §3, §4 — BẮT BUỘC)
// ----------------------------------------------------------------------------
//   Hằng dùng: Cfg::DRIFT_THRESHOLD_PCT (10%), Cfg::CALIB_INTERVAL_SEC (30 ngày).
//   Key NVS:   Cfg::NVS_NAMESPACE + NVS_KEY_BL_TEMP/BL_HUMI/BL_PM25/BL_CO2/
//              NVS_KEY_LAST_CALIB_TS (config.hpp §12).
//
//   6.1 So lệch mỗi chu kỳ (chỉ trên kênh có cờ ready):
//         dev% = |current − baseline| / |baseline| · 100
//       Tính cho T, RH, PM2.5, CO2. Nếu BẤT KỲ kênh nào dev% > DRIFT_THRESHOLD_PCT
//       → set data.calib_needed = true (kèm lý do, ví dụ "CALIB_DRIFT_TEMP").
//
//   6.2 Quá hạn thời gian: now − last_calib_timestamp > CALIB_INTERVAL_SEC
//       → set data.calib_needed = true (lý do "CALIB_OVERDUE_30D").
//       (Lấy 'now' theo §7.)
//
//   6.3 KHÔNG tự bù trừ, KHÔNG tự ghi đè baseline (CLAUDE.md §3, §4): khi
//       calib_needed=true chỉ PHÁT CỜ + để hệ thống cảnh báo; chờ người dùng.
//
//   6.4 Xác nhận tái hiệu chuẩn (confirmRecalibration — §1):
//       - Kích hoạt khi nhận lệnh MQTT "confirm_calib" (NetworkManager nhận →
//         command-callback do main.cpp đăng ký gọi DataFusion::confirmRecalibration).
//       - Ghi baseline mới = giá trị ổn định hiện tại; last_calib_timestamp = now;
//         nvs_set_* + nvs_commit(); clear calib_needed.
//
//   6.5 Khởi tạo baseline lần đầu (init không thấy key NVS): chốt baseline từ MẪU
//       ỔN ĐỊNH ĐẦU TIÊN (tất cả *_ready và đã qua warmup) — coi như hiệu chuẩn
//       gốc tại hiện trường; ghi NVS + timestamp. KHÔNG dùng mẫu warmup làm baseline.
//
//   6.6 Phát cảnh báo MQTT: DataFusion KHÔNG tự publish. Khi data.calib_needed=true,
//       NetworkManager::buildJson() chèn field "calib_alert": true (đối số
//       alert_reason), và task mạng gọi NetworkManager::publishAlert() (xem §9).
//       Đáp ứng NFR đẩy cảnh báo ≤ Cfg::ALERT_MAX_LATENCY_MS (3 s, CLAUDE.md §3).
//
// ============================================================================
// 7. NGUỒN THỜI GIAN (đồng bộ với đặc tả sẵn có trong DataFusion.cpp)
// ----------------------------------------------------------------------------
//   Lấy 'now' (Unix giây) khi so hạn 30 ngày / khi ghi last_calib_timestamp:
//     - Nếu NetworkManager::isTimeSynced() == true (NetworkManager.hpp):
//           now = time(NULL);                         // Unix UTC thật (qua SNTP)
//     - Ngược lại:
//           now = esp_timer_get_time() / 1000000LL;   // giây-từ-boot (tạm thời)
//   (Trùng khớp khối comment đầu DataFusion.cpp — GIỮ NGUYÊN file .cpp đó.)
//
// ============================================================================
// 8. QUYẾT ĐỊNH CẢNH BÁO LED / BUZZER (computeAlertLevel)
// ----------------------------------------------------------------------------
//   ĐẦU VÀO: data.aqi_category (§3.4), data.co2_ppm, data.pm2_5.
//   Hằng ngưỡng: Cfg::ALERT_CO2_PPM, Cfg::ALERT_PM25_UGM3 (config.hpp §8).
//   Quy tắc tổng hợp:
//     - aqi_category ≥ 4  → CRITICAL.
//     - co2_ppm > ALERT_CO2_PPM  HOẶC  pm2_5 > ALERT_PM25_UGM3 → ≥ WARNING
//       (nâng lên CRITICAL nếu vượt biên xa / category 5).
//     - còn lại → NONE.
//   GHI: kết quả lưu nội bộ (getAlertLevel) — DataFusion KHÔNG chạm GPIO.
//   main.cpp sở hữu gpio_config() (GPIO25/26/27 LED, GPIO32 Buzzer — CLAUDE.md §5)
//   và lái output theo AlertLevel + aqi_category trong ≤ ALERT_MAX_LATENCY_MS (3 s).
//   Chỉ ánh xạ category→LED khi data.sensors_ready để tránh báo động giả lúc warmup.
//
// ============================================================================
// 9. LIÊN KẾT MODULE — HÀM/DỮ LIỆU DÙNG TỪ FILE NÀO
// ----------------------------------------------------------------------------
//   ĐỌC/GHI struct AirData ............ DataStructures.hpp (in-place; trường §11).
//   Hằng số cấu hình .................. config.hpp (namespace Cfg; bổ sung §10).
//   Tiền xử lý đầu vào ................ Filters::process()   — Filters.hpp
//                                        (DataFusion CHẠY SAU, giả định đã sạch).
//   Cờ readiness/raw .................. do SensorManager::readAll() set — SensorManager.hpp.
//   Đồng bộ thời gian ................. NetworkManager::isTimeSynced() — NetworkManager.hpp (§7).
//   Phát cảnh báo/calib_alert ......... NetworkManager::publishAlert() / buildJson()
//                                        — NetworkManager.hpp; GỌI TỪ task mạng,
//                                        KHÔNG từ DataFusion (§6.6).
//   Đăng ký lệnh "confirm_calib" ...... NetworkManager::setCommandCallback() —
//                                        GỌI TRONG main.cpp (CLAUDE.md §2 ghi chú);
//                                        callback → DataFusion::confirmRecalibration().
//   NVS (baseline + timestamp) ........ nvs_open/nvs_get_*/nvs_set_*/nvs_commit
//                                        ("nvs_flash.h"); nvs_flash_init() ĐÃ gọi ở
//                                        app_main() trong main.cpp.
//   Thời gian thực .................... time() ("time.h") / esp_timer_get_time()
//                                        ("esp_timer.h") — §7.
//   Logging .......................... ESP_LOGI/W/E ("esp_log.h") — KHÔNG Serial.print,
//                                        KHÔNG xoá log khi commit (CLAUDE.md §4, §6.5).
//   Hiển thị/Lưu trữ hạ nguồn ......... DisplayManager / StorageHelper đọc các trường
//                                        DataFusion ghi (chỉ đọc; chưa hiện thực).
//
//   #include dự kiến khi triển khai header: "DataStructures.hpp", "config.hpp".
//   (Tránh kéo NetworkManager.hpp vào để không tạo phụ thuộc vòng — tương tác mạng
//    đi qua cờ AirData + dây nối ở main.cpp.)
//
// ============================================================================
// 10. HẰNG SỐ CẦN BỔ SUNG VÀO config.hpp (namespace Cfg) — KHÔNG hardcode .cpp
// ----------------------------------------------------------------------------
//   ĐÃ CÓ (tái dùng): AQI_GOOD_MAX..AQI_VERY_BAD_MAX, ALERT_CO2_PPM, ALERT_PM25_UGM3,
//     DRIFT_THRESHOLD_PCT, CALIB_INTERVAL_SEC, NVS_KEY_BL_*, NVS_KEY_LAST_CALIB_TS,
//     ACCURACY_INDEX_PCT, MAX_CYCLE_TIME_MS, ALERT_MAX_LATENCY_MS.
//
//   CẦN THÊM (đề xuất, đặt trong các mục tương ứng của config.hpp):
//     §7 AQI:
//       - AQI_INDEX_BP[]  = {0,50,100,150,200,300,500}      (điểm gãy chỉ số)
//       - AQI_PM25_BP[]   = {0,25,50,80,150,250,500}        (nồng độ PM2.5 µg/m³)
//       - AQI_PM10_BP[]   = {0,50,150,250,350,420,600}      (nồng độ PM10 µg/m³)
//       - AQI_MAX_INDEX   = 500                              (clamp trần)
//     §2b/§4 TVOC:
//       - TVOC_CURVE_A, TVOC_CURVE_B   (hoặc TVOC_LOG_SCALE/OFFSET)
//       - TVOC_GAS_R0_DEFAULT_OHM      (R0_gas mặc định khi NVS trống)
//       - NVS_KEY_BL_GAS               (key NVS lưu R0_gas — nếu lưu lâu dài §4.1)
//       - (tuỳ chọn) hệ số bù RH cho gas nếu khác MQ-135.
//     §5 Comfort:
//       - COMFORT_DI_K1 = 0.55f, COMFORT_DI_K2 = 14.5f
//       - COMFORT_DI_OK / WARM / HOT / SEVERE  (ngưỡng dải §5.2)
//   (Giá trị curve TVOC và R0 cần HIỆU CHUẨN thực tế — ghi rõ trong comment config.)
//
// ============================================================================
// 11. TRƯỜNG AirData ĐỌC/GHI & QUY TẮC READINESS (DataStructures.hpp)
// ----------------------------------------------------------------------------
//   ĐỌC : temperature, humidity, pressure, gas_resistance, pm2_5, pm10, co2_ppm,
//          bme680_ready, pms5003_ready, mq135_ready, sensors_ready, data_valid, timestamp.
//   GHI : aqi, aqi_category, comfort_index, tvoc_ppm, calib_needed,
//          last_calib_timestamp (CHỈ trong confirmRecalibration / init lần đầu).
//
//   QUY TẮC: mỗi chỉ số chỉ tính khi cảm biến nguồn đã ready (§1 process). Khi chưa
//   ready → giữ output ở SENTINEL an toàn (khuyến nghị NAN cho float, category=0,
//   calib_needed KHÔNG bật do warmup) để Display/Network biết "chưa hợp lệ" và
//   KHÔNG kích cảnh báo giả. KHÔNG bao giờ feed giá trị warmup vào AQI/Drift/Alert
//   (đồng nhất triết lý Filters/SensorManager).
//
// ============================================================================
// 12. RÀNG BUỘC PHI CHỨC NĂNG & THREADING (CLAUDE.md §3, §4)
// ----------------------------------------------------------------------------
//   - NON-BLOCKING: process() thuần tính toán O(1); KHÔNG vTaskDelay, KHÔNG chờ I/O.
//     Định kỳ điều phối bằng task + queue/esp_timer ở main.cpp (KHÔNG ở đây).
//   - 300 ms: phần DataFusion phải nhỏ so với ngân sách chu kỳ tổng.
//   - NVS chỉ ghi khi confirmRecalibration (hiếm) → không ảnh hưởng chu kỳ thường.
//   - ĐỒNG BỘ: nếu confirmRecalibration (chạy trong context callback MQTT/task mạng)
//     và process() (task xử lý) có thể chạm baseline/NVS đồng thời → BẢO VỆ baseline
//     dùng cấp/khoá phù hợp (SemaphoreHandle_t mutex, hoặc đẩy lệnh qua queue về
//     task xử lý). Tránh đọc/ghi baseline đua dữ liệu giữa 2 core.
//   - Stack: nếu DataFusion chạy trong task riêng, kiểm tra uxTaskGetStackHighWaterMark()
//     trước release (CLAUDE.md §6.5).
//
// ============================================================================
// 13. LOGGING / DEBUG (CLAUDE.md §4, §6)
// ----------------------------------------------------------------------------
//   - Dùng TAG riêng (ví dụ "DataFusion") với ESP_LOGI/W/E.
//   - ESP_LOGW khi phát hiện drift hoặc quá hạn calib; ESP_LOGI khi chốt baseline mới.
//   - KHÔNG xoá/comment ESP_LOG* khi commit (nguồn debug song song JTAG — §6.5).
//   - Các biến nên theo dõi qua JTAG: data.aqi, data.tvoc_ppm, baseline NVS, dev%,
//     AlertLevel — phục vụ nghiệm thu sai số ≤ 10% (CLAUDE.md §3).
// ============================================================================

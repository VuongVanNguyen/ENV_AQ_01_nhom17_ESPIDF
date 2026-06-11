#pragma once

#include "DataStructures.hpp"
#include "config.hpp"

#include "esp_err.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <cstdint>
#include <cstddef>
#include <cstring>

class DataFusion {
public:
    enum class AqiCategory : uint8_t {
        GOOD       = 0,
        MODERATE   = 1,
        POOR       = 2,
        BAD        = 3,
        VERY_BAD   = 4,
        HAZARDOUS  = 5,
    };

    enum class AlertLevel : uint8_t {
        NONE     = 0,
        WARNING  = 1,
        CRITICAL = 2,
    };

    DataFusion();
    ~DataFusion();

    DataFusion(const DataFusion&)            = delete;
    DataFusion& operator=(const DataFusion&) = delete;

    esp_err_t init();
    void process(AirData &data, bool time_synced = false);
    esp_err_t confirmRecalibration(AirData &data, bool time_synced = false);

    AlertLevel getAlertLevel() const;
    AqiCategory lastCategory() const;
    bool hasBaseline() const;
    int64_t lastCalibrationTimestamp() const;
    const char *calibReason() const;

private:
    struct Baseline {
        float temperature;
        float humidity;
        float pm25;
        float co2;
        float gas_resistance;
    } baseline_;

    int64_t last_calib_ts_;
    bool has_baseline_;
    AlertLevel last_alert_level_;
    AqiCategory last_category_;
    char last_calib_reason_[32];

    nvs_handle_t nvs_handle_;
    mutable SemaphoreHandle_t mutex_;

    static constexpr const char *kNoCalibReason = "NONE";

    void setSafeSentinel(AirData &data) const;
    void setReason(const char *reason);
    int64_t getNow(bool time_synced) const;

    esp_err_t loadBaseline();
    esp_err_t saveBaseline(int64_t timestamp);

    void initializeBaselineFromData(const AirData &data, bool time_synced);
    void computeAqi(AirData &data);
    void computeComfort(AirData &data) const;
    void driftSelfCheck(AirData &data, bool time_synced);
    void computeAlertLevel(AirData &data);
    float computeAqiSubindex(float concentration, const float *breakpoints) const;
    bool hasFiniteValue(float value) const;
};
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
//   GHI CHÚ: Các hằng Cfg::AQI_GOOD_MAX/MODERATE_MAX/... (config.hpp §7) là
//   ngưỡng NỒNG ĐỘ PM2.5 (µg/m³) — KHÔNG dùng để suy ra aqi_category (khác đơn
//   vị với data.aqi). aqi_category PHẢI dùng Cfg::AQI_CAT_BP[] (config.hpp §7,
//   dải CHỈ SỐ 0–500) như mô tả ở §3.4. AQI số (data.aqi) BẮT BUỘC tính bằng
//   nội suy §3.1 để đạt sai số lặp ≤ 10% (CLAUDE.md §3 — chỉ tiêu nghiệm thu AQI).
//
// ============================================================================
// 5. THUẬT TOÁN — COMFORT INDEX (từ Nhiệt độ / Độ ẩm)
// ----------------------------------------------------------------------------
//   ĐẦU VÀO: data.temperature (°C), data.humidity (%RH); khi data.bme680_ready.
//
//   5.1 Chỉ số nhiệt-ẩm THI (Temperature-Humidity Index, °C):
//         THI = T − 0.55·(1 − 0.01·RH)·(T − 14.5)
//       → ghi vào data.comfort_index (float). (Có thể chuẩn hoá về thang 0–100
//         nếu Display cần — quyết định ở DisplayManager, KHÔNG ở đây.)
//
//   5.2 Diễn giải dải DI (để Display/Alert dùng, hằng ngưỡng ở config.hpp §14):
//         DI < 21  : dễ chịu | 21–24 : hơi nóng | 24–27 : nóng khó chịu
//         27–29 : rất khó chịu | ≥ 29 : nguy cơ stress nhiệt.
//
//   5.3 Hệ số 0.55 / 14.5 / 0.01 (và các ngưỡng dải) KHAI BÁO TRONG config.hpp
//       (§14) — không hardcode (CLAUDE.md §4). NFR: sai số lặp Comfort ≤ 10%
//       (CLAUDE.md §3).
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
//   Hằng số cấu hình .................. config.hpp (namespace Cfg; §7/§11/§14).
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
// 10. HẰNG SỐ TRONG config.hpp (namespace Cfg) — KHÔNG hardcode .cpp
// ----------------------------------------------------------------------------
//   TÁI DÙNG: AQI_GOOD_MAX..AQI_VERY_BAD_MAX (ngưỡng nồng độ PM2.5 — KHÔNG dùng
//     để phân loại theo data.aqi), ALERT_CO2_PPM, ALERT_PM25_UGM3,
//     DRIFT_THRESHOLD_PCT, CALIB_INTERVAL_SEC, NVS_KEY_BL_*, NVS_KEY_LAST_CALIB_TS,
//     ACCURACY_INDEX_PCT, ACCURACY_TEMP_C, MAX_CYCLE_TIME_MS, ALERT_MAX_LATENCY_MS.
//
//   §7 AQI: AQI_INDEX_BP[], AQI_PM25_BP[], AQI_PM10_BP[] (điểm gãy nội suy §3.1),
//     AQI_CAT_BP[] = {50,100,150,200,300} (phân loại category §3.4),
//     AQI_MAX_INDEX = 500 (clamp trần §3.5).
//   §11 Drift: DRIFT_TEMP_ABS_C = ACCURACY_TEMP_C (ngưỡng tuyệt đối drift T, §6.1).
//   §14 Comfort: COMFORT_DI_K1 = 0.55f, COMFORT_DI_K2 = 14.5f, COMFORT_DI_RH_SCALE
//     = 0.01f, COMFORT_DI_OK/WARM/HOT/SEVERE (ngưỡng dải §5.2).
//
//   Baseline.gas_resistance / NVS_KEY_BL_GAS: GIỮ NGUYÊN (không xóa). Hiện
//     chưa có consumer sau khi bỏ TVOC, nhưng để dành cho gas drift self-check
//     tương lai (so data.gas_resistance vs baseline, như T/RH/PM2.5/CO2). Vô
//     hại, không gây lỗi biên dịch. In/publish gas dùng data.gas_resistance
//     (live, đã có ở NetworkManager), KHÔNG dùng baseline.
//
// ============================================================================
// 11. TRƯỜNG AirData ĐỌC/GHI & QUY TẮC READINESS (DataStructures.hpp)
// ----------------------------------------------------------------------------
//   ĐỌC : temperature, humidity, pressure, gas_resistance, pm2_5, pm10, co2_ppm,
//          bme680_ready, pms5003_ready, mq135_ready, sensors_ready, data_valid, timestamp.
//   GHI : aqi, aqi_category, comfort_index, calib_needed,
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
//   - Các biến nên theo dõi qua JTAG: data.aqi, data.comfort_index, data.co2_ppm,
//     baseline NVS, dev%, AlertLevel — phục vụ nghiệm thu sai số ≤ 10% (CLAUDE.md §3).
//
// ============================================================
// 14. KẾT QUẢ RÀ SOÁT (AUDIT) — ĐÃ FIX (2026-06-11)
//     (checklist chi tiết: main/DataFusion_CHECKLIST.md — tất cả mục ĐÃ XONG)
// ----------------------------------------------------------------------------
//   [AQI-CAT] FIXED: computeAqi() nay phân loại category bằng Cfg::AQI_CAT_BP[]
//     (dải CHỈ SỐ 0–500: 50/100/150/200/300, §3.4) thay vì Cfg::AQI_*_MAX (ngưỡng
//     nồng độ PM2.5 — vẫn giữ nguyên trong config.hpp cho mục đích khác, không
//     dùng để phân loại data.aqi nữa).
//
//   [COMFORT-CONST] FIXED: computeComfort() đọc Cfg::COMFORT_DI_K1/K2/RH_SCALE
//     (config.hpp §14) thay vì hardcode 0.55f/14.5f/0.01f. Ngưỡng dải §5.2
//     (COMFORT_DI_OK/WARM/HOT/SEVERE) cũng đã có trong config.hpp §14 cho
//     DisplayManager.
//
//   [ALERT-STALE] FIXED: process() ở nhánh !data_valid nay reset
//     last_alert_level_ = AlertLevel::NONE trước khi return.
//
//   [ALERT-WARMUP] FIXED: computeAlertLevel() nhánh CRITICAL theo aqi_category
//     (>= VERY_BAD) nay được gate bằng data.sensors_ready (§8); nhánh trùng lặp
//     "aqi_category == HAZARDOUS → CRITICAL" đã gộp vào nhánh trên (HAZARDOUS >=
//     VERY_BAD nên đã được bao quát, tránh vòng kiểm tra ungated thứ hai).
//
//   [DRIFT-CELSIUS] FIXED: nhánh nhiệt độ trong driftSelfCheck() đổi sang sai số
//     tuyệt đối dev_abs = |T - baseline_T| > Cfg::DRIFT_TEMP_ABS_C (= ACCURACY_TEMP_C
//     = 0.5f, config.hpp §11), bỏ guard != 0.0f cho T; RH/PM2.5/CO2 vẫn dùng dev%.
//
//   [PM-FINITE] FIXED: bỏ guard isfinite no-op trên giá trị PM (uint16_t) — PM đã
//     được kiểm tra range bởi Filters.cpp (Cfg::SANITY_PM_MIN/MAX) trước khi tới
//     DataFusion.
//
//   [SUBIDX-COUNT] FIXED: computeAqiSubindex() dùng std::size(Cfg::AQI_INDEX_BP)
//     thay vì hardcode kCount=7; trần clamp dùng Cfg::AQI_MAX_INDEX.
// ============================================================================

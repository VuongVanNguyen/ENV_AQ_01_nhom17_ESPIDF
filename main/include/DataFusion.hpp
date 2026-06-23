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

    // Thang Discomfort Index (Thom DI) ngoài đời thực — 6 mức, đối xứng với AqiCategory.
    enum class ComfortCategory : uint8_t {
        COMFORTABLE  = 0,  // DI < OK            : dễ chịu
        SLIGHTLY_HOT = 1,  // OK   <= DI < WARM  : hơi nóng
        HOT          = 2,  // WARM <= DI < HOT   : nóng khó chịu
        VERY_HOT     = 3,  // HOT  <= DI < SEVERE: rất khó chịu
        HEAT_STRESS  = 4,  // SEVERE <= DI < DANGER: nguy cơ stress nhiệt
        DANGER       = 5,  // DI >= DANGER       : cấp cứu y tế / nguy hiểm tính mạng
    };

    enum class AlertLevel : uint8_t {
        NONE     = 0,
        WARNING  = 1,
        CRITICAL = 2,
    };

    // Bitmask cho AirData::alert_flags (DataStructures.hpp) — mỗi bit set ĐỘC
    // LẬP khi điều kiện tương ứng đang active, BỔ SUNG cho alert_reason (chỉ
    // mang lý do CAO NHẤT). Dùng để phát hiện nhiều điều kiện CRITICAL/WARNING
    // xảy ra đồng thời (ví dụ AQI_HAZARDOUS + COMFORT_DANGER cùng lúc).
    static constexpr uint16_t FLAG_AQI_BAD          = 1u << 0;
    static constexpr uint16_t FLAG_AQI_HAZARDOUS    = 1u << 1;
    static constexpr uint16_t FLAG_COMFORT_VERY_HOT = 1u << 2;
    static constexpr uint16_t FLAG_COMFORT_DANGER   = 1u << 3;
    static constexpr uint16_t FLAG_CO2_WARNING      = 1u << 4;
    static constexpr uint16_t FLAG_CO2_CRITICAL     = 1u << 5;
    static constexpr uint16_t FLAG_CALIB_NEEDED     = 1u << 6;

    DataFusion();
    ~DataFusion();

    DataFusion(const DataFusion&)            = delete;
    DataFusion& operator=(const DataFusion&) = delete;

    esp_err_t init();
    void process(AirData &data, bool time_synced = false);
    esp_err_t confirmRecalibration(AirData &data, bool time_synced = false);

    // Ghi baseline đang "dirty" xuống NVS — TÁCH khỏi process() (pipeline ≤300ms,
    // CLAUDE.md §3/§4). process()→initializeBaselineGroup() chỉ chốt baseline vào
    // RAM + bật cờ dirty (O(1), non-blocking); main.cpp PHẢI gọi hàm này định kỳ ở
    // task ƯU TIÊN THẤP (ngoài đường tới hạn cảm biến) để thực hiện nvs_commit chậm.
    // Idempotent: không dirty → trả ESP_OK ngay, không chạm flash.
    // (confirmRecalibration vẫn ghi NVS ĐỒNG BỘ — nó chạy trong task sự kiện MQTT,
    //  ngoài pipeline, nên giữ nguyên ngữ nghĩa lỗi đồng bộ.)
    esp_err_t persistBaselineIfDirty();

    bool hasBaseline() const;

private:
    struct Baseline {
        float temperature;
        float humidity;
        float pm25;
        float pm10;
        float co2;
        float pressure;
        float aqi;
        float comfort_index;
    } baseline_;

    int64_t last_calib_ts_;

    // Mốc 30 ngày chờ ghi NVS (deferred): initializeBaselineGroup() đặt giá trị
    // này + bật baseline_dirty_; persistBaselineIfDirty() đọc ra để nvs_set_i64.
    int64_t pending_calib_ts_;

    // Bitmask 3 nhóm baseline — bit set = nhóm đó đã có baseline hợp lệ.
    // hasBaseline() trả về (baseline_mask_ != 0). Xem §6.5.
    uint8_t baseline_mask_;

    // true = baseline_/baseline_mask_ trong RAM đã đổi nhưng CHƯA ghi NVS.
    // Bật bởi initializeBaselineGroup() (trong pipeline, không được block);
    // xoá bởi persistBaselineIfDirty() (low-prio) hoặc confirmRecalibration()
    // (ghi đồng bộ). Truy cập dưới mutex_.
    bool baseline_dirty_;
    static constexpr uint8_t BASELINE_BME680_BIT  = 1u << 0; // temperature, humidity, pressure, comfort_index
    static constexpr uint8_t BASELINE_PMS5003_BIT = 1u << 1; // pm25, pm10, aqi
    static constexpr uint8_t BASELINE_MQ135_BIT   = 1u << 2; // co2

    AlertLevel last_alert_level_;

    // Mốc esp_timer_get_time() (µs) của lần alert_level THỰC SỰ đổi mức gần
    // nhất — sở hữu bởi DataFusion (như last_calib_ts_), KHÔNG dựa vào AirData
    // tự giữ giá trị giữa các chu kỳ: taskSensor (main.cpp) tạo "AirData local{}"
    // MỚI mỗi chu kỳ (zero-init), nên field trong data sẽ mất giá trị nếu không
    // được gán lại TỪ ĐÂY mỗi lần process() chạy (CLAUDE.md §3 ALERT_MAX_LATENCY_MS).
    int64_t alert_level_changed_us_;

    AqiCategory last_category_;
    ComfortCategory last_comfort_category_;
    char last_calib_reason_[32];

    nvs_handle_t nvs_handle_;
    mutable SemaphoreHandle_t mutex_;

    static constexpr const char *kNoCalibReason = "NONE";
    static constexpr const char *kNoAlertReason = "NONE";

    void setSafeSentinel(AirData &data) const;
    void setReason(const char *reason);
    int64_t getNow(bool time_synced) const;

    esp_err_t loadBaseline();
    esp_err_t saveBaseline(int64_t timestamp);
    // Thân ghi NVS thuần (nvs_set_* + commit) — nhận baseline/mask/ts qua tham số,
    // KHÔNG cập nhật last_calib_ts_ (caller lo). Dùng chung bởi saveBaseline()
    // (ghi đồng bộ trong confirmRecalibration) và persistBaselineIfDirty() (flush
    // cờ dirty của baseline-init). Cả hai caller gọi khi ĐANG giữ mutex_.
    esp_err_t writeBaselineToNvs(const Baseline &bl, uint8_t mask, int64_t timestamp);

    void initializeBaselineGroup(uint8_t group_bit, const AirData &data, bool time_synced);
    void computeAqi(AirData &data);
    void computeComfort(AirData &data);
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
//   3.4 aqi_category suy ra TỪ GIÁ TRỊ data.aqi theo dải chỉ số AQI (0–500),
//       so sánh tuần tự với Cfg::AQI_GOOD_MAX/MODERATE_MAX/POOR_MAX/BAD_MAX/
//       VERY_BAD_MAX (config.hpp §7, đến từ Kconfig, mặc định 50/100/150/200/300):
//         ≤GOOD_MAX→0, ≤MODERATE_MAX→1, ≤POOR_MAX→2, ≤BAD_MAX→3,
//         ≤VERY_BAD_MAX→4, else→5.
//       → ghi vào data.aqi_category (uint8_t, theo enum AqiCategory §2).
//
//   3.5 Biên/clamp: C_x vượt điểm gãy cao nhất → kẹp AQI = 500 (Nguy hại).
//       C_x âm/không hợp lệ đã bị Filters loại; nếu vẫn gặp → coi như chưa ready.
//
//   GHI CHÚ: Cfg::AQI_GOOD_MAX/MODERATE_MAX/... (config.hpp §7) là ngưỡng CHỈ SỐ
//   AQI (0–500, cùng đơn vị với data.aqi) — KHÔNG phải nồng độ PM2.5. AQI số
//   (data.aqi) BẮT BUỘC tính bằng nội suy §3.1 để đạt sai số lặp ≤ 10%
//   (CLAUDE.md §3 — chỉ tiêu nghiệm thu AQI).
//
// ============================================================================
// 5. THUẬT TOÁN — COMFORT INDEX (từ Nhiệt độ / Độ ẩm)
// ----------------------------------------------------------------------------
//   ĐẦU VÀO: data.temperature (°C), data.humidity (%RH); khi data.bme680_ready.
//
//   5.1 Chỉ số nhiệt-ẩm THI (Discomfort Index — Thom DI, °C):
//         THI = T − 0.55·(1 − 0.01·RH)·(T − 14.5)
//       → ghi vào data.comfort_index (float).
//
//   5.2 comfort_category suy ra TỪ GIÁ TRỊ data.comfort_index theo thang DI
//       ngoài đời thực (Thom Discomfort Index, 6 mức — đối xứng với AQI §3.4),
//       so sánh tuần tự với Cfg::COMFORT_DI_OK/WARM/HOT/SEVERE/DANGER
//       (config.hpp §14):
//         DI < OK            → 0 COMFORTABLE  (dễ chịu)
//         OK   <= DI < WARM  → 1 SLIGHTLY_HOT  (hơi nóng)
//         WARM <= DI < HOT   → 2 HOT           (nóng khó chịu)
//         HOT  <= DI < SEVERE→ 3 VERY_HOT      (rất khó chịu)
//         SEVERE <= DI < DANGER → 4 HEAT_STRESS (nguy cơ stress nhiệt)
//         DI >= DANGER       → 5 DANGER        (cấp cứu y tế / nguy hiểm tính mạng)
//       → ghi vào data.comfort_category (uint8_t, theo enum ComfortCategory §2).
//       Dùng bởi computeAlertLevel (§8) VÀ DisplayManager (chỉ map số→nhãn,
//       KHÔNG tự so ngưỡng).
//
//   5.3 Hệ số 0.55 / 14.5 / 0.01 và các ngưỡng dải DI KHAI BÁO TRONG config.hpp
//       (§14) — không hardcode (CLAUDE.md §4). NFR: sai số lặp Comfort ≤ 10%
//       (CLAUDE.md §3).
//
// ============================================================================
// 6. DRIFT SELF-CHECK + VÒNG ĐỜI BASELINE NVS (CLAUDE.md §3, §4 — BẮT BUỘC)
// ----------------------------------------------------------------------------
//   Hằng dùng: Cfg::DRIFT_THRESHOLD_PCT (10%), Cfg::CALIB_INTERVAL_SEC (30 ngày).
//   Key NVS:   Cfg::NVS_NAMESPACE + NVS_KEY_BL_TEMP/BL_HUMI/BL_PM25/BL_CO2/
//              NVS_KEY_BL_MASK/NVS_KEY_LAST_CALIB_TS (config.hpp §12, §14).
//
//   BASELINE THEO 3 NHÓM ĐỘC LẬP (baseline_mask_, bit per nhóm — xem class
//   private members): mỗi nhóm được chốt baseline NGAY khi cờ *_ready tương
//   ứng của nhóm đó lần đầu = true, KHÔNG chờ 2 nhóm còn lại (CLAUDE.md đề
//   xuất: cảm biến nào warmup xong dùng ngay).
//     - BASELINE_BME680_BIT  ⊂ {temperature, humidity, pressure,
//       comfort_index} — gate bởi data.bme680_ready.
//     - BASELINE_PMS5003_BIT ⊂ {pm25, pm10, aqi}        — gate bởi data.pms5003_ready.
//     - BASELINE_MQ135_BIT   ⊂ {co2}                     — gate bởi data.mq135_ready.
//   hasBaseline() == (baseline_mask_ != 0): "có ít nhất 1 nhóm đã baseline".
//
//   6.1 So lệch mỗi chu kỳ (chỉ trên kênh có cờ ready VÀ nhóm tương ứng đã có
//       baseline — baseline_mask_ & BIT_NHÓM, tránh so với baseline_ rỗng/0):
//         dev% = |current − baseline| / |baseline| · 100
//       Tính cho PM2.5, PM10, CO2. Nếu BẤT KỲ kênh nào dev% > DRIFT_THRESHOLD_PCT
//       → set data.calib_needed = true (kèm lý do, ví dụ "CALIB_DRIFT_PM25"
//       hoặc "CALIB_DRIFT_PM10").
//       T và RH dùng SAI SỐ TUYỆT ĐỐI thay vì dev% (xem §10/§11):
//         |T_now − T_baseline|  > DRIFT_TEMP_ABS_C  (0.5°C)  → "CALIB_DRIFT_TEMP"
//         |RH_now − RH_baseline| > DRIFT_HUMI_ABS_PCT (3%RH) → "CALIB_DRIFT_HUMI"
//
//   6.1b "Sai số lặp" chỉ số suy diễn (CLAUDE.md §3 — AQI/Comfort(THI)/CO2 ≤ 10%):
//       CO2 đã được phủ bởi §6.1 (cùng DRIFT_THRESHOLD_PCT=10%). Với AQI và
//       comfort_index (chỉ khi pms5003_ready / bme680_ready, nhóm tương ứng đã
//       có baseline, và giá trị hữu hạn, baseline ≠ 0 để tránh chia 0):
//         dev% = |current − baseline| / |baseline| · 100
//       Nếu dev% > ACCURACY_INDEX_PCT (10%) → calib_needed=true, lý do
//       "CALIB_DRIFT_AQI" / "CALIB_DRIFT_COMFORT". baseline_.aqi/comfort_index
//       được chốt SAU khi computeAqi()/computeComfort() đã chạy trong process()
//       (initializeBaselineGroup/confirmRecalibration phải đọc data.aqi/
//       data.comfort_index ĐÃ TÍNH của chu kỳ hiện tại — process() gọi
//       computeAqi/computeComfort TRƯỚC initializeBaselineGroup/driftSelfCheck).
//
//   6.2 Quá hạn thời gian: now − last_calib_timestamp > CALIB_INTERVAL_SEC
//       → set data.calib_needed = true (lý do "CALIB_OVERDUE_30D").
//       (Lấy 'now' theo §7.) last_calib_timestamp chỉ được set khi NHÓM
//       BASELINE ĐẦU TIÊN (bao giờ) được chốt (baseline_mask_ 0 → non-zero) —
//       các nhóm chốt sau đó KHÔNG đẩy lại mốc này (xem §6.5).
//
//   6.3 KHÔNG tự bù trừ, KHÔNG tự ghi đè baseline (CLAUDE.md §3, §4): khi
//       calib_needed=true chỉ PHÁT CỜ + để hệ thống cảnh báo; chờ người dùng.
//
//   6.4 Xác nhận tái hiệu chuẩn (confirmRecalibration — §1):
//       - Kích hoạt khi nhận lệnh MQTT "confirm_calib" (NetworkManager nhận →
//         command-callback do main.cpp đăng ký gọi DataFusion::confirmRecalibration).
//       - PER-GROUP: chỉ reset baseline của (các) nhóm hiện đang *_ready —
//         set bit tương ứng trong baseline_mask_. Nhóm chưa ready giữ baseline
//         cũ (nếu có); nếu drift do nhóm đó gây ra, chu kỳ process() kế tiếp
//         sẽ driftSelfCheck() và re-assert calib_needed=true.
//       - Yêu cầu tối thiểu: ÍT NHẤT 1 trong 3 cờ *_ready = true, nếu không trả
//         ESP_ERR_INVALID_STATE (không có gì để hiệu chuẩn).
//       - last_calib_timestamp = now (LUÔN cập nhật — đây là hành động chủ
//         động của người dùng, khác với mốc "lần đầu" ở §6.5/§6.2);
//         nvs_set_* (bao gồm bl_mask) + nvs_commit(); clear calib_needed.
//
//   6.5 Khởi tạo baseline lần đầu (init không thấy key NVS hoặc thiếu bit
//       trong baseline_mask_): mỗi nhóm tự chốt baseline từ MẪU ỔN ĐỊNH ĐẦU
//       TIÊN của riêng nhóm đó (khi *_ready tương ứng lần đầu = true sau
//       warmup) — KHÔNG chờ 2 nhóm còn lại, KHÔNG dùng mẫu warmup làm baseline.
//       Mỗi lần 1 nhóm được chốt: cập nhật baseline + bl_mask vào RAM và BẬT CỜ
//       baseline_dirty_ (KHÔNG nvs_commit trong process() — pipeline ≤300ms,
//       §12); nếu đây là nhóm ĐẦU TIÊN (baseline_mask_ trước đó == 0) thì đồng
//       thời đặt last_calib_timestamp = now (RAM) — các nhóm chốt sau giữ nguyên
//       timestamp này. Việc ghi 8 blob + bl_mask + ts xuống flash do
//       persistBaselineIfDirty() (task ưu tiên thấp ở main.cpp) thực hiện sau.
//       Backward-compat: NVS cũ (trước khi có bl_mask) nhưng có đủ 8 blob →
//       coi baseline_mask_ = 0b111 (tất cả nhóm hợp lệ, vì code cũ chỉ ghi khi
//       cả 3 cảm biến ready).
//
//   6.6 Phát cảnh báo MQTT: DataFusion KHÔNG tự publish. Khi data.calib_needed=true,
//       NetworkManager::buildJson() chèn field "calib_alert": true VÀ
//       "calib_reason" mang lý do drift cụ thể (CALIB_DRIFT_TEMP/PM25/.../
//       CALIB_OVERDUE_30D — computeAlertLevel ghi từ last_calib_reason_, ĐỘC
//       LẬP với alert_reason, xem §8), task mạng gọi
//       NetworkManager::publishAlert() (§9).
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
//   ĐẦU VÀO: data.aqi_category (§3.4), data.comfort_category (§5.2),
//            data.co2_ppm.
//   Hằng ngưỡng: Cfg::ALERT_CO2_PPM, Cfg::ALERT_CO2_CRITICAL_PPM (config.hpp §8).
//   Quy tắc tổng hợp:
//     - aqi_category ≥ 4 (VERY_BAD)        → CRITICAL.
//     - aqi_category == 3 (BAD)            → ≥ WARNING.
//     - comfort_category == 5 (DANGER)     → CRITICAL (nguy cơ say nắng/sốc nhiệt).
//     - comfort_category == 4 (HEAT_STRESS)→ ≥ WARNING.
//     - co2_ppm > ALERT_CO2_CRITICAL_PPM    → CRITICAL.
//     - co2_ppm > ALERT_CO2_PPM             → ≥ WARNING.
//     - calib_needed == true                → alert_level ≥ WARNING (chỉ nâng
//       alert_reason nếu chưa có mức cao hơn — AQI/Comfort/CO2 CRITICAL vẫn
//       thắng và giữ alert_reason của chúng). RIÊNG data.calib_reason LUÔN
//       được ghi = last_calib_reason_ (lý do drift CỤ THỂ do driftSelfCheck()
//       set ngay trước đó — "CALIB_DRIFT_TEMP"/"CALIB_DRIFT_PM25"/.../
//       "CALIB_OVERDUE_30D"), bất kể alert_reason đang là gì — không dùng
//       nhãn chung "CALIB_NEEDED".
//     - còn lại → alert_level=NONE, alert_reason="NONE"; calib_reason="NONE"
//       khi calib_needed=false.
//   PM2.5/PM10 không có ngưỡng cảnh báo riêng: hai giá trị này đã được gộp
//   vào aqi_category thông qua AQI_PM25_BP/AQI_PM10_BP (§3.4), nên kiểm tra
//   thêm ngưỡng nồng độ thô sẽ trùng lặp với quy tắc AQI ở trên.
//   GHI: data.alert_level (0/1/2) + data.alert_reason (lý do CAO NHẤT trong
//   chu kỳ, ví dụ "AQI_HAZARDOUS"/"COMFORT_DANGER"/"CO2_CRITICAL"/"NONE", §11)
//   + data.calib_reason (lý do hiệu chuẩn CỤ THỂ, ĐỘC LẬP với alert_reason,
//   §11) + data.alert_flags (bitmask FLAG_* — set ĐỘC LẬP cho TỪNG điều kiện
//   đang active, song song với consider()/alert_reason, để KHÔNG mất thông
//   tin khi nhiều điều kiện CRITICAL/WARNING xảy ra đồng thời, §11) VÀ lưu
//   nội bộ (last_alert_level_). DataFusion KHÔNG chạm GPIO.
//   main.cpp sở hữu gpio_config() (GPIO25/26/27 LED, GPIO32 Buzzer — CLAUDE.md §5)
//   và lái output theo AlertLevel + aqi_category trong ≤ ALERT_MAX_LATENCY_MS (3 s).
//   NetworkManager::buildJson() đưa alert_level/alert_reason/calib_reason vào
//   MỌI payload (publishData VÀ publishAlert) để dashboard theo dõi mức nguy
//   hiểm real-time.
//   aqi_category chỉ xét khi data.pms5003_ready: PMS5003_WARMUP_MS = 30s, nên
//   AQI sẵn sàng rất sớm và cảnh báo AQI nguy hại không bị trễ bởi các cảm
//   biến khác (MQ135_WARMUP_MS = 20 phút) — phù hợp độ trễ cảnh báo ≤ 3s
//   (CLAUDE.md §3). comfort_category chỉ xét khi
//   data.bme680_ready (computeComfort đã set sentinel COMFORTABLE=0 khi chưa
//   ready, nên không cần thêm guard riêng).
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
//   TÁI DÙNG: ALERT_CO2_PPM, ALERT_CO2_CRITICAL_PPM, DRIFT_THRESHOLD_PCT,
//     CALIB_INTERVAL_SEC, NVS_KEY_BL_*, NVS_KEY_LAST_CALIB_TS,
//     ACCURACY_INDEX_PCT, ACCURACY_TEMP_C, MAX_CYCLE_TIME_MS, ALERT_MAX_LATENCY_MS.
//
//   §7 AQI: AQI_INDEX_BP[], AQI_PM25_BP[], AQI_PM10_BP[] (điểm gãy nội suy §3.1),
//     AQI_GOOD_MAX..AQI_VERY_BAD_MAX = 50/100/150/200/300 (ngưỡng CHỈ SỐ AQI,
//     đến từ Kconfig, dùng phân loại category §3.4),
//     AQI_MAX_INDEX = 500 (clamp trần §3.5).
//   §11 Drift: DRIFT_TEMP_ABS_C = ACCURACY_TEMP_C (ngưỡng tuyệt đối drift T, §6.1).
//     DRIFT_HUMI_ABS_PCT = ACCURACY_HUMI_RH (ngưỡng tuyệt đối drift RH, §6.1).
//     ACCURACY_INDEX_PCT (10%) = ngưỡng dev% cho AQI/comfort_index (§6.1b).
//   §14 Comfort: COMFORT_DI_K1 = 0.55f, COMFORT_DI_K2 = 14.5f, COMFORT_DI_RH_SCALE
//     = 0.01f, COMFORT_DI_OK/WARM/HOT/SEVERE/DANGER (ngưỡng dải DI 6 mức §5.2).
//
//   Baseline.pressure / NVS_KEY_BL_PRESSURE: GIỮ NGUYÊN (không xóa). Hiện
//     chưa có consumer, nhưng để dành cho pressure drift self-check tương lai
//     (so data.pressure vs baseline, như T/RH/PM2.5/CO2). Vô hại, không gây
//     lỗi biên dịch. In/publish pressure dùng data.pressure (live, đã có ở
//     NetworkManager), KHÔNG dùng baseline.
//
// ============================================================================
// 11. TRƯỜNG AirData ĐỌC/GHI & QUY TẮC READINESS (DataStructures.hpp)
// ----------------------------------------------------------------------------
//   ĐỌC : temperature, humidity, pressure, pm2_5, pm10, co2_ppm,
//          bme680_ready, pms5003_ready, mq135_ready, data_valid, timestamp.
//   GHI : aqi, aqi_category, comfort_index, comfort_category, calib_needed,
//          alert_level, alert_reason, alert_flags, calib_reason,
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
//   - 300 ms: phần DataFusion phải nhỏ so với ngân sách chu kỳ tổng. process()
//     thuần RAM/O(1): KHÔNG nvs_commit trong pipeline. Baseline-init chỉ chốt RAM
//     + bật baseline_dirty_; flash do persistBaselineIfDirty() (low-prio) ghi sau.
//   - NVS ghi ở 2 nơi NGOÀI pipeline: (a) confirmRecalibration (hiếm, task MQTT,
//     ĐỒNG BỘ để trả lỗi flash); (b) persistBaselineIfDirty (task ưu tiên thấp,
//     flush cờ dirty của baseline-init) → không nơi nào nằm trên đường ≤300ms.
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
//   - Các biến nên theo dõi qua JTAG: data.aqi, data.comfort_index,
//     data.comfort_category, data.co2_ppm, baseline NVS, dev%, AlertLevel —
//     phục vụ nghiệm thu sai số ≤ 10% (CLAUDE.md §3).


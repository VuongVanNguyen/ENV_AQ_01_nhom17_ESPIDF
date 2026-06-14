#pragma once

#include <cstdint>

// ============================================================
// AirData — Struct chia sẻ giữa tất cả module trong project.
//   SensorManager  → ghi (raw + readiness flags + timestamp)
//   Filters        → đọc/ghi (làm sạch + cập nhật ngược)
//   DataFusion     → ghi (aqi, comfort, calib_needed)
//   DisplayManager → đọc (hiển thị)
//   NetworkManager → đọc (publish JSON)
//   StorageHelper  → đọc (log CSV / offline queue)
// ============================================================
struct AirData {
    // ---- BME680 ----
    float temperature;      // °C
    float humidity;         // %RH
    float pressure;         // hPa
    float gas_resistance;   // Ω (chỉ hợp lệ khi bme680_ready=true)

    // ---- PMS5003 ----
    uint16_t pm1_0;         // µg/m³
    uint16_t pm2_5;         // µg/m³
    uint16_t pm10;          // µg/m³

    // ---- MQ-135 ----
    float co2_ppm;          // ppm (CO2 quy đổi + bù T/RH tại SensorManager, EMA tại Filters)

    // ---- Chỉ số tính toán ----
    float aqi;              // AQI theo tiêu chuẩn Việt Nam
    uint8_t aqi_category;   // 0=Tốt 1=Trung bình 2=Kém 3=Xấu 4=Rất xấu 5=Nguy hại
    float comfort_index;    // THI (Discomfort Index, °C)
    uint8_t comfort_category; // 0=Dễ chịu 1=Hơi nóng 2=Nóng khó chịu 3=Rất khó chịu 4=Stress nhiệt 5=Cấp cứu

    // ---- Cảnh báo vượt ngưỡng (DataFusion::computeAlertLevel) ----
    //   alert_level: 0=NONE 1=WARNING 2=CRITICAL (DataFusion::AlertLevel).
    //   alert_reason: lý do CỤ THỂ gây ra alert_level cao nhất trong chu kỳ —
    //   "AQI_HAZARDOUS"/"AQI_BAD", "COMFORT_DANGER"/"COMFORT_VERY_HOT",
    //   "CO2_CRITICAL"/"CO2_WARNING", hoặc lý do drift hiệu chuẩn nếu đó là
    //   nguyên nhân CAO NHẤT trong chu kỳ, hoặc "NONE". NetworkManager đưa
    //   trường này vào JSON (publishData VÀ publishAlert) để dashboard theo
    //   dõi mức độ nguy hiểm real-time.
    uint8_t alert_level;
    char alert_reason[24];

    //   alert_flags: bitmask BỔ SUNG cho alert_reason — mỗi bit set ĐỘC LẬP
    //   khi điều kiện tương ứng đang active trong chu kỳ, cho phép dashboard
    //   phát hiện NHIỀU điều kiện CRITICAL/WARNING xảy ra ĐỒNG THỜI (mà
    //   alert_reason chỉ thể hiện được lý do CAO NHẤT). Tên hằng FLAG_* định
    //   nghĩa tại DataFusion.hpp:
    //     bit0 0x0001 FLAG_AQI_BAD          aqi_category == BAD
    //     bit1 0x0002 FLAG_AQI_HAZARDOUS    aqi_category >= VERY_BAD
    //     bit2 0x0004 FLAG_COMFORT_VERY_HOT comfort_category == HEAT_STRESS
    //     bit3 0x0008 FLAG_COMFORT_DANGER   comfort_category >= DANGER
    //     bit4 0x0010 FLAG_CO2_WARNING      co2_ppm > ALERT_CO2_PPM
    //     bit5 0x0020 FLAG_CO2_CRITICAL     co2_ppm > ALERT_CO2_CRITICAL_PPM
    //     bit6 0x0040 FLAG_CALIB_NEEDED     calib_needed == true
    //   alert_flags = 0 khi data_valid=false (đồng nhất với alert_level=NONE).
    uint16_t alert_flags;

    // ---- Trạng thái hiệu chuẩn ----
    //   calib_needed: cờ Drift Self-Check (DataFusion::driftSelfCheck).
    //   calib_reason: lý do hiệu chuẩn CỤ THỂ — "CALIB_DRIFT_TEMP"/
    //   "CALIB_DRIFT_HUMI"/"CALIB_DRIFT_PM25"/"CALIB_DRIFT_PM10"/
    //   "CALIB_DRIFT_CO2"/"CALIB_DRIFT_AQI"/"CALIB_DRIFT_COMFORT"/
    //   "CALIB_OVERDUE_30D", hoặc "NONE" khi calib_needed=false. LUÔN được
    //   ghi ĐỘC LẬP với alert_reason — kể cả khi alert_reason đang mang lý do
    //   AQI/Comfort/CO2 ở mức CRITICAL cao hơn (calib chỉ nâng alert_level lên
    //   WARNING), calib_reason vẫn cho biết kênh nào drift.
    bool calib_needed;
    char calib_reason[24];
    int64_t last_calib_timestamp; // Unix time (giây)

    // ---- Trạng thái sẵn sàng của TỪNG cảm biến ----
    //   Module hạ nguồn (Filters/DataFusion/Display/Network) phải tự
    //   kiểm tra cờ tương ứng trước khi tin tưởng giá trị raw — tránh
    //   feed giá trị rác từ heater chưa ổn định / fan PMS chưa quay /
    //   MOX MQ-135 chưa preheat đủ. Mỗi cờ độc lập: cảm biến nào ready
    //   trước thì dữ liệu của riêng cảm biến đó được dùng ngay (baseline/
    //   drift-check/hiển thị), không chờ các cảm biến còn lại.
    bool bme680_ready;      // heater stable + gas_valid + qua warmup
    bool pms5003_ready;     // fan đã quay đủ lâu + frame có dữ liệu khác 0
    bool mq135_ready;       // đã preheat đủ thời gian

    // ---- Metadata ----
    int64_t timestamp;      // Unix time khi lấy mẫu
    bool data_valid;        // false nếu cycle readAll() trả lỗi cứng
};

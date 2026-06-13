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

    // ---- Trạng thái hiệu chuẩn ----
    bool calib_needed;
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

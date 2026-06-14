#pragma once

// ============================================================
// Filters.hpp — Module lọc nhiễu tín hiệu cảm biến ENV-AQ-01
//
//   Kiến trúc hai tầng (CLAUDE.md Mục 7.1):
//     Tầng 1 — Sanity check range vật lý cố định (Cfg::SANITY_*).
//     Tầng 2 — EMA (7 kênh) / SMA (humidity, Cfg::FILTER_WINDOW_SIZE).
//
//   Vị trí trong luồng dữ liệu:
//     SensorManager::readAll() → Filters::process() → DataFusion
//
//   Sử dụng:
//     - Tạo đúng 1 instance Filters trong main.cpp, dùng suốt vòng đời.
//     - Gọi process(data) ngay sau mỗi SensorManager::readAll().
//     - Làm sạch AirData tại chỗ (in-place) — không cấp phát struct mới.
//
//   Ranh giới trách nhiệm:
//     KHÔNG tính AQI/Comfort (thuộc DataFusion).
//     KHÔNG bù T/RH cho MQ-135 (SensorManager đã làm trước).
//     KHÔNG đọc cảm biến, không I/O, không blocking.
// ============================================================

#include "DataStructures.hpp"
#include "config.hpp"

#include <array>
#include <cstddef>

class Filters {
public:
    Filters();
    ~Filters() = default;

    // Lọc in-place toàn bộ 8 kênh của AirData.
    // Trình tự: data_valid → readiness flags → Tầng 1 (range) → Tầng 2 (EMA/SMA)
    //           → ghi ngược (làm tròn + clamp cho PM uint16_t).
    void process(AirData &data);

    // Xoá toàn bộ trạng thái filter (dùng khi tái hiệu chuẩn / reset chuỗi lọc).
    void reset();

    // Truy vấn trạng thái filter hiện tại (debug/JTAG, không tác dụng phụ).
    // Trả true nếu tất cả kênh đã qua warm-start; false nếu bất kỳ kênh nào chưa sẵn sàng.
    bool getFilterState(float &temp, float &humi, float &press, float &gas,
                        float &co2,  float &pm1,  float &pm25,  float &pm10) const;

private:
    // EMA state: 1 float y[n-1] + 1 bool initialized per channel
    struct EmaState {
        float prev;
        bool  initialized;
    };

    // SMA state: ring buffer + write index + valid count + cached last output
    struct SmaState {
        std::array<float, Cfg::FILTER_WINDOW_SIZE> buf;
        size_t write_idx;
        size_t count;
        float  last_output;
    };

    // Filter state cho từng kênh
    EmaState ema_temp_;
    SmaState sma_humi_;
    EmaState ema_press_;
    EmaState ema_gas_;
    EmaState ema_co2_;
    EmaState ema_pm1_;
    EmaState ema_pm25_;
    EmaState ema_pm10_;

    // Cờ ready chu kỳ trước — phát hiện chuyển trạng thái false→true (spec §5.2)
    bool prev_bme680_ready_;
    bool prev_mq135_ready_;
    bool prev_pms5003_ready_;

    // ---- EMA helpers ----
    // Warm-start: gán thẳng x vào state (y[-1] = x[0]), trả về x
    static float emaWarmStart(EmaState &s, float x);
    // Update bình thường: y[n] = α·x + (1−α)·y[n−1]
    static float emaUpdate(EmaState &s, float x, float alpha);

    // ---- SMA helper ----
    // Nạp x vào ring buffer, trả về trung bình trên số mẫu hiện có
    static float smaUpdate(SmaState &s, float x);

    // ---- Sanity check helper ----
    static bool inRange(float x, float lo, float hi);

    // ---- Reset per sensor group ----
    void resetBme680State();
    void resetMq135State();
    void resetPms5003State();

    // ---- Per-group processing ----
    void processBme680(AirData &data);
    void processMq135(AirData &data);
    void processPms5003(AirData &data);
};

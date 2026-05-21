#include "DataFusion.hpp"

// ============================================================
// QUAN TRỌNG — Yêu cầu đề bài §4 (Drift Self-Check + Data Persistence):
//
// Khi ghi AirData::last_calib_timestamp (xác nhận tái hiệu chuẩn):
//   - Nếu SNTP đã sync (networkManager.isTimeSynced() == true):
//       last_calib_timestamp = time(NULL);   // Unix UTC — chính xác cho chu kỳ 30 ngày
//   - Nếu chưa sync:
//       last_calib_timestamp = esp_timer_get_time() / 1000000LL;  // seconds-from-boot
//       → Chu kỳ 30 ngày vẫn đo được trong 1 session liên tục,
//         nhưng reset sau reboot. Chấp nhận được ở giai đoạn phát triển;
//         cần SNTP để chạy đúng trong triển khai thực tế.
//
// Hằng ngưỡng: Cfg::CALIB_INTERVAL_SEC = 30 * 86400 giây (config.hpp)
// Key NVS lưu: Cfg::NVS_KEY_LAST_CALIB_TS (config.hpp)
// ============================================================

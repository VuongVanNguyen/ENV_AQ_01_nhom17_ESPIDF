#include "StorageHelper.hpp"

// ============================================================
// QUAN TRỌNG — Yêu cầu đề bài §4 (Data Persistence):
//
// Khi đọc/ghi last_calib_timestamp từ NVS (nvs_get_i64 / nvs_set_i64):
//   - Giá trị phải là Unix time (giây) để tính chu kỳ 30 ngày đúng qua reboot.
//   - Nếu SNTP chưa sync lúc ghi, dùng esp_timer_get_time() / 1000000LL làm
//     fallback — nhưng giá trị này sẽ không hợp lệ sau reboot (reset về 0).
//   - Khi đọc lại từ NVS sau reboot: nếu giá trị đọc được < 1577836800
//     (ngưỡng 1/1/2020), coi là fallback cũ → trigger calib_needed ngay,
//     không tính chu kỳ 30 ngày.
//
// Key NVS: Cfg::NVS_KEY_LAST_CALIB_TS, namespace: Cfg::NVS_NAMESPACE (config.hpp)
// ============================================================

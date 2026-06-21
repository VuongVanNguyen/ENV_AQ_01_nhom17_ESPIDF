#pragma once

// ============================================================================
// StorageHelper.hpp — ĐẶC TẢ MODULE LƯU TRỮ SD CARD (ENV-AQ-01)
//
//   Vị trí trong luồng dữ liệu (CLAUDE.md §2):
//     SensorManager → Filters → DataFusion → DisplayManager + NetworkManager
//                                                            + StorageHelper (SD)
//
//   StorageHelper là điểm CUỐI (sink) của pipeline, CHỈ-ĐỌC struct AirData
//   (DataStructures.hpp §12: "StorageHelper → đọc (log CSV / offline queue)").
//   Module KHÔNG tính toán chỉ số, KHÔNG chạm GPIO/I2C/cảm biến — chỉ ghi file.
//
//   BA TRÁCH NHIỆM CHÍNH (CLAUDE.md §2 + §4):
//     (A) LOG DỮ LIỆU ĐỊNH KỲ : ghi mỗi mẫu AirData thành 1 dòng CSV vào
//         Cfg::SD_LOG_FILE (/sdcard/airdata.csv).                         → §3
//     (B) EVENT LOG (rời rạc)  : ghi các SỰ KIỆN ĐỘC LẬP (đổi alert/calib,
//         connect/disconnect MQTT) vào file CSV riêng SD_EVENT_LOG_FILE
//         (/sdcard/events.csv) qua logEvent(...).                         → §4
//     (C) OFFLINE BUFFER       : khi mất MQTT, xếp AirData vào hàng đợi bền
//         vững trên SD (Cfg::SD_OFFLINE_QUEUE) rồi phát lại khi reconnect. → §5
//
//   NGUYÊN TẮC TỐI THƯỢNG (CLAUDE.md §3 NFR + §4 Non-blocking):
//     MỌI thao tác SD (mount, fopen/fwrite/fsync/rename/remove) là BLOCKING
//     I/O hàng chục–hàng trăm ms → TUYỆT ĐỐI KHÔNG gọi trực tiếp trong
//     pipeline ≤300ms. Toàn bộ I/O chạy trong MỘT task riêng
//     (Cfg::TASK_PRIO_STORAGE / TASK_STACK_STORAGE_WORDS) nhận lệnh qua
//     FreeRTOS queue. API public của module chỉ ENQUEUE (O(1), non-blocking)
//     rồi trả về ngay — xem §6.
//
//   #include dự kiến khi triển khai .cpp (KHÔNG kéo NetworkManager.hpp vào
//   header → tránh phụ thuộc vòng; tương tác mạng đi qua trạng thái + dây nối
//   ở main.cpp / taskNetwork, giống triết lý DataFusion.hpp §9):
//     "StorageHelper.hpp", "DataStructures.hpp", "config.hpp",
//     "esp_err.h", "esp_log.h", "esp_vfs_fat.h", "sdmmc_cmd.h",
//     "driver/sdspi_host.h", "driver/spi_common.h",
//     "freertos/FreeRTOS.h", "freertos/task.h", "freertos/queue.h",
//     <ctime> / "esp_timer.h" (nguồn thời gian §7), <cstdio>, <cstring>.
//
// ============================================================================
// 0. NỘI DUNG ĐÃ ĐẶC TẢ SẴN TRONG StorageHelper.cpp (GIỮ NGUYÊN — gộp lên đây)
// ----------------------------------------------------------------------------
//   StorageHelper.cpp hiện chứa ghi chú về last_calib_timestamp trong NVS
//   (Data Persistence, CLAUDE.md §4). Ghi chú đó được GIỮ NGUYÊN trong .cpp và
//   nhắc lại ở đây để liên kết module:
//     - last_calib_timestamp là Unix time (giây), kiểu int64_t, do DataFusion
//       sở hữu & ghi vào NVS (DataFusion.hpp §6 — nvs_set_i64, key
//       Cfg::NVS_KEY_LAST_CALIB_TS, namespace Cfg::NVS_NAMESPACE).
//     - StorageHelper KHÔNG ghi NVS calibration. Module chỉ ĐỌC trường
//       AirData::last_calib_timestamp (đã được DataFusion nạp/ghi) để in vào
//       cột CSV (§3) và đính kèm Event Log khi có sự kiện calib (§4).
//     - Cảnh báo "fallback timestamp" trong .cpp (giá trị < 1577836800 =
//       1/1/2020 ⇒ chưa có SNTP thật) áp dụng cho cả cột timestamp ghi ra SD:
//       nếu nguồn thời gian CHƯA hợp lệ (NetworkManager::isTimeSynced()==false),
//       cột timestamp ghi giá trị giây-từ-boot và PHẢI gắn cờ phân biệt (xem §7)
//       để hậu kỳ không nhầm là Unix time thật.
//
// ============================================================================
// 1. API PUBLIC (ĐẶC TẢ CHỮ KÝ — class StorageHelper, không copyable như các module khác)
// ----------------------------------------------------------------------------
//   Quy ước chung với SensorManager/NetworkManager/DataFusion: class
//   non-copyable (xoá copy ctor + operator=), trả esp_err_t cho hàm có thể lỗi.
//
//   esp_err_t init();
//     - Gọi ĐÚNG 1 LẦN ở app_main() (main.cpp), SAU nvs_flash_init().
//     - Khởi tạo SPI bus + mount SD (sdspi, §2); tạo storage queue; tạo
//       storage task (§6); mở/khởi tạo 2 file CSV và ghi header nếu file mới
//       (§3.2, §4.2); khôi phục & (nếu có mạng) phát lại offline queue tồn dư
//       từ phiên trước (§5.4).
//     - KHÔNG block pipeline: nếu thẻ SD vắng/hỏng → trả lỗi (ESP_FAIL/
//       ESP_ERR_NOT_FOUND) NHƯNG hệ thống vẫn chạy được; module vào chế độ
//       "SD_ABSENT" (§13) — mọi enqueue sau đó bị drop êm + ESP_LOGW, không
//       làm sập pipeline.
//
//   esp_err_t logData(const AirData &data);
//     - Gọi TỪ pipeline (hoặc taskStorage tự bơm theo nhịp §3.1). Non-blocking:
//       CHỈ copy AirData vào StorageMsg{kind=DATA} rồi xQueueSend timeout 0.
//     - Trả ESP_ERR_TIMEOUT/ESP_FAIL nếu queue đầy (caller KHÔNG block; drop +
//       đếm dropped, §11). KHÔNG tự fopen tại đây.
//
//   esp_err_t logEvent(EventType type, const AirData &snapshot);
//     - Ghi 1 SỰ KIỆN rời rạc (§4). Non-blocking: copy {kind=EVENT, type,
//       trường liên quan trích từ snapshot} vào queue. Việc fopen/fwrite do
//       taskStorage làm. type ∈ EventType (§4.1).
//
//   esp_err_t bufferOffline(const AirData &data);
//     - Gọi TỪ taskNetwork khi NetworkManager::publishData()/publishAlert()
//       trả ESP_ERR_INVALID_STATE (MQTT chưa connected — NetworkManager.hpp).
//       Non-blocking enqueue {kind=OFFLINE_PUSH} → taskStorage append record
//       vào Cfg::SD_OFFLINE_QUEUE (§5.2).
//
//   esp_err_t drainOffline(NetworkManager &net);   // hoặc callback re-publish
//     - Gọi TỪ taskNetwork NGAY khi isConnected() chuyển false→true. Yêu cầu
//       taskStorage đọc tuần tự offline queue và phát lại từng record qua
//       NetworkManager::publishData() cho tới khi rỗng/lỗi (§5.3).
//     - GHI CHÚ phụ thuộc vòng: để KHÔNG #include NetworkManager.hpp trong
//       header này, ưu tiên nhận một std::function<esp_err_t(const AirData&)>
//       (con trỏ tới net.publishData) thay vì tham chiếu trực tiếp
//       NetworkManager — quyết định cuối ở .cpp; cả hai cách đều giữ
//       StorageHelper KHÔNG biết chi tiết NetworkManager.
//
//   bool   isMounted() const;        // thẻ SD đã mount thành công?
//   size_t offlineCount() const;     // số record đang tồn trong offline queue
//                                    // (taskNetwork dùng để biết còn phải drain)
//
//   PRIVATE (gợi ý — chi tiết để .cpp tự quyết, mô tả vai trò):
//     - sdmmc_card_t *card_;                 // handle thẻ (do sdspi mount trả về)
//     - QueueHandle_t  storage_queue_;       // hàng lệnh tới taskStorage (§6)
//     - TaskHandle_t   storage_task_;        // task ghi SD duy nhất
//     - volatile bool  mounted_;             // trạng thái mount (§13)
//     - size_t         offline_count_;       // đếm record offline (đọc khi init §5.4)
//     - uint32_t       dropped_data_, dropped_event_;  // thống kê queue-full (§11)
//     - static void    storageTask(void *arg);          // thân task (§6)
//     - esp_err_t      mountCard();          // §2
//     - esp_err_t      ensureHeader(path, header);  // ghi header nếu file mới (§3.2/§4.2)
//     - esp_err_t      writeDataRow(const AirData&);    // §3.3
//     - esp_err_t      writeEventRow(EventType, const AirData&); // §4.3
//     - esp_err_t      pushOfflineRecord(const AirData&);   // §5.2
//     - esp_err_t      drainOfflineRecords(publish_fn);     // §5.3
//
//   StorageMsg (struct nội bộ truyền qua queue) — đặt trong .cpp:
//     enum class MsgKind { DATA, EVENT, OFFLINE_PUSH, OFFLINE_DRAIN };
//     gồm: MsgKind kind; EventType evt; AirData payload; (publish_fn cho DRAIN).
//     TRUYỀN BẢN COPY của AirData (by value) — KHÔNG truyền con trỏ tới
//     shared_data của pipeline (tránh data race khi pipeline ghi đè, [XM-7/9]).
//
// ============================================================================
// 2. MOUNT THẺ SD — SPI MODE (CLAUDE.md §5 Pin Mapping + config.hpp §4)
// ----------------------------------------------------------------------------
//   ⚠️ ĐỐI CHIẾU MÂU THUẪN TÀI LIỆU: bảng module CLAUDE.md §2 ghi
//   "esp_vfs_fat_sdmmc_mount / sdmmc_*", NHƯNG §5 Pin Mapping + config.hpp §4
//   dùng BUS SPI (SCK=18, MISO=19, MOSI=23, CS=5, SPI2_HOST) và §5 nói rõ
//   "mount qua esp_vfs_fat_sdspi_mount()". ⇒ ĐƯỜNG SPI (sdspi) là CHÍNH XÁC.
//   Hai cái không mâu thuẫn thực sự: esp_vfs_fat_sdspi_mount() VẪN trả về
//   handle sdmmc_card_t* và dùng API sdmmc_* chung — chỉ khác lớp host (SDSPI).
//
//   Trình tự mount (taskStorage::init, một lần):
//     2.1 spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA) với
//         bus_cfg.{mosi,miso,sclk} = Cfg::SD_SPI_MOSI_PIN/MISO_PIN/SCK_PIN,
//         quartz/cs để mặc định. SPI host = Cfg::SD_SPI_HOST (= 1 = SPI2_HOST).
//         ⚠️ SPI2_HOST PHẢI KHÔNG xung đột với bất kỳ thiết bị SPI nào khác
//         (dự án chỉ có SD trên SPI → an toàn). KHÔNG đụng I2C (BME680/PCF8574
//         ở I2C_NUM_0) hay UART2 (PMS5003) hay ADC1 (MQ135).
//     2.2 sdspi_device_config_t dev_cfg; dev_cfg.gpio_cs = Cfg::SD_SPI_CS_PIN;
//         dev_cfg.host_id = SPI2_HOST.
//     2.3 esp_vfs_fat_sdspi_mount(Cfg::SD_MOUNT_POINT, &host, &dev_cfg,
//         &mount_cfg, &card_) với mount_cfg.format_if_mount_failed = false
//         (KHÔNG tự format thẻ người dùng — chỉ cảnh báo), max_files đủ cho
//         data.csv + events.csv + offline.bin mở đồng thời (≥3 →
//         Cfg::SD_MAX_OPEN_FILES, §9), allocation_unit_size hợp lý (16–32 KB).
//     2.4 Thành công → mounted_=true, ESP_LOGI in dung lượng thẻ
//         (sdmmc_card_print_info). Thất bại → mounted_=false, ESP_LOGE, vào
//         chế độ SD_ABSENT (§13). KHÔNG ESP_ERROR_CHECK (không được sập hệ
//         thống chỉ vì thiếu thẻ — quan trắc vẫn phải hiển thị + publish MQTT).
//
//   GPIO JTAG (12/13/14/15) KHÔNG trùng chân SD (CLAUDE.md §6.2) → an toàn debug.
//
// ============================================================================
// 3. LOG DỮ LIỆU ĐỊNH KỲ — CSV (CLAUDE.md §2, §4 Logging vs Event Log)
// ----------------------------------------------------------------------------
//   3.1 NHỊP GHI: KHÔNG ghi mỗi cycle pipeline (≤300ms → mòn thẻ + I/O thừa).
//       Ghi theo chu kỳ Cfg::SD_LOG_INTERVAL_MS (§9, từ Kconfig — ví dụ 30–60s,
//       BỘI SỐ của SENSOR_READ_INTERVAL_MS). Hai cách triển khai (chọn ở .cpp):
//         (a) main.cpp/pipeline gọi logData() đúng theo nhịp đó; hoặc
//         (b) taskStorage tự dùng esp_timer/queue-timeout đếm nhịp và chỉ ghi
//             snapshot AirData mới nhất (lấy qua airDataQueue [XM-9]).
//       Dù cách nào: ENQUEUE non-blocking, fwrite trong taskStorage (§6).
//
//   3.2 HEADER (ghi 1 lần khi tạo file mới — ensureHeader so sánh kích thước/
//       tồn tại). Thứ tự cột CỐ ĐỊNH, khớp đúng các trường đọc ở §10:
//         timestamp,time_valid,temperature,humidity,pressure,
//         pm1_0,pm2_5,pm10,co2_ppm,aqi,aqi_category,
//         comfort_index,comfort_category,alert_level,alert_reason,
//         alert_flags,calib_needed,calib_reason,
//         bme680_ready,pms5003_ready,mq135_ready,data_valid
//
//   3.3 MỘT DÒNG = MỘT MẪU:
//       - timestamp: AirData::timestamp (Unix giây nếu time_valid=1, §7).
//       - time_valid: 0/1 — cờ phân biệt Unix-thật vs giây-từ-boot (§7, §0).
//       - float in cố định số lẻ (vd %.2f cho T/RH/P/comfort, %.1f cho co2/aqi)
//         để hậu kỳ ổn định; uintX in nguyên.
//       - alert_reason / calib_reason: in NGUYÊN chuỗi (đã là token an toàn
//         "AQI_HAZARDOUS"/"CALIB_DRIFT_PM25"/"NONE"… — DataFusion.hpp §8/§11);
//         vẫn PHẢI khử ',' và '\n' phòng hờ (CSV-escape, §3.4).
//       - alert_flags: in dạng số (vd hex 0x%04X hoặc thập phân) — bitmask
//         FLAG_* (DataFusion.hpp §… / DataStructures.hpp) để hậu kỳ tách.
//       - Trường chưa hợp lệ do warmup: AirData chứa SENTINEL (NAN float,
//         category=0 — DataFusion.hpp §11). In NAN thành ô rỗng hoặc "nan"
//         NHẤT QUÁN; KHÔNG bịa 0.0 (sai lệch phân tích). bme680_ready/
//         pms5003_ready/mq135_ready trong CÙNG dòng cho biết ô nào đáng tin.
//
//   3.4 CSV-SAFE: mọi field chuỗi phải KHÔNG chứa ',' '"' '\n'. Token của
//       DataFusion vốn an toàn, nhưng helper ghi PHẢI tự bảo vệ (thay thế hoặc
//       quote) để 1 chuỗi lỗi không phá vỡ cấu trúc cột.
//
//   3.5 BỀN VỮNG GHI: sau mỗi N dòng (hoặc mỗi lần ghi) gọi fflush()+fsync()
//       (Cfg::SD_FSYNC_EVERY_N_ROWS, §9) để hạn chế mất dữ liệu khi mất điện
//       đột ngột — cân bằng giữa an toàn và tuổi thọ thẻ. fopen append "a",
//       đóng/mở lại theo chính sách .cpp (giữ mở để giảm latency, hoặc mở-ghi-
//       đóng để an toàn). KHÔNG giữ con trỏ FILE* mở xuyên suốt nếu rủi ro
//       corrupt khi rút thẻ nóng — ưu tiên fsync thường xuyên.
//
//   3.6 XOAY VÒNG FILE (chống đầy thẻ): khi airdata.csv vượt
//       Cfg::SD_LOG_MAX_BYTES (§9) → rename sang tên có hậu tố (vd
//       airdata.1.csv) và mở file mới với header. Giữ tối đa
//       Cfg::SD_LOG_MAX_FILES bản, xoá bản cũ nhất. (Tùy chọn nhưng KHUYẾN
//       NGHỊ cho vận hành dài hạn 30 ngày — CLAUDE.md §3 Ổn định dài hạn.)
//
// ============================================================================
// 4. EVENT LOG — SỰ KIỆN RỜI RẠC (CLAUDE.md §4 "Event Log" — BẮT BUỘC)
// ----------------------------------------------------------------------------
//   File RIÊNG: Cfg::SD_EVENT_LOG_FILE (/sdcard/events.csv) — TÁCH khỏi log dữ
//   liệu định kỳ §3. Chỉ ghi khi TRẠNG THÁI ĐỔI (edge-triggered), KHÔNG ghi
//   mỗi chu kỳ (khác hẳn §3).
//
//   4.1 EventType (enum, khai báo trong header này — public, để main/taskNetwork
//       gọi logEvent): các sự kiện CLAUDE.md §4 yêu cầu:
//         ALERT_LEVEL_CHANGED   // alert_level đổi (NONE↔WARNING↔CRITICAL)
//         ALERT_REASON_CHANGED  // alert_reason / alert_flags đổi
//         CALIB_NEEDED_SET      // calib_needed false→true (kèm calib_reason)
//         CALIB_CONFIRMED       // người dùng xác nhận confirm_calib
//                               //   (DataFusion::confirmRecalibration OK, §4.4)
//         MQTT_CONNECTED        // kết nối broker (NetworkManager isConnected ↑)
//         MQTT_DISCONNECTED     // mất kết nối broker (↓)
//         SYSTEM_BOOT           // (khuyến nghị) mốc khởi động — phục vụ truy vết
//         SD_LOG_ROTATED        // (nếu bật §3.6) — truy vết file
//
//   4.2 HEADER events.csv (ghi 1 lần khi tạo mới):
//         timestamp,time_valid,event,alert_level,alert_reason,alert_flags,
//         calib_needed,calib_reason
//       (event = tên EventType dạng chuỗi; các cột sau là ngữ cảnh tại thời
//        điểm sự kiện, lấy từ snapshot AirData — bỏ trống cho sự kiện không
//        liên quan, vd MQTT_*).
//
//   4.3 PHÁT HIỆN EDGE: StorageHelper KHÔNG tự so sánh chu kỳ-trước. CHỦ THỂ
//       sinh sự kiện gọi logEvent() đúng lúc đổi trạng thái — khớp dây nối có
//       sẵn ở main.cpp [XM-11] (taskNetwork giữ last_published_level và chỉ
//       publishAlert khi đổi): cùng nhịp đó, taskNetwork/pipeline gọi
//       logEvent(ALERT_LEVEL_CHANGED, snapshot). Tương tự:
//         - taskNetwork phát hiện isConnected() đổi → logEvent(MQTT_CONNECTED/
//           DISCONNECTED) (đồng thời trigger drainOffline/bufferOffline §5).
//         - cmd_callback "confirm_calib" [XM-8]: SAU khi
//           DataFusion::confirmRecalibration() trả ESP_OK → logEvent(
//           CALIB_CONFIRMED, snapshot). Nếu trả ESP_ERR_INVALID_STATE thì KHÔNG
//           ghi (không có gì để hiệu chuẩn — DataFusion.hpp §6.4).
//         - pipeline phát hiện data.calib_needed false→true →
//           logEvent(CALIB_NEEDED_SET, snapshot) (calib_reason trong snapshot
//           cho biết kênh drift — DataFusion.hpp §6/§8).
//       StorageHelper chỉ chịu trách nhiệm GHI; phát hiện edge là của caller.
//
//   4.4 CALIB_CONFIRMED và NVS: sự kiện này tương quan với việc DataFusion ghi
//       last_calib_timestamp mới vào NVS (§0). Cột timestamp của dòng event =
//       thời điểm xác nhận (now §7); để truy vết, có thể ghi kèm giá trị
//       last_calib_timestamp mới (đọc từ snapshot.last_calib_timestamp) — đây
//       là lý do EventType này ĐỌC trường NVS-derived của AirData (§10).
//
//   4.5 cùng quy tắc CSV-safe (§3.4) + fsync (§3.5). events.csv KHÔNG xoay vòng
//       theo dung lượng như §3.6 (mật độ thấp), nhưng vẫn cap
//       Cfg::SD_EVENT_MAX_BYTES (§9) nếu cần.
//
// ============================================================================
// 5. OFFLINE BUFFER — HÀNG ĐỢI BỀN VỮNG TRÊN SD (CLAUDE.md §4 "Offline Buffer")
// ----------------------------------------------------------------------------
//   MỤC TIÊU: "Nếu mất kết nối MQTT, dữ liệu phải được lưu vào hàng đợi trên
//   thẻ SD qua storage_helper, dùng VFS FAT" (CLAUDE.md §4). Khi có mạng lại,
//   PHÁT LẠI để dashboard không mất mẫu.
//
//   5.1 ĐỊNH DẠNG: file nhị phân Cfg::SD_OFFLINE_QUEUE (/sdcard/offline_queue.bin)
//       gồm các RECORD CỐ ĐỊNH KÍCH THƯỚC = sizeof(AirData) (POD, append-only).
//       Lý do nhị phân (không CSV): nhanh, không mất độ chính xác float, phát
//       lại = publishData(record) tái dựng JSON y hệt lúc online. (CLAUDE.md §1
//       gợi std::queue cho Offline Buffer; ở đây dùng FILE-BACKED queue để
//       BỀN qua reboot — RAM std::queue không sống sót mất điện. Có thể kèm 1
//       ring RAM nhỏ làm staging, nhưng nguồn-sự-thật là file SD.)
//       ⚠️ Phụ thuộc layout AirData: nếu struct đổi, format cũ không tương
//       thích → ghi MAGIC+VERSION ở đầu file (Cfg::OFFLINE_MAGIC, §9); khi đọc
//       version lệch → bỏ file cũ + ESP_LOGW (không phát lại rác).
//
//   5.2 ENQUEUE (bufferOffline → taskStorage pushOfflineRecord):
//       - Append sizeof(AirData) byte vào cuối file ("ab"), fsync, offline_count_++.
//       - CHẶN ĐẦY THẺ: nếu offline_count_ ≥ Cfg::OFFLINE_QUEUE_MAX_RECORDS (§9)
//         → drop record CŨ NHẤT (head) theo chính sách ring/compaction (§5.5),
//         ESP_LOGW. KHÔNG để offline queue ăn hết thẻ làm hỏng log §3/§4.
//
//   5.3 DRAIN (drainOffline → taskStorage drainOfflineRecords), gọi khi MQTT
//       reconnect:
//       - Đọc tuần tự từ HEAD; với mỗi record gọi publish_fn(record)
//         (= NetworkManager::publishData, NetworkManager.hpp).
//         · ESP_OK            → advance head (record đã gửi).
//         · ESP_ERR_INVALID_STATE (rớt mạng giữa chừng) → DỪNG, giữ phần còn
//           lại cho lần reconnect sau (KHÔNG mất dữ liệu).
//       - Khi rỗng: truncate/remove file, offline_count_=0, ESP_LOGI.
//       - PHÁT LẠI THEO LÔ + nhường CPU: gửi tối đa Cfg::OFFLINE_DRAIN_BATCH
//         (§9) record mỗi lượt rồi yield (không độc chiếm taskStorage / không
//         flood broker). KHÔNG dùng vTaskDelay làm logic timing nghiệp vụ —
//         dùng nhịp queue/esp_timer (CLAUDE.md §4 Non-blocking).
//       - THỨ TỰ: FIFO (mẫu cũ gửi trước) để chuỗi thời gian liền mạch.
//
//   5.4 KHÔI PHỤC SAU REBOOT (init §1): nếu Cfg::SD_OFFLINE_QUEUE tồn tại lúc
//       mount → đọc MAGIC/VERSION (§5.1), đếm record → offline_count_. KHÔNG tự
//       phát lại tại init (chưa chắc có mạng); để taskNetwork gọi drainOffline
//       khi isConnected() lần đầu = true.
//
//   5.5 HEAD/TAIL & COMPACTION: tránh ghi đè giữa file. Cách đơn giản & an
//       toàn: lưu head-offset trong file phụ Cfg::SD_OFFLINE_HEAD (vd
//       offline_queue.hdr) hoặc nén (compact) bằng cách ghi phần còn lại sang
//       file tạm rồi rename khi head vượt ngưỡng. Chi tiết để .cpp; YÊU CẦU:
//       nguyên tử (rename) để mất điện giữa chừng không hỏng queue.
//
// ============================================================================
// 6. MÔ HÌNH THREADING — TASK GHI SD DUY NHẤT + QUEUE (CLAUDE.md §3 §4 — CỐT LÕI)
// ----------------------------------------------------------------------------
//   6.1 storage_queue_ = xQueueCreate(Cfg::STORAGE_QUEUE_LEN (§9),
//       sizeof(StorageMsg)). StorageMsg mang BẢN COPY AirData (by value) —
//       KHÔNG con trỏ tới shared_data ([XM-7/9] main.cpp): pipeline ghi đè
//       shared_data liên tục, truyền con trỏ sẽ data-race giữa 2 core.
//
//   6.2 storage_task_ = xTaskCreate(storageTask, "storage",
//       Cfg::TASK_STACK_STORAGE_WORDS, this, Cfg::TASK_PRIO_STORAGE, &handle).
//       TASK_PRIO_STORAGE = 2 (THẤP NHẤT — config.hpp §15: sensor5>net4>disp3>
//       storage2) ⇒ I/O SD chậm KHÔNG cản pipeline real-time hay cảnh báo ≤3s.
//
//   6.3 Thân storageTask: vòng for(;;) { xQueueReceive(blocking, portMAX_DELAY);
//       switch(msg.kind){ DATA→writeDataRow; EVENT→writeEventRow;
//       OFFLINE_PUSH→pushOfflineRecord; OFFLINE_DRAIN→drainOfflineRecords; } }.
//       Đây là NƠI DUY NHẤT chạm fopen/fwrite/fsync → tự nhiên SERIALIZE mọi
//       truy cập 3 file, KHÔNG cần mutex file riêng. (xQueueReceive blocking ở
//       đây HỢP LỆ — là cơ chế đồng bộ chờ việc, KHÔNG phải vTaskDelay timing
//       nghiệp vụ mà CLAUDE.md §4 cấm.)
//
//   6.4 BACK-PRESSURE: API public (logData/logEvent/bufferOffline) xQueueSend
//       timeout 0 (non-blocking) — pipeline KHÔNG BAO GIỜ chờ SD. Queue đầy →
//       drop + đếm dropped_* + ESP_LOGW tiết chế (không spam mỗi lần). Mất 1
//       dòng log ÍT nghiêm trọng hơn trễ pipeline >300ms (ưu tiên NFR §3).
//
//   6.5 STACK CHECK: trước release gọi uxTaskGetStackHighWaterMark(storage_task_)
//       (CLAUDE.md §6.5) — fopen/snprintf/FAT tốn stack; chỉnh
//       TASK_STACK_STORAGE_WORDS nếu high-water-mark thấp.
//
// ============================================================================
// 7. NGUỒN THỜI GIAN (NHẤT QUÁN với DataFusion.hpp §7 + StorageHelper.cpp §0)
// ----------------------------------------------------------------------------
//   Cột timestamp (CSV §3 & event §4) PHẢI là Unix time thật mới có ý nghĩa
//   chuỗi thời gian. Quy tắc:
//     - StorageHelper ƯU TIÊN dùng AirData::timestamp do SensorManager/pipeline
//       đã set (DataStructures.hpp). Trường này = time(NULL) khi
//       NetworkManager::isTimeSynced()==true, ngược lại = giây-từ-boot.
//     - time_valid (cột §3.2/§4.2): suy ra như DataFusion.cpp/§0 — nếu
//       timestamp < 1577836800 (1/1/2020) ⇒ coi là giây-từ-boot ⇒ time_valid=0;
//       else time_valid=1. Cho phép hậu kỳ lọc/căn lại mốc sau khi SNTP sync.
//     - StorageHelper KHÔNG tự gọi SNTP và KHÔNG #include NetworkManager.hpp;
//       nó CHỈ đọc timestamp đã có trong AirData (tránh phụ thuộc vòng, giống
//       DataFusion.hpp §9). Nếu cần "now" cho event không gắn AirData (vd
//       MQTT_*), caller truyền snapshot có timestamp hợp lệ, hoặc dùng
//       time(NULL)/esp_timer_get_time()/1000000LL ngay tại caller.
//
// ============================================================================
// 8. LIÊN KẾT MODULE — HÀM/DỮ LIỆU DÙNG TỪ FILE NÀO (giống DataFusion.hpp §9)
// ----------------------------------------------------------------------------
//   ĐỌC struct AirData ................ DataStructures.hpp (CHỈ-ĐỌC; trường §10).
//   Hằng số cấu hình .................. config.hpp (namespace Cfg; §4/§9/§15).
//   Nguồn dữ liệu (đã sạch+tính) ...... pipeline SensorManager→Filters→DataFusion;
//                                        StorageHelper là sink, KHÔNG gọi ngược
//                                        lên các module này.
//   Trạng thái kết nối MQTT ........... NetworkManager::isConnected() —
//                                        NetworkManager.hpp; taskNetwork (main.cpp)
//                                        dùng nó để quyết định bufferOffline (§5.2)
//                                        / drainOffline (§5.3) / logEvent(MQTT_*).
//   Phát lại record offline ........... NetworkManager::publishData(record) —
//                                        NetworkManager.hpp; GỌI TRONG drain (§5.3)
//                                        qua publish_fn (không #include header đó).
//   Thời gian thực .................... AirData::timestamp (đã set sẵn) /
//                                        time() ("time.h") / esp_timer_get_time()
//                                        ("esp_timer.h") — §7.
//   NVS last_calib_timestamp .......... DataFusion sở hữu+ghi (DataFusion.hpp §6,
//                                        nvs_set_i64 key Cfg::NVS_KEY_LAST_CALIB_TS);
//                                        StorageHelper CHỈ đọc qua AirData (§0/§4.4),
//                                        KHÔNG mở NVS.
//   VFS FAT / SD / SPI ................ esp_vfs_fat_sdspi_mount() ("esp_vfs_fat.h"),
//                                        sdspi_host ("driver/sdspi_host.h"),
//                                        spi_bus ("driver/spi_common.h"),
//                                        sdmmc_card_t/sdmmc_card_print_info
//                                        ("sdmmc_cmd.h") — §2.
//   FreeRTOS task/queue .............. "freertos/task.h" + "freertos/queue.h" — §6.
//   Logging ........................... ESP_LOGI/W/E ("esp_log.h"), TAG="StorageHelper";
//                                        KHÔNG Serial.print, KHÔNG xoá log khi
//                                        commit (CLAUDE.md §4, §6.5).
//   Dây nối khởi tạo & gọi ............ main.cpp: StorageHelper::init() SAU
//                                        nvs_flash_init(); tạo taskNetwork wiring
//                                        gọi bufferOffline/drainOffline/logEvent
//                                        (xem [XM-9/10/11] có sẵn trong main.cpp).
//
// ============================================================================
// 9. HẰNG SỐ TRONG config.hpp (namespace Cfg) — KHÔNG hardcode .cpp (CLAUDE.md §4)
// ----------------------------------------------------------------------------
//   ĐÃ CÓ (config.hpp §4 + §15) — TÁI DÙNG, KHÔNG khai báo lại:
//     SD_SPI_SCK_PIN/MISO_PIN/MOSI_PIN/CS_PIN, SD_SPI_HOST (=SPI2_HOST),
//     SD_MOUNT_POINT (/sdcard), SD_LOG_FILE (/sdcard/airdata.csv),
//     SD_OFFLINE_QUEUE (/sdcard/offline_queue.bin),
//     TASK_STACK_STORAGE_WORDS (4096), TASK_PRIO_STORAGE (2),
//     MAX_CYCLE_TIME_MS (300), ALERT_MAX_LATENCY_MS (3000).
//
//   CẦN BỔ SUNG vào config.hpp §4 (SD CARD) — hằng MỚI, đặt ở ĐÓ (KHÔNG ở .cpp):
//     SD_EVENT_LOG_FILE     = "/sdcard/events.csv"     // Event Log §4 (CLAUDE.md §4)
//     SD_OFFLINE_HEAD       = "/sdcard/offline_queue.hdr" // head-offset §5.5 (nếu dùng)
//     SD_LOG_INTERVAL_MS    = (Kconfig, vd 30000)        // nhịp ghi data §3.1
//     SD_FSYNC_EVERY_N_ROWS = (vd 1..10)                 // chính sách fsync §3.5
//     SD_MAX_OPEN_FILES     = (vd 4)                      // max_files khi mount §2.3
//     SD_LOG_MAX_BYTES      = (vd 5*1024*1024)            // ngưỡng xoay vòng §3.6
//     SD_LOG_MAX_FILES      = (vd 5)                      // số bản giữ lại §3.6
//     SD_EVENT_MAX_BYTES    = (vd 1*1024*1024)            // cap events.csv §4.5
//     STORAGE_QUEUE_LEN     = (vd 16)                     // độ dài storage_queue_ §6.1
//     OFFLINE_QUEUE_MAX_RECORDS = (vd 2000)              // trần ring offline §5.2
//     OFFLINE_DRAIN_BATCH   = (vd 20)                     // lô phát lại §5.3
//     OFFLINE_MAGIC / OFFLINE_FORMAT_VERSION             // MAGIC+VERSION §5.1
//   (Giá trị Kconfig-điều-chỉnh-được → khai báo qua Kconfig.projbuild + CONFIG_*;
//    hằng cố định → inline constexpr trong Cfg. KHÔNG hardcode literal trong .cpp.)
//
// ============================================================================
// 10. TRƯỜNG AirData ĐỌC (DataStructures.hpp) — StorageHelper CHỈ-ĐỌC, KHÔNG GHI
// ----------------------------------------------------------------------------
//   ĐỌC (data log §3 + offline record §5 = toàn struct; event §4 = tập con):
//     temperature, humidity, pressure, pm1_0, pm2_5, pm10, co2_ppm,
//     aqi, aqi_category, comfort_index, comfort_category,
//     alert_level, alert_reason[24], alert_flags, calib_needed, calib_reason[24],
//     last_calib_timestamp, bme680_ready, pms5003_ready, mq135_ready,
//     timestamp, data_valid.
//   GHI : KHÔNG — StorageHelper TUYỆT ĐỐI không sửa AirData (sink chỉ-đọc;
//          mọi field do SensorManager/Filters/DataFusion sở hữu).
//   READINESS: ghi RA SD cả khi *_ready=false (giá trị có thể là SENTINEL/NAN,
//   DataFusion.hpp §11) NHƯNG PHẢI ghi kèm cờ *_ready trong cùng dòng (§3.3) để
//   hậu kỳ biết ô nào tin được — KHÔNG tự lọc bỏ (lưu trữ thô phục vụ truy vết/
//   nghiệm thu sai số ≤10%, CLAUDE.md §3). data_valid=false (cycle lỗi cứng):
//   vẫn ghi 1 dòng đánh dấu để không "mất mẫu im lặng", hoặc bỏ qua tùy chính
//   sách .cpp — NHƯNG phải NHẤT QUÁN và ghi rõ trong code.
//
// ============================================================================
// 11. RÀNG BUỘC PHI CHỨC NĂNG & ĐỘ BỀN (CLAUDE.md §3 NFR + §4)
// ----------------------------------------------------------------------------
//   - NON-BLOCKING (§4): pipeline ≤300ms (NFR §3) KHÔNG được chờ SD. API public
//     chỉ enqueue O(1); mọi I/O ở taskStorage prio thấp (§6). KHÔNG vTaskDelay
//     làm logic nghiệp vụ; nhịp ghi dùng esp_timer/queue (§3.1, §5.3).
//   - ĐỘ TRỄ CẢNH BÁO ≤3s (§3): Event Log/offline KHÔNG nằm trên đường tới hạn
//     cảnh báo — LED/Buzzer do main.cpp lái, publishAlert do taskNetwork; ghi SD
//     CHẬM cũng không làm trễ cảnh báo (task tách rời, prio thấp).
//   - ĐỘ BỀN DỮ LIỆU: fsync định kỳ (§3.5); thao tác đổi file (rotate §3.6,
//     compaction offline §5.5) PHẢI nguyên tử (rename) — mất điện giữa chừng
//     không corrupt log/queue. Offline queue PHẢI sống sót reboot (§5.4).
//   - QUEUE-FULL/THẺ-ĐẦY/THẺ-VẮNG: degrade êm (drop+ESP_LOGW+đếm dropped_*),
//     KHÔNG ESP_ERROR_CHECK gây panic (§2.4, §6.4, §13). Quan trắc + hiển thị +
//     MQTT phải tiếp tục dù SD hỏng.
//   - TUỔI THỌ THẺ: không ghi mỗi cycle (§3.1); ghi theo lô; allocation unit
//     hợp lý (§2.3) — tránh mòn flash thẻ trong vận hành 30 ngày liên tục.
//
// ============================================================================
// 12. LOGGING / DEBUG (CLAUDE.md §4, §6 — JTAG song song)
// ----------------------------------------------------------------------------
//   - TAG riêng "StorageHelper" với ESP_LOGI/W/E.
//   - ESP_LOGI: mount OK + dung lượng thẻ, tạo file+header, drain xong N record,
//     rotate file. ESP_LOGW: queue đầy (tiết chế), thẻ vắng/đầy, version offline
//     lệch, drop record. ESP_LOGE: mount fail, lỗi fwrite/fopen cứng.
//   - KHÔNG xoá/comment ESP_LOG* khi commit (CLAUDE.md §6.5 — nguồn debug song
//     song JTAG). Mọi panic/exception phải investigate bằng call stack (§6.5).
//   - Biến nên theo dõi qua JTAG: mounted_, offline_count_, dropped_data_/
//     dropped_event_, high-water-mark storage_task_ (§6.5), card_->csd dung
//     lượng — phục vụ kiểm tra rò rỉ/đầy thẻ khi nghiệm thu.
//
// ============================================================================
// 13. CHẾ ĐỘ THẺ SD VẮNG/HỎNG ("SD_ABSENT") — KHÔNG LÀM SẬP HỆ THỐNG
// ----------------------------------------------------------------------------
//   Mount fail (§2.4) hoặc lỗi I/O cứng giữa chừng → mounted_=false. Khi đó:
//     - init() trả lỗi nhưng app_main() CHỈ ESP_LOGE rồi tiếp tục tạo các task
//       khác (KHÔNG ESP_ERROR_CHECK trên StorageHelper::init()).
//     - logData/logEvent/bufferOffline: trả lỗi êm + ESP_LOGW tiết chế, drop dữ
//       liệu (không enqueue) — KHÔNG block, KHÔNG crash.
//     - (Tùy chọn) thử remount định kỳ qua taskStorage (nhịp esp_timer/queue,
//       KHÔNG vTaskDelay nghiệp vụ) để tự phục hồi khi cắm lại thẻ; thành công
//       → logEvent(SYSTEM_BOOT/ghi chú remount) + tiếp tục ghi.
//   Triết lý: chức năng quan trắc lõi (đọc cảm biến, hiển thị LCD, publish MQTT,
//   cảnh báo LED/Buzzer) KHÔNG phụ thuộc SD — SD chỉ là lưu trữ phụ trợ/bền vững.
// ============================================================================

// ============================================================================
// 14. KHAI BÁO C++ — TRIỂN KHAI THEO ĐẶC TẢ §0-§13 PHÍA TRÊN (KHÔNG xoá §0-§13)
// ============================================================================

#include "DataStructures.hpp"
#include "config.hpp"

#include "esp_err.h"
#include "sdmmc_cmd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <cstdint>
#include <cstddef>
#include <functional>

// Callback phát lại 1 record offline — trỏ tới NetworkManager::publishData()
// (NetworkManager.hpp §1). Truyền vào drainOffline() để StorageHelper KHÔNG
// #include NetworkManager.hpp (tránh phụ thuộc vòng, §1/§8, giống triết lý
// DataFusion.hpp §9).
using PublishFn = std::function<esp_err_t(const AirData &)>;

// StorageHelper — sink CHỈ-ĐỌC của pipeline (§0): log CSV định kỳ (§3),
// event log rời rạc (§4), offline buffer bền vững trên SD (§5), trên MỘT
// task riêng (§6). Non-copyable như SensorManager/NetworkManager/DataFusion.
class StorageHelper {
public:
    // §4.1 — sự kiện rời rạc (edge-triggered) ghi vào SD_EVENT_LOG_FILE
    // qua logEvent(). Nested public: caller dùng StorageHelper::EventType::*.
    enum class EventType : uint8_t {
        ALERT_LEVEL_CHANGED  = 0, // alert_level đổi (NONE<->WARNING<->CRITICAL)
        ALERT_REASON_CHANGED = 1, // alert_reason / alert_flags đổi
        CALIB_NEEDED_SET     = 2, // calib_needed false->true (kèm calib_reason)
        CALIB_CONFIRMED      = 3, // confirm_calib OK (DataFusion::confirmRecalibration, §4.4)
        MQTT_CONNECTED       = 4, // NetworkManager::isConnected() false->true
        MQTT_DISCONNECTED    = 5, // NetworkManager::isConnected() true->false
        SYSTEM_BOOT          = 6, // mốc khởi động — phục vụ truy vết
        SD_LOG_ROTATED       = 7, // airdata.csv vừa xoay vòng (§3.6)
    };

    StorageHelper();
    ~StorageHelper();

    StorageHelper(const StorageHelper&)            = delete;
    StorageHelper& operator=(const StorageHelper&) = delete;

    // §1 — gọi ĐÚNG 1 LẦN ở app_main(), SAU nvs_flash_init(). Mount SD (§2),
    // tạo storage_queue_ + storage_task_ (§6), ensureHeader cho 2 file CSV
    // (§3.2/§4.2), khôi phục offline_count_ từ SD_OFFLINE_QUEUE (§5.4).
    // Trả lỗi (KHÔNG ESP_ERROR_CHECK ở caller) nếu SD vắng/hỏng — module vào
    // chế độ SD_ABSENT (§13), KHÔNG làm sập pipeline.
    esp_err_t init();

    // §1/§3 — non-blocking: enqueue {kind=DATA, payload=data} (xQueueSend
    // timeout 0). Trả ESP_ERR_TIMEOUT/ESP_FAIL nếu queue đầy (drop + đếm
    // dropped_data_, §6.4/§11) hoặc SD_ABSENT.
    esp_err_t logData(const AirData &data);

    // §1/§4 — non-blocking: enqueue {kind=EVENT, evt=type, payload=snapshot}.
    // CHỦ THỂ gọi (pipeline/taskNetwork/cmd_callback) chịu trách nhiệm phát
    // hiện edge (§4.3) — StorageHelper chỉ ghi.
    esp_err_t logEvent(EventType type, const AirData &snapshot);

    // §1/§5.2 — non-blocking: enqueue {kind=OFFLINE_PUSH, payload=data}.
    // Gọi từ taskNetwork khi NetworkManager::publishData()/publishAlert()
    // trả ESP_ERR_INVALID_STATE (MQTT chưa connected).
    esp_err_t bufferOffline(const AirData &data);

    // §1/§5.3 — gọi từ taskNetwork NGAY khi isConnected() chuyển false->true.
    // publish_fn = wrapper gọi NetworkManager::publishData (bind tại call
    // site trong main.cpp, §8) — enqueue {kind=OFFLINE_DRAIN}; storageTask
    // phát lại tuần tự theo lô (Cfg::OFFLINE_DRAIN_BATCH, §5.3).
    esp_err_t drainOffline(PublishFn publish_fn);

    bool   isMounted() const;     // §1 — thẻ SD đã mount thành công? (SD_ABSENT, §13)
    size_t offlineCount() const;  // §1 — số record tồn trong offline queue (§5.4)

private:
    // ---- Trạng thái SD/queue/task (§1 — truy cập từ nhiều task, volatile/atomic-ish) ----
    sdmmc_card_t *card_;            // handle thẻ (sdspi mount, §2)
    QueueHandle_t storage_queue_;   // hàng lệnh tới storageTask (§6.1)
    TaskHandle_t  storage_task_;    // task ghi SD duy nhất (§6.2)
    volatile bool mounted_;         // trạng thái mount — SD_ABSENT khi false (§13)
    size_t        offline_count_;   // số record offline hiện tại (§5)
    uint32_t      dropped_data_;    // queue-full khi logData/bufferOffline (§6.4/§11)
    uint32_t      dropped_event_;   // queue-full khi logEvent (§6.4/§11)

    // ---- Trạng thái nội bộ storageTask (CHỈ truy cập trong storageTask — §6.3) ----
    long      data_file_bytes_;          // kích thước hiện tại SD_LOG_FILE (§3.6)
    long      event_file_bytes_;         // kích thước hiện tại SD_EVENT_LOG_FILE (§4.5)
    uint32_t  data_rows_pending_fsync_;  // số dòng chưa fsync (§3.5)
    PublishFn drain_publish_fn_;         // publish_fn gần nhất từ drainOffline() (§5.3)
    int64_t   last_log_timestamp_;       // AirData::timestamp của dòng airdata.csv gần nhất
                                          // — gating nhịp ghi theo SD_LOG_INTERVAL_MS (§3.1)

    // ---- Thân task (§6) ----
    static void storageTask(void *arg);
    void taskLoop();

    // ---- Mount SD qua SPI (§2) ----
    esp_err_t mountCard();

    // ---- CSV: log dữ liệu (§3) + event log (§4) ----
    esp_err_t ensureHeader(const char *path, const char *header, long &size_out);
    esp_err_t writeDataRow(const AirData &data);
    esp_err_t writeEventRow(EventType type, const AirData &snapshot);
    esp_err_t rotateDataLog(const AirData &data);  // §3.6 — vượt SD_LOG_MAX_BYTES

    // ---- Offline buffer (§5) ----
    esp_err_t pushOfflineRecord(const AirData &data);              // §5.2
    esp_err_t drainOfflineRecords(const PublishFn &publish_fn);    // §5.3
    esp_err_t restoreOfflineState();                               // §5.4 (gọi từ init)
    esp_err_t readOfflineHead(uint32_t &head_index) const;         // §5.5
    esp_err_t writeOfflineHead(uint32_t head_index) const;         // §5.5
    esp_err_t compactOfflineQueue(uint32_t head, long total_records); // §5.5 — nén khi head lớn

    // ---- Tiện ích chung (§3.4 CSV-safe, §7 time_valid) ----
    static size_t      csvEscape(const char *in, char *out, size_t out_sz);
    static int         timeValidOf(int64_t timestamp);
    static const char *eventTypeName(EventType type);
};
// ============================================================================

#include "StorageHelper.hpp"

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>

static const char *TAG = "StorageHelper";

static constexpr int64_t kY2020Epoch = 1577836800;
static constexpr int32_t kFallbackLogKey = -1;

static constexpr const char *kDataHeader =
    "timestamp,time_valid,temperature,humidity,pressure,"
    "pm1_0,pm2_5,pm10,co2_ppm,aqi,aqi_category,"
    "comfort_index,comfort_category,co2_category,alert_level,alert_reason,"
    "alert_flags,calib_needed,calib_reason,"
    "bme680_ready,pms5003_ready,mq135_ready,data_valid,cycle_time_ms\n";

static constexpr const char *kEventHeader =
    "timestamp,time_valid,event,alert_level,alert_reason,alert_flags,"
    "calib_needed,calib_reason\n";

enum class MsgKind : uint8_t { DATA, EVENT, OFFLINE_PUSH, OFFLINE_DRAIN, SHUTDOWN };

struct StorageMsg {
    MsgKind   kind;
    StorageHelper::EventType evt;
    AirData   payload;
};

struct OfflineFileHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
};

static void fmtFloat(float v, int prec, char *buf, size_t n) {
    if (!std::isfinite(v)) {
        buf[0] = '\0';
    } else {
        snprintf(buf, n, "%.*f", prec, v);
    }
}

static void rotatedPath(char *out, size_t out_sz, int n) {
    const char *base = Cfg::SD_LOG_FALLBACK_FILE;
    const char *ext = strrchr(base, '.');
    size_t base_len = ext ? static_cast<size_t>(ext - base) : strlen(base);
    if (base_len >= out_sz) base_len = out_sz - 1;
    snprintf(out, out_sz, "%.*s.%d%s", static_cast<int>(base_len), base, n,
             ext ? ext : "");
}

StorageHelper::StorageHelper()
    : card_(nullptr),
      storage_queue_(nullptr),
      storage_task_(nullptr),
      shutdown_done_(nullptr),
      mounted_(false),
      offline_count_(0),
      dropped_data_(0),
      dropped_event_(0),
      data_file_bytes_(0),
      event_file_bytes_(0),
      data_rows_pending_fsync_(0),
      drain_publish_fn_(nullptr),
      last_log_timestamp_(0),
      active_log_day_(INT32_MIN),
      active_log_path_{} {}

StorageHelper::~StorageHelper() {
    if (storage_task_) {
        vTaskDelete(storage_task_);
    }
    if (storage_queue_) {
        vQueueDelete(storage_queue_);
    }
    if (shutdown_done_) {
        vSemaphoreDelete(shutdown_done_);
    }
    if (mounted_ && card_) {
        esp_vfs_fat_sdcard_unmount(Cfg::SD_MOUNT_POINT, card_);
    }
}

esp_err_t StorageHelper::init() {
    storage_queue_ = xQueueCreate(Cfg::STORAGE_QUEUE_LEN, sizeof(StorageMsg));
    if (!storage_queue_) {
        ESP_LOGE(TAG, "Không thể tạo storage_queue_ (ESP_ERR_NO_MEM)");
        return ESP_ERR_NO_MEM;
    }

    shutdown_done_ = xSemaphoreCreateBinary();
    if (!shutdown_done_) {
        ESP_LOGE(TAG, "Không thể tạo shutdown_done_ (ESP_ERR_NO_MEM)");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = mountCard();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mount SD thất bại (%s) — vào chế độ SD_ABSENT (§13)",
                 esp_err_to_name(err));
        mounted_ = false;
    } else {
        mounted_ = true;
        sdmmc_card_print_info(stdout, card_);

        ensureHeader(Cfg::SD_EVENT_LOG_FILE, kEventHeader, event_file_bytes_);

        restoreOfflineState();

        ESP_LOGI(TAG,
                 "SD mounted OK — events.csv=%ld B, offline_count=%u",
                 event_file_bytes_,
                 static_cast<unsigned>(offline_count_));
    }

    BaseType_t ok = xTaskCreate(taskStorage, "storage",
                                 Cfg::TASK_STACK_STORAGE_WORDS, this,
                                 Cfg::TASK_PRIO_STORAGE, &storage_task_);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Không thể tạo storage_task_ (ESP_ERR_NO_MEM)");
        return ESP_ERR_NO_MEM;
    }

    return mounted_ ? ESP_OK : ESP_FAIL;
}

esp_err_t StorageHelper::mountCard() {

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = Cfg::SD_SPI_MOSI_PIN;
    bus_cfg.miso_io_num = Cfg::SD_SPI_MISO_PIN;
    bus_cfg.sclk_io_num = Cfg::SD_SPI_SCK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    esp_err_t err = spi_bus_initialize(
        static_cast<spi_host_device_t>(Cfg::SD_SPI_HOST), &bus_cfg,
        SDSPI_DEFAULT_DMA);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize lỗi: %s", esp_err_to_name(err));
        return err;
    }

    sdspi_device_config_t dev_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev_cfg.gpio_cs = static_cast<gpio_num_t>(Cfg::SD_SPI_CS_PIN);
    dev_cfg.host_id = static_cast<spi_host_device_t>(Cfg::SD_SPI_HOST);

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = Cfg::SD_SPI_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {};
    mount_cfg.format_if_mount_failed = false;
    mount_cfg.max_files = Cfg::SD_MAX_OPEN_FILES;
    mount_cfg.allocation_unit_size = 16 * 1024;

    err = esp_vfs_fat_sdspi_mount(Cfg::SD_MOUNT_POINT, &host, &dev_cfg,
                                   &mount_cfg, &card_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_vfs_fat_sdspi_mount lỗi: %s", esp_err_to_name(err));
        card_ = nullptr;
        return err;
    }

    return ESP_OK;
}

esp_err_t StorageHelper::ensureHeader(const char *path, const char *header,
                                       long &size_out) {
    struct stat st {};
    if (stat(path, &st) == 0 && st.st_size > 0) {
        size_out = static_cast<long>(st.st_size);
        return ESP_OK;
    }

    FILE *f = fopen(path, "a");
    if (!f) {
        ESP_LOGE(TAG, "Không thể tạo file %s", path);
        size_out = 0;
        return ESP_FAIL;
    }

    size_t written = fwrite(header, 1, strlen(header), f);
    fflush(f);
    fsync(fileno(f));
    fclose(f);

    size_out = static_cast<long>(written);
    ESP_LOGI(TAG, "Tạo file %s với header (%u byte)", path,
             static_cast<unsigned>(written));
    return ESP_OK;
}

void StorageHelper::buildLogPath(int64_t timestamp, char *out, size_t out_sz,
                                  int32_t &day_key) {
    if (timeValidOf(timestamp)) {
        time_t t = static_cast<time_t>(timestamp);
        struct tm tm_info{};
        localtime_r(&t, &tm_info);
        day_key = (tm_info.tm_year + 1900) * 10000 +
                  (tm_info.tm_mon + 1) * 100 + tm_info.tm_mday;
        snprintf(out, out_sz, "%s/%02d%02d%02d.csv", Cfg::SD_MOUNT_POINT,
                 (tm_info.tm_year + 1900) % 100, tm_info.tm_mon + 1,
                 tm_info.tm_mday);
    } else {
        day_key = kFallbackLogKey;
        snprintf(out, out_sz, "%s", Cfg::SD_LOG_FALLBACK_FILE);
    }
}

esp_err_t StorageHelper::writeDataRow(const AirData &data) {
    if (!mounted_) return ESP_ERR_INVALID_STATE;

    if (last_log_timestamp_ != 0) {
        int64_t delta_ms = (data.timestamp - last_log_timestamp_) * 1000;
        if (delta_ms < static_cast<int64_t>(Cfg::SD_LOG_INTERVAL_MS)) {
            return ESP_OK;
        }
    }

    int32_t day_key;
    char path[sizeof(active_log_path_)];
    buildLogPath(data.timestamp, path, sizeof(path), day_key);
    if (day_key != active_log_day_) {
        strncpy(active_log_path_, path, sizeof(active_log_path_) - 1);
        active_log_path_[sizeof(active_log_path_) - 1] = '\0';
        active_log_day_ = day_key;
        ensureHeader(active_log_path_, kDataHeader, data_file_bytes_);
    }

    char t_buf[16], h_buf[16], p_buf[16], co2_buf[16], aqi_buf[16], ci_buf[16];
    fmtFloat(data.temperature, 2, t_buf, sizeof(t_buf));
    fmtFloat(data.humidity, 2, h_buf, sizeof(h_buf));
    fmtFloat(data.pressure, 2, p_buf, sizeof(p_buf));
    fmtFloat(data.co2_ppm, 1, co2_buf, sizeof(co2_buf));
    fmtFloat(data.aqi, 1, aqi_buf, sizeof(aqi_buf));
    fmtFloat(data.comfort_index, 2, ci_buf, sizeof(ci_buf));

    char alert_reason_esc[48], calib_reason_esc[48];
    csvEscape(data.alert_reason, alert_reason_esc, sizeof(alert_reason_esc));
    csvEscape(data.calib_reason, calib_reason_esc, sizeof(calib_reason_esc));

    FILE *f = fopen(active_log_path_, "a");
    if (!f) {
        ESP_LOGE(TAG, "Không thể mở %s để ghi", active_log_path_);
        return ESP_FAIL;
    }

    int n = fprintf(
        f,
        "%lld,%d,%s,%s,%s,"
        "%u,%u,%u,%s,%s,%u,"
        "%s,%u,%u,%u,%s,"
        "0x%04X,%d,%s,"
        "%d,%d,%d,%d,%u\n",
        static_cast<long long>(data.timestamp), timeValidOf(data.timestamp),
        t_buf, h_buf, p_buf, static_cast<unsigned>(data.pm1_0),
        static_cast<unsigned>(data.pm2_5), static_cast<unsigned>(data.pm10),
        co2_buf, aqi_buf, static_cast<unsigned>(data.aqi_category), ci_buf,
        static_cast<unsigned>(data.comfort_category),
        static_cast<unsigned>(data.co2_category),
        static_cast<unsigned>(data.alert_level), alert_reason_esc,
        static_cast<unsigned>(data.alert_flags), data.calib_needed ? 1 : 0,
        calib_reason_esc, data.bme680_ready ? 1 : 0,
        data.pms5003_ready ? 1 : 0, data.mq135_ready ? 1 : 0,
        data.data_valid ? 1 : 0, static_cast<unsigned>(data.cycle_time_ms));

    if (n < 0) {
        ESP_LOGE(TAG, "fprintf lỗi khi ghi %s", active_log_path_);
        fclose(f);
        return ESP_FAIL;
    }

    data_file_bytes_ += n;
    last_log_timestamp_ = data.timestamp;

    bool write_err = false;
    data_rows_pending_fsync_++;
    if (data_rows_pending_fsync_ >= Cfg::SD_FSYNC_EVERY_N_ROWS) {
        if (fflush(f) != 0 || fsync(fileno(f)) != 0) {
            write_err = true;
        }
        data_rows_pending_fsync_ = 0;
    }
    if (fclose(f) != 0) {
        write_err = true;
    }

    if (write_err) {
        ESP_LOGE(TAG, "Ghi %s thất bại (errno=%d) — có thể thẻ đầy hoặc lỗi phần cứng",
                 active_log_path_, errno);
        return ESP_FAIL;
    }

    if (day_key == kFallbackLogKey && data_file_bytes_ >= Cfg::SD_LOG_MAX_BYTES) {
        rotateDataLog(data);
    }

    return ESP_OK;
}

esp_err_t StorageHelper::rotateDataLog(const AirData &data) {
    char oldest[80], src[80], dst[80];

    rotatedPath(oldest, sizeof(oldest), Cfg::SD_LOG_MAX_FILES - 1);
    remove(oldest);

    for (int i = Cfg::SD_LOG_MAX_FILES - 2; i >= 1; --i) {
        rotatedPath(src, sizeof(src), i);
        rotatedPath(dst, sizeof(dst), i + 1);
        rename(src, dst);
    }

    rotatedPath(dst, sizeof(dst), 1);
    if (rename(Cfg::SD_LOG_FALLBACK_FILE, dst) != 0) {
        ESP_LOGW(TAG, "Rotate: rename %s -> %s lỗi (errno=%d)",
                 Cfg::SD_LOG_FALLBACK_FILE, dst, errno);
    }

    data_file_bytes_ = 0;
    data_rows_pending_fsync_ = 0;
    esp_err_t err = ensureHeader(Cfg::SD_LOG_FALLBACK_FILE, kDataHeader, data_file_bytes_);

    ESP_LOGI(TAG, "SD_LOG_FALLBACK_FILE rotated (> %ld B) -> %s",
             static_cast<long>(Cfg::SD_LOG_MAX_BYTES), dst);

    writeEventRow(EventType::SD_LOG_ROTATED, data);

    return err;
}

esp_err_t StorageHelper::writeEventRow(EventType type,
                                        const AirData &snapshot) {
    if (!mounted_) return ESP_ERR_INVALID_STATE;

    char alert_reason_esc[48], calib_reason_esc[48];
    csvEscape(snapshot.alert_reason, alert_reason_esc, sizeof(alert_reason_esc));
    csvEscape(snapshot.calib_reason, calib_reason_esc, sizeof(calib_reason_esc));

    FILE *f = fopen(Cfg::SD_EVENT_LOG_FILE, "a");
    if (!f) {
        ESP_LOGE(TAG, "Không thể mở %s để ghi", Cfg::SD_EVENT_LOG_FILE);
        return ESP_FAIL;
    }

    int n = fprintf(f, "%lld,%d,%s,%u,%s,0x%04X,%d,%s\n",
                    static_cast<long long>(snapshot.timestamp),
                    timeValidOf(snapshot.timestamp), eventTypeName(type),
                    static_cast<unsigned>(snapshot.alert_level),
                    alert_reason_esc,
                    static_cast<unsigned>(snapshot.alert_flags),
                    snapshot.calib_needed ? 1 : 0, calib_reason_esc);

    if (n < 0) {
        ESP_LOGE(TAG, "fprintf lỗi khi ghi %s", Cfg::SD_EVENT_LOG_FILE);
        fclose(f);
        return ESP_FAIL;
    }

    bool write_err = (fflush(f) != 0) || (fsync(fileno(f)) != 0);
    if (fclose(f) != 0) {
        write_err = true;
    }

    if (write_err) {
        ESP_LOGE(TAG, "Ghi %s thất bại (errno=%d) — có thể thẻ đầy hoặc lỗi phần cứng",
                 Cfg::SD_EVENT_LOG_FILE, errno);
        return ESP_FAIL;
    }

    event_file_bytes_ += n;
    if (event_file_bytes_ >= Cfg::SD_EVENT_MAX_BYTES) {
        ESP_LOGW(TAG,
                 "%s vượt %ld byte — events.csv không tự xoay vòng (§4.5), "
                 "cần dọn thủ công",
                 Cfg::SD_EVENT_LOG_FILE,
                 static_cast<long>(Cfg::SD_EVENT_MAX_BYTES));
    }

    ESP_LOGI(TAG, "Event: %s", eventTypeName(type));
    return ESP_OK;
}

const char *StorageHelper::eventTypeName(EventType type) {
    switch (type) {
        case EventType::ALERT_LEVEL_CHANGED:    return "ALERT_LEVEL_CHANGED";
        case EventType::ALERT_REASON_CHANGED:   return "ALERT_REASON_CHANGED";
        case EventType::CALIB_NEEDED_SET:       return "CALIB_NEEDED_SET";
        case EventType::CALIB_CONFIRMED:        return "CALIB_CONFIRMED";
        case EventType::MQTT_CONNECTED:         return "MQTT_CONNECTED";
        case EventType::MQTT_DISCONNECTED:      return "MQTT_DISCONNECTED";
        case EventType::SYSTEM_BOOT:            return "SYSTEM_BOOT";
        case EventType::SD_LOG_ROTATED:         return "SD_LOG_ROTATED";
        case EventType::ALERT_LATENCY_EXCEEDED: return "ALERT_LATENCY_EXCEEDED";
        case EventType::WARMUP_SKIPPED:         return "WARMUP_SKIPPED";
    }
    return "UNKNOWN";
}

esp_err_t StorageHelper::pushOfflineRecord(const AirData &data) {
    if (!mounted_) return ESP_ERR_INVALID_STATE;

    struct stat st {};
    if (stat(Cfg::SD_OFFLINE_QUEUE, &st) != 0 || st.st_size == 0) {

        FILE *hf = fopen(Cfg::SD_OFFLINE_QUEUE, "wb");
        if (!hf) {
            ESP_LOGE(TAG, "Không thể tạo %s", Cfg::SD_OFFLINE_QUEUE);
            return ESP_FAIL;
        }
        OfflineFileHeader hdr{Cfg::OFFLINE_MAGIC, Cfg::OFFLINE_FORMAT_VERSION, 0};
        fwrite(&hdr, sizeof(hdr), 1, hf);
        fflush(hf);
        fsync(fileno(hf));
        fclose(hf);
        writeOfflineHead(0);
        offline_count_ = 0;
    }

    FILE *f = fopen(Cfg::SD_OFFLINE_QUEUE, "ab");
    if (!f) {
        ESP_LOGE(TAG, "Không thể mở %s để append", Cfg::SD_OFFLINE_QUEUE);
        return ESP_FAIL;
    }
    size_t written = fwrite(&data, sizeof(AirData), 1, f);
    fflush(f);
    fsync(fileno(f));
    fclose(f);

    if (written != 1) {
        ESP_LOGE(TAG, "Ghi offline record thất bại (%s)", Cfg::SD_OFFLINE_QUEUE);
        return ESP_FAIL;
    }

    offline_count_++;

    if (offline_count_ > Cfg::OFFLINE_QUEUE_MAX_RECORDS) {

        uint32_t head = 0;
        esp_err_t read_err = readOfflineHead(head);
        if (read_err != ESP_OK && read_err != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Đọc %s lỗi (%s) — bỏ qua drop lượt này",
                     Cfg::SD_OFFLINE_HEAD, esp_err_to_name(read_err));
        } else {
            esp_err_t write_err = writeOfflineHead(head + 1);
            if (write_err != ESP_OK) {
                ESP_LOGE(TAG, "Ghi %s lỗi — chưa drop được, sẽ thử lại",
                         Cfg::SD_OFFLINE_HEAD);
            } else {
                offline_count_--;
                ESP_LOGW(TAG,
                         "Offline queue đầy (> %u) — drop record cũ nhất (head=%u)",
                         static_cast<unsigned>(Cfg::OFFLINE_QUEUE_MAX_RECORDS),
                         static_cast<unsigned>(head + 1));
            }
        }
    }

    return ESP_OK;
}

esp_err_t StorageHelper::drainOfflineRecords(const PublishFn &publish_fn) {
    if (!mounted_ || !publish_fn) return ESP_ERR_INVALID_STATE;

    struct stat st {};
    if (stat(Cfg::SD_OFFLINE_QUEUE, &st) != 0) {
        offline_count_ = 0;
        return ESP_OK;
    }

    long total_records =
        (static_cast<long>(st.st_size) -
         static_cast<long>(sizeof(OfflineFileHeader))) /
        static_cast<long>(sizeof(AirData));
    if (total_records < 0) total_records = 0;

    uint32_t head = 0;
    readOfflineHead(head);
    if (head > static_cast<uint32_t>(total_records)) {
        head = static_cast<uint32_t>(total_records);
    }

    FILE *f = fopen(Cfg::SD_OFFLINE_QUEUE, "rb");
    if (!f) {
        ESP_LOGW(TAG, "Không thể mở %s để drain", Cfg::SD_OFFLINE_QUEUE);
        return ESP_FAIL;
    }

    size_t sent = 0;
    AirData record;
    while (head < static_cast<uint32_t>(total_records) &&
           sent < Cfg::OFFLINE_DRAIN_BATCH) {
        long offset = static_cast<long>(sizeof(OfflineFileHeader)) +
                      static_cast<long>(head) *
                          static_cast<long>(sizeof(AirData));
        if (fseek(f, offset, SEEK_SET) != 0) break;
        if (fread(&record, sizeof(AirData), 1, f) != 1) break;

        esp_err_t pub_err = publish_fn(record);
        if (pub_err == ESP_OK) {
            head++;
            sent++;

            vTaskDelay(pdMS_TO_TICKS(Cfg::OFFLINE_DRAIN_PACE_MS));
        } else if (pub_err == ESP_ERR_INVALID_STATE) {

            ESP_LOGW(TAG,
                     "Drain dừng giữa chừng (mất kết nối) — còn %ld record",
                     total_records - static_cast<long>(head));
            break;
        } else {
            ESP_LOGW(TAG, "publish_fn lỗi (%s) — dừng drain, giữ record",
                     esp_err_to_name(pub_err));
            break;
        }
    }
    fclose(f);

    if (head >= static_cast<uint32_t>(total_records)) {

        remove(Cfg::SD_OFFLINE_QUEUE);
        remove(Cfg::SD_OFFLINE_HEAD);
        offline_count_ = 0;
        ESP_LOGI(TAG, "Offline queue đã drain hết (%u record phát lại)",
                 static_cast<unsigned>(sent));
    } else {
        writeOfflineHead(head);
        offline_count_ =
            static_cast<size_t>(total_records - static_cast<long>(head));
        ESP_LOGI(TAG, "Đã phát lại %u record offline, còn %u",
                 static_cast<unsigned>(sent),
                 static_cast<unsigned>(offline_count_));

        if (head >= Cfg::OFFLINE_DRAIN_BATCH) {
            compactOfflineQueue(head, total_records);
        }

        if (sent == Cfg::OFFLINE_DRAIN_BATCH) {
            StorageMsg again{};
            again.kind = MsgKind::OFFLINE_DRAIN;
            xQueueSend(storage_queue_, &again, 0);
        }
    }

    return ESP_OK;
}

esp_err_t StorageHelper::restoreOfflineState() {
    offline_count_ = 0;

    struct stat st {};
    if (stat(Cfg::SD_OFFLINE_QUEUE, &st) != 0) {
        return ESP_OK;
    }

    FILE *f = fopen(Cfg::SD_OFFLINE_QUEUE, "rb");
    if (!f) {
        ESP_LOGW(TAG, "Không thể mở %s để khôi phục", Cfg::SD_OFFLINE_QUEUE);
        return ESP_FAIL;
    }
    OfflineFileHeader hdr{};
    size_t n = fread(&hdr, sizeof(hdr), 1, f);
    fclose(f);

    if (n != 1 || hdr.magic != Cfg::OFFLINE_MAGIC ||
        hdr.version != Cfg::OFFLINE_FORMAT_VERSION) {

        ESP_LOGW(TAG, "%s: MAGIC/VERSION không khớp — bỏ file cũ",
                 Cfg::SD_OFFLINE_QUEUE);
        remove(Cfg::SD_OFFLINE_QUEUE);
        remove(Cfg::SD_OFFLINE_HEAD);
        return ESP_OK;
    }

    long total_records =
        (static_cast<long>(st.st_size) -
         static_cast<long>(sizeof(OfflineFileHeader))) /
        static_cast<long>(sizeof(AirData));
    if (total_records < 0) total_records = 0;

    uint32_t head = 0;
    readOfflineHead(head);
    if (head > static_cast<uint32_t>(total_records)) {
        head = static_cast<uint32_t>(total_records);
    }

    offline_count_ = static_cast<size_t>(total_records - static_cast<long>(head));
    ESP_LOGI(TAG, "Khôi phục offline queue: %u record tồn dư (head=%u, total=%ld)",
             static_cast<unsigned>(offline_count_),
             static_cast<unsigned>(head), total_records);
    return ESP_OK;
}

esp_err_t StorageHelper::readOfflineHead(uint32_t &head_index) const {
    head_index = 0;
    FILE *f = fopen(Cfg::SD_OFFLINE_HEAD, "rb");
    if (!f) return ESP_ERR_NOT_FOUND;
    size_t n = fread(&head_index, sizeof(head_index), 1, f);
    fclose(f);
    return (n == 1) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

esp_err_t StorageHelper::writeOfflineHead(uint32_t head_index) const {
    FILE *f = fopen(Cfg::SD_OFFLINE_HEAD, "wb");
    if (!f) return ESP_FAIL;
    size_t n = fwrite(&head_index, sizeof(head_index), 1, f);
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    return (n == 1) ? ESP_OK : ESP_FAIL;
}

esp_err_t StorageHelper::compactOfflineQueue(uint32_t head, long total_records) {
    long remaining = total_records - static_cast<long>(head);
    if (remaining <= 0) {
        remove(Cfg::SD_OFFLINE_QUEUE);
        remove(Cfg::SD_OFFLINE_HEAD);
        return ESP_OK;
    }

    FILE *src = fopen(Cfg::SD_OFFLINE_QUEUE, "rb");
    if (!src) return ESP_FAIL;

    char tmp_path[96];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", Cfg::SD_OFFLINE_QUEUE);
    FILE *dst = fopen(tmp_path, "wb");
    if (!dst) {
        fclose(src);
        return ESP_FAIL;
    }

    OfflineFileHeader hdr{Cfg::OFFLINE_MAGIC, Cfg::OFFLINE_FORMAT_VERSION, 0};
    fwrite(&hdr, sizeof(hdr), 1, dst);

    long offset = static_cast<long>(sizeof(OfflineFileHeader)) +
                  static_cast<long>(head) * static_cast<long>(sizeof(AirData));
    fseek(src, offset, SEEK_SET);

    AirData rec;
    for (long i = 0; i < remaining; ++i) {
        if (fread(&rec, sizeof(AirData), 1, src) != 1) break;
        fwrite(&rec, sizeof(AirData), 1, dst);
    }
    fflush(dst);
    fsync(fileno(dst));
    fclose(dst);
    fclose(src);

    remove(Cfg::SD_OFFLINE_QUEUE);
    if (rename(tmp_path, Cfg::SD_OFFLINE_QUEUE) != 0) {
        ESP_LOGW(TAG, "Compact: rename %s -> %s lỗi (errno=%d)", tmp_path,
                 Cfg::SD_OFFLINE_QUEUE, errno);
        remove(tmp_path);
        return ESP_FAIL;
    }
    writeOfflineHead(0);
    ESP_LOGI(TAG, "Offline queue compacted: %ld record giữ lại", remaining);
    return ESP_OK;
}

esp_err_t StorageHelper::logData(const AirData &data) {
    if (!mounted_) {
        dropped_data_++;
        if (dropped_data_ % 50 == 1) {
            ESP_LOGW(TAG, "SD_ABSENT — drop logData (dropped_data_=%u)",
                     static_cast<unsigned>(dropped_data_));
        }
        return ESP_ERR_INVALID_STATE;
    }

    StorageMsg msg{};
    msg.kind = MsgKind::DATA;
    msg.payload = data;

    if (xQueueSend(storage_queue_, &msg, 0) != pdTRUE) {
        dropped_data_++;
        if (dropped_data_ % 50 == 1) {
            ESP_LOGW(TAG, "storage_queue_ đầy — drop logData (dropped_data_=%u)",
                     static_cast<unsigned>(dropped_data_));
        }
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t StorageHelper::logEvent(EventType type, const AirData &snapshot) {
    if (!mounted_) {
        dropped_event_++;
        if (dropped_event_ % 50 == 1) {
            ESP_LOGW(TAG, "SD_ABSENT — drop logEvent (dropped_event_=%u)",
                     static_cast<unsigned>(dropped_event_));
        }
        return ESP_ERR_INVALID_STATE;
    }

    StorageMsg msg{};
    msg.kind = MsgKind::EVENT;
    msg.evt = type;
    msg.payload = snapshot;

    if (xQueueSend(storage_queue_, &msg, 0) != pdTRUE) {
        dropped_event_++;
        if (dropped_event_ % 50 == 1) {
            ESP_LOGW(TAG, "storage_queue_ đầy — drop logEvent (dropped_event_=%u)",
                     static_cast<unsigned>(dropped_event_));
        }
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t StorageHelper::bufferOffline(const AirData &data) {
    if (!mounted_) {
        dropped_data_++;
        if (dropped_data_ % 50 == 1) {
            ESP_LOGW(TAG, "SD_ABSENT — drop bufferOffline (dropped_data_=%u)",
                     static_cast<unsigned>(dropped_data_));
        }
        return ESP_ERR_INVALID_STATE;
    }

    StorageMsg msg{};
    msg.kind = MsgKind::OFFLINE_PUSH;
    msg.payload = data;

    if (xQueueSend(storage_queue_, &msg, 0) != pdTRUE) {
        dropped_data_++;
        if (dropped_data_ % 50 == 1) {
            ESP_LOGW(TAG, "storage_queue_ đầy — drop bufferOffline (dropped_data_=%u)",
                     static_cast<unsigned>(dropped_data_));
        }
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t StorageHelper::drainOffline(PublishFn publish_fn) {
    if (!mounted_ || !publish_fn) return ESP_ERR_INVALID_STATE;

    drain_publish_fn_ = publish_fn;

    StorageMsg msg{};
    msg.kind = MsgKind::OFFLINE_DRAIN;

    if (xQueueSend(storage_queue_, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "storage_queue_ đầy — drop drainOffline request");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t StorageHelper::prepareShutdown() {
    if (!mounted_) return ESP_OK;

    StorageMsg msg{};
    msg.kind = MsgKind::SHUTDOWN;

    if (xQueueSend(storage_queue_, &msg, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "storage_queue_ đầy — drop prepareShutdown request");
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(shutdown_done_, pdMS_TO_TICKS(Cfg::SD_SHUTDOWN_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "prepareShutdown timeout sau %ums", (unsigned)Cfg::SD_SHUTDOWN_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "SD đã unmount an toàn — sẵn sàng rút nguồn");
    return ESP_OK;
}

bool StorageHelper::isMounted() const { return mounted_; }

size_t StorageHelper::offlineCount() const { return offline_count_; }

void StorageHelper::taskStorage(void *arg) {
    static_cast<StorageHelper *>(arg)->taskLoop();
}

void StorageHelper::taskLoop() {
    StorageMsg msg;
    for (;;) {

        if (xQueueReceive(storage_queue_, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (msg.kind) {
            case MsgKind::DATA:
                writeDataRow(msg.payload);
                break;
            case MsgKind::EVENT:
                writeEventRow(msg.evt, msg.payload);
                break;
            case MsgKind::OFFLINE_PUSH:
                pushOfflineRecord(msg.payload);
                break;
            case MsgKind::OFFLINE_DRAIN:
                drainOfflineRecords(drain_publish_fn_);
                break;
            case MsgKind::SHUTDOWN:
                if (mounted_ && card_) {
                    esp_vfs_fat_sdcard_unmount(Cfg::SD_MOUNT_POINT, card_);
                    card_ = nullptr;
                }
                mounted_ = false;
                xSemaphoreGive(shutdown_done_);
                break;
        }
    }
}

size_t StorageHelper::csvEscape(const char *in, char *out, size_t out_sz) {
    if (out_sz == 0) return 0;
    size_t j = 0;
    for (size_t i = 0; in[i] != '\0' && j + 1 < out_sz; ++i) {
        char c = in[i];
        switch (c) {
            case ',':  out[j++] = ';';  break;
            case '"':  out[j++] = '\''; break;
            case '\n':
            case '\r': out[j++] = ' ';  break;
            default:   out[j++] = c;    break;
        }
    }
    out[j] = '\0';
    return j;
}

int StorageHelper::timeValidOf(int64_t timestamp) {
    return (timestamp < kY2020Epoch) ? 0 : 1;
}

# Known Issues — MQTT / Task Watchdog (2026-07-02)

Phát hiện từ log thực tế khi Wi-Fi/MQTT chập chờn kéo dài. Bug rename() trên
StorageHelper (offline queue compaction, mục 2), comment sai enum
`esp_mqtt_error_type_t` + thiếu chi tiết chẩn đoán transport error, backoff
reconnect quá dài so với NFR cảnh báo (mục 3), và publish blocking gây Task
Watchdog (mục 1) đã được fix trực tiếp trong code. Mục 3 và mục 1 **cần theo
dõi log thực tế sau khi flash** (xem phần "rủi ro đã cân nhắc" ở mỗi mục).

## 1. `esp_mqtt_client_publish()` blocking → Task Watchdog trigger trên task `network`

- **File:** `main/NetworkManager.cpp:216` (`publishData`/`publishJson`), tương tự
  `main/NetworkManager.cpp:342` (phản hồi RPC lệnh).
- **Triệu chứng quan sát:**
  ```
  E mqtt_client: Writing didn't complete in specified timeout: errno=119
  E mqtt_client: Error to resend data
  ...
  E task_wdt: Task watchdog got triggered ... - network (CPU 0/1)
  ```
- **Nguyên nhân:** `esp_mqtt_client_publish(..., qos=1, ...)` là API **blocking**,
  có thể chờ tới `network_timeout_ms` (mặc định 10000ms) nếu TCP write không
  hoàn tất — đúng tình huống khi Wi-Fi/MQTT vừa rớt/kết nối lại còn chập chờn.
  `taskNetwork` (`main/main.cpp:218-298`) gọi hàm này trực tiếp trong vòng lặp
  chính và chỉ `esp_task_wdt_reset()` **sau khi** publish trả về
  (`main/main.cpp:296`) — không có reset nào trong lúc chờ.
  `CONFIG_ESP_TASK_WDT_TIMEOUT_S=10` (`sdkconfig:1722`) gần như trùng khớp với
  `network_timeout_ms` mặc định của mqtt client → 1 lần publish bị nghẽn là đủ
  (hoặc cộng dồn với json build/logEvent/persistBaselineIfDirty cùng chu kỳ) để
  vượt mốc WDT. Khi mạng còn bất ổn kéo dài, gần như mọi chu kỳ đều có nguy cơ
  lặp lại → WDT trigger liên tục (không phải domino thật, mà là cùng 1 causal
  path lặp lại mỗi chu kỳ).
- **Trạng thái:** **đã fix** — thay `esp_mqtt_client_publish()` bằng
  `esp_mqtt_client_enqueue(mqtt_, topic, json, n, /*qos=*/1, /*retain=*/0,
  /*store=*/true)` ở cả 2 nơi: `publishJson()` (dữ liệu định kỳ + alert,
  `NetworkManager.cpp`) và callback phản hồi RPC lệnh (chạy trong chính
  context của mqtt task nội bộ — blocking ở đây còn rủi ro hơn vì có thể tự
  nghẽn task đang lo việc gửi/nhận MQTT của chính nó). API `enqueue()` chỉ đẩy
  message vào outbox nội bộ và trả về gần như ngay lập tức (không chờ I/O
  mạng) — việc gửi/retry thực sự do task nội bộ của thư viện esp-mqtt xử lý
  bất đồng bộ. Giữ nguyên QoS 1 nhờ `store=true`, không đổi hành vi publish
  quan sát được từ broker, chỉ loại bỏ khả năng block caller task quá lâu.
- **Giới hạn outbox tường minh (`cfg.outbox.limit`, `initMqtt()`):** mặc định
  thư viện (`limit=0`) là **không giới hạn** — outbox có thể phình tới hết
  heap nếu mạng chập chờn kéo dài mà `MQTT_EVENT_DISCONNECTED` chưa kịp fire
  (khoảng trống giữa "network thực sự xấu" và "app phát hiện mất kết nối"),
  gây rủi ro OOM khó lường thay vì một lỗi rõ ràng. Đặt
  `Cfg::MQTT_OUTBOX_LIMIT_BYTES = 16*1024` (`config.hpp`) — đủ chứa ~20+ bản
  tin (mỗi bản tin ~`MQTT_JSON_BUF_LEN` (640B) + topic string + overhead nội
  bộ outbox, ước lượng ~700B/msg), bao trùm cửa sổ phát hiện mất kết nối xấu
  nhất theo TCP keep-alive vừa cấu hình ở mục 3 (~35s: `idle=20s +
  interval=5s*count=3`) ở nhịp publish 2s (`SENSOR_READ_INTERVAL_MS`). Vượt
  giới hạn → `enqueue()` trả `-2`, `publishJson()` coi như lỗi (rơi vào cùng
  nhánh xử lý lỗi hiện có, không phải luồng chính nên chấp nhận rơi rụng thay
  vì OOM).
- **Kiểm tra ngân sách bộ nhớ ESP32 trước khi áp dụng:** dự án không dùng
  SPIRAM (`CONFIG_ESP32_SPIRAM_SUPPORT` tắt, `sdkconfig:3442`) — toàn bộ nằm
  trong 520KB SRAM on-chip. Không dùng BT. Kết nối broker qua `mqtt://` (TCP
  thường, `sdkconfig:709`) chứ không phải `mqtts://` nên **không** phát sinh
  buffer TLS runtime cho session MQTT (mbedTLS trong `sdkconfig` chỉ phục vụ
  mục đích khác, không active cho kết nối này). Tổng stack 4 task ứng dụng đã
  cấp cố định (`config.hpp`): sensor 4096w + network 6144w + display 3072w +
  storage 4096w = 17408 words (~68KB) + main task 3584w (~14KB) + 2 idle task
  1536w (~12KB) — không đổi bởi fix này. 16KB outbox mới cộng thêm là mức nhỏ
  so với heap khả dụng thực tế lúc runtime (thường còn hàng trăm KB sau khi
  WiFi driver cấp phát buffer động) — **không có dấu hiệu quá tải bộ nhớ**,
  nhưng nên xác nhận lại bằng `esp_get_free_heap_size()` /
  `esp_get_minimum_free_heap_size()` sau khi flash thực tế (không build/flash
  trong phiên làm việc này).
- **Rủi ro đã cân nhắc, không chặn fix:** `enqueue()` chỉ đảm bảo "đã xếp hàng
  vào outbox thành công", KHÔNG còn đảm bảo đồng bộ tại thời điểm gọi rằng
  broker đã nhận — khác với `publish()` cũ (blocking, kết quả trả về phản ánh
  gần hơn trạng thái gửi thật). Hệ quả: nếu mạng xấu nhưng `mqtt_connected_`
  (flag app-level, cập nhật qua event) chưa kịp chuyển `false`, bản tin vẫn
  được coi là "publish OK" (không rơi vào nhánh `bufferOffline` ở
  `main.cpp:262`) dù thực tế có thể còn nằm trong outbox và bị esp-mqtt tự
  loại bỏ âm thầm sau `CONFIG_MQTT_OUTBOX_EXPIRED_TIMEOUT_MS=30000`
  (`sdkconfig.defaults:24`) nếu không gửi kịp — không có `ESP_LOGE` riêng cho
  từng bản tin bị rớt kiểu này. Không phải mất dữ liệu nhiều hơn trước (bản
  tin timeout cũ cũng bị drop, cũng không buffer), chỉ là **giảm khả năng
  quan sát qua log** đúng lúc mạng chập chờn — **cần theo dõi log sau khi
  flash**: nếu nghi ngờ dữ liệu tới broker trễ mà không thấy log lỗi tương
  ứng, dùng `esp_mqtt_client_get_outbox_size()` để debug thời gian thực thay
  vì suy luận từ return code của `publishJson()`.

## 2. (Đã fix) FatFs `rename()` không ghi đè file đích khi nén offline queue

- **File:** `main/StorageHelper.cpp:697` (`compactOfflineQueue`).
- **Triệu chứng:** `Compact: rename ... lỗi (errno=17)` — EEXIST.
- **Nguyên nhân:** code giả định `rename()` có ngữ nghĩa POSIX (tự động ghi đè
  file đích nếu đã tồn tại), nhưng FatFs (`f_rename()` — driver đứng sau
  `esp_vfs_fat` cho SD card) trả `FR_EXIST` nếu đích đã tồn tại. Vì
  `offline_queue.bin` luôn tồn tại tại thời điểm nén, `rename()` fail 100% mọi
  lần gọi — bug có sẵn từ trước, chỉ lộ ra khi `offline_count_` vượt
  `Cfg::OFFLINE_QUEUE_MAX_RECORDS` (2000, `config.hpp:143`) đủ để kích hoạt
  compaction lần đầu (đợt mất mạng kéo dài khiến offline buffer dồn nhiều hơn
  bình thường).
- **Hệ quả nếu không fix:** offline queue không bao giờ được nén lại, file
  phình to dần trên SD card theo thời gian (không mất dữ liệu, nhưng tốn
  dung lượng và I/O đọc/ghi ngày càng lớn).
- **Trạng thái:** **đã fix** — thêm `remove(Cfg::SD_OFFLINE_QUEUE)` trước
  `rename()` để xoá đích trước khi ghi đè. Đánh đổi: mất tính atomic tuyệt đối
  nếu mất điện đúng lúc giữa `remove()` và `rename()` — giới hạn cố hữu của
  FAT, không tránh được hoàn toàn bằng API chuẩn.

## 3. `transport_read(): EOF` / `errno=128` (ENOTCONN) — broker đóng kết nối TCP giữa chừng

- **Log quan sát:**
  ```
  E mqtt_client: esp_mqtt_handle_transport_read_error: transport_read(): EOF
  E mqtt_client: esp_mqtt_handle_transport_read_error: transport_read() error: errno=128
  E NetworkManager: MQTT loại lỗi=1
  E mqtt_client: mqtt_process_receive: mqtt_message_receive() returned -2
  ```
- **Ý nghĩa từng dòng:** `EOF` = lần `read()` đầu tiên trên socket TCP tới broker trả về
  0 byte (peer gửi FIN — broker hoặc thiết bị trung gian chủ động đóng kết nối,
  không phải ESP32 chủ động ngắt). `errno=128` = `ENOTCONN`, xảy ra ở lần `read()`
  **kế tiếp** sau khi socket đã đóng — hệ quả của EOF, không phải nguyên nhân độc
  lập. `mqtt_message_receive() returned -2` là cách esp-mqtt lan truyền lỗi đọc
  transport lên state machine để trigger `MQTT_EVENT_DISCONNECTED` + reconnect.
  `MQTT loại lỗi=1` = `MQTT_ERROR_TYPE_TCP_TRANSPORT` (không phải giá trị "PAHO"
  như comment cũ ghi sai — xem mục sửa bên dưới), đúng với bản chất lỗi tầng
  transport quan sát được.
- **Đây có phải bug logic không?** Không, tự nó không phải crash/bug logic — `cfg.network.disable_auto_reconnect = false`
  (`main/NetworkManager.cpp:154`) nên esp-mqtt tự động reconnect, và
  `taskNetwork` (`main/main.cpp`) đã buffer dữ liệu xuống SD qua `StorageHelper`
  khi `isConnected()==false` nên không mất dữ liệu. Nguyên nhân gốc (vì sao
  broker/network đóng TCP session) nằm ngoài phạm vi code.
- **Đối chiếu lại giả thuyết "idle timeout" (đã LOẠI BỎ):** giả thuyết ban đầu
  cho rằng broker/NAT đóng session do idle rồi đề xuất rút ngắn
  `session.keepalive` — kiểm tra lại `sdkconfig` cho thấy giả thuyết này **không
  đủ căn cứ**: broker thật đang dùng là `mqtt.thingsboard.cloud:1883`
  (`sdkconfig:709`, không phải `broker.hivemq.com` mặc định trong
  `Kconfig.projbuild`), và `taskNetwork` gọi `publishData()` mỗi
  `SENSOR_READ_INTERVAL_MS` (mặc định 2000ms, `sdkconfig:741`) — tức kết nối có
  traffic đều đặn mỗi ~2 giây, không hề đủ "idle" để chạm ngưỡng timeout của
  NAT/firewall/broker thông thường (thường >= 60s). Do đó **không áp dụng** fix
  rút ngắn `session.keepalive` như từng đề xuất — không có cơ sở nó giải quyết
  đúng nguyên nhân, và không hợp lý khi áp một fix chưa được chứng minh cho một
  bug lặp lại thường xuyên (yêu cầu người phụ trách 2026-07-02).
- **Nguyên nhân khả dĩ nhất sau khi loại giả thuyết idle:** broker chủ động gửi
  FIN dù đang có traffic → phù hợp với đặc điểm ThingsBoard **Cloud** (dịch vụ
  SaaS multi-tenant dùng chung, không phải broker tự host) — có thể do tái cấp
  phát/cân bằng tải kết nối phía server, giới hạn rate-limit theo
  device/tenant của gói Cloud đang dùng, hoặc do hạ tầng mạng giữa ESP32 và
  Internet chập chờn (khớp bối cảnh đầu file). Đây là nguyên nhân **ngoài tầm
  kiểm soát của code thiết bị** — không thể "fix" triệt để từ phía ESP32.
- **Bug thật sự tìm thấy — comment sai về `esp_mqtt_error_type_t`:** `main/NetworkManager.cpp:350`
  (trước khi sửa) ghi `// error_type: 0=TCP transport, 1=PAHO, 2=TLS, 3=DNS` —
  sai cả 4 giá trị so với enum thật trong
  `managed_components/espressif__mqtt/include/mqtt_client.h:117-122`:
  `0=MQTT_ERROR_TYPE_NONE, 1=MQTT_ERROR_TYPE_TCP_TRANSPORT (bao gồm cả lỗi TLS,
  xem alias MQTT_ERROR_TYPE_ESP_TLS), 2=MQTT_ERROR_TYPE_CONNECTION_REFUSED,
  3=MQTT_ERROR_TYPE_SUBSCRIBE_FAILED`. Comment sai này có thể khiến người debug
  sau này đọc nhầm giá trị `error_type` khác (vd. 2) thành "lỗi TLS" trong khi
  thực tế là broker từ chối kết nối (`CONNECTION_REFUSED`) — hai hướng chẩn
  đoán hoàn toàn khác nhau.
- **Trạng thái:** **đã fix** trong `main/NetworkManager.cpp` (case
  `MQTT_EVENT_ERROR`) — sửa lại comment cho khớp enum thật, đồng thời log thêm
  `esp_transport_sock_errno` / `esp_tls_last_esp_err` / `esp_tls_stack_err` từ
  `evt->error_handle` để `ESP_LOGE` của `NetworkManager` tự đủ thông tin chẩn
  đoán (errno gốc, lỗi TLS nếu có) mà không phải đối chiếu log rời rạc của
  component `mqtt_client`.
- **Fix áp dụng (2026-07-02) — vì lỗi lặp lại thường xuyên trên thực tế:** không thể
  ngăn broker chủ động đóng kết nối (nguyên nhân ngoài code), nên fix tập
  trung vào **giảm thời gian thiết bị ở trạng thái ngắt kết nối** thay vì cố
  ngăn việc ngắt xảy ra:
  1. **`cfg.network.reconnect_timeout_ms = 3000`** (`main/NetworkManager.cpp`,
     `initMqtt()`) — mặc định của esp-mqtt là 10000ms, tự nó đã vượt
     `Cfg::ALERT_MAX_LATENCY_MS` (3000ms, §3) trước cả khi thử reconnect lần
     đầu. Rút xuống 3000ms để không tự làm hỏng NFR cảnh báo mỗi khi broker
     ngắt rồi có lại ngay. Vẫn đủ lớn để tránh spam broker khi mất mạng thật
     sự kéo dài (lúc đó `esp_wifi_connect()` retry của tầng Wi-Fi thất bại
     nhanh, không tốn full TCP timeout).
  2. **TCP-level keep-alive** (`cfg.network.tcp_keep_alive_cfg`, `keep_alive_enable=true`,
     `idle=20s`, `interval=5s`, `count=3`) — bổ sung phòng trường hợp KHÁC với
     log đã quan sát: socket "chết" âm thầm (NAT/router drop mapping, không
     gửi FIN) mà esp-mqtt không tự biết cho tới I/O kế tiếp. Không thay thế
     MQTT keepalive (`session.keepalive`, tầng ứng dụng) — bổ sung ở tầng TCP.
  - **Đối chiếu CLAUDE.md trước khi áp dụng:** không vi phạm §3 (Non-blocking —
    cả 2 thay đổi chỉ là struct config truyền vào `esp_mqtt_client_init()`,
    không thêm blocking call nào); không vi phạm NFR công suất <=2.0W — vài
    gói TCP keep-alive probe mỗi 20s không đáng kể so với chu kỳ publish thực
    tế đã ~2s/lần (radio vốn đã thức thường xuyên hơn nhiều); không đụng tới
    quy tắc I2C/WDT/NVS ở các mục khác. Trực tiếp phục vụ NFR §3 "đẩy sự kiện
    lên Cloud <=3s" bằng cách giảm backoff trước lần reconnect đầu.
  - **Rủi ro đã cân nhắc, không chặn fix:** reconnect nhanh hơn (3s so với 10s)
    về lý thuyết có thể làm tệ hơn nếu nguyên nhân là ThingsBoard Cloud
    rate-limit theo tần suất kết nối — 3s vẫn là mức thận trọng (không phải
    reconnect ngay lập tức/backoff=0), nhưng **cần theo dõi log sau khi
    flash**: nếu tần suất `MQTT_EVENT_DISCONNECTED` KHÔNG giảm hoặc tăng lên
    sau fix, đó là dấu hiệu rate-limit phía ThingsBoard Cloud (kiểm tra tenant
    profile / device rate limit trên dashboard ThingsBoard) chứ không phải vấn
    đề có thể giải quyết thêm từ phía firmware.

## Ghi chú

- Bug #1 và #2 **độc lập về nguyên nhân**, chỉ tình cờ cùng lộ diện trong một
  đợt Wi-Fi/MQTT chập chờn kéo dài (đợt mất mạng → offline buffer đầy → nén
  bị lỗi; đồng thời reconnect liên tục → publish blocking → WDT). Không phải
  một chuỗi domino nhân-quả duy nhất.
- Không có bug nào trong danh sách này liên quan đến `StorageHelper::taskStorage`
  bị WDT — task này **không đăng ký `esp_task_wdt_add`** theo đúng chủ đích
  thiết kế (CLAUDE.md §4, mục "StorageHelper (I/O chậm)").

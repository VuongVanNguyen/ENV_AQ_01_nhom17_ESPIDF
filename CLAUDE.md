# CLAUDE.md - ENV-AQ-01 Project Instructions

File này cung cấp hướng dẫn cho Claude khi làm việc trong repository của dự án ENV-AQ-01: Trạm Quan Trắc Không Khí Đa Thông Số.

## 1. Tổng quan dự án (Project Overview)
Hệ thống quan trắc dựa trên ESP32, thu thập dữ liệu từ BME680, PMS5003, và MQ-135. Tính toán các chỉ số đầu ra: **AQI** (theo tiêu chuẩn US EPA), **Comfort Index** (THI — Temperature-Humidity Index, từ nhiệt độ và độ ẩm), **nồng độ CO2**, cùng Nhiệt độ (T), Độ ẩm (RH) và Áp suất (P); hiển thị lên LCD và truyền dữ liệu qua MQTT. Không còn tính chỉ số TVOC.
- **Framework:** ESP-IDF (Espressif IoT Development Framework), phát triển thông qua **ESP-IDF Extension trên Visual Studio Code**.
- **Build System:** CMake + Ninja (được quản lý bởi ESP-IDF toolchain).
- **Ngôn ngữ lập trình:** C++ (chuẩn C++17), sử dụng trực tiếp ESP-IDF APIs — không dùng Arduino HAL hay bất kỳ Arduino wrapper nào. Lý do chọn C++: kiến trúc đa module của dự án (SensorManager, NetworkManager, Filters...) hưởng lợi rõ từ encapsulation (class), RAII, và STL (`std::queue` cho Offline Buffer, `std::array` cho filter window). Overhead RAM/Flash của C++ trên ESP32 (520KB SRAM, 4MB+ Flash) là không đáng kể. Entry point bắt buộc khai báo `extern "C" void app_main()`.
- **Mục tiêu chính:** Độ chính xác cao, hoạt động thời gian thực, tiết kiệm GPIO.

## 2. Kiến trúc & Phân tách Module
Toàn bộ module chia sẻ struct `AirData` (định nghĩa trong `main/include/DataStructures.hpp`).

**Luồng dữ liệu:** `SensorManager` (Đọc thô) → `Filters` (Lọc nhiễu) → `DataFusion` (Tính AQI, Comfort Index/THI, CO2) → `DisplayManager` (LCD) + `NetworkManager` (MQTT) + `StorageHelper` (SD Card).

**Cấu trúc thư mục chuẩn ESP-IDF:**
```
project_root/
├── CMakeLists.txt              # Top-level CMake (chỉ gọi cmake_minimum_required + project())
├── sdkconfig                   # Cấu hình ESP-IDF (sinh ra bởi idf.py menuconfig, KHÔNG chỉnh tay)
├── sdkconfig.defaults          # Giá trị mặc định cho sdkconfig (commit lên git)
├── Kconfig.projbuild           # Khai báo CONFIG_* cho project (WiFi, MQTT, AQI threshold...)
├── main/
│   ├── CMakeLists.txt          # Khai báo SRCS và INCLUDE_DIRS — xem nội dung bên dưới
│   ├── include/                # Toàn bộ file header (.hpp) đặt tại đây
│   │   ├── DataStructures.hpp  # Định nghĩa struct AirData dùng chung toàn project
│   │   ├── config.hpp          # Các hằng số của namespace Cfg được đặt ở đây 
│   │   ├── SensorManager.hpp
│   │   ├── Filters.hpp
│   │   ├── DataFusion.hpp
│   │   ├── DisplayManager.hpp
│   │   ├── NetworkManager.hpp
│   │   └── StorageHelper.hpp
│   ├── SensorManager.cpp       # Toàn bộ file implementation (.cpp) đặt tại đây
│   ├── Filters.cpp
│   ├── DataFusion.cpp
│   ├── DisplayManager.cpp
│   ├── NetworkManager.cpp
│   ├── StorageHelper.cpp
│   └── main.cpp                # extern "C" void app_main(); khởi tạo và tạo FreeRTOS tasks
└── components/                 # (Tuỳ chọn) Các ESP-IDF managed components bên thứ ba
```
| File/Module | Trách nhiệm chính | ESP-IDF APIs liên quan |
| :--- | :--- | :--- |
| `main/SensorManager.cpp` | Driver BME680 (I2C), PMS5003 (UART), MQ-135 (ADC). | `bme680` + `i2cdev` (esp-idf-lib), `uart_*`, `adc_oneshot_*` |
| `main/Filters.cpp` | Lọc nhiễu tín hiệu cảm biến: EMA cho T/P/Gas (BME680), SMA cho RH (BME680). Outlier rejection dùng sanity check theo range vật lý — không dùng delta-based threshold. | — |
| `main/DataFusion.cpp` | Hợp nhất dữ liệu, tính AQI (US EPA) và Comfort Index theo công thức/thang đo THI (Temperature-Humidity Index, từ T/RH); Drift Self-Check. Không tính TVOC. | — |
| `main/DisplayManager.cpp` | Điều khiển LCD 16x2 thông qua IC mở rộng chân PCF8574 (I2C). Hiển thị 3 chỉ số chính AQI, THI (Comfort Index), CO2; thêm T/RH nếu vừa màn hình, nếu không thì luân phiên (rotate) trang hiển thị. | `i2c_master_*` |
| `main/NetworkManager.cpp` | Quản lý WiFi, MQTT (JSON payload), xử lý Buffer khi mất mạng. | `esp_wifi_*`, `esp_mqtt_client_*` |
| `main/StorageHelper.cpp` | Ghi log dữ liệu định kỳ vào thẻ SD (`.csv`) và ghi nhật ký sự kiện (Event Log) rời rạc. | `esp_vfs_fat_sdmmc_mount`, `sdmmc_*` |
| `main/main.cpp` | Khởi tạo hệ thống (`extern "C" void app_main()`), tạo và điều phối bằng FreeRTOS tasks — không dùng `vTaskDelay()` làm logic chính. | `xTaskCreate`, `esp_event_loop_*` |

> **Lưu ý:** `NetworkManager::setCommandCallback()` phải được gọi trong `main.cpp` — không phải `DataFusion`.

## 3. Ràng buộc kỹ thuật (Constraints & NFR)
- **Năng lượng:** Công suất trung bình <= 2.0W. Sử dụng Modem-sleep (`esp_wifi_set_ps(WIFI_PS_MODEM)`) khi nhàn rỗi.
- **Thời gian thực:** Tổng chu kỳ đọc + xử lý dữ liệu phải hoàn tất trong <= 300ms.
- **Độ trễ:** Cảnh báo (Buzzer/LED) và đẩy sự kiện lên Cloud <= 3s.
- **Hiển thị:** LCD 16x2 cập nhật thông số mỗi 2-5 giây, đảm bảo không gây trễ bus I2C. Ưu tiên hiển thị 3 chỉ số chính: **AQI**, **THI** (Comfort Index) và **nồng độ CO2**. Nhiệt độ (T) và Độ ẩm (RH) được hiển thị thêm nếu vừa màn hình; nếu không đủ chỗ thì chuyển sang chế độ hiển thị luân phiên (rotate theo trang) giữa nhóm chỉ số chính và nhóm T/RH.
- **Độ chính xác sau hiệu chuẩn:** Sai số tổng T ≤ ±0.5°C, RH ≤ ±3%RH. Sai số lặp lại của các chỉ số suy diễn (AQI, Comfort Index/THI, CO2) ≤ 10%. Đây là chỉ tiêu kiểm thử nghiệm thu bắt buộc.
- **Ổn định dài hạn:** Chu kỳ tự kiểm tra độ trôi tham số hiệu chuẩn tối đa 30 ngày. Khi sai lệch phát hiện > 10% so với baseline, hệ thống phải phát cảnh báo yêu cầu tái hiệu chuẩn — không được âm thầm bù trừ sai số mà không thông báo.

## 4. Quy tắc lập trình quan trọng (Key Rules)

- **Non-blocking:** Tuyệt đối không dùng `vTaskDelay()` làm logic timing cho nghiệp vụ chính. Mọi tác vụ định kỳ phải dùng `esp_timer_create()` / `esp_timer_start_periodic()` hoặc FreeRTOS Task kết hợp với `xQueueReceive()` / Event Group để đồng bộ — không blocking toàn bộ task.

- **Watchdog Timer (WDT) — phòng WDT reset:** WDT về bản chất là lỗi **cấp độ task**, nên trách nhiệm chia làm hai vai trò và **cả hai đều bắt buộc**, không cái nào thay thế cái nào:
  - **`main.cpp` (cấu hình + kiến trúc, làm một lần):** cấu hình Task WDT (TWDT) qua `idf.py menuconfig` (`CONFIG_ESP_TASK_WDT_INIT`, `CONFIG_ESP_TASK_WDT_TIMEOUT_S`); phát hiện reset do WDT ở đầu `app_main()` bằng `esp_reset_reason()` (`ESP_RST_TASK_WDT` / `ESP_RST_INT_WDT` / `ESP_RST_WDT`) → `ESP_LOGE` + ghi Event Log `SYSTEM_BOOT` để truy vết, **không reset im lặng** (§6.5); tạo task với priority hợp lý để không task nào độc chiếm core.
  - **Từng module/task (tự nuôi watchdog mỗi chu kỳ):** mọi task có vòng lặp vô hạn dài đăng ký `esp_task_wdt_add(NULL)` ở đầu thân task và gọi `esp_task_wdt_reset()` mỗi vòng lặp, với chu kỳ **ngắn hơn** `CONFIG_ESP_TASK_WDT_TIMEOUT_S`. Không busy-spin giữ CPU: đọc UART (PMS5003) / I2C (BME680) phải có timeout hoặc chờ-trên-tín-hiệu (queue/notify/event group), không `while(1)` chờ data; task prio cao phải nhường CPU đủ để IDLE0/IDLE1 chạy. Trường hợp dùng `vTaskDelay()` để **yield cuối chu kỳ** là hợp lệ (khác với cấm dùng nó làm logic timing nghiệp vụ ở trên).
  - **Interrupt WDT (INT_WDT):** không tắt ngắt / không giữ spinlock hay `I2C_DEV_TAKE_MUTEX` quá lâu; truy cập I2C đi qua `i2cdev`, ISR (nếu có) phải ngắn.
  - **StorageHelper (I/O chậm):** `storageTask` ghi SD có thể block fsync/rename hàng trăm ms → **HOẶC** không add vào TWDT, **HOẶC** đặt timeout đủ rộng để tránh false-positive WDT. Đặt `CONFIG_ESP_TASK_WDT_TIMEOUT_S` lớn hơn nhịp của task subscribe chậm nhất.

- **I2C Shared Bus (esp-idf-lib `i2cdev`):** BME680 và PCF8574 dùng chung **một bus I2C duy nhất**, quản lý qua thư viện `i2cdev` của esp-idf-lib. Subsystem được khởi tạo một lần bằng `i2cdev_init()` (idempotent — có guard static, gọi nhiều lần không leak; `SensorManager::init()` gọi trước, `DisplayManager::init()` chạy sau). Mỗi thiết bị đăng ký bằng hàm `*_init_desc()` riêng (`bme680_init_desc()`, `pcf8574_init_desc()`) tạo ra một `i2c_dev_t`. **Tất cả thiết bị phải dùng cùng một I2C port** (`I2C_NUM_0`) để chia sẻ đúng bus. BME680: địa chỉ **0x76** (SDO nối GND). PCF8574: địa chỉ **0x20** (A0/A1/A2 nối GND). Thread-safety giữa nhiều task được `i2cdev` tự bảo đảm bằng **mutex per-port nội bộ** (`i2c_dev_create_mutex` / `I2C_DEV_TAKE_MUTEX`) — không cần tự tạo `SemaphoreHandle_t` thủ công, nhưng **bắt buộc mọi truy cập I2C phải đi qua API i2cdev**.

- **LCD Control (esp-idf-lib `hd44780` + `pcf8574`):** Giao tiếp với LCD 16x2 qua PCF8574 bằng driver `hd44780` (chế độ 4-bit) của esp-idf-lib, với callback ghi byte xuống `pcf8574_port_write()`. **Tuyệt đối không dùng `LiquidCrystal_I2C`** hay bất kỳ wrapper Arduino nào. Pinout PCF8574→LCD khai báo trong `hd44780_t` phải khớp §5 (RS=P0, E=P2, D4–D7=P4–P7; RW=P1 luôn giữ LOW = write-mode; backlight = P3). Chỉ cập nhật vùng ký tự **thực sự thay đổi** (dirty-check giữa shadow buffer và nội dung đang hiển thị) để giảm tải bus I2C.

- **Data Persistence:** Hiệu chuẩn (Offset/Gain) phải được lưu vào NVS (Non-Volatile Storage) thông qua `nvs_flash_init()` và `nvs_open()` / `nvs_set_*` / `nvs_commit()`. NVS **phải lưu thêm** `last_calib_timestamp` (Unix time, kiểu `int64_t`) để tính chu kỳ 30 ngày.

- **Drift Self-Check:** `SensorManager` hoặc `DataFusion` so sánh đọc hiện tại với baseline NVS mỗi chu kỳ. Nếu lệch > 10% hoặc quá 30 ngày từ `last_calib_timestamp` (lấy thời gian thực qua SNTP hoặc `esp_timer_get_time()`), phải set cờ `calib_needed = true` trong struct `AirData` và publish cảnh báo qua MQTT với field `"calib_alert": true`. Không ghi đè baseline tự động — chỉ người dùng mới được xác nhận tái hiệu chuẩn.

- **Offline Buffer:** Nếu mất kết nối MQTT, dữ liệu phải được lưu vào hàng đợi trên thẻ SD thông qua `storage_helper`, sử dụng VFS FAT (`esp_vfs_fat_sdmmc_mount()`).

- **Event Log:** Ngoài log dữ liệu định kỳ (`Cfg::SD_LOG_FILE`), `StorageHelper` phải cung cấp `logEvent(...)` ghi các **sự kiện rời rạc** — thay đổi `alert_level`/`alert_reason`/`alert_flags`, `calib_needed`/`calib_reason` (bao gồm khi `confirm_calib` được xác nhận), và kết nối/mất kết nối MQTT — vào một file CSV riêng trên SD card (ví dụ `/sdcard/events.csv`), mỗi dòng kèm `timestamp` (Unix time) và loại sự kiện. Việc ghi phải thực hiện qua task riêng (`Cfg::TASK_PRIO_STORAGE`/`TASK_STACK_STORAGE_WORDS`) nhận dữ liệu qua queue từ pipeline chính — không gọi trực tiếp API SD (blocking I/O) trong pipeline để không vi phạm NFR ≤300ms (§3).

- **Logging:** Dùng `ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE` (từ `esp_log.h`) thay vì `Serial.print()`. Mức log có thể cấu hình qua `idf.py menuconfig` → Component config → Log output. Lưu ý: `ESP_LOG*` là log debug tức thời (UART/console, không lưu trữ lâu dài) — khác với Event Log bền vững trên SD card nói trên.

- **Cấu hình dự án:** Các tham số cấu hình (WiFi SSID/Password, MQTT broker URL, ngưỡng AQI...) phải được định nghĩa trong `Kconfig.projbuild` và truy cập qua macro `CONFIG_*` được sinh tự động — không hardcode trực tiếp trong source code.

## 5. Cấu hình phần cứng (Pin Mapping)

| Bus/Chân | Pins | Thiết bị | Ghi chú |
| :--- | :--- | :--- | :--- |
| **I2C SDA** | GPIO21 | BME680, PCF8574 | Bus dùng chung (`I2C_NUM_0`), quản lý qua `i2cdev` (esp-idf-lib) |
| **I2C SCL** | GPIO22 | BME680, PCF8574 | Bus dùng chung |
| **UART** | RX=16, TX=17, SET=4 | PMS5003 | Dùng UART port 1 hoặc 2 (`UART_NUM_1` / `UART_NUM_2`) |
| **ADC1** | GPIO34 | MQ-135 | Dùng `adc_oneshot_*` API; tránh xung đột với WiFi (dùng ADC1, không dùng ADC2) |
| **SPI** | SCK=18, MISO=19, MOSI=23, CS=5 | SD Card | Dùng SPI host `SPI2_HOST`; mount qua `esp_vfs_fat_sdspi_mount()` |
| **Output** | GPIO25 (LED RED), GPIO26 (LED YELLOW), GPIO27 (LED GREEN), GPIO32 (Buzzer) | Cảnh báo | Cấu hình qua `gpio_config()` với mode `GPIO_MODE_OUTPUT` |
| **Input** | GPIO33 | Nút bấm shutdown an toàn | `GPIO_MODE_INPUT`, pull-up nội, active LOW, ngắt `GPIO_INTR_NEGEDGE`; giữ ≥2s để xác nhận (chống rung tiếp điểm/nhấn nhầm) |

**Kết nối PCF8574 → LCD 16x2:**
- P0: RS, P1: RW (giữ LOW = write), P2: E, P3: Backlight (BL)
- P4–P7: D4–D7 (Chế độ 4-bit)
- A0, A1, A2 nối GND → Địa chỉ I2C: 0x20

## 6. Cấu hình Debug phần cứng (Hardware Debugging)

Dự án yêu cầu khả năng debug trực tiếp trên MCU (on-chip debug) — không chỉ dựa vào log UART. Chuỗi công cụ debug chuẩn:

**JTAG Adapter → OpenOCD → GDB → ESP-IDF Extension (VSCode)**

### 6.1 JTAG Adapter được hỗ trợ
Ưu tiên sử dụng một trong các adapter sau (được Espressif hỗ trợ chính thức với OpenOCD):

| Adapter | Ghi chú |
| :--- | :--- |
| **ESP-Prog** | Adapter chính thức của Espressif, khuyến nghị dùng cho dự án này |
| **ESP32-WROVER-KIT** (onboard) | Nếu dùng kit tích hợp |
| FT2232H-based adapters | FTDI chip, hỗ trợ tốt |
| J-Link | Hỗ trợ qua OpenOCD, tốc độ cao |

### 6.2 Kết nối JTAG — Pin Mapping ESP32
JTAG sử dụng các GPIO mặc định sau trên ESP32 (không được dùng các pin này cho mục đích khác trong dự án):

| JTAG Signal | ESP32 GPIO | Ghi chú |
| :--- | :--- | :--- |
| TDI | GPIO12 | |
| TDO | GPIO15 | |
| TCK | GPIO13 | |
| TMS | GPIO14 | |
| GND | GND | |

> ⚠️ **Lưu ý:** Các GPIO JTAG trên ESP32 (12, 13, 14, 15) bị chiếm dụng khi debug. Đảm bảo không có phần cứng nào của dự án (sensor, output) được nối vào các chân này.

### 6.3 Cấu hình trong ESP-IDF Extension (VSCode)
ESP-IDF Extension tích hợp sẵn OpenOCD và GDB — không cần cài thêm. Cần tạo/kiểm tra file cấu hình sau trong thư mục dự án:

**`.vscode/launch.json`** (sinh tự động bởi ESP-IDF Extension, kiểm tra các trường quan trọng):
```json
{
  "configurations": [
    {
      "type": "espidf",
      "name": "ESP-IDF Debug",
      "request": "launch",
      "openOcdConfigFiles": [
        "board/esp32-wrover-kit-3.3v.cfg"
      ]
    }
  ]
}
```
> Thay `openOcdConfigFiles` bằng config file phù hợp với adapter đang dùng (ví dụ: `interface/esp_prog.cfg` + `target/esp32.cfg` nếu dùng ESP-Prog).

**Khởi động debug:** Menu **Run → Start Debugging (F5)** trong VSCode sau khi đã build project. ESP-IDF Extension tự động:
1. Khởi động OpenOCD server (cổng mặc định 3333/4444).
2. Kết nối GDB tới OpenOCD.
3. Flash firmware lên ESP32 (nếu cấu hình `"flashingType": "openOCD"`).
4. Dừng tại `app_main()` hoặc breakpoint đã đặt.

### 6.4 Tính năng debug có thể sử dụng
- **Breakpoints** trên bất kỳ dòng C++ nào, kể cả trong FreeRTOS task callback.
- **Watch variables / Expressions:** Theo dõi giá trị `AirData`, giá trị ADC thô, trạng thái kết nối MQTT theo thời gian thực.
- **Call stack:** Xem toàn bộ call stack tại điểm dừng, kể cả khi crash (panic handler).
- **FreeRTOS Task Inspector:** ESP-IDF Extension hỗ trợ xem danh sách task đang chạy, stack usage của từng task — hữu ích để debug stack overflow.
- **Postmortem debug (Core Dump):** Khi firmware crash, ESP32 có thể lưu core dump vào flash partition riêng. Phân tích sau bằng: `idf.py coredump-info` hoặc `idf.py coredump-debug`.

### 6.5 Quy tắc bắt buộc khi debug
- **Không xoá/comment các `ESP_LOG*` call** khi commit — đây là nguồn thông tin debug song song với JTAG.
- **Mọi panic/exception phải được investigate bằng call stack** — không được chỉ reset lại mà không xác định nguyên nhân.
- **Stack size của mỗi FreeRTOS task phải được kiểm tra** bằng `uxTaskGetStackHighWaterMark()` trong quá trình debug, trước khi release, để tránh stack overflow trong vận hành thực tế.


# CLAUDE.md - ENV-AQ-01 Project Instructions

File này cung cấp hướng dẫn cho Claude khi làm việc trong repository của dự án ENV-AQ-01: Trạm Quan Trắc Không Khí Đa Thông Số.

## 1. Tổng quan dự án (Project Overview)
Hệ thống quan trắc dựa trên ESP32, thu thập dữ liệu từ BME680, PMS5003, và MQ-135. Tính toán chỉ số AQI (theo tiêu chuẩn Việt Nam), hiển thị lên LCD và truyền dữ liệu qua MQTT.
- **Framework:** ESP-IDF (Espressif IoT Development Framework), phát triển thông qua **ESP-IDF Extension trên Visual Studio Code**.
- **Build System:** CMake + Ninja (được quản lý bởi ESP-IDF toolchain).
- **Ngôn ngữ lập trình:** C++ (chuẩn C++17), sử dụng trực tiếp ESP-IDF APIs — không dùng Arduino HAL hay bất kỳ Arduino wrapper nào. Lý do chọn C++: kiến trúc đa module của dự án (SensorManager, NetworkManager, Filters...) hưởng lợi rõ từ encapsulation (class), RAII, và STL (`std::queue` cho Offline Buffer, `std::array` cho filter window). Overhead RAM/Flash của C++ trên ESP32 (520KB SRAM, 4MB+ Flash) là không đáng kể. Entry point bắt buộc khai báo `extern "C" void app_main()`.
- **Mục tiêu chính:** Độ chính xác cao, hoạt động thời gian thực, tiết kiệm GPIO.

## 2. Kiến trúc & Phân tách Module
Toàn bộ module chia sẻ struct `AirData` (định nghĩa trong `main/include/DataStructures.hpp`).

**Luồng dữ liệu:** `SensorManager` (Đọc thô) → `Filters` (Lọc nhiễu) → `DataFusion` (Tính AQI/Comfort) → `DisplayManager` (LCD) + `NetworkManager` (MQTT) + `StorageHelper` (SD Card).

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

**Nội dung `main/CMakeLists.txt`:**
```cmake
idf_component_register(
    SRCS
        "SensorManager.cpp"
        "Filters.cpp"
        "DataFusion.cpp"
        "DisplayManager.cpp"
        "NetworkManager.cpp"
        "StorageHelper.cpp"
        "main.cpp"
    INCLUDE_DIRS
        "include"               # Bắt buộc khai báo tường minh — ESP-IDF không tự scan
)
```
> ⚠️ **Lưu ý:** Khác với PlatformIO (tự động nhận diện `include/`), ESP-IDF **bắt buộc phải khai báo `INCLUDE_DIRS`** trong `idf_component_register()`. Nếu thiếu dòng này, compiler sẽ báo lỗi không tìm thấy header khi `#include "SensorManager.hpp"`.

| File/Module | Trách nhiệm chính | ESP-IDF APIs liên quan |
| :--- | :--- | :--- |
| `main/SensorManager.cpp` | Driver BME680 (I2C), PMS5003 (UART), MQ-135 (ADC). | `i2c_master_*`, `uart_*`, `adc_oneshot_*` |
| `main/Filters.cpp` | Lọc nhiễu tín hiệu cảm biến: EMA cho T/P/Gas (BME680), SMA cho RH (BME680). Outlier rejection dùng sanity check theo range vật lý — không dùng delta-based threshold. Tham số từng kênh xem **Mục 7**. | — |
| `main/DataFusion.cpp` | Hợp nhất dữ liệu, tính AQI (VN) và Comfort Index; Drift Self-Check. | — |
| `main/DisplayManager.cpp` | Điều khiển LCD 16x2 thông qua IC mở rộng chân PCF8574 (I2C). | `i2c_master_*` |
| `main/NetworkManager.cpp` | Quản lý WiFi, MQTT (JSON payload), xử lý Buffer khi mất mạng. | `esp_wifi_*`, `esp_mqtt_client_*` |
| `main/StorageHelper.cpp` | Ghi log dữ liệu vào thẻ SD (định dạng .csv). | `esp_vfs_fat_sdmmc_mount`, `sdmmc_*` |
| `main/main.cpp` | Khởi tạo hệ thống (`extern "C" void app_main()`), tạo và điều phối bằng FreeRTOS tasks — không dùng `vTaskDelay()` làm logic chính. | `xTaskCreate`, `esp_event_loop_*` |

> **Lưu ý:** `NetworkManager::setCommandCallback()` phải được gọi trong `main.cpp` — không phải `DataFusion`.

## 3. Ràng buộc kỹ thuật (Constraints & NFR)
- **Năng lượng:** Công suất trung bình <= 2.0W. Sử dụng Modem-sleep (`esp_wifi_set_ps(WIFI_PS_MODEM)`) khi nhàn rỗi.
- **Thời gian thực:** Tổng chu kỳ đọc + xử lý dữ liệu phải hoàn tất trong <= 300ms.
- **Độ trễ:** Cảnh báo (Buzzer/LED) và đẩy sự kiện lên Cloud <= 3s.
- **Hiển thị:** LCD cập nhật thông số mỗi 2-5 giây, đảm bảo không gây trễ bus I2C.
- **Độ chính xác sau hiệu chuẩn:** Sai số tổng T ≤ ±0.5°C, RH ≤ ±3%RH. Sai số lặp lại của các chỉ số suy diễn (AQI, TVOC, Comfort Index) ≤ 10%. Đây là chỉ tiêu kiểm thử nghiệm thu bắt buộc.
- **Ổn định dài hạn:** Chu kỳ tự kiểm tra độ trôi tham số hiệu chuẩn tối đa 30 ngày. Khi sai lệch phát hiện > 10% so với baseline, hệ thống phải phát cảnh báo yêu cầu tái hiệu chuẩn — không được âm thầm bù trừ sai số mà không thông báo.

## 4. Quy tắc lập trình quan trọng (Key Rules)

- **Non-blocking:** Tuyệt đối không dùng `vTaskDelay()` làm logic timing cho nghiệp vụ chính. Mọi tác vụ định kỳ phải dùng `esp_timer_create()` / `esp_timer_start_periodic()` hoặc FreeRTOS Task kết hợp với `xQueueReceive()` / Event Group để đồng bộ — không blocking toàn bộ task.

- **I2C Shared Bus:** BME680 và PCF8574 dùng chung bus I2C, được khởi tạo một lần duy nhất bằng `i2c_master_bus_create()`. Dùng `i2c_master_bus_add_device()` để đăng ký từng thiết bị riêng. BME680: địa chỉ **0x76** (SDO nối GND). PCF8574: địa chỉ **0x20** (A0/A1/A2 nối GND). Mọi truy cập I2C từ nhiều task phải được bảo vệ bằng `SemaphoreHandle_t` (mutex).

- **LCD Control:** Giao tiếp với LCD 16x2 thông qua PCF8574 bằng cách tự implement giao thức 4-bit trực tiếp qua `i2c_master_transmit()` — không dùng thư viện `LiquidCrystal_I2C` (thư viện Arduino). Chỉ cập nhật màn hình khi dữ liệu thay đổi để giảm tải bus I2C.

- **Data Persistence:** Hiệu chuẩn (Offset/Gain) phải được lưu vào NVS (Non-Volatile Storage) thông qua `nvs_flash_init()` và `nvs_open()` / `nvs_set_*` / `nvs_commit()`. NVS **phải lưu thêm** `last_calib_timestamp` (Unix time, kiểu `int64_t`) để tính chu kỳ 30 ngày.

- **Drift Self-Check:** `SensorManager` hoặc `DataFusion` so sánh đọc hiện tại với baseline NVS mỗi chu kỳ. Nếu lệch > 10% hoặc quá 30 ngày từ `last_calib_timestamp` (lấy thời gian thực qua SNTP hoặc `esp_timer_get_time()`), phải set cờ `calib_needed = true` trong struct `AirData` và publish cảnh báo qua MQTT với field `"calib_alert": true`. Không ghi đè baseline tự động — chỉ người dùng mới được xác nhận tái hiệu chuẩn.

- **Offline Buffer:** Nếu mất kết nối MQTT, dữ liệu phải được lưu vào hàng đợi trên thẻ SD thông qua `storage_helper`, sử dụng VFS FAT (`esp_vfs_fat_sdmmc_mount()`).

- **Logging:** Dùng `ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE` (từ `esp_log.h`) thay vì `Serial.print()`. Mức log có thể cấu hình qua `idf.py menuconfig` → Component config → Log output.

- **Cấu hình dự án:** Các tham số cấu hình (WiFi SSID/Password, MQTT broker URL, ngưỡng AQI...) phải được định nghĩa trong `Kconfig.projbuild` và truy cập qua macro `CONFIG_*` được sinh tự động — không hardcode trực tiếp trong source code.

## 5. Cấu hình phần cứng (Pin Mapping)

| Bus/Chân | Pins | Thiết bị | Ghi chú |
| :--- | :--- | :--- | :--- |
| **I2C SDA** | GPIO21 | BME680, PCF8574 | Bus dùng chung, khởi tạo qua `i2c_master_bus_create()` |
| **I2C SCL** | GPIO22 | BME680, PCF8574 | Bus dùng chung |
| **UART** | RX=16, TX=17, SET=4 | PMS5003 | Dùng UART port 1 hoặc 2 (`UART_NUM_1` / `UART_NUM_2`) |
| **ADC1** | GPIO34 | MQ-135 | Dùng `adc_oneshot_*` API; tránh xung đột với WiFi (dùng ADC1, không dùng ADC2) |
| **SPI** | SCK=18, MISO=19, MOSI=23, CS=5 | SD Card | Dùng SPI host `SPI2_HOST`; mount qua `esp_vfs_fat_sdspi_mount()` |
| **Output** | GPIO25 (LED RED), GPIO26 (LED YELLOW), GPIO27 (LED GREEN), GPIO32 (Buzzer) | Cảnh báo | Cấu hình qua `gpio_config()` với mode `GPIO_MODE_OUTPUT` |

**Kết nối PCF8574 → LCD 16x2:**
- P0: RS, P1: RW, P2: E
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

## 7. Thiết kế Bộ lọc (Filter Design)

### 7.1 Kiến trúc hai tầng

```
Raw sample → [Tầng 1: Sanity Check] → reject nếu giá trị vật lý vô lý (hardware fault)
                                      ↓ pass
                              [Tầng 2: EMA / SMA] → smoothed output → AirData
```

**Không dùng delta-based outlier rejection trong `Filters.cpp`** — threshold theo ΔX không phân biệt được sự kiện môi trường thật (ví dụ: Gas drop ~40% khi có VOC) với hardware glitch. Tầng 1 chỉ loại các giá trị nằm ngoài range vật lý khả dĩ của sensor.

Gas baseline tracker (drift compensation dài hạn) **thuộc `DataFusion`**, không thuộc `Filters`.

### 7.2 BME680 — đã xác nhận qua thực nghiệm

**Sampling rate:** 2s/sample

| Kênh | Filter | Tham số | Cơ sở thực nghiệm |
| :--- | :--- | :--- | :--- |
| Temperature | EMA | α = 0.10 | stdT = 0.005°C, tín hiệu rất sạch, thay đổi chậm |
| Humidity | SMA | window = 3–5 | Có oscillation pattern (ADC quantization / HVAC); recovery breath event chỉ 3–4s |
| Pressure | EMA | α = 0.10 | Ổn định, ít thay đổi |
| Gas resistance | EMA | α = 0.25 | Drop ~43% trong 10s (~5 samples); α=0.25 bắt được ~76% biên độ drop |

**Kết quả perturbation test (hà hơi 10s):**
- Drop: ~174k → 99k Ω (~43%, ~75kΩ) trong 10s
- Recovery: ~90–102s
- dGas per sample khi drop: ~11–15k Ω/sample → vượt mọi delta threshold thực tế → xác nhận không dùng delta-based rejection

**Sanity check range — BME680:**

| Kênh | Min | Max |
| :--- | :--- | :--- |
| Temperature | -40°C | 85°C |
| Humidity | 0% | 100% |
| Pressure | 300 hPa | 1100 hPa |
| Gas resistance | 1 kΩ | 500 kΩ |

### 7.3 MQ-135 — tham số lý thuyết

**Sampling rate:** 2s/sample

| Kênh | Filter | Tham số | Cơ sở lý thuyết |
| :--- | :--- | :--- | :--- |
| CO2 ppm (sau bù T/RH) | EMA | α = 0.20 | MOX có quán tính nhiệt lớn (~0.8W heater) → lag-1 autocorrelation cao; α=0.20 bắt 75% biên độ sự kiện trong ~6.4 sample (~13s) — đủ bám sự kiện CO2 thổi hơi thở (~30s) |

**Đặc điểm noise dự kiến:**
- MQ-135 **không có internal averaging** (khác PMS5003) — output là tín hiệu analog thuần của phần tử MOX qua ADC 12-bit.
- Quán tính nhiệt của gốm MOX tạo ra **lag-1 autocorrelation cao**: tín hiệu thay đổi liên tục, chậm → EMA tốt hơn SMA (SMA có độ trễ phẳng không khớp hệ thống first-order).
- Không có impulse spike đơn lẻ — khối lượng nhiệt lớn ngăn nhảy đột ngột trong 1 sample → **Median không cần thiết**.
- α=0.20 thận trọng hơn BME680 gas (α=0.25) vì heater MQ-135 lớn hơn nhiều (~0.8W vs ~4mW), hằng số thời gian nhiệt dài hơn → tín hiệu thay đổi chậm hơn → cần α nhỏ hơn.

**Lưu ý bù T/RH và R0:**
- T/RH compensation (`SensorManager::mq135CorrectionFactor`) chạy trước EMA — loại bỏ drift chậm do môi trường thay đổi.

**Sanity check range — MQ-135:**

| Kênh | Min | Max |
| :--- | :--- | :--- |
| CO2 ppm | 100 ppm | 5000 ppm |

Không dùng delta-based rejection — noise analog liên tục, không có spike để reject. Tầng 1 chỉ là sanity check range.

### 7.4 PMS5003 — đã xác nhận qua thực nghiệm

**Sampling rate:** 1s/sample (sensor tự output 1 frame/giây)

| Kênh | Filter | Tham số | Cơ sở thực nghiệm |
| :--- | :--- | :--- | :--- |
| PM1.0 | EMA | α = 0.50 | std ≈ 0.3–0.5 µg/m³, noise tương đồng PM2.5 |
| PM2.5 | EMA | α = 0.50 | std ≈ 0.6–0.8 µg/m³, dPM25 thường = 0 hoặc ±1 |
| PM10  | EMA | α = 0.30 | std ≈ 0.7–1+ µg/m³, noisier ~1.5× PM2.5 do hạt lớn ít hơn |

**Đặc điểm noise quan sát được:**
- PMS5003 đã thực hiện heavy internal averaging trong firmware chip trước khi xuất UART — output là tín hiệu đã lọc sẵn.
- Noise hoàn toàn là **integer quantization** (±1 LSB): dPM max = 3 µg/m³ kể cả khi có perturbation (giũ gối).
- **Không có impulse spike** — spike_rate = 0 tại mọi ngưỡng trong suốt quá trình test.
- mad/std ratio ≈ 0.6 (thấp hơn Gaussian lý thuyết 0.80 vì phần lớn delta = 0).
- PM10 noisier hơn PM2.5 do counting statistics kém ổn định hơn (ít hạt lớn trong không khí).

**Kết quả perturbation test (giũ gối gần sensor):**
- Rise: chậm, dPM25 không vượt quá 3 µg/m³/sample ngay cả khi perturbation mạnh.
- Decay: chậm tương tự, monotonic.
- Xác nhận: sensor không có impulse noise, không cần Median filter.

**Sanity check range — PMS5003:**

| Kênh | Min | Max |
| :--- | :--- | :--- |
| PM1.0 | 0 µg/m³ | 500 µg/m³ |
| PM2.5 | 0 µg/m³ | 500 µg/m³ |
| PM10  | 0 µg/m³ | 500 µg/m³ |

Không dùng delta-based rejection (không có spike để reject). Tầng 1 chỉ là sanity check range.

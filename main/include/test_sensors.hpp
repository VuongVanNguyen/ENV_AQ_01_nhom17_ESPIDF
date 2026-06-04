#pragma once

// Khai báo các FreeRTOS task function cho từng chế độ test cảm biến.
// Được include trong main.cpp và bảo vệ bởi #ifdef CONFIG_TEST_MODE_*.
// Mỗi task tự init phần cứng riêng — không phụ thuộc module production.

void test_bme680_task(void *arg);
void test_pms5003_task(void *arg);
void test_mq135_task(void *arg);

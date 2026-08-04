#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include "RTClib.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

// 引入 DBC 解碼標頭檔
extern "C" {
#if __has_include("bms4_x_cannetv1_08.h")
#include "bms4_x_cannetv1_08.h"
#define HAS_DBC 1
#else
#define HAS_DBC 0
#endif
}

// ==========================================
// 硬體接腳定義
// ==========================================
const int SPI_CS_PIN_1 = 5;
const int CAN_INT_PIN_1 = 4;

const int SPI_CS_PIN_2 = 16;
const int CAN_INT_PIN_2 = 17;

#define L2B_01VEXTREMUM_ID 0x1810F401

MCP_CAN CAN1(SPI_CS_PIN_1);
MCP_CAN CAN2(SPI_CS_PIN_2);
RTC_DS3231 rtc;

// ==========================================
// FreeRTOS 與 Light Sleep 相關變數
// ==========================================
SemaphoreHandle_t rxSemaphore1;
SemaphoreHandle_t rxSemaphore2;

const uint32_t SLEEP_TIMEOUT_MS = 5000;
volatile uint32_t lastActivityTimeMs = 0;

// ==========================================
// 中斷服務常式 (ISR)
// ==========================================
void IRAM_ATTR canISR1() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  lastActivityTimeMs = (uint32_t)(esp_timer_get_time() / 1000);
  xSemaphoreGiveFromISR(rxSemaphore1, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR canISR2() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  lastActivityTimeMs = (uint32_t)(esp_timer_get_time() / 1000);
  xSemaphoreGiveFromISR(rxSemaphore2, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// 取得當前時間字串
String getTimeString() {
  DateTime now = rtc.now();
  char buf[30];
  snprintf(buf, sizeof(buf), "[%04d/%02d/%02d %02d:%02d:%02d]", 
           now.year(), now.month(), now.day(), 
           now.hour(), now.minute(), now.second());
  return String(buf);
}

// ==========================================
// CAN 1 接收 Task
// ==========================================
void canRxTask1(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(rxSemaphore1, portMAX_DELAY) == pdTRUE) {
      while (CAN_MSGAVAIL == CAN1.checkReceive()) {
        long unsigned int rxId;
        unsigned char len = 0;
        unsigned char rxBuf[8];

        CAN1.readMsgBuf(&rxId, &len, rxBuf);
        uint32_t maskedId = rxId & 0x1FFFFFFF;

        String timestamp = getTimeString();
        Serial.printf("%s [CAN 1] 收到封包 ID: 0x%X\n", timestamp.c_str(), maskedId);

#if HAS_DBC
        if (maskedId == L2B_01VEXTREMUM_ID) {
          struct bms4_x_cannetv1_08_l2_b_01_vextremum_t rx_data;
          bms4_x_cannetv1_08_l2_b_01_vextremum_unpack(&rx_data, rxBuf, len);
          Serial.printf("    L2B_UCellHi1: %.3f V\n", rx_data.l2_b_u_cell_hi1 * 0.001);
          Serial.printf("    L2B_UCellLo1: %.3f V\n", rx_data.l2_b_u_cell_lo1 * 0.001);
        }
#endif
      }
    }
  }
}

// ==========================================
// CAN 2 接收 Task
// ==========================================
void canRxTask2(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(rxSemaphore2, portMAX_DELAY) == pdTRUE) {
      while (CAN_MSGAVAIL == CAN2.checkReceive()) {
        long unsigned int rxId;
        unsigned char len = 0;
        unsigned char rxBuf[8];

        CAN2.readMsgBuf(&rxId, &len, rxBuf);
        uint32_t maskedId = rxId & 0x1FFFFFFF;

        String timestamp = getTimeString();
        Serial.printf("%s [CAN 2] 收到封包 ID: 0x%X\n", timestamp.c_str(), maskedId);

#if HAS_DBC
        if (maskedId == L2B_01VEXTREMUM_ID) {
          struct bms4_x_cannetv1_08_l2_b_01_vextremum_t rx_data;
          bms4_x_cannetv1_08_l2_b_01_vextremum_unpack(&rx_data, rxBuf, len);
          Serial.printf("    L2B_UCellHi1: %.3f V\n", rx_data.l2_b_u_cell_hi1 * 0.001);
          Serial.printf("    L2B_UCellLo1: %.3f V\n", rx_data.l2_b_u_cell_lo1 * 0.001);
        }
#endif
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n==============================================");
  Serial.println("=== 雙通道 CAN + DS3231 + Light Sleep 測試 ===");
  Serial.println("==============================================\n");

  // 初始化 I2C 與 RTC
  Wire.begin(21, 22);
  if (!rtc.begin(&Wire)) {
    Serial.println("找不到 DS3231 RTC 模組，請檢查 I2C 接線!");
    Serial.flush();
    while (1) delay(10);
  }
  
  DateTime now = rtc.now();
  // 校時邏輯：如果 RTC 斷電，或者年份錯亂 (小於 2024)，強制寫入編譯時間
  if (rtc.lostPower() || now.year() < 2024) {
    Serial.println("RTC 時間異常，正在強制寫入編譯時間！");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // 如果您發現時間還是不對 (例如是昨天編譯的時間)，請取消註解並修改下方這行，上傳一次後再註解掉：
    // rtc.adjust(DateTime(2024, 7, 28, 12, 0, 0)); 
  } else {
    Serial.printf("RTC 目前時間: %04d/%02d/%02d %02d:%02d:%02d\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  }

  // 初始化 CAN
  if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN1.setMode(MCP_NORMAL);
    Serial.println("CAN 1 初始化成功!");
  }
  if (CAN2.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN2.setMode(MCP_NORMAL);
    Serial.println("CAN 2 初始化成功!");
  }

  rxSemaphore1 = xSemaphoreCreateBinary();
  rxSemaphore2 = xSemaphoreCreateBinary();

  pinMode(CAN_INT_PIN_1, INPUT_PULLUP);
  pinMode(CAN_INT_PIN_2, INPUT_PULLUP);

  // 綁定硬體中斷
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_1), canISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_2), canISR2, FALLING);

  xTaskCreate(canRxTask1, "CAN1_Task", 4096, NULL, 5, NULL);
  xTaskCreate(canRxTask2, "CAN2_Task", 4096, NULL, 5, NULL);

  lastActivityTimeMs = (uint32_t)(esp_timer_get_time() / 1000);
}

void loop() {
  uint32_t currentTimeMs = (uint32_t)(esp_timer_get_time() / 1000);

  if ((currentTimeMs - lastActivityTimeMs) > SLEEP_TIMEOUT_MS) {
    
    // 【防呆機制】防止 INT 已經是 LOW 時進入休眠導致無窮迴圈當機
    if (digitalRead(CAN_INT_PIN_1) == LOW || digitalRead(CAN_INT_PIN_2) == LOW) {
        if (digitalRead(CAN_INT_PIN_1) == LOW) xSemaphoreGive(rxSemaphore1);
        if (digitalRead(CAN_INT_PIN_2) == LOW) xSemaphoreGive(rxSemaphore2);
        lastActivityTimeMs = currentTimeMs;
        delay(10);
        return; 
    }

    Serial.println("\n[Sleep] 閒置超過 5 秒，準備進入 Light Sleep...");
    Serial.flush();
    delay(50); // 確保 UART 輸出完畢

    // 1. 完全解除 Arduino 中斷綁定，避免與 RTC 喚醒衝突引發 Watchdog Timeout
    detachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_1));
    detachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_2));

    // 2. 開啟 GPIO LOW Level 喚醒
    gpio_wakeup_enable((gpio_num_t)CAN_INT_PIN_1, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)CAN_INT_PIN_2, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    // 3. 進入休眠
    esp_light_sleep_start();

    // 4. 醒來後，立刻關閉 GPIO 喚醒設定，避免後續一般中斷時產生干擾
    gpio_wakeup_disable((gpio_num_t)CAN_INT_PIN_1);
    gpio_wakeup_disable((gpio_num_t)CAN_INT_PIN_2);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);

    // 5. 重新綁定 Arduino 的 FALLING 硬體中斷
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_1), canISR1, FALLING);
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_2), canISR2, FALLING);

    lastActivityTimeMs = (uint32_t)(esp_timer_get_time() / 1000);
    Serial.println("\n[Wakeup] 從 Light Sleep 喚醒！");

    // 6. 手動檢查 (防止休眠期間被拉低後，錯過 FALLING 邊緣觸發)
    if (digitalRead(CAN_INT_PIN_1) == LOW) xSemaphoreGive(rxSemaphore1);
    if (digitalRead(CAN_INT_PIN_2) == LOW) xSemaphoreGive(rxSemaphore2);
  }

  delay(10); 
}
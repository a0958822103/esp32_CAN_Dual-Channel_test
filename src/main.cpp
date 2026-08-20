#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include "RTClib.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/gpio.h"
#include "SdFat.h"
#include <atomic>

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
// 硬體接腳定義 (CAN 模組使用預設 VSPI)
// ==========================================
const int SPI_CS_PIN_1 = 5;
const int CAN_INT_PIN_1 = 4;

const int SPI_CS_PIN_2 = 16;
const int CAN_INT_PIN_2 = 17;

// ==========================================
// 硬體接腳定義 (SD 卡專用第二組 HSPI，避開衝突)
// ==========================================
const int SD_SCK_PIN = 25;
const int SD_MISO_PIN = 26;
const int SD_MOSI_PIN = 33;
const int SD_CS_PIN = 27;

MCP_CAN CAN1(SPI_CS_PIN_1);
MCP_CAN CAN2(SPI_CS_PIN_2);
RTC_DS3231 rtc;

// 建立 SD 卡專用的硬體 SPI (HSPI) 與 SdFat 物件
SPIClass sdSPI(HSPI);
SdFs SD;

// ==========================================
// FreeRTOS 與 Light Sleep 相關變數
// ==========================================
SemaphoreHandle_t rxSemaphore1;
SemaphoreHandle_t rxSemaphore2;

// 定義 SD 卡寫入佇列的資料結構
struct LogMessage {
  char payload[128]; 
};
QueueHandle_t sdQueue;

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
  snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d", 
           now.year(), now.month(), now.day(), 
           now.hour(), now.minute(), now.second());
  return String(buf);
}

// ==========================================
// SD 卡非同步寫入 Task (消費者)
// ==========================================
void sdWriteTask(void *pvParameters) {
  for (;;) {
    LogMessage logMsg;
    // 阻塞等待佇列中的資料 (不消耗 CPU)
    if (xQueueReceive(sdQueue, &logMsg, portMAX_DELAY) == pdTRUE) {
      FsFile dataFile = SD.open("can_log.csv", O_WRITE | O_CREAT | O_APPEND);
      if (dataFile) {
        dataFile.println(logMsg.payload);
        dataFile.close();
      } else {
        Serial.println("SD 卡寫入失敗，請檢查卡片或連線！");
      }
    }
  }
}

// ==========================================
// 封包解析與寫入佇列函式
// ==========================================
std::atomic<uint32_t> globalPacketSeq{0};

void enqueue_log(const char* payload) {
  LogMessage logMsg;
  strlcpy(logMsg.payload, payload, sizeof(logMsg.payload));
  // 終端機也印出一份方便監看
  Serial.println(logMsg.payload);
  xQueueSend(sdQueue, &logMsg, 0);
}

void parse_and_log_signals(const char* channel, uint32_t maskedId, uint8_t* rxBuf, uint8_t len, const String& timestamp, uint32_t packetSeq) {
  char buf[128];
  
#if HAS_DBC
  if (maskedId == BMS4_X_CANNETV1_08_L2_B_01_VEXTREMUM_FRAME_ID) {
    struct bms4_x_cannetv1_08_l2_b_01_vextremum_t data;
    bms4_x_cannetv1_08_l2_b_01_vextremum_unpack(&data, rxBuf, len);
    const char* msgName = "L2B_01Vextremum";
    
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCellHi1,%.3f,V", timestamp.c_str(), packetSeq, channel, maskedId, msgName, bms4_x_cannetv1_08_l2_b_01_vextremum_l2_b_u_cell_hi1_decode(data.l2_b_u_cell_hi1));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCellLo1,%.3f,V", timestamp.c_str(), packetSeq, channel, maskedId, msgName, bms4_x_cannetv1_08_l2_b_01_vextremum_l2_b_u_cell_lo1_decode(data.l2_b_u_cell_lo1));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UMdl1,%.2f,V", timestamp.c_str(), packetSeq, channel, maskedId, msgName, bms4_x_cannetv1_08_l2_b_01_vextremum_l2_b_u_mdl1_decode(data.l2_b_u_mdl1));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCellHiNr1,%d,-", timestamp.c_str(), packetSeq, channel, maskedId, msgName, (int)bms4_x_cannetv1_08_l2_b_01_vextremum_l2_b_u_cell_hi_nr1_decode(data.l2_b_u_cell_hi_nr1));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCellLNr1,%d,-", timestamp.c_str(), packetSeq, channel, maskedId, msgName, (int)bms4_x_cannetv1_08_l2_b_01_vextremum_l2_b_u_cell_l_nr1_decode(data.l2_b_u_cell_l_nr1));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_LifeUExtremum1,%d,-", timestamp.c_str(), packetSeq, channel, maskedId, msgName, (int)bms4_x_cannetv1_08_l2_b_01_vextremum_l2_b_life_u_extremum1_decode(data.l2_b_life_u_extremum1));
    enqueue_log(buf);
    return;
  }
  
  if (maskedId == BMS4_X_CANNETV1_08_L2_B_01_V_SIG1_FRAME_ID) {
    struct bms4_x_cannetv1_08_l2_b_01_v_sig1_t data;
    bms4_x_cannetv1_08_l2_b_01_v_sig1_unpack(&data, rxBuf, len);
    const char* msgName = "L2B_01VSig1";
    
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCell1Nr01,%.3f,V", timestamp.c_str(), packetSeq, channel, maskedId, msgName, bms4_x_cannetv1_08_l2_b_01_v_sig1_l2_b_u_cell1_nr01_decode(data.l2_b_u_cell1_nr01));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCell1Nr01Vld,%d,-", timestamp.c_str(), packetSeq, channel, maskedId, msgName, (int)bms4_x_cannetv1_08_l2_b_01_v_sig1_l2_b_u_cell1_nr01_vld_decode(data.l2_b_u_cell1_nr01_vld));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCell1Nr02,%.3f,V", timestamp.c_str(), packetSeq, channel, maskedId, msgName, bms4_x_cannetv1_08_l2_b_01_v_sig1_l2_b_u_cell1_nr02_decode(data.l2_b_u_cell1_nr02));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCell1Nr02Vld,%d,-", timestamp.c_str(), packetSeq, channel, maskedId, msgName, (int)bms4_x_cannetv1_08_l2_b_01_v_sig1_l2_b_u_cell1_nr02_vld_decode(data.l2_b_u_cell1_nr02_vld));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCell1Nr03,%.3f,V", timestamp.c_str(), packetSeq, channel, maskedId, msgName, bms4_x_cannetv1_08_l2_b_01_v_sig1_l2_b_u_cell1_nr03_decode(data.l2_b_u_cell1_nr03));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCell1Nr03Vld,%d,-", timestamp.c_str(), packetSeq, channel, maskedId, msgName, (int)bms4_x_cannetv1_08_l2_b_01_v_sig1_l2_b_u_cell1_nr03_vld_decode(data.l2_b_u_cell1_nr03_vld));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCell1Nr04,%.3f,V", timestamp.c_str(), packetSeq, channel, maskedId, msgName, bms4_x_cannetv1_08_l2_b_01_v_sig1_l2_b_u_cell1_nr04_decode(data.l2_b_u_cell1_nr04));
    enqueue_log(buf);
    snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,%s,L2B_UCell1Nr04Vld,%d,-", timestamp.c_str(), packetSeq, channel, maskedId, msgName, (int)bms4_x_cannetv1_08_l2_b_01_v_sig1_l2_b_u_cell1_nr04_vld_decode(data.l2_b_u_cell1_nr04_vld));
    enqueue_log(buf);
    return;
  }
#endif

  // 若非上述已知 ID，或是未啟用 DBC，則輸出 RAW DATA
  char hexBuf[24] = {0};
  for(int i=0; i<len; i++) {
    sprintf(&hexBuf[i*3], "%02X ", rxBuf[i]);
  }
  if(len > 0) hexBuf[len*3 - 1] = '\0';
  
  snprintf(buf, sizeof(buf), "%s,%u,%s,0x%X,UNKNOWN_MSG,RAW_DATA,%s,-", timestamp.c_str(), packetSeq, channel, maskedId, hexBuf);
  enqueue_log(buf);
}

// ==========================================
// CAN 1 接收 Task (生產者)
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
        
        // 取得唯一的封包序號
        uint32_t seq = globalPacketSeq.fetch_add(1);
        parse_and_log_signals("CAN1", maskedId, rxBuf, len, timestamp, seq);
      }
    }
  }
}

// ==========================================
// CAN 2 接收 Task (生產者)
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
        
        // 取得唯一的封包序號
        uint32_t seq = globalPacketSeq.fetch_add(1);
        parse_and_log_signals("CAN2", maskedId, rxBuf, len, timestamp, seq);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n==============================================");
  Serial.println("=== 雙通道 CAN + DS3231 + SD 卡 (雙 SPI 隔離架構) ===");
  Serial.println("==============================================\n");

  // 初始化 I2C 與 RTC
  Wire.begin(21, 22);
  if (!rtc.begin(&Wire)) {
    Serial.println("找不到 DS3231 RTC 模組，請檢查 I2C 接線!");
    Serial.flush();
    while (1) delay(10);
  }
  
  DateTime now = rtc.now();
  if (rtc.lostPower() || now.year() < 2024) {
    Serial.println("RTC 時間異常，正在強制寫入編譯時間！");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } else {
    Serial.printf("RTC 目前時間: %04d/%02d/%02d %02d:%02d:%02d\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  }

  // ==========================================
  // 初始化 CAN 模組 (第一組 SPI: VSPI 預設)
  // ==========================================
  pinMode(SPI_CS_PIN_1, OUTPUT); digitalWrite(SPI_CS_PIN_1, HIGH);
  pinMode(SPI_CS_PIN_2, OUTPUT); digitalWrite(SPI_CS_PIN_2, HIGH);
  
  bool can1_ok = false;
  if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN1.setMode(MCP_NORMAL);
    Serial.println("CAN 1 初始化成功!");
    can1_ok = true;
  } else {
    Serial.println("CAN 1 初始化失敗! (請檢查接線)");
  }
  
  bool can2_ok = false;
  if (CAN2.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN2.setMode(MCP_NORMAL);
    Serial.println("CAN 2 初始化成功!");
    can2_ok = true;
  } else {
    Serial.println("CAN 2 初始化失敗! (請檢查接線)");
  }

  // ==========================================
  // 初始化 SD 卡 (第二組 SPI: HSPI 物理隔離)
  // ==========================================
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  
  Serial.println("正在嘗試初始化 SD 卡 (SdFat - HSPI)...");
  if (!SD.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(4), &sdSPI))) {
    Serial.println("=========================================");
    Serial.println("SD 卡初始化失敗！請確認 VCC 接 5V 以及腳位是否為 25, 26, 33, 27。");
    SD.printSdError(&Serial);
    Serial.println("=========================================");
  } else {
    Serial.println("SD 卡初始化成功！(獨立 SPI 匯流排)");
    
    // 檢查 CSV 檔案是否存在，如果不存在 (或大小為 0)，則寫入標題列
    Serial.println("正在檢查/建立 CSV 檔案...");
    if (!SD.exists("can_log.csv")) {
      FsFile dataFile = SD.open("can_log.csv", O_WRITE | O_CREAT);
      if (dataFile) {
        dataFile.println("Timestamp,Packet_Seq,Channel,CAN_ID,Message,Signal,Value,Unit");
        dataFile.close();
      }
    }
    Serial.println("CSV 檔案準備就緒！");
  }

  // ==========================================
  // 建立 FreeRTOS 物件與安全中斷綁定
  // ==========================================
  if (can1_ok) rxSemaphore1 = xSemaphoreCreateBinary();
  if (can2_ok) rxSemaphore2 = xSemaphoreCreateBinary();
  
  // 建立深度為 100 的日誌佇列 (因為一個封包可能展開成多列)
  sdQueue = xQueueCreate(100, sizeof(LogMessage));

  pinMode(CAN_INT_PIN_1, INPUT_PULLUP);
  pinMode(CAN_INT_PIN_2, INPUT_PULLUP);

  // 只有在 CAN 模組初始化成功時，才綁定中斷與 Task，否則會因為懸空腳位導致 LoadProhibited 崩潰！
  if (can1_ok) {
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_1), canISR1, FALLING);
    xTaskCreate(canRxTask1, "CAN1_Task", 4096, NULL, 5, NULL);
  }
  
  if (can2_ok) {
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_2), canISR2, FALLING);
    xTaskCreate(canRxTask2, "CAN2_Task", 4096, NULL, 5, NULL);
  }

  // 永遠啟動 SD 寫入 Task
  xTaskCreate(sdWriteTask, "SD_Task", 4096, NULL, 2, NULL);

  lastActivityTimeMs = (uint32_t)(esp_timer_get_time() / 1000);
}

void loop() {
  uint32_t currentTimeMs = (uint32_t)(esp_timer_get_time() / 1000);

  if ((currentTimeMs - lastActivityTimeMs) > SLEEP_TIMEOUT_MS) {
    
    // 【防呆機制】除了檢查 INT 腳位，還要檢查 sdQueue 是否為空
    // 確保所有收到的資料都寫入 SD 卡後，才允許睡覺！
    if (digitalRead(CAN_INT_PIN_1) == LOW || 
        digitalRead(CAN_INT_PIN_2) == LOW || 
        uxQueueMessagesWaiting(sdQueue) > 0) {
        
        if (digitalRead(CAN_INT_PIN_1) == LOW) xSemaphoreGive(rxSemaphore1);
        if (digitalRead(CAN_INT_PIN_2) == LOW) xSemaphoreGive(rxSemaphore2);
        
        lastActivityTimeMs = currentTimeMs;
        delay(10);
        return; 
    }

    Serial.println("\n[Sleep] 閒置超過 5 秒且 SD 卡寫入完畢，準備進入 Light Sleep...");
    Serial.flush();
    delay(50); // 確保 UART 輸出完畢

    // 1. 完全解除 Arduino 中斷綁定
    detachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_1));
    detachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_2));

    // 2. 開啟 GPIO LOW Level 喚醒
    gpio_wakeup_enable((gpio_num_t)CAN_INT_PIN_1, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)CAN_INT_PIN_2, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    // 3. 進入休眠
    esp_light_sleep_start();

    // 4. 醒來後立刻關閉 GPIO 喚醒設定
    gpio_wakeup_disable((gpio_num_t)CAN_INT_PIN_1);
    gpio_wakeup_disable((gpio_num_t)CAN_INT_PIN_2);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);

    // 5. 重新綁定 Arduino 的 FALLING 硬體中斷
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_1), canISR1, FALLING);
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN_2), canISR2, FALLING);

    lastActivityTimeMs = (uint32_t)(esp_timer_get_time() / 1000);
    Serial.println("\n[Wakeup] 從 Light Sleep 喚醒！");

    // 6. 手動檢查防止錯失
    if (digitalRead(CAN_INT_PIN_1) == LOW) xSemaphoreGive(rxSemaphore1);
    if (digitalRead(CAN_INT_PIN_2) == LOW) xSemaphoreGive(rxSemaphore2);
  }

  delay(10); 
}
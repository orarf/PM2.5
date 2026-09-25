#include <Arduino.h>
#include <string.h>
#include <HardwareSerial.h>
#include "RS485.h"
#include <PubSubClient.h>
#include <SIM76xx.h>
#include <GSMClient.h>
#include <GPS.h>
#include <ArduinoJson.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include "Font5x7.h"
#include "ScrollDisplay.h"
// ================================================================
//  WATCHDOG
// ================================================================
#define WDT_TIMEOUT_SEC 60

// ================================================================
//  MQTT
// ================================================================

const char* mqtt_server  = "tb-dev.tricommtha.com";
const int   mqtt_port    = 1883;
const char* access_token = "xxxxxxxxxxxxxxxxxxxxxx";

// ================================================================
//  RS485 / MODBUS  (ใช้ library RS485.h ของบอร์ด AIS 4G แทน ModbusMaster)
// ================================================================
// RS485_RX / RS485_TX / RS485_DIR ถูก define ไว้แล้วใน RS485.h
//   RS485_RX  = 16
//   RS485_TX  = 17
//   RS485_DIR = 4
#define MODBUS_ID   120
#define MODBUS_BAUD 9600

#define REG_START     500
#define REG_COUNT     10

#define REG_IDX_HUMI  0
#define REG_IDX_TEMP  1
#define REG_IDX_NOISE 2
#define REG_IDX_PM25  3
#define REG_IDX_PM10  4

HardwareSerial rs485_uart(2);
RS485Class     rs485(&rs485_uart);

// ================================================================
//  HUB75 PIN — 2 แผง chain แนวนอน (ซ้าย=แผงบน, ขวา=แผงล่าง)
//  Physical: 64 wide × 16 tall
//  Virtual framebuffer: 32 wide × 32 tall (บน=y0-15, ล่าง=y16-31)
// ================================================================
#define R1_PIN   25
#define G1_PIN   26
#define B1_PIN   27
#define R2_PIN   21
#define G2_PIN   22
#define B2_PIN   23
#define A_PIN    19
#define B_PIN    18
#define C_PIN    5
#define D_PIN    -1
#define E_PIN    -1
#define CLK_PIN  15
#define LAT_PIN  32
#define OE_PIN   33

// 2 panels 32×16 stack บน-ล่าง
// library config: width=32, height=16, chain=2
#define PANEL_W      32
#define PANEL_H      16
#define PANEL_CHAIN   2

// Virtual canvas = ขนาดรวม 32×32
#define VIRT_WIDTH   32
#define VIRT_HEIGHT  32   // PANEL_H * PANEL_CHAIN

// ความสว่างจอ (0-255) — ยิ่งต่ำ ยิ่งกินกระแสน้อยลง ช่วยลด ghosting/สีซ้อน
// ที่เกิดจากไฟตกคร่อมสาย/ครอสทอล์คตอนหลายพิกเซลติดพร้อมกัน
// (เทียบเท่ากับการลดแรงดัน PSU ที่เคยทดสอบแล้วได้ผล แต่ทำแบบนี้ปลอดภัยกับ
//  IC ขับ LED ในแผงมากกว่า เพราะยังจ่ายไฟ 5V ตามสเปกอยู่)
// ลองปรับตัวเลขนี้ลง/ขึ้นแล้วอัปโหลดใหม่ เพื่อหาจุดที่ภาพไม่เพี้ยนแต่ยังสว่างพอ
#define PANEL_BRIGHTNESS   255

MatrixPanel_I2S_DMA  *matrix     = nullptr;

// ================================================================
//  PM2.5 LEVEL THRESHOLDS — 5 ระดับ
//
//    Level 0 : 0 – 15   → CYAN   (Very Good)  ← สีเดียวกับ Humidity
//    Level 1 : 16 – 25  → GREEN  (Good)
//    Level 2 : 26 – 37  → YELLOW (Moderate)
//    Level 3 : 38 – 75  → ORANGE (Unhealthy)
//    Level 4 : > 75     → RED    (Hazardous)
// ================================================================
#define PM25_BLUE_MAX     15.0f
#define PM25_GREEN_MAX    25.0f
#define PM25_YELLOW_MAX   37.0f
#define PM25_ORANGE_MAX   75.0f

int  pm25Level     = -2;   // -2 = ยังไม่มีค่า, -1 = error, 0..4 = ระดับสี
int  pm25LevelPrev = -2;

// ── Debounce กัน publish ถี่ตอนค่าคาบรอยต่อสี (เช่น 25↔26) ──
// ต้องอ่านได้ "ระดับใหม่" ต่อเนื่องครบจำนวนรอบนี้ก่อน ถึง commit + publish
// (2 รอบ × 10s ≈ ต้องนิ่ง ~20s ; เพิ่มเป็น 3 = ~30s ถ้ายังถี่)
#define PM25_LEVEL_DEBOUNCE   2
int  pm25LevelCandidate      = -2;
int  pm25LevelCandidateCount = 0;

// ================================================================
//  TEST MODE — จำลองค่า PM2.5 / T / H เพื่อทดสอบสี/สถานะบนจอ LED
//  ทุกค่าวนตามลิสต์ที่กำหนดไว้ตายตัว (ไม่ใช่ sweep/step แล้ว)
//
//  TEST_MODE_PM25 = 1  → ไม่อ่านค่าจริงจาก RS485 ใช้ค่าจำลองแทน
//  TEST_MODE_PM25 = 0  → กลับไปอ่านค่าจริงจาก sensor ตามปกติ
//
//  PM2.5 : วนตามลิสต์ 5,15,16,25,26,37,38,75,76,90,120,500,750,1000
//          (ค่าคู่ 15/16, 25/26, 37/38, 75/76 คือค่าคาบเส้นสี BLUE/GREEN/YELLOW/ORANGE/RED)
//  T     : วนตามลิสต์ -10, 10, 25, 33, 100
//  H     : วนตามลิสต์ 5, 10, 50, 150, 200
// ================================================================
#define TEST_MODE_PM25         0      // <-- ใช้งานจริง: อ่านค่าจาก RS485 sensor (ตั้ง 1 เพื่อกลับไปโหมดทดสอบ)
#define TEST_PM25_INTERVAL  13000UL   // เปลี่ยนค่าทุกกี่ ms (ให้เวลาดู scroll ของ T/H จบรอบก่อนเปลี่ยนค่าใหม่)

// PM2.5 — วนตามลิสต์ checkpoint (ค่าคาบเส้นสีทุกระดับ + ค่ากระโดดหลักร้อย/พัน)
const int testPM25List[] = {5, 15, 16, 25, 26, 37, 38, 75, 76, 90, 120, 500, 750, 1000};
#define TEST_PM25_LIST_LEN  (sizeof(testPM25List) / sizeof(testPM25List[0]))

// T — °C : วนตามลิสต์ค่าที่กำหนด
const int testTempList[] = {-10, 10, 25, 33, 100};
#define TEST_TEMP_LIST_LEN  (sizeof(testTempList) / sizeof(testTempList[0]))

// H — % : วนตามลิสต์ค่าที่กำหนด
const int testHumiList[] = {5, 10, 50, 150, 200};
#define TEST_HUMI_LIST_LEN  (sizeof(testHumiList) / sizeof(testHumiList[0]))

int  testPM25Idx   = 0;
int  testPM25Value = testPM25List[0];

int  testTempIdx  = 0;
int  testTempValue = testTempList[0];

int  testHumiIdx  = 0;
int  testHumiValue = testHumiList[0];

// ================================================================
//  GSM / MQTT
// ================================================================
GSMClient    gsm_client;
PubSubClient client(gsm_client);

// ================================================================
//  TIMING
// ================================================================
unsigned long t_modbus         = 0;
unsigned long t_gps            = 0;
unsigned long t_publish        = 0;
unsigned long t_display        = 0;
unsigned long lastNetworkCheck = 0;
unsigned long lastMQTTCheck    = 0;

#if TEST_MODE_PM25
  #define MODBUS_INTERVAL     TEST_PM25_INTERVAL   // โหมดทดสอบ: เปลี่ยนค่าไวขึ้นเพื่อดูผล
#else
  #define MODBUS_INTERVAL     10000UL   // อ่าน sensor (RS485) ทุก 10 วินาที
#endif
#define GPS_INTERVAL          15000UL   // อ่าน GPS ทุก 15 วินาที
#define PUBLISH_INTERVAL     900000UL   // ส่งค่าขึ้น MQTT ทุก 15 นาที (รวม sensor+GPS)
#define DISPLAY_UPDATE        60000UL   // จอ LED อัปเดตทุก 1 นาที

#define BOOT_PUBLISH_DELAY    60000UL   // หลังเปิดเครื่อง 1 นาที ส่งค่าทั้งหมดขึ้นครั้งแรก
bool bootPublishDone = false;

const unsigned long networkInterval = 30000UL;

// ── MQTT reconnect backoff ──
// ต่อไม่ติด (broker ล่ม / token ผิด) → ถอยจังหวะแบบ exponential กัน retry ถี่ๆ
// 10 → 20 → 40 → 60s (ค้างที่ 60) ; ต่อติด/เน็ตกลับมาปกติ → รีเซ็ตเป็น 10s
#define MQTT_BACKOFF_MIN   10000UL    // เริ่มที่ 10 วิ (เท่าเดิม)
#define MQTT_BACKOFF_MAX   60000UL    // เพดาน 60 วิ
unsigned long mqttBackoff = MQTT_BACKOFF_MIN;

// ================================================================
//  SENSOR DATA
// ================================================================
float humi = 0, temp = 0, pm25 = 0, pm10 = 0;
bool  sensorOK = false;

float disp_humi = 0, disp_temp = 0, disp_pm25 = 0;
bool  hasDisplayData = false;

// ================================================================
//  MODBUS FAIL COUNTER
// ================================================================
#define MODBUS_RETRY_COUNT  2
#define MODBUS_RETRY_DELAY  200
#define MODBUS_FAIL_MAX     2

int  modbusFailCount  = 0;
bool sensorErrorState = false;

// ================================================================
//  GPS DATA
// ================================================================
double latitude  = 0;
double longitude = 0;
bool   gps_fix   = false;

bool networkReady = false;

// ================================================================
//  FRAMEBUFFER 32×32 (virtual canvas)
//  fb[y][x]  y=0..31, x=0..31
// ================================================================
uint16_t fb[VIRT_HEIGHT][VIRT_WIDTH];
uint16_t COLOR_WHITE, COLOR_RED, COLOR_GREEN;
uint16_t COLOR_YELLOW, COLOR_CYAN, COLOR_ORANGE;

void fbClear() {
  memset(fb, 0, sizeof(fb));
}

void fbSetPixel(int16_t x, int16_t y, uint16_t color) {
  if (x >= 0 && x < VIRT_WIDTH && y >= 0 && y < VIRT_HEIGHT)
    fb[y][x] = color;
}

// ================================================================
//  fbFlush — Virtual 32×32 → Physical 64×16 (chain=2 แนวนอน)
//
//  chain=2 library map:
//    panel0 = physical x= 0..31  (DATA ออกก่อน = แผงล่างตาม wiring)
//    panel1 = physical x=32..63  (DATA ออกหลัง = แผงบนตาม wiring)
//
//  Virtual:
//    y= 0..15 = zone บน  → panel1 (x=32..63, y=0..15)
//    y=16..31 = zone ล่าง → panel0 (x= 0..31, y=0..15)
// ================================================================
void fbFlush() {
  for (int vy = 0; vy < VIRT_HEIGHT; vy++) {
    for (int vx = 0; vx < VIRT_WIDTH; vx++) {
      int px, py;
      if (vy < 16) {
        // zone บน → panel0 (x=0..31)
        px = vx;
        py = vy;
      } else {
        // zone ล่าง → panel1 (x=32..63)
        px = vx + 32;
        py = vy - 16;
      }
      matrix->drawPixel(px, py, fb[vy][vx]);
    }
  }
  matrix->flipDMABuffer();  // สลับบัฟเฟอร์ทีเดียวหลังวาดครบเฟรม กัน "สีซ้อน/ตัวหนังสือฉีก" ระหว่างวาด
}

// ================================================================
//  Font5x7 Draw
//  bit0 = row0 = บนสุด  →  ใช้ (1 << row) ถูกต้องแล้ว
// ================================================================
void fbDrawChar(int16_t x, int16_t y, char c, uint16_t color) {
  if (c < FONT5X7_FIRST_CHAR || c > FONT5X7_LAST_CHAR) return;
  const uint8_t *g = font5x7[c - FONT5X7_FIRST_CHAR];
  // font5x7: แต่ละ byte = 1 column (5 bytes = 5 columns)
  // bit0 = row บนสุด, bit6 = row ล่างสุด
  for (int col = 0; col < FONT5X7_CHAR_WIDTH; col++) {
    uint8_t line = g[col];
    for (int row = 0; row < FONT5X7_CHAR_HEIGHT; row++) {
      if (line & (1 << row)) {
        // col → x offset, row → y offset
        fbSetPixel(x + col, y + row, color);
      }
    }
  }
}

void fbDrawString(int16_t x, int16_t y, const char* str, uint16_t color) {
  int16_t cx = x;
  while (*str) {
    char c = *str++;
    if (c >= 'a' && c <= 'z') c -= 32;   // to uppercase
    fbDrawChar(cx, y, c, color);
    cx += FONT5X7_CHAR_SPACING;
  }
}

// วาด ° ที่ (x,y) คืน x ถัดไป
int16_t drawDegree(int16_t x, int16_t y, uint16_t color) {
  for (int col = 0; col < FONT5X7_DEGREE_WIDTH; col++) {
    uint8_t line = font5x7_degree[col];
    for (int row = 0; row < 3; row++) {
      if (line & (1 << row)) fbSetPixel(x + col, y + row, color);
    }
  }
  return x + FONT5X7_DEGREE_WIDTH + 1;
}

// ================================================================
//  SCROLLING TEXT — state ของแต่ละ zone (PM2.5 / T / H)
//  struct ScrollState + ฟังก์ชัน scrollTick()/prepareScroll() อยู่ใน
//  ScrollDisplay.h (แยกไฟล์เพื่อเลี่ยงปัญหา Arduino auto-prototype)
// ================================================================
ScrollState scrollPM25, scrollT, scrollH;

// ================================================================
//  PM25 COLOR / LEVEL — 5 ระดับ
// ================================================================
uint16_t getPM25Color(float val) {
  if (val <= PM25_BLUE_MAX)   return COLOR_CYAN;   // 0–15  : สีเดียวกับ Humidity
  if (val <= PM25_GREEN_MAX)  return COLOR_GREEN;  // 16–25
  if (val <= PM25_YELLOW_MAX) return COLOR_YELLOW; // 26–37
  if (val <= PM25_ORANGE_MAX) return COLOR_ORANGE; // 38–75
  return COLOR_RED;                                // > 75
}

// คำนวณระดับสีจากค่า pm25 ปัจจุบัน (return อย่างเดียว ไม่ set global)
int computePM25Level() {
  if      (pm25 <= PM25_BLUE_MAX)   return 0;  // BLUE
  else if (pm25 <= PM25_GREEN_MAX)  return 1;  // GREEN
  else if (pm25 <= PM25_YELLOW_MAX) return 2;  // YELLOW
  else if (pm25 <= PM25_ORANGE_MAX) return 3;  // ORANGE
  else                              return 4;  // RED
}

// ================================================================
//  DISPLAY LAYOUT — Virtual 32×32
//
//  Zone บน  (y= 0–15):
//    y= 0  "PM2.5"   WHITE    ← row 0 font อยู่ที่ y=0
//    y= 8  ค่า PM2.5          ← เหลือ 7px (row 0–6) พอดี
//
//  Zone ล่าง (y=16–31):
//    y=16  "T:xx°"  YELLOW
//    y=24  "H:xx%"  CYAN     ← เลื่อนจาก 25→24 ให้ font 7px ไม่ล้น y=31
// ================================================================
void drawZoneLabel() {
  fbDrawString(1, 0, "PM2.5", COLOR_WHITE);  // ชิดบนสุด
}

void drawZonePM25(bool visible) {
  for (int y = 7; y < 16; y++)
    for (int x = 0; x < VIRT_WIDTH; x++)
      fb[y][x] = 0;
  if (!visible) { scrollPM25.active = false; return; }

  char buf[12];
  snprintf(buf, sizeof(buf), "%d", (int)disp_pm25);
  int len     = strlen(buf);
  int textW   = (len - 1) * FONT5X7_CHAR_SPACING + FONT5X7_CHAR_WIDTH;
  int staticX = (VIRT_WIDTH - textW) / 2;      // ค่าปกติ: จัดกึ่งกลาง
  if (staticX < 0) staticX = 0;

  int16_t drawX = prepareScroll(scrollPM25, buf, textW, staticX, VIRT_WIDTH);
  fbDrawString(drawX, 8, buf, getPM25Color(disp_pm25));  // y=8
}

void drawZoneTH() {
  for (int y = 16; y < VIRT_HEIGHT; y++)
    for (int x = 0; x < VIRT_WIDTH; x++)
      fb[y][x] = 0;

  // T:xx° — y=17 (ความกว้างรวม = ข้อความ + สัญลักษณ์องศา)
  char tBuf[12];
  snprintf(tBuf, sizeof(tBuf), "T:%d", (int)disp_temp);
  int tTextW    = strlen(tBuf) * FONT5X7_CHAR_SPACING;
  int tContentW = tTextW + FONT5X7_DEGREE_WIDTH + 1;
  int tStaticX  = 1;

  int16_t tDrawX = prepareScroll(scrollT, tBuf, tContentW, tStaticX, VIRT_WIDTH);
  fbDrawString(tDrawX, 17, tBuf, COLOR_YELLOW);
  drawDegree(tDrawX + tTextW, 17, COLOR_YELLOW);

  // H:xx% — y=25
  char hBuf[12];
  snprintf(hBuf, sizeof(hBuf), "H:%d%%", (int)disp_humi);
  int hTextW   = (strlen(hBuf) - 1) * FONT5X7_CHAR_SPACING + FONT5X7_CHAR_WIDTH;
  int hStaticX = 1;

  int16_t hDrawX = prepareScroll(scrollH, hBuf, hTextW, hStaticX, VIRT_WIDTH);
  fbDrawString(hDrawX, 25, hBuf, COLOR_CYAN);
}

void displaySensorFull() {
  fbClear();
  drawZoneLabel();
  drawZonePM25(true);
  drawZoneTH();
  fbFlush();
}

void drawFullWithPM25(bool pm25Visible) {
  fbClear();
  drawZoneLabel();
  drawZonePM25(pm25Visible);
  drawZoneTH();
  fbFlush();
}

void displaySensorError() {
  fbClear();
  // "ERROR" กึ่งกลาง zone บน
  fbDrawString(1, 4,  "ERROR", COLOR_RED);
  // "RS485" กึ่งกลาง zone ล่าง
  fbDrawString(1, 20, "RS485", COLOR_RED);
  fbFlush();
}

// ================================================================
//  DISPLAY — Flash State Machine REMOVED
//  (ตามที่ผู้ใช้ต้องการ: ไม่กระพริบในทุกระดับ)
//
//  ฟังก์ชัน stub เก็บไว้เพื่อความเข้ากันได้ของ code เดิม
//  (ถูกเรียกจาก readModbus() และ checkPM25LevelChange())
// ================================================================
void resetFlash() {
  // no-op — ไม่มี flash state แล้ว
}

// ================================================================
//  updateDisplay — ไม่มีการกระพริบแล้ว แสดงค่าคงที่เท่านั้น
// ================================================================
void updateDisplay() {
  unsigned long now = millis();

  if (sensorErrorState) {
    if (now - t_display >= DISPLAY_UPDATE) {
      t_display = now;
      displaySensorError();
    }
    return;
  }

  if (!sensorOK && !hasDisplayData) {
    return;
  }

  // ขณะมี zone ใดกำลัง scroll (ข้อความยาวเกินจอ) ให้ redraw ถี่ขึ้นเพื่อให้ภาพเลื่อนลื่น
  bool anyScrolling = scrollPM25.active || scrollT.active || scrollH.active;
  unsigned long interval = anyScrolling ? DISPLAY_UPDATE_SCROLL : DISPLAY_UPDATE;

  if (now - t_display >= interval) {
    t_display = now;
    displaySensorFull();
  }
}

// ================================================================
//  MQTT PUBLISH
// ================================================================
void publishSensorError() {
  if (!networkReady || !client.connected()) return;
  JsonDocument doc;
  doc["temp"]   = 0;
  doc["humi"]   = 0;
  doc["pm25"]   = 0;
  doc["pm10"]   = 0;
  doc["blue"]   = false;
  doc["green"]  = false;
  doc["yellow"] = false;
  doc["orange"] = false;
  doc["red"]    = false;
  if (gps_fix) {
    doc["lat"] = serialized(String(latitude,  7));
    doc["lon"] = serialized(String(longitude, 7));
  }
  char buf[300];
  serializeJson(doc, buf);
  Serial.println("[MQTT] Publish ERROR state:");
  Serial.println(buf);
  client.publish("v1/devices/me/telemetry", buf);
}

void publishPM25Status() {
  if (!networkReady || !client.connected()) return;
  JsonDocument alarm;
  alarm["blue"]   = (pm25Level == 0);
  alarm["green"]  = (pm25Level == 1);
  alarm["yellow"] = (pm25Level == 2);
  alarm["orange"] = (pm25Level == 3);
  alarm["red"]    = (pm25Level == 4);
  char buf[150];
  serializeJson(alarm, buf);
  client.publish("v1/devices/me/telemetry", buf);
  Serial.printf("[PM25] blue=%s green=%s yellow=%s orange=%s red=%s\n",
    pm25Level == 0 ? "true" : "false",
    pm25Level == 1 ? "true" : "false",
    pm25Level == 2 ? "true" : "false",
    pm25Level == 3 ? "true" : "false",
    pm25Level == 4 ? "true" : "false");
}

// ================================================================
//  PM2.5 LEVEL CHANGE — พร้อม debounce กัน publish ถี่ตอนค่าคาบเส้นสี
//
//  หลักการ:
//    - สีบนจอคำนวณจากค่าจริง (getPM25Color/disp_pm25) → เปลี่ยนทันทีเสมอ
//    - ระดับที่ publish ขึ้น MQTT ต้องอ่านได้ "ระดับใหม่" นิ่งครบ
//      PM25_LEVEL_DEBOUNCE รอบก่อน ถึงจะ commit + publish
//    - ค่าครั้งแรก / ฟื้นจาก ERROR (pm25Level < 0) → commit ทันที ไม่ต้องรอ
// ================================================================
void commitPM25Level(int level) {
  pm25LevelPrev           = pm25Level;
  pm25Level               = level;
  pm25LevelCandidate      = level;
  pm25LevelCandidateCount = 0;

  const char* label;
  switch (level) {
    case 0:  label = "Very Good (BLUE)";   break;
    case 1:  label = "Good (GREEN)";       break;
    case 2:  label = "Moderate (YELLOW)";  break;
    case 3:  label = "Unhealthy (ORANGE)"; break;
    case 4:  label = "Hazardous (RED)";    break;
    default: label = "Unknown";            break;
  }
  Serial.printf("[PM25] Level committed → %s\n", label);

  publishPM25Status();
  displaySensorFull();
}

void checkPM25LevelChange() {
  int candidate = computePM25Level();

  // ระดับที่อ่านได้ = ระดับที่ publish ไปแล้ว → ไม่มีอะไรเปลี่ยน รีเซ็ตตัวนับ
  if (candidate == pm25Level) {
    pm25LevelCandidate      = candidate;
    pm25LevelCandidateCount = 0;
    return;
  }

  // ค่าครั้งแรก / เพิ่งฟื้นจาก ERROR (pm25Level < 0) → ยอมรับทันที ไม่ต้อง debounce
  if (pm25Level < 0) {
    commitPM25Level(candidate);
    return;
  }

  // ต่างจากระดับที่ publish อยู่ → เริ่ม/นับ debounce
  if (candidate == pm25LevelCandidate) {
    pm25LevelCandidateCount++;
  } else {
    pm25LevelCandidate      = candidate;
    pm25LevelCandidateCount = 1;
  }

  // ยังนิ่งไม่ครบรอบ → รอก่อน (ยังไม่ publish; จอยังโชว์สีตามค่าจริงอยู่แล้ว)
  if (pm25LevelCandidateCount < PM25_LEVEL_DEBOUNCE) {
    Serial.printf("[PM25] candidate=%d (%d/%d) — waiting to confirm\n",
                  candidate, pm25LevelCandidateCount, PM25_LEVEL_DEBOUNCE);
    return;
  }

  // นิ่งครบรอบ → commit + publish
  commitPM25Level(candidate);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Received on: "); Serial.println(topic);
}

// ================================================================
//  GSM / NETWORK
// ================================================================
void initGSM() {
  Serial.println("[GSM] Initializing...");
  networkReady = false;
  esp_task_wdt_reset();
  if (GSM.begin()) {
    IPAddress ip = Network.getDeviceIP();
    Serial.print("[GSM] Connected | IP: ");
    Serial.println(ip);
    if (ip == IPAddress(0, 0, 0, 0)) {
      // GSM.begin() คืน true แต่ไม่ได้ IP จริง = ยังไม่ได้ data attach จริง
      // (มักเป็นเพราะ eSIM ไม่มีแพ็กเกจ data/ยังไม่เปิดใช้งาน) ไม่ถือว่าพร้อมใช้งาน
      Serial.println("[GSM] WARNING: ได้ IP 0.0.0.0 — ยังไม่มี data session จริง จะลองใหม่รอบหน้า");
      networkReady = false;
    } else {
      networkReady = true;
    }
  } else {
    Serial.println("[GSM] Failed — will retry");
  }
  esp_task_wdt_reset();
}

void checkNetwork() {
  if (millis() - lastNetworkCheck < networkInterval) return;
  lastNetworkCheck = millis();
  if (!networkReady) {
    Serial.println("[NET] GSM not ready — reconnecting...");
    initGSM();
  }
}

// ================================================================
//  reconnectMQTT — พร้อม exponential backoff กัน retry ถี่ๆ ตอนต่อไม่ติด
// ================================================================
void reconnectMQTT() {
  if (millis() - lastMQTTCheck < mqttBackoff) return;
  lastMQTTCheck = millis();

  if (!networkReady || client.connected()) {
    mqttBackoff = MQTT_BACKOFF_MIN;   // เน็ต/MQTT ปกติดีอยู่แล้ว → รีเซ็ต backoff
    return;
  }

  esp_task_wdt_reset();
  Serial.printf("[MQTT] Connecting... (retry ทุก %lus)\n", mqttBackoff / 1000);
  if (client.connect("AIS_GATEWAY", access_token, NULL)) {
    Serial.println("[MQTT] Connected");
    mqttBackoff = MQTT_BACKOFF_MIN;   // สำเร็จ → รีเซ็ต backoff กลับค่าเริ่มต้น
    client.subscribe("v1/devices/me/rpc/request/+");
    if (sensorErrorState) publishSensorError();
    else if (sensorOK)    publishPM25Status();
  } else {
    Serial.printf("[MQTT] Fail state: %d — backoff %lus → ", client.state(), mqttBackoff / 1000);
    mqttBackoff *= 2;                                       // ต่อไม่ติด → ถอยเป็น 2 เท่า
    if (mqttBackoff > MQTT_BACKOFF_MAX) mqttBackoff = MQTT_BACKOFF_MAX;
    Serial.printf("%lus\n", mqttBackoff / 1000);
  }
  esp_task_wdt_reset();
}

// ================================================================
//  TEST MODE — จำลองค่า PM2.5 / T / H เพื่อทดสอบสี/สถานะบนจอ LED
//  ทุกค่าวนตามลิสต์ checkpoint ที่กำหนดไว้ตายตัว
// ================================================================
void readModbusTest() {
  esp_task_wdt_reset();

  // ---- PM2.5: วนตามลิสต์ checkpoint ----
  testPM25Idx = (testPM25Idx + 1) % (int)TEST_PM25_LIST_LEN;
  testPM25Value = testPM25List[testPM25Idx];

  // ---- T: วนตามลิสต์ -10, 10, 25, 33, 100 ----
  testTempIdx = (testTempIdx + 1) % (int)TEST_TEMP_LIST_LEN;
  testTempValue = testTempList[testTempIdx];

  // ---- H: วนตามลิสต์ 5, 10, 50, 150, 200 ----
  testHumiIdx = (testHumiIdx + 1) % (int)TEST_HUMI_LIST_LEN;
  testHumiValue = testHumiList[testHumiIdx];

  modbusFailCount  = 0;
  sensorOK         = true;
  bool wasError    = sensorErrorState;
  sensorErrorState = false;

  humi = (float)testHumiValue;
  temp = (float)testTempValue;
  pm25 = (float)testPM25Value;
  pm10 = pm25 * 1.2f;           // ค่าอ้างอิงคร่าวๆ ไม่มีผลต่อสีจอ

  disp_humi = humi;
  disp_temp = temp;
  disp_pm25 = pm25;
  hasDisplayData = true;

  Serial.printf("[TEST] PM2.5=%.0f  T=%.0f  H=%.0f\n", pm25, temp, humi);

  if (wasError) {
    Serial.println("[Sensor] Recovered from ERROR");
    resetFlash();
    pm25LevelPrev = -2;
    t_display     = millis();
  }

  checkPM25LevelChange();  // อัปเดตสี/ระดับถ้าเปลี่ยน (มี debounce ในตัว)
  displaySensorFull();     // บังคับ refresh ตัวเลขบนจอทุกรอบ แม้สียังไม่เปลี่ยน
}

// ================================================================
//  RS485 / MODBUS  (ใช้ RS485.h ของบอร์ด AIS 4G)
// ================================================================
void readModbusReal() {
  long raw_humi = -1, raw_temp = -1, raw_pm25 = -1, raw_pm10 = -1;
  bool ok = false;

  for (int attempt = 1; attempt <= MODBUS_RETRY_COUNT; attempt++) {
    esp_task_wdt_reset();

    raw_humi = rs485.holdingRegisterRead(MODBUS_ID, REG_START + REG_IDX_HUMI);
    raw_temp = rs485.holdingRegisterRead(MODBUS_ID, REG_START + REG_IDX_TEMP);
    raw_pm25 = rs485.holdingRegisterRead(MODBUS_ID, REG_START + REG_IDX_PM25);
    raw_pm10 = rs485.holdingRegisterRead(MODBUS_ID, REG_START + REG_IDX_PM10);

    if (raw_humi >= 0 && raw_temp >= 0 && raw_pm25 >= 0 && raw_pm10 >= 0) {
      ok = true;
      break;
    }

    Serial.printf("[Modbus] Retry %d/%d — humi=%ld temp=%ld pm25=%ld pm10=%ld\n",
                  attempt, MODBUS_RETRY_COUNT, raw_humi, raw_temp, raw_pm25, raw_pm10);
    delay(MODBUS_RETRY_DELAY);
  }
  esp_task_wdt_reset();

  if (ok) {
    modbusFailCount  = 0;
    sensorOK         = true;
    bool wasError    = sensorErrorState;
    sensorErrorState = false;

    humi = raw_humi / 10.0f;
    temp = raw_temp / 10.0f;
    pm25 = (float)raw_pm25;
    pm10 = (float)raw_pm10;

    disp_humi = humi;
    disp_temp = temp;
    disp_pm25 = pm25;
    hasDisplayData = true;

    Serial.printf("[Sensor] T:%.1f°C  H:%.1f%%  PM2.5:%.0f  PM10:%.0f\n",
                  temp, humi, pm25, pm10);

    if (wasError) {
      Serial.println("[Sensor] Recovered from ERROR");
      resetFlash();
      pm25LevelPrev = -2;
      t_display     = millis();
    }

    checkPM25LevelChange();

  } else {
    modbusFailCount++;
    sensorOK = false;
    Serial.printf("[Modbus] All retries failed (%d/%d)\n",
                  modbusFailCount, MODBUS_FAIL_MAX);

    if (modbusFailCount >= MODBUS_FAIL_MAX && !sensorErrorState) {
      sensorErrorState = true;
      humi = 0; temp = 0; pm25 = 0; pm10 = 0;
      pm25Level     = -1;
      pm25LevelPrev = -2;
      pm25LevelCandidate      = -2;   // เคลียร์ debounce ตอนเข้า ERROR
      pm25LevelCandidateCount = 0;
      Serial.println("[Modbus] !! ERROR STATE — sensor offline");
      resetFlash();
      displaySensorError();
      publishSensorError();
    }
  }
}

// ================================================================
//  readModbus() — จุดเรียกใช้งานหลัก (ถูกเรียกจาก setup()/loop())
//  สลับไปใช้โหมดทดสอบหรือโหมดจริงตาม TEST_MODE_PM25
// ================================================================
void readModbus() {
#if TEST_MODE_PM25
  readModbusTest();
#else
  readModbusReal();
#endif
}

// ================================================================
//  GPS
// ================================================================
void readGPS() {
  esp_task_wdt_reset();
  if (GPS.available()) {
    latitude  = GPS.latitude();
    longitude = GPS.longitude();
    gps_fix   = true;
    Serial.printf("[GPS] Fix: %.7f, %.7f\n", latitude, longitude);
  } else {
    gps_fix = false;
    Serial.println("[GPS] No Fix");
  }
}

// ================================================================
//  PUBLISH SENSOR DATA (รวม Humi/Temp/PM2.5/PM10/GPS ในครั้งเดียว)
// ================================================================
void publishData() {
  Serial.println("==============================");
  Serial.println("[Publish] Cycle start");
  Serial.printf("[Publish] sensorOK=%d sensorErrorState=%d gps_fix=%d\n",
                sensorOK, sensorErrorState, gps_fix);

  if (sensorErrorState) {
    Serial.println("[Publish] Sensor in ERROR state -> publishSensorError()");
    publishSensorError();
    Serial.println("==============================");
    return;
  }
  if (!sensorOK) {
    Serial.println("[Publish] Sensor not OK yet -> skip");
    Serial.println("==============================");
    return;
  }

  Serial.printf("[Publish] Humi=%.1f%%  Temp=%.1fC  PM2.5=%.1f  PM10=%.1f\n",
                humi, temp, pm25, pm10);
  if (gps_fix) {
    Serial.printf("[Publish] GPS Lat=%.7f Lon=%.7f\n", latitude, longitude);
  } else {
    Serial.println("[Publish] GPS no fix — lat/lon omitted");
  }

  if (networkReady && client.connected()) {
    JsonDocument doc;
    doc["temp"]   = serialized(String(temp, 1));
    doc["humi"]   = serialized(String(humi, 1));
    doc["pm25"]   = serialized(String(pm25, 1));
    doc["pm10"]   = serialized(String(pm10, 1));
    doc["blue"]   = (pm25Level == 0);
    doc["green"]  = (pm25Level == 1);
    doc["yellow"] = (pm25Level == 2);
    doc["orange"] = (pm25Level == 3);
    doc["red"]    = (pm25Level == 4);
    if (gps_fix) {
      doc["lat"] = serialized(String(latitude,  7));
      doc["lon"] = serialized(String(longitude, 7));
    }
    char buf[300];
    serializeJson(doc, buf);
    Serial.println("[MQTT] Publish payload:");
    Serial.println(buf);
    bool sent = client.publish("v1/devices/me/telemetry", buf);
    Serial.printf("[MQTT] Publish result: %s\n", sent ? "OK" : "FAILED");

  } else {
    Serial.println("[Offline] Network/MQTT not ready — skip publish (ไม่มีการเก็บค้างส่งอีกต่อไป)");
  }
  Serial.println("==============================");
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== IoT Gateway HUB75 | PM2.5 Monitor | RS-BYH-M ===");

  // ── Watchdog ──────────────────────────────────────────────────
  // Arduino-ESP32 core auto-init TWDT ของตัวเองไว้ก่อนแล้ว (timeout สั้น
  // กว่าที่เราต้องการมาก) เห็นได้จาก log "TWDT already initialized" —
  // ถ้าไม่ deinit ตัวเดิมก่อน esp_task_wdt_init() ของเราด้านล่างจะ fail
  // เงียบๆ แล้วระบบจะยังใช้ timeout สั้นเดิมอยู่ ทำให้ WDT panic ก่อนเวลา
  esp_task_wdt_deinit();
  const esp_task_wdt_config_t wdt_config = {
    .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  Serial.printf("[WDT] Watchdog initialized (%d sec)\n", WDT_TIMEOUT_SEC);

  // ── HUB75 Matrix ──────────────────────────────────────────────
  // P10 32×16 ×2 stack บน-ล่าง = virtual 32×32
  // chain=1 เพราะสายสัญญาณต่อแนวตั้ง ไม่ใช่แนวนอน
  // height=32 = รวม 2 แผง
  HUB75_I2S_CFG::i2s_pins _pins = {
    R1_PIN, G1_PIN, B1_PIN,
    R2_PIN, G2_PIN, B2_PIN,
    A_PIN,  B_PIN,  C_PIN,
    D_PIN,  E_PIN,
    LAT_PIN, OE_PIN, CLK_PIN
  };
  HUB75_I2S_CFG mxconfig(PANEL_W, PANEL_H, PANEL_CHAIN, _pins);
  // chain=2, width=32, height=16 → physical = 64×16
  // panel0 (x=0..31) = DATA ออกก่อน = แผงล่าง (ตาม wiring)
  // panel1 (x=32..63) = DATA ออกหลัง = แผงบน
  mxconfig.clkphase    = false;
  mxconfig.driver      = HUB75_I2S_CFG::SHIFTREG;
  mxconfig.double_buff = true;   // เปิด double buffer เพื่อกัน "สีซ้อน/ตัวอักษรฉีก" ระหว่างวาด
  mxconfig.i2sspeed    = HUB75_I2S_CFG::HZ_8M;  // ค่านี้เป็น clock ต่ำสุดที่ library รองรับ (ไม่มี HZ_5M จริง)

  matrix = new MatrixPanel_I2S_DMA(mxconfig);
  matrix->begin();
  matrix->setBrightness8(PANEL_BRIGHTNESS);  // ปรับค่าที่ #define PANEL_BRIGHTNESS ด้านบนไฟล์
  matrix->fillScreen(matrix->color565(255,0,0)); matrix->flipDMABuffer(); delay(1200); // แดงล้วน
  matrix->fillScreen(matrix->color565(0,255,0)); matrix->flipDMABuffer(); delay(1200); // เขียวล้วน
  matrix->fillScreen(matrix->color565(0,0,255)); matrix->flipDMABuffer(); delay(1200); // น้ำเงินล้วน
  matrix->fillScreen(matrix->color565(255,255,255)); matrix->flipDMABuffer(); delay(1200); // ขาวล้วน
  matrix->fillScreen(0);
  matrix->flipDMABuffer();   // ให้จอเริ่มต้นจากบัฟเฟอร์ที่เคลียร์แล้วจริงๆ

  COLOR_WHITE  = matrix->color565(255, 255, 255);
  COLOR_RED    = matrix->color565(255, 0,   0  );
  COLOR_GREEN  = matrix->color565(0,   255, 0  );
  COLOR_YELLOW = matrix->color565(255, 255, 0  );
  COLOR_CYAN   = matrix->color565(0,   200, 255);
  COLOR_ORANGE = matrix->color565(255, 128, 0  );
  Serial.println("[LED] Matrix initialized (32×32 / P10 stack)");

  // ── RS485 / Modbus (RS485.h) ─────────────────────────────────
  rs485.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX, RS485_TX, RS485_DIR);
  Serial.printf("[Modbus] ID=%d  Baud=%d  Retry=%dx%dms  FailMax=%d\n",
                MODBUS_ID, MODBUS_BAUD,
                MODBUS_RETRY_COUNT, MODBUS_RETRY_DELAY, MODBUS_FAIL_MAX);

  // ── GSM / MQTT ────────────────────────────────────────────────
  initGSM();
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(60);
  client.setSocketTimeout(15);
  client.setBufferSize(512);
  client.setCallback(mqttCallback);
  Serial.println("[MQTT] Client configured");

  // ── GPS ───────────────────────────────────────────────────────
  GPS.begin();
  Serial.println("[GPS] Initialized");

  hasDisplayData = false;

  // ── Splash Screen ─────────────────────────────────────────────
  fbClear();
  fbDrawString(7,  4,  "IOT",   COLOR_GREEN);   // zone บน กึ่งกลาง
  fbDrawString(1,  20, "READY", COLOR_YELLOW);  // zone ล่าง
  fbFlush();
  delay(2000);

  // ── Placeholder รอค่า sensor ──────────────────────────────────
  fbClear();
  fbDrawString(1,  0,  "PM2.5", COLOR_WHITE);
  fbDrawString(10, 8,  "---",   COLOR_WHITE);
  fbDrawString(1,  17, "T:--",  COLOR_YELLOW);
  drawDegree(25,   17, COLOR_YELLOW);
  fbDrawString(1,  25, "H:--%", COLOR_CYAN);
  fbFlush();

  resetFlash();
  t_display = millis();

  esp_task_wdt_reset();
  readModbus();

  Serial.println("=== Setup Complete ===\n");
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  esp_task_wdt_reset();

  client.loop();
  unsigned long now = millis();

  checkNetwork();
  reconnectMQTT();

  if (now - t_modbus >= MODBUS_INTERVAL) {
    t_modbus = now;
    readModbus();
  }

  if (now - t_gps >= GPS_INTERVAL) {
    t_gps = now;
    readGPS();
  }

  // ── Boot Publish: 1 นาทีหลังเปิดเครื่อง ส่งค่าทั้งหมดขึ้นครั้งแรก ──
  if (!bootPublishDone && now >= BOOT_PUBLISH_DELAY) {
    bootPublishDone = true;
    Serial.println("[Boot] 1 minute elapsed — sending initial publish");
    publishData();
    t_publish = now;   // reset รอบปกติให้นับใหม่จากจุดนี้
  }

  // ── Publish รอบปกติ: รวม Humi/Temp/PM2.5/PM10/GPS ทุก 15 นาที ──
  if (bootPublishDone && (now - t_publish >= PUBLISH_INTERVAL)) {
    t_publish = now;
    publishData();
  }

  updateDisplay();

  delay(50);
}

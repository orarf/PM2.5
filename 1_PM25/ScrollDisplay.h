#pragma once
#include <Arduino.h>
#include <string.h>

// ================================================================
//  ScrollDisplay.h — ตัวช่วยเลื่อนข้อความ (marquee) บนจอ LED
//
//  ใช้เมื่อข้อความ (เช่น ค่า PM2.5 / T / H) กว้างเกินความกว้างจอ
//    - ถ้าข้อความพอดีจอ  → วาดนิ่งตามตำแหน่งปกติ
//    - ถ้าข้อความยาวเกิน → เลื่อนจากขวาไปซ้ายวนซ้ำ
//
//  หมายเหตุ: อยู่แยกเป็น .h ต่างหาก (ไม่ได้เขียนไว้ใน .ino) เพราะ
//  Arduino IDE จะ auto-generate function prototype ของฟังก์ชันใน .ino
//  ไปแปะไว้บนสุดของไฟล์ ก่อนที่ struct ScrollState จะถูกประกาศ ทำให้
//  คอมไพล์ error "was not declared in this scope" — การย้ายมาไว้ใน
//  header ที่ #include เข้ามาแก้ปัญหานี้ได้ เพราะเนื้อหาที่ #include
//  จะถูกแทรกเข้าไปจริงๆ ก่อนจุดที่ Arduino แทรก prototype อัตโนมัติ
// ================================================================

#define SCROLL_STEP_PX         1     // เลื่อนทีละกี่ px ต่อ tick
#define SCROLL_INTERVAL_MS   130UL   // 1 tick ทุกกี่ ms (ยิ่งน้อยยิ่งเลื่อนเร็ว)
#define SCROLL_GAP_PX          8     // ช่องว่างก่อนวนข้อความกลับมาเริ่มใหม่ทางขวา
#define DISPLAY_UPDATE_SCROLL  90UL  // ความถี่ redraw จอ ขณะมี zone ใดกำลัง scroll

struct ScrollState {
  int16_t       x           = 0;     // ตำแหน่งวาดปัจจุบัน
  unsigned long lastMove     = 0;
  bool          active       = false;  // true = กำลัง scroll อยู่ตอนนี้
  char          lastText[16] = "";     // ข้อความล่าสุดที่วาด (ไว้เช็คว่าค่าค่าเปลี่ยนไหม)
};

// เดินตำแหน่ง scroll ไป 1 tick ถ้าถึงเวลา แล้วคืน x ปัจจุบันที่ใช้วาด
inline int16_t scrollTick(ScrollState &s, int16_t contentWidth, int16_t panelWidth) {
  unsigned long now = millis();
  if (now - s.lastMove >= SCROLL_INTERVAL_MS) {
    s.lastMove = now;
    s.x -= SCROLL_STEP_PX;
    // เลื่อนพ้นขอบซ้ายจนหมดข้อความ + เว้นช่องว่าง → วนกลับไปเริ่มใหม่ทางขวา
    if (s.x < -contentWidth - SCROLL_GAP_PX) s.x = panelWidth;
  }
  return s.x;
}

// เตรียม/อัปเดต state ของ zone ก่อนวาดข้อความ
// - ถ้าข้อความเปลี่ยน (ค่าที่อ่านได้ใหม่)   → เริ่ม scroll ใหม่จากขวาสุดเสมอ
// - ถ้าข้อความพอดีจอ                       → หยุด scroll วาดนิ่งที่ staticX
// - ถ้าข้อความยาวเกินจอ (ไม่เปลี่ยน)        → เดิน scroll ต่อจากตำแหน่งเดิม
// คืนค่า x ที่ควรใช้วาดข้อความในรอบนี้
inline int16_t prepareScroll(ScrollState &s, const char* text,
                              int16_t contentWidth, int16_t staticX,
                              int16_t panelWidth) {
  bool needsScroll = contentWidth > panelWidth;

  if (strcmp(text, s.lastText) != 0) {
    strncpy(s.lastText, text, sizeof(s.lastText) - 1);
    s.lastText[sizeof(s.lastText) - 1] = '\0';
    s.x      = panelWidth;
    s.active = needsScroll;
    return needsScroll ? s.x : staticX;
  }

  if (!needsScroll) {
    s.active = false;
    return staticX;
  }

  s.active = true;
  return scrollTick(s, contentWidth, panelWidth);
}

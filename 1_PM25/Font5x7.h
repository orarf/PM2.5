#pragma once
#include <stdint.h>

// ================================================================
//  Font5x7 — 5x7 pixel bitmap font (ASCII 32–90)
//  ครอบคลุม: space ! " # $ % & ' ( ) * + , - . /
//             0–9 : ; < = > ? @
//             A–Z
// ================================================================

#define FONT5X7_CHAR_WIDTH   5
#define FONT5X7_CHAR_HEIGHT  7
#define FONT5X7_CHAR_SPACING 6   // width + 1 px gap
#define FONT5X7_FIRST_CHAR   32  // ASCII space
#define FONT5X7_LAST_CHAR    90  // ASCII Z

extern const uint8_t font5x7[][FONT5X7_CHAR_WIDTH];

// ================================================================
//  Degree symbol ° bitmap (3x3 px, วางที่มุมบน)
//
//   col:  0  1  2
//  row 0: .  X  .
//  row 1: X  .  X
//  row 2: .  X  .
//
//  bit encoding (bit0 = row0):
//    col0 = 0b010 = 0x02
//    col1 = 0b101 = 0x05
//    col2 = 0b010 = 0x02
// ================================================================
#define FONT5X7_DEGREE_WIDTH   3
extern const uint8_t font5x7_degree[FONT5X7_DEGREE_WIDTH];

// ================================================================
//  ใช้ใน fbDrawString แทน "°C" — เรียกแล้วคืน x ถัดไป
//  ตัวอย่าง:
//    int cx = 0;
//    cx = fbDrawDegreeC(cx, 16, COLOR_YELLOW, setPixelFn);
// ================================================================
//
//  *** ผู้ใช้ต้อง implement fbDrawDegreeC() ใน main.cpp เอง
//      เพราะต้องเรียก fbSetPixel() ซึ่งอยู่ใน main.cpp ***
//
//  ตัวอย่าง implementation:
//
//  // วาด ° ที่ (x,y) คืนค่า x หลังจากวาดเสร็จ
//  int16_t drawDegree(int16_t x, int16_t y, uint16_t color) {
//    for (int col = 0; col < FONT5X7_DEGREE_WIDTH; col++) {
//      uint8_t line = font5x7_degree[col];
//      for (int row = 0; row < 3; row++)
//        if (line & (1 << row)) fbSetPixel(x + col, y + row, color);
//    }
//    return x + FONT5X7_DEGREE_WIDTH + 1;  // +1 gap
//  }
//
//  // วาด °C ที่ (x,y)
//  void drawDegreeC(int16_t x, int16_t y, uint16_t color) {
//    int16_t cx = drawDegree(x, y, color);
//    fbDrawChar(cx, y, 'C', color);
//  }
// ================================================================

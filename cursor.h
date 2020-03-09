#ifndef _CURSOR_H
#define _CURSOR_H

#include "neslib.h"
#include <inttypes.h>

#define OFF                0
#define ON                 1
#define CURSOR_FRAME_DELAY 5
#define TILE_MODE          0
#define PIXEL_MODE         1

#define MAP_SIZE_X        32
#define MAP_SIZE_Y        32

typedef struct Cursor {
  uint16_t x;
  uint16_t y;
  
  uint8_t mx;
  uint8_t my;
   
  byte state;
  byte value;  
  byte mode;
  
  char sprite;
  
  uint8_t framecount;
} Cursor;

extern Cursor cursor;

void __fastcall__ cursor_init(byte mode, char sprite);
void __fastcall__ cursor_move(void);

#endif // cursor.h
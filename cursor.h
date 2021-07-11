/* 
============================================================
Depth-first Search Algorithm - Proof-of-concept - NES MMC3
Copyright 2018 - 2021 Ninja Dynamics - See license below
============================================================
Creative Commons - Attribution 3.0 Unported
https://creativecommons.org/licenses/by/3.0/legalcode
You are free to:
------------------------------------------------------------
    Share - copy and redistribute the material in any
    medium or format.
    Adapt - remix, transform, and build upon the material
    for any purpose, even commercially.
Under the following terms:
------------------------------------------------------------
    Attribution - You must give appropriate credit,
    provide a link to the license, and indicate if
    changes were made. You may do so in any reasonable
    manner, but not in any way that suggests the licensor
    endorses you or your use.
    No additional restrictions — You may not apply legal
    terms or technological measures that legally restrict
    others from doing anything the license permits.
============================================================
*/

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
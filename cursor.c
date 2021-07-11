/* 
============================================================
Depth-first Search Algorithm - NES Proof-of-concept
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

#include "cursor.h"

Cursor cursor;

static byte gamepad_state;

void __fastcall__ cursor_init(byte mode, char sprite) {
  cursor.x          = 128;
  cursor.y          = 128;
  cursor.mx         = cursor.x / 8;
  cursor.my         = cursor.y / 8;
  cursor.value      = NULL;
  cursor.state      = OFF;
  cursor.sprite     = sprite;
  cursor.mode       = mode;
  cursor.framecount = CURSOR_FRAME_DELAY;
}

#define CURSOR_SCREEN_POSITION \
  ((int16_t)cursor.x - (int16_t)0)//camera.x)

#define CURSOR_LEFT \
  (cursor.mx > 0 && (gamepad_state & PAD_LEFT) && CURSOR_SCREEN_POSITION > 0 + 8)

#define CURSOR_RIGHT \
  (cursor.mx < MAP_SIZE_X && (gamepad_state & PAD_RIGHT) && CURSOR_SCREEN_POSITION < 256 - 16)

#define CURSOR_UP \
  (cursor.my > 0 && (gamepad_state & PAD_UP))

#define CURSOR_DOWN \
  (cursor.my < MAP_SIZE_Y && (gamepad_state & PAD_DOWN))

void __fastcall__ cursor_move(void) {
  gamepad_state = pad_poll(0);
  if (cursor.mode == TILE_MODE) {
    if (cursor.framecount > 0) {
      --cursor.framecount;
      return;
    }
    if (CURSOR_LEFT) {
      cursor.x  -= 8;
      cursor.mx -= 1;
      cursor.framecount = CURSOR_FRAME_DELAY;
    }
    else if (CURSOR_RIGHT) {
      cursor.x  += 8;
      cursor.mx += 1;
      cursor.framecount = CURSOR_FRAME_DELAY;
    }  
    if (CURSOR_UP) {
      cursor.y  -= 8;
      cursor.my -= 1;
      cursor.framecount = CURSOR_FRAME_DELAY;
    }  
    else if (CURSOR_DOWN) {
      cursor.y  += 8;
      cursor.my += 1;
      cursor.framecount = CURSOR_FRAME_DELAY;
    }   
  }
  else {
    if (CURSOR_LEFT) {
      cursor.x  -= 1;
      cursor.mx = cursor.x / 8;
      cursor.framecount = CURSOR_FRAME_DELAY;
    }
    else if (CURSOR_RIGHT) {
      cursor.x  += 1;
      cursor.mx = cursor.x / 8;
      cursor.framecount = CURSOR_FRAME_DELAY;
    }  
    if (CURSOR_UP) {
      cursor.y  -= 1;
      cursor.mx = cursor.x / 8;
      cursor.framecount = CURSOR_FRAME_DELAY;
    }  
    else if (CURSOR_DOWN) {
      cursor.y  += 1;
      cursor.mx = cursor.x / 8;
      cursor.framecount = CURSOR_FRAME_DELAY;
    }
  }
}

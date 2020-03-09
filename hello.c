
/*
A simple "hello world" example.
Set the screen background color and palette colors.
Then write a message to the nametable.
Finally, turn on the PPU to display video.
*/

#include <stdio.h>

#include "neslib.h"

#include "dfs.h"
//#link "dfs.c"


#include "cursor.h"
//#link "cursor.c"

// link the pattern table into CHR ROM
//#link "chr_generic.s"

// main function, run after console reset

static int16_t i, wp;
static uint8_t x, y;

char value[6];

/*{pal:"nes",layout:"nes"}*/
const char PALETTE[32] = { 
  0x21,			// screen color

  0x17,0x28,0x30,0x0,	// background palette 0
  0x1c,0x20,0x2c,0x0,	// background palette 1
  0x00,0x10,0x20,0x0,	// background palette 2
  0x06,0x16,0x26,0x0,   // background palette 3

  0x16,0x35,0x24,0x0,	// sprite palette 0
  0x00,0x37,0x25,0x0,	// sprite palette 1
  0x0d,0x2d,0x3a,0x0,	// sprite palette 2
  0x0d,0x27,0x2a	// sprite palette 3
};

unsigned char sprid;

uint8_t sx, sy, dx, dy;

byte pad;

void draw_map(void) {
  for (y = 0; y < 30; ++y) {
    for (x = 0; x < 32; ++x) {
      vram_adr(NTADR_A(x, y));
      vram_write(&area[y][x], 1);
    }
  }
}

void draw_path(void) {
  if (!wp) {
    vram_adr(NTADR_A(3, 2));
    vram_write("No solution", 11);    
  }
  else {
    vram_adr(NTADR_A(3, 2));
    vram_write("           ", 11);              
  }
  for (i = 0; i < wp; ++i) {    
    x = waypointX[i];
    y = waypointY[i];   
    vram_adr(NTADR_A(x, y));
    vram_write("+", 1);   
  }
}

void main(void) {
  
  sx = 0;
  sy = 0;
  dx = 0;
  dy = 0;

  pal_all(PALETTE);  
  
  // Draw area
  draw_map();
    
  //wp = solve(28, 25, 18, 4);
  wp = solve(28, 25, 18, 4);
  
  if (!wp) {
    vram_adr(NTADR_A(3, 2));
    vram_write("No solution", 11);    
  }  
  
  for (i = 0; i < wp; ++i) {    
    x = waypointX[i];
    y = waypointY[i];   
    vram_adr(NTADR_A(x, y));
    vram_write("+", 1);   
  }  
  
  cursor_init(TILE_MODE, 0x10);
  cursor.state = OFF;
  
  // enable PPU rendering (turn on screen)
  ppu_on_all();

  // infinite loop
  i = 0;
  while (1) {     
    oam_clear();
    
    if (cursor.state == ON) { 
      pad = pad_trigger(0);
      if (pad & PAD_A) {
        if (!sx && !sy) {
          wp = 0;
          sx = cursor.mx;
          sy = cursor.my;
          ppu_off();
          draw_map();
          ppu_on_all();
        }
        else if (sx && sy && !dx && !dy) {
          dx = cursor.mx;
          dy = cursor.my;
          wp = solve(sx, sy, dx, dy);          
          ppu_off();
          draw_map();
          draw_path();          
          ppu_on_all();          
        }  
        else {
          wp = 0;
          sx = 0;
          sy = 0;
          dx = 0;
          dy = 0;          
          ppu_off();
          draw_map();
          ppu_on_all();                    
        }
      }
      if (pad & PAD_B) {
        wp = 0;
        sx = 0;
        sy = 0;
        dx = 0;
        dy = 0;
        ppu_off();
        draw_map();
        ppu_on_all();
      }      
      cursor_move();
      if (sx && sy) {
        sprid = oam_spr(sx*8, sy*8 - 1, 'S', 0, sprid);
      }
      if (dx && dy) {
        sprid = oam_spr(dx*8, dy*8 - 1, 'F', 0, sprid);
      }      
      sprid = oam_spr(cursor.x, cursor.y - 1, cursor.sprite, 0, sprid);
    }
    
    if (wp) {
      x = waypointX[i];
      y = waypointY[i];
      sprid = oam_spr(x*8, y*8-1, 0x18, 0, sprid);
      if (++i == wp) i = 0; 
      delay(2);
    }
    ppu_wait_nmi();    
  };
}

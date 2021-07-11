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


#include "neslib.h"
#include "dfs.h"
//#link "dfs.c"

#include "cursor.h"
//#link "cursor.c"

//#link "chr_generic.s"

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

static uint16_t wp_i, wp;
static uint8_t  x, y, sprite;

static uint8_t px, py;
static int8_t  vx, vy;

static uint8_t sx, sy, dx, dy;

static byte pad;

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
  for (wp_i = 0; wp_i < wp; ++wp_i) {    
    x = waypointX[wp_i];
    y = waypointY[wp_i];   
    vram_adr(NTADR_A(x, y));
    vram_write("+", 1);   
  }
}

byte framecount;

void main(void) {
  
  sx = 0;
  sy = 0;
  dx = 0;
  dy = 0;

  pal_all(PALETTE);  
  
  // Draw area
  draw_map();
/*    
  //wp = solve(28, 25, 18, 4);
  //wp = solve(28, 25, 16, 4);
  //wp = solve(18, 4, 28, 25);
  
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
*/  
  cursor_init(TILE_MODE, 0x10);
  cursor.state = ON;
  
  // enable PPU rendering (turn on screen)
  ppu_on_all();

  // infinite loop  
  while (1) {     
    oam_clear();
    sprid = 0;
    
    if (cursor.state == ON) { 
      pad = pad_trigger(0);
      if (pad & PAD_A) {
        if ((!sx && !sy) || (dx && dy)) {
          wp = 0;
          sx = cursor.mx;
          sy = cursor.my;
          dx = NULL;
          dy = NULL;
          ppu_off();
          draw_map();
          ppu_on_all();
        }
        else if ((sx && sy) && (!dx && !dy)) {
          // - - - - -
          px = sx * 8;
          py = sy * 8;
          // - - - - -
          dx = cursor.mx;
          dy = cursor.my;
          wp = solve(sx, sy, dx, dy);          
          ppu_off();
          draw_map();
          draw_path();          
          ppu_on_all();          
          sprite = 0x18;
          wp_i = 0;
        }
      }
      if (pad & PAD_B) {
        wp = 0;
        sx = NULL;
        sy = NULL;
        dx = NULL;
        dy = NULL;
        ppu_off();
        draw_map();
        ppu_on_all();
      }      
      cursor_move();    
      sprid = oam_spr(cursor.x, cursor.y - 1, cursor.sprite, 0, sprid);
    }
    
    if (wp) {
      x = waypointX[wp_i];
      y = waypointY[wp_i];
            
      vx = 0;      
      if (px != x * 8) vx = (px > x * 8) ? -1 : 1;
      
      vy = 0;
      if (py != y * 8) vy = (py > y * 8) ? -1 : 1;
      
      px += vx;
      py += vy;
      
      //sprid = oam_spr(x*8, y*8-1, 0x18, 0, sprid);            
        
      sprid = oam_spr(px, py-1, sprite, 0, sprid);
      if (++framecount == 16) {
        framecount = 0;
        if (++sprite > 0x18 + 2) {          
          sprite = 0x18;
        }
      }
            
      if (px == x * 8 && py == y * 8) {        
        if (++wp_i >= wp) {wp_i = 0; px = sx*8; py = sy*8; }
      }
    }
    if (sx && sy) {
      sprid = oam_spr(sx*8, sy*8 - 1, 'S', 3, sprid);
    }
    if (dx && dy) {
      sprid = oam_spr(dx*8, dy*8 - 1, 'F', 3, sprid);
    }      
    ppu_wait_nmi();    
  };
}

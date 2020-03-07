
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

// link the pattern table into CHR ROM
//#link "chr_generic.s"

// main function, run after console reset

static int16_t i, x, y, wp;

static int16_t c, k;

char value[6];

#define COST(a, b) ( \
  ABS_DIFF(waypointX[a], waypointX[b]) + \
  ABS_DIFF(waypointY[a], waypointY[b]) \
)

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

void main(void) {

  pal_all(PALETTE);  
  
  // Draw area
  for (y = 0; y < 30; ++y) {
    for (x = 0; x < 32; ++x) {
      vram_adr(NTADR_A(x, y));
      vram_write(&area[y][x], 1);
    }
  }
  
  wp = solve(28, 25, 14, 4); 
  
  k = 1; i = 0;  
  while (i < wp) {
    for (c = i + 1; c < wp; ++c) {
      if (c - i > 1 && COST(i, c) == 1) {        
        waypointX[k] = waypointX[c];
        waypointY[k] = waypointY[c];
        ++k;
        i = c;
        continue;
      }
    }
    ++i;
    waypointX[k] = waypointX[i];
    waypointY[k] = waypointY[i];
    ++k;
  }
  
  if (!wp) {
    vram_adr(NTADR_A(3, 2));
    vram_write("No solution", 11);    
  }  
  
  for (i = 0; i < k; ++i) {    
    x = waypointX[i];
    y = waypointY[i];   
    vram_adr(NTADR_A(x, y));
    vram_write("+", 1);   
  }  
  
  // enable PPU rendering (turn on screen)
  ppu_on_all();

  // infinite loop
  while (1) {
    for (i = 0; i < k; ++i) {    
      x = waypointX[i];
      y = waypointY[i];
      oam_spr(x*8, y*8-1, 0x18, 0, 0);
      delay(2);      
    }    
    ppu_wait_nmi();
  };
}

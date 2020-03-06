
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

char value[6];

void main(void) {

  // set palette colors
  pal_col(0,0x21);	// set screen to dark blue
  pal_col(1,0x17);	// fuchsia
  pal_col(2,0x28);	// grey
  pal_col(3,0x30);	// white

  // Draw area
  for (y = 0; y < 30; ++y) {
    for (x = 0; x < 32; ++x) {
      vram_adr(NTADR_A(x, y));
      vram_write(&area[y][x], 1);
    }
  }
  
  wp = solve(2, 25, 28, 3);   
  
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
  
  // enable PPU rendering (turn on screen)
  ppu_on_all();

  // infinite loop
  while (1) {
    for (i = 0; i < wp; ++i) {    
      x = waypointX[i];
      y = waypointY[i];   
      oam_spr(x*8, y*8-1, 0x01, 0, 0);
      delay(2);      
    }    
    ppu_wait_nmi();
  };
}

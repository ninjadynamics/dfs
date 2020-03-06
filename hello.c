
/*
A simple "hello world" example.
Set the screen background color and palette colors.
Then write a message to the nametable.
Finally, turn on the PPU to display video.
*/

#include "neslib.h"

#include "dfs.h"
//#link "dfs.c"

// link the pattern table into CHR ROM
//#link "chr_generic.s"

// main function, run after console reset

static uint8_t i, x, y, wp;

char value[2];

void main(void) {

  // set palette colors
  pal_col(0,0x02);	// set screen to dark blue
  pal_col(1,0x14);	// fuchsia
  pal_col(2,0x20);	// grey
  pal_col(3,0x30);	// white

  // Draw area
  for (y = 0; y < 30; ++y) {
    for (x = 0; x < 32; ++x) {
      vram_adr(NTADR_A(x, y));
      vram_write(&area[y][x], 1);
    }
  }

  initialize_solver();
  wp = solve(3, 3, 10, 3);
  
  for (i = 0; i < wp; ++i) {    
    vram_adr(NTADR_A(waypointX[i], waypointY[i]));
    vram_write("@", 1);    
  }

  value[0] = '0' + wp;
  
  vram_adr(NTADR_A(4, 3));
  vram_write(value, 1);    

  
  // enable PPU rendering (turn on screen)
  ppu_on_all();

  // infinite loop
  while (1) ;
}

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

#include "dfs.h"

#define ONE                       (byte)1
#define BIT_ON(v, n)              (v |= ONE << n)
#define BIT_OFF(v, n)             (v &= ~(ONE << n))
#define BIT_VALUE(v, n)           ((v >> n) & ONE)
#define BIT_TOGGLE(v, n)          (v ^= ONE << n)

#define BIT_ARRAY_SET(a, i)       BIT_ON(    a[(i) / 8], ((i) % 8) )
#define BIT_ARRAY_UNSET(a, i)     BIT_OFF(   a[(i) / 8], ((i) % 8) )
#define BIT_ARRAY_VALUE(a, i)     BIT_VALUE( a[(i) / 8], ((i) % 8) )

#define SIZE_X 32
#define SIZE_Y 32

typedef uint8_t bit8_t;

const char area[32][32] = {
  {" !               X       X    ! "},
  {" 1               X       X    0 "},
  {" 2               X       XXX  _ "},
  {" 3               X XX      XXX| "},
  {" 4    XXXXX      X            | "},
  {" 5         X     X            | "},
  {" 6X         XXX  XX   XXXXX   | "},
  {" 7 X          XXXXXXXXXXXXX   | "},
  {" 8  X                         _ "},
  {" 9             X              | "},
  {" 0          XXX X  X          | "},
  {" 1XX                      XXXX| "},
  {" 2        X          X   X    | "},
  {" 3      X    X   X    XXXX    | "},
  {" 4 XX XXXX          X         _ "},
  {" 5      X                     | "},
  {" 6       X     XXXX   X XXX   | "},
  {" 7    X      X        X XX    | "},
  {" 8        XX         XX  X    | "},
  {" 9XXXXXXXXXXXXXXXXXXXXXXXXXX  | "},
  {" 0                            _ "},
  {" 1  XXXXXXXXXXXXXXXXXXXXXXXXXX| "},
  {" 2                            | "},
  {" 3                X     XXX   | "},
  {" 4X              X X    X X   | "},
  {" 5  X      XXXX   X     XXX   | "},
  {" 6  X   X                     | "},
  {"X723456789012345678901234567890X"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"}
};

#define stack    (*(volatile uint16_t (*)[STACK_SIZE])(0x6000))

// Stack index goes negative
static int16_t   stack_index;

// Waypoint indexes go negative
static int16_t   waypointX_index;
static int16_t   waypointY_index;

// Always positive [0..1023]
static uint16_t  index;
static uint16_t  destIndex;
static uint16_t  newIndex;

// Always positive
static uint16_t  i; 
static uint16_t  c, k;
static uint16_t  num_nodes;
static uint16_t  end;

// Always positive [0..31]
static uint8_t   x;
static uint8_t   y;
static uint8_t   tmp;

// Distances go negative
static int8_t    distX;
static int8_t    distY;

// Visited index goes negative
static bit8_t    visited[1024/8];
static int16_t   visited_index;

// Always positive [0..31]
static uint8_t   startX, startY;
static uint8_t   destX, destY;

// Control variables
static bool      is_horizontal;
static uint8_t   pass;


#define VALUE_AT(x, y) ( \
  area[y][x] != ' ' ? 1 : 0 \
)

#define PUSH(s, v) ( \
  s[++s##_index] = (v) \
)

#define POP(s) ( \
  s[s##_index--] = NULL \
)

#define TOP(s) ( \
  s[s##_index] \
)

#define EMPTY(s) ( \
  s##_index < 0 \
)

#define SET_VISITED_AT(i) (\
  visited_index = i, \
  BIT_ARRAY_SET(visited, visited_index) \
)

#define NOT_VISITED(i) (\
  visited_index = i, \
  BIT_ARRAY_VALUE(visited, visited_index) == 0 \
)

int16_t __fastcall__ solve(uint8_t sx, uint8_t sy, uint8_t dx, uint8_t dy) {

  // Init
  pass = 0;
  
  // Return if invalid destination
  if ((sx == dx && sy == dy) || VALUE_AT(sx, sy) || VALUE_AT(dx, dy)) return false;  
  
  // Start solving
  solve:
  ++pass;

  // Reset the stack
  for (stack_index = 0; stack_index < STACK_SIZE; ++stack_index) {
    stack[stack_index] = NULL;
  }
  
  // Reset the waypoints
  for (stack_index = 0; stack_index < STACK_SIZE; ++stack_index) {
    waypointX[stack_index] = NULL;
    waypointY[stack_index] = NULL;
  }  

  // Reset the visited map
  for (visited_index = 0; visited_index < SIZE_OF_ARRAY(visited); ++visited_index) {
    visited[visited_index] = NULL;
  }

  // Get the start point
  startX = sx;
  startY = sy;

  // Get the end point
  destX = dx;
  destY = dy;

  // Start (x, y) index
  index = (startY * SIZE_X) + startX;

  // Destination (x, y) index
  destIndex = (destY * SIZE_X) + destX;

  // Empty the stack
  stack_index = -1;

  // Empty the waypoints
  waypointX_index = -1;
  waypointY_index = -1;

  //Initializes the result and sets the first stack frame
  PUSH(stack, index);

  while (!EMPTY(stack)) {

    //Try again in reverse
    if (stack_index > STACK_SIZE - 1) {
      if (pass < 2) {
        x  = dx; y  = dy;
        dx = sx; dy = sy;
        sx =  x; sy =  y;
        goto solve;
      }      
      return NULL;
    }

    //Gets the index from the top of the Stack
    index = TOP(stack);

    //Marks as visited
    SET_VISITED_AT(index + 1);

    //Computes the is_horizontal and vertical distances    
    y = index / SIZE_X;
    x = index % SIZE_X;
    distX = destX - x;
    distY = destY - y;

    //If the goal is reached...
    if (index == destIndex) {
      PUSH(waypointX, x);
      PUSH(waypointY, y);
      ++stack_index;
      break;
    }

    //Selects the axis to follow
    is_horizontal = ABS(distX) > ABS(distY);

    //Selects the next cell to visit
    newIndex = 0;
    if (is_horizontal) {
      //The is_horizontal axis is explored first
      if (distX > 0) { //On the left -> to the right
        if (x != (SIZE_X - 1) && VALUE_AT(x + 1, y) == 0) {
          newIndex = index + 1;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      } else { //On the right -> to the left
        if (x != 0 && VALUE_AT(x - 1, y) == 0) {
          newIndex = index - 1;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      }
      //Vertical axis
      if (distY > 0) { //Too low -> to the top
        if (y != (SIZE_Y - 1) && VALUE_AT(x, y + 1) == 0) {
          newIndex = index + SIZE_X;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
        if (y != 0 && VALUE_AT(x, y - 1) == 0) {
          newIndex = index - SIZE_X;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      } else { //'Too high -> to the bottom
        if (y != 0 && VALUE_AT(x, y - 1) == 0) {
          newIndex = index - SIZE_X;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
        if (y != (SIZE_Y - 1) && VALUE_AT(x, y + 1) == 0) {
          newIndex = index + SIZE_X;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      }
      //Last possible direction
      if (distX > 0) {
        if (x != 0 && VALUE_AT(x - 1, y) == 0) {
          newIndex = index - 1;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      } else {
        if (x != (SIZE_X - 1) && VALUE_AT(x + 1, y) == 0) {
          newIndex = index + 1;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      }
    } else {
      //The vertical axis is explored first
      if (distY > 0) { //Too low -> to the top
        if (y != (SIZE_Y - 1) && VALUE_AT(x, y + 1) == 0) {
          newIndex = index + SIZE_X;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      } else { //Too high -> to the bottom
        if (y != 0 && VALUE_AT(x, y - 1) == 0) {
          newIndex = index - SIZE_X;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      }
      //Horizontal axis
      if (distX > 0) { //On the left -> to the right
        if (x != (SIZE_X - 1) && VALUE_AT(x + 1, y) == 0) {
          newIndex = index + 1;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
        if (x != 0 && VALUE_AT(x - 1, y) == 0) {
          newIndex = index - 1;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      } else { //On the right -> to the left
        if (x != 0 && VALUE_AT(x - 1, y) == 0) {
          newIndex = index - 1;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
        if (x != (SIZE_X - 1) && VALUE_AT(x + 1, y) == 0) {
          newIndex = index + 1;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      }
      //Last possible direction
      if (distY > 0) {
        if (y != 0 && VALUE_AT(x, y - 1) == 0) {
          newIndex = index - SIZE_X;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      } else {
        if (y != (SIZE_Y - 1) && VALUE_AT(x, y + 1) == 0) {
          newIndex = index + SIZE_X;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      }
    }

    //Removes the current cell from the solution array
    if (stack_index >= 0) {
      POP(stack);
      POP(waypointX);
      POP(waypointY);
    }
  }
  
  // Impossibru!! No solution
  if (stack_index < 0) return 0;
  
  // Give it another go:
  // Sometimes [B to A] path is better than [A to B]
  if (pass > 1) {
    end = stack_index - 1;
    for (i = 0; i < stack_index / 2; ++i) {
      tmp = waypointX[i];
      waypointX[i] = waypointX[end];
      waypointX[end] = tmp;
      --end;
    }
    end = stack_index - 1;
    for (i = 0; i < stack_index / 2; ++i) {
      tmp = waypointY[i];
      waypointY[i] = waypointY[end];
      waypointY[end] = tmp;
      --end;
    }   
  }
 
  // Optimize path    
  #define COST(a, b) ( \
    sx = waypointX[a], sy = waypointY[a], \
    dx = waypointX[b], dy = waypointY[b], \
    ABS_DIFF(sx, dx) + \
    ABS_DIFF(sy, dy) \
  )  
  num_nodes = stack_index;
  do {
    pass = TRUE;
    k = 0; i = 0;
    while (i < num_nodes) {
      // WORKAROUND: I don't understand this cast to 32-bit
      // but it seems to be absolutely necessary! WTF!?
      for (c = (uint32_t)i + 2; c < num_nodes; ++c) {
        if (COST(i, c) == 1) {        
          i = c - 1;
          pass = FALSE;
          break;
        }
      } 
      ++i; ++k;
      waypointX[k] = waypointX[i];
      waypointY[k] = waypointY[i];   
    }   
    num_nodes = k;  
  } while (!pass);
  
  return num_nodes;
  //Based on Informatix's SDA algorithm: www.b4x.com/android/forum/threads/optimization-with-b4a.57913
}
#include "dfs.h"

#define ONE                       (byte)1
#define BIT_ON(v, n)              (v |= ONE << n)
#define BIT_OFF(v, n)             (v &= ~(ONE << n))
#define BIT_VALUE(v, n)           ((v >> n) & ONE)
#define BIT_TOGGLE(v, n)          (v ^= ONE << n)
#define BIT_ARRAY_SET(a, i)       BIT_ON(a[(i) >> 3], (i & 0x07))
#define BIT_ARRAY_UNSET(a, i)     BIT_OFF(a[(i) >> 3], (i & 0x07))
#define BIT_ARRAY_VALUE(a, i)     BIT_VALUE(a[(i) >> 3], (i & 0x07))

#define SIZE_X 32
#define SIZE_Y 32

typedef uint8_t bit8_t;

const char area[32][32] = {
  {" !               X            ! "},
  {" 1               X            0 "},
  {" 2               X            _ "},
  {" 3               X XX         | "},
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
  {" 3                            | "},
  {" 4X                           | "},
  {" 5  X      XXXX               | "},
  {" 6  X   X                     | "},
  {"X723456789012345678901234567890X"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"}  
};


uint8_t            waypointX[256];
uint8_t            waypointY[256];

static int16_t     waypointX_index;
static int16_t     waypointY_index;

uint16_t    highest;

//static uint16_t    stack[256];
#define stack    (*(volatile int16_t (*)[1024])(0x6000))

static int16_t   stack_index;

static int16_t   x;
static int16_t   y; 
static int16_t   distX;
static int16_t   distY;

static bit8_t    visited[1024/8];
static uint16_t  visited_index;

static int16_t   startX, startY;
static int16_t   destX, destY;

static int16_t   index;
static int16_t   destIndex;
static int16_t   newIndex;

static bool      is_horizontal;

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
  s##_index == -1 \
)

#define SET_VISITED_AT(i) (\
  visited_index = i, \
  BIT_ARRAY_SET(visited, visited_index) \
)

#define NOT_VISITED(i) (\
  visited_index = i, \
  BIT_ARRAY_VALUE(visited, visited_index) == 0 \
)

static int16_t result;
int16_t __fastcall__ solve(uint8_t sx, uint8_t sy, uint8_t dx, uint8_t dy) {  
    
  // Return if invalid destination
  if ((sx == dx && sy == dy) || VALUE_AT(dx, dy)) return false;
  
  // Reset the stack
  for (stack_index = 0; stack_index < SIZE_OF_ARRAY(stack); ++stack_index) {
    stack[stack_index] = NULL;
  }  
  
  // Reset the visited map
  for (visited_index = 0; visited_index < SIZE_OF_ARRAY(visited); ++visited_index) {
    visited[visited_index] = NULL;
  }    
  
  startX = sx;
  startY = sy;
  
  destX  = dx;
  destY  = dy;    
  
  // Start (x, y) index
  index = (startY * SIZE_X) + startX;
  
  // Destination (x, y) index 
  destIndex = (destY  * SIZE_X) + destX;  
  
  // Empty the stack
  stack_index = -1;
  
  // Empty the waypoints
  waypointX_index = -1;
  waypointY_index = -1;
  
  //Initializes the result and sets the first stack frame
  result = false;
  PUSH(stack, index);  
  
  while (!EMPTY(stack)) {    
    
    if (stack_index + 1 > highest) highest = stack_index + 1;
    
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
      result = true;
      break;
    }

    //Selects the axis to follow    
    is_horizontal = ABS(distX) > ABS(distY);
    
    /*    
    is_horizontal = false;
    if (distX > 0) {
      if (distY > 0) {
        is_horizontal = (distX > distY);
      } else {
        is_horizontal = (distX > -distY);
      }
    } else {
      if (distY > 0) {
        is_horizontal = (-distX > distY);
      } else {
        is_horizontal = (-distX > -distY);
      }
    }*/

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
    POP(stack);
    
    if ((waypointX_index + 1) + (waypointY_index + 1) >= 2) {
      POP(waypointX);
      POP(waypointY);
    }    
    result = false;    
  }
  result = (waypointX_index + 1);
  return result;
  //Based on Informatix's SDA algorithm: www.b4x.com/android/forum/threads/optimization-with-b4a.57913
}
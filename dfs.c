#include "dfs.h"

#define ONE                       (byte)1
#define BIT_ON(v, n)              (v |= ONE << n)
#define BIT_OFF(v, n)             (v &= ~(ONE << n))
#define BIT_VALUE(v, n)           ((v >> n) & ONE)
#define BIT_TOGGLE(v, n)          (v ^= ONE << n)
#define BIT_ARRAY_SET(a, i)       BIT_ON(a[(i) >> 3], (i & 0x07))
#define BIT_ARRAY_UNSET(a, i)     BIT_OFF(a[(i) >> 3], (i & 0x07))
#define BIT_ARRAY_VALUE(a, i)     BIT_VALUE(a[(i) >> 3], (i & 0x07))

#define sizeX 32
#define sizeY 32

typedef uint8_t bit8_t;

const char area[32][32] = {
  {" !                            ! "},
  {" 1                            0 "},
  {" 2                            _ "},
  {" 3                 XX         | "},
  {" 4    XXXXX                   | "},
  {" 5         X                  | "},
  {" 6X         XXX  XX   XXXXX   | "},
  {" 7 X                          | "},
  {" 8  X                         _ "},
  {" 9                            | "},
  {" 0          XX     X          | "},
  {" 1XXXXXXXX         XXXXX XXXXX| "},
  {" 2                            | "},
  {" 3           X                | "},
  {" 4 XX XXXXXXXX                _ "},
  {" 5                            | "},
  {" 6             XXXX   X  XX   | "},
  {" 7                    X       | "},
  {" 8        XX         XX       | "},
  {" 9                            | "},
  {" 0               XXX          _ "},
  {" 1          XXX               | "},
  {" 2                            | "},
  {" 3     XXX               XXX  | "},
  {" 4                            | "},
  {" 5         XXXX               | "},
  {" 6  X   X                     | "},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"},
  {"X7XXXXXXXXXXXXXXXXXXXXXXXXXXXXFX"}  
};

static uint16_t stack[128];

static uint8_t  waypointX_index;
static uint8_t  waypointY_index;
static uint8_t  stack_index;

static uint8_t  x;
static uint8_t  y; 
static int16_t  distX;
static int16_t  distY;

static bit8_t   map[1024/8];
static uint16_t value_index;

static bit8_t   visited[1024/8];
static uint16_t visited_index;

static uint8_t  startX, startY;
static uint8_t  destX, destY;

static uint16_t index;
static uint16_t destIndex;
static uint16_t newIndex;

static bool     is_horizontal;
static bool     result;

uint8_t         waypointX[128];
uint8_t         waypointY[128];


#define VALUE_AT(x, y) ( \
  value_index = (y * sizeX) + x, \
  BIT_ARRAY_VALUE(map, value_index) \
)

#define PUSH(s, v) ( \
  s[++s##_index] = (v) \
)

#define POP(s) ( \
  s[s##_index--] \
)

#define TOP(s) ( \
  s[s##_index] \
)

#define EMPTY(s) ( \
 (int8_t)s##_index < 0 \
)

#define SET_VISITED(i) (\
  visited_index = index + 1, \
  BIT_ARRAY_SET(visited, visited_index) \
)

#define NOT_VISITED(i) (\
  visited_index = index + 1, \
  !BIT_ARRAY_VALUE(visited, visited_index) \
)

void __fastcall__ initialize_solver(void) {  
  for (y = 0; y < sizeX; ++y) {
    for (x = 0; x < sizeY; ++x) {
      value_index = (y * sizeX) + x;
      area[y][x] != ' ' ? 
      	BIT_ARRAY_SET(map, value_index) 
        : 
      	BIT_ARRAY_UNSET(map, value_index);
    }
  }
}

bool __fastcall__ solve(uint8_t sx, uint8_t sy, uint8_t dx, uint8_t dy) {
  startX = sx,
  startY = sy,
  destX  = dx,
  destY  = dy,
  
  
  index = (startY * sizeX) + startX;
  
  stack_index     = -1;
  waypointX_index = -1;
  waypointY_index = -1;  
  
  //Initializes the result and sets the first stack frame
  result = false;
  PUSH(stack, index);   
  
  while (!EMPTY(stack)) {
    //Gets the index from the top of the Stack
    index = TOP(stack);

    //Marks as visited    
    SET_VISITED(index + 1);    

    //Computes the is_horizontal and vertical distances
    y = index / sizeX;
    x = index % sizeX;
    distX = destX - x;
    distY = destY - y;

    //If the goal is reached...
    if (index == destIndex) {
      PUSH(waypointX, x);
      PUSH(waypointX, y);
      result = true;
      while (!EMPTY(stack)) {
        POP(stack);        
      }
      break;
    }

    //Selects the axis to follow
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
    }

    //Selects the next cell to visit
    newIndex = 0;
    if (is_horizontal) {
      //The is_horizontal axis is explored first
      if (distX > 0) { //On the left -> to the right
        if (x != (sizeX - 1) && VALUE_AT(x + 1, y) == 0) {
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
        if (y != (sizeY - 1) && VALUE_AT(x, y + 1) == 0) {
          newIndex = index + sizeX;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
        if (y != 0 && VALUE_AT(x, y - 1) == 0) {
          newIndex = index - sizeX;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      } else { //'Too high -> to the bottom
        if (y != 0 && VALUE_AT(x, y - 1) == 0) {
          newIndex = index - sizeX;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
        if (y != (sizeY - 1) && VALUE_AT(x, y + 1) == 0) {
          newIndex = index + sizeX;
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
        if (x != (sizeX - 1) && VALUE_AT(x + 1, y) == 0) {
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
        if (y != (sizeY - 1) && VALUE_AT(x, y + 1) == 0) {
          newIndex = index + sizeX;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      } else { //Too high -> to the bottom
        if (y != 0 && VALUE_AT(x, y - 1) == 0) {
          newIndex = index - sizeX;
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
        if (x != (sizeX - 1) && VALUE_AT(x + 1, y) == 0) {
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
        if (x != (sizeX - 1) && VALUE_AT(x + 1, y) == 0) {
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
          newIndex = index - sizeX;
          if (NOT_VISITED(newIndex + 1)) {
            PUSH(waypointX, x);
            PUSH(waypointY, y);
            PUSH(stack, newIndex);
            continue;
          }
        }
      } else {
        if (y != (sizeY - 1) && VALUE_AT(x, y + 1) == 0) {
          newIndex = index + sizeX;
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
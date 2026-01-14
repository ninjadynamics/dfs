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
    No additional restrictions -- You may not apply legal
    terms or technological measures that legally restrict
    others from doing anything the license permits.
============================================================
*/

#include "dfs.h"

#define ONE                       (byte)1
#define BIT_ON(v, n)              (v |= (ONE << (n)))
#define BIT_OFF(v, n)             (v &= (byte)~(ONE << (n)))
#define BIT_VALUE(v, n)           (((v) >> (n)) & ONE)

#define BIT_ARRAY_SET(a, i)       BIT_ON(    (a)[(i) / 8], ((i) % 8) )
#define BIT_ARRAY_UNSET(a, i)     BIT_OFF(   (a)[(i) / 8], ((i) % 8) )
#define BIT_ARRAY_VALUE(a, i)     BIT_VALUE( (a)[(i) / 8], ((i) % 8) )

#define SIZE_X 32
#define SIZE_Y 30 /* last 2 lines never displayed */

typedef uint8_t bit8_t;

const char area[32][32] = {
  {"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"},
  {"X                              X"},
  {"X                X          X  X"},
  {"X                X XX  XX XXXX X"},
  {"X     XXXXX      X             X"},
  {"X          X     X   X      X  X"},
  {"X X         XXX  XX   XXXXX    X"},
  {"X  X          XXXXXXXXXXXXX    X"},
  {"X   X                          X"},
  {"X              X               X"},
  {"X           XXX X  X           X"},
  {"X XX                      XXXX X"},
  {"X         X          X   X     X"},
  {"X       X    X   X    XXXX     X"},
  {"X  XX XXXX          X          X"},
  {"X       X                      X"},
  {"X        X     XXXX   X XXX    X"},
  {"X     X      X        X XX     X"},
  {"X         XX         XX  X     X"},
  {"X XXXXXXXXXXXXXXXXXXXXXXXXXX   X"},
  {"X                              X"},
  {"X   XXXXXXXXXXXXXXXXXXXXXXXXXX X"},
  {"X                              X"},
  {"X                 X     XXX    X"},
  {"X X              X X    X X    X"},
  {"X   X      XXXX   X     XXX    X"},
  {"X   X   X                      X"},
  {"X                              X"},
  {"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"},
  {"                                "},
  {"                                "},
  {"                                "}
};

#define stack    (*(volatile uint16_t (*)[STACK_SIZE])(0x6000))

/* Stack index goes negative */
static int16_t   stack_index;

/* Waypoint indexes go negative */
static int16_t   waypointX_index;
static int16_t   waypointY_index;

/* Always positive */
static uint16_t  index;
static uint16_t  destIndex;
static uint16_t  newIndex;

/* Always positive */
static uint16_t  i;
static uint16_t  c, k;
static uint16_t  num_nodes;
static uint16_t  end;

/* Always positive [0..31] */
static uint8_t   x;
static uint8_t   y;
static uint8_t   tmp;

/* Distances go negative */
static int8_t    distX;
static int8_t    distY;

/* Visited */
#define CELL_COUNT      (SIZE_X * SIZE_Y)
#define VISITED_BYTES   ((CELL_COUNT + 7) / 8)
static bit8_t    visited[VISITED_BYTES];
static uint16_t  visited_index;

/* Always positive [0..31] */
static uint8_t   startX, startY;
static uint8_t   destX, destY;

/* Control variables */
static bool      is_horizontal;
static uint8_t   pass;

#define VALUE_AT(x_, y_) ( \
  area[(y_)][(x_)] != ' ' ? 1 : 0 \
)

/* NOTE: Do not clear elements on POP; clearing is wasted cycles on NES. */
#define PUSH(s, v) ( \
  (s)[++s##_index] = (v) \
)

#define POP(s) ( \
  --s##_index \
)

#define TOP(s) ( \
  (s)[s##_index] \
)

#define EMPTY(s) ( \
  (s##_index) < 0 \
)

#define SET_VISITED_AT(i_) ( \
  visited_index = (uint16_t)(i_), \
  BIT_ARRAY_SET(visited, visited_index) \
)

#define NOT_VISITED(i_) ( \
  visited_index = (uint16_t)(i_), \
  (BIT_ARRAY_VALUE(visited, visited_index) == 0) \
)

#define IN_BOUNDS_X(x_) ((x_) < SIZE_X)
#define IN_BOUNDS_Y(y_) ((y_) < SIZE_Y)

int16_t __fastcall__ solve(uint8_t sx, uint8_t sy, uint8_t dx, uint8_t dy) {

  /* Init */
  pass = 0;

  /* Reject invalid / degenerate requests */
  if ((sx == dx && sy == dy)) return 0;
  if (!IN_BOUNDS_X(sx) || !IN_BOUNDS_X(dx) || !IN_BOUNDS_Y(sy) || !IN_BOUNDS_Y(dy)) return 0;
  if (VALUE_AT(sx, sy) || VALUE_AT(dx, dy)) return 0;

solve:
  ++pass;

  /* Reset visited map (small: 120 bytes for 32x30) */
  for (visited_index = 0; visited_index < (uint16_t)SIZE_OF_ARRAY(visited); ++visited_index) {
    visited[visited_index] = 0;
  }

  /* Get the start point */
  startX = sx;
  startY = sy;

  /* Get the end point */
  destX = dx;
  destY = dy;

  /* Start (x, y) index */
  index = (uint16_t)((startY * SIZE_X) + startX);

  /* Destination (x, y) index */
  destIndex = (uint16_t)((destY * SIZE_X) + destX);

  /* Empty the stack and waypoints (logical reset only, no memory clearing) */
  stack_index = -1;
  waypointX_index = -1;
  waypointY_index = -1;

  /* Push start and mark visited on push (prevents duplicates) */
  PUSH(stack, index);
  SET_VISITED_AT(index);

  while (!EMPTY(stack)) {

    /* Guard: avoid writing past arrays (check BEFORE pushing). */
    if (stack_index >= (STACK_SIZE - 1)) {
      if (pass < 2) {
        x  = dx; y  = dy;
        dx = sx; dy = sy;
        sx = x;  sy = y;
        goto solve;
      }
      return 0;
    }

    /* Get current */
    index = TOP(stack);

    /* Compute coordinates */
    y = (uint8_t)(index / SIZE_X);
    x = (uint8_t)(index % SIZE_X);

    /* If the goal is reached... */
    if (index == destIndex) {
      /* Include goal in waypoints */
      if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
        PUSH(waypointX, x);
        PUSH(waypointY, y);
      }
      break;
    }

    /* Compute distances */
    distX = (int8_t)destX - (int8_t)x;
    distY = (int8_t)destY - (int8_t)y;

    /* Select axis */
    is_horizontal = (ABS(distX) > ABS(distY));

    /* Select next cell to visit */
    newIndex = 0;

    if (is_horizontal) {

      /* Horizontal first */
      if (distX > 0) { /* left -> right */
        if (x != (SIZE_X - 1) && VALUE_AT((uint8_t)(x + 1), y) == 0) {
          newIndex = (uint16_t)(index + 1);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      } else { /* right -> left */
        if (x != 0 && VALUE_AT((uint8_t)(x - 1), y) == 0) {
          newIndex = (uint16_t)(index - 1);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      }

      /* Vertical axis */
      if (distY > 0) { /* low -> up (in your coordinate system) */
        if (y != (SIZE_Y - 1) && VALUE_AT(x, (uint8_t)(y + 1)) == 0) {
          newIndex = (uint16_t)(index + SIZE_X);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
        if (y != 0 && VALUE_AT(x, (uint8_t)(y - 1)) == 0) {
          newIndex = (uint16_t)(index - SIZE_X);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      } else { /* high -> down */
        if (y != 0 && VALUE_AT(x, (uint8_t)(y - 1)) == 0) {
          newIndex = (uint16_t)(index - SIZE_X);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
        if (y != (SIZE_Y - 1) && VALUE_AT(x, (uint8_t)(y + 1)) == 0) {
          newIndex = (uint16_t)(index + SIZE_X);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      }

      /* Last possible direction */
      if (distX > 0) {
        if (x != 0 && VALUE_AT((uint8_t)(x - 1), y) == 0) {
          newIndex = (uint16_t)(index - 1);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      } else {
        if (x != (SIZE_X - 1) && VALUE_AT((uint8_t)(x + 1), y) == 0) {
          newIndex = (uint16_t)(index + 1);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      }

    } else {

      /* Vertical first */
      if (distY > 0) { /* low -> up */
        if (y != (SIZE_Y - 1) && VALUE_AT(x, (uint8_t)(y + 1)) == 0) {
          newIndex = (uint16_t)(index + SIZE_X);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      } else { /* high -> down */
        if (y != 0 && VALUE_AT(x, (uint8_t)(y - 1)) == 0) {
          newIndex = (uint16_t)(index - SIZE_X);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      }

      /* Horizontal axis */
      if (distX > 0) { /* left -> right */
        if (x != (SIZE_X - 1) && VALUE_AT((uint8_t)(x + 1), y) == 0) {
          newIndex = (uint16_t)(index + 1);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
        if (x != 0 && VALUE_AT((uint8_t)(x - 1), y) == 0) {
          newIndex = (uint16_t)(index - 1);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      } else { /* right -> left */
        if (x != 0 && VALUE_AT((uint8_t)(x - 1), y) == 0) {
          newIndex = (uint16_t)(index - 1);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
        if (x != (SIZE_X - 1) && VALUE_AT((uint8_t)(x + 1), y) == 0) {
          newIndex = (uint16_t)(index + 1);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      }

      /* Last possible direction */
      if (distY > 0) {
        if (y != 0 && VALUE_AT(x, (uint8_t)(y - 1)) == 0) {
          newIndex = (uint16_t)(index - SIZE_X);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      } else {
        if (y != (SIZE_Y - 1) && VALUE_AT(x, (uint8_t)(y + 1)) == 0) {
          newIndex = (uint16_t)(index + SIZE_X);
          if (NOT_VISITED(newIndex)) {
            if (waypointX_index < (STACK_SIZE - 1) && waypointY_index < (STACK_SIZE - 1)) {
              PUSH(waypointX, x);
              PUSH(waypointY, y);
            }
            PUSH(stack, newIndex);
            SET_VISITED_AT(newIndex);
            continue;
          }
        }
      }
    }

    /* Backtrack: pop current frame + corresponding waypoint entry */
    if (stack_index >= 0) {
      POP(stack);
      if (waypointX_index >= 0) POP(waypointX);
      if (waypointY_index >= 0) POP(waypointY);
    }
  }

  /* No solution */
  if (EMPTY(stack)) return 0;
  if (waypointX_index < 0 || waypointY_index < 0) return 0;

  /* Path length is waypoint count */
  num_nodes = (uint16_t)(waypointX_index + 1);

  /* Reverse waypoint order on second pass (kept from original behavior) */
  if (pass > 1) {
    end = (uint16_t)(num_nodes - 1);

    for (i = 0; i < (uint16_t)(num_nodes / 2); ++i) {
      tmp = waypointX[i];
      waypointX[i] = waypointX[end];
      waypointX[end] = tmp;
      --end;
    }

    end = (uint16_t)(num_nodes - 1);
    for (i = 0; i < (uint16_t)(num_nodes / 2); ++i) {
      tmp = waypointY[i];
      waypointY[i] = waypointY[end];
      waypointY[end] = tmp;
      --end;
    }
  }

  /* Optimize path */
  #define COST(a, b) ( \
    sx = waypointX[(a)], sy = waypointY[(a)], \
    dx = waypointX[(b)], dy = waypointY[(b)], \
    (uint16_t)(ABS_DIFF(sx, dx) + ABS_DIFF(sy, dy)) \
  )

  do {
    pass = TRUE;
    k = 0;
    i = 0;
    while (i < num_nodes) {
      /* Keep your original cast workaround */
      for (c = (uint32_t)i + 2; c < num_nodes; ++c) {
        if (COST(i, c) == 1) {
          i = (uint16_t)(c - 1);
          pass = FALSE;
          break;
        }
      }
      ++i;
      ++k;
      if (i < num_nodes) {
        waypointX[k] = waypointX[i];
        waypointY[k] = waypointY[i];
      }
    }
    num_nodes = k;
  } while (!pass);

  return (int16_t)num_nodes;
}

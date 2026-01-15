/*
============================================================
Depth-first Search Algorithm - NES Proof-of-concept
Copyright 2018 - 2026 Ninja Dynamics - See license below
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
#include "area.h"

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

#define stack    (*(volatile uint16_t (*)[STACK_SIZE])(0x6000))

/* Stack index goes negative */
static int16_t   stack_index;

/* Waypoint indexes go negative */
static int16_t   waypoint_index;

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
static bool      done;
static uint8_t   pass;

/* Optimization: cache bounds and distance values */
static uint8_t   can_right, can_left, can_down, can_up;
static uint8_t   abs_distX, abs_distY;
static uint8_t   new_x, new_y;

#define IS_SOLID(x_, y_) ( \
  area[(y_)][(x_)] != ' ' \
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

int16_t __fastcall__ solve_dfs(uint8_t sx, uint8_t sy, uint8_t dx, uint8_t dy) {
  /* Init */
  pass = 0;
  
  /* Reject invalid / degenerate requests */
  if ((sx == dx && sy == dy)) return 0;
  if (!IN_BOUNDS_X(sx) || !IN_BOUNDS_X(dx) || !IN_BOUNDS_Y(sy) || !IN_BOUNDS_Y(dy)) return 0;
  if (IS_SOLID(sx, sy) || IS_SOLID(dx, dy)) return 0;

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
  waypoint_index = -1;
  
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
      if (waypoint_index < (STACK_SIZE - 1)) {
        ++waypoint_index;
        waypointX[waypoint_index] = x;
        waypointY[waypoint_index] = y;
      }
      break;
    }
    
    /* Compute distances */
    distX = (int8_t)destX - (int8_t)x;
    distY = (int8_t)destY - (int8_t)y;
    
    /* Cache absolute distances */
    abs_distX = ABS(distX);
    abs_distY = ABS(distY);
    
    /* Select axis */
    is_horizontal = (abs_distX > abs_distY);
    
    /* Cache bounds checks */
    can_right = (x < (SIZE_X - 1));
    can_left = (x > 0);
    can_down = (y < (SIZE_Y - 1));
    can_up = (y > 0);
    
    /* Select next cell to visit */
    newIndex = 0;
    
    if (is_horizontal) {
      /* Horizontal first */
      if (distX > 0) { /* left -> right */
        if (can_right) {
          new_x = x + 1;
          newIndex = index + 1;
          if (NOT_VISITED(newIndex) && IS_SOLID(new_x, y) == 0) {
            goto push_node;
          }
        }
      } else { /* right -> left */
        if (can_left) {
          new_x = x - 1;
          newIndex = index - 1;
          if (NOT_VISITED(newIndex) && IS_SOLID(new_x, y) == 0) {
            goto push_node;
          }
        }
      }
      
      /* Vertical axis */
      if (distY > 0) { /* low -> up (in your coordinate system) */
        if (can_down) {
          new_y = y + 1;
          newIndex = index + SIZE_X;
          if (NOT_VISITED(newIndex) && IS_SOLID(x, new_y) == 0) {
            goto push_node;
          }
        }
        if (can_up) {
          new_y = y - 1;
          newIndex = index - SIZE_X;
          if (NOT_VISITED(newIndex) && IS_SOLID(x, new_y) == 0) {
            goto push_node;
          }
        }
      } else { /* high -> down */
        if (can_up) {
          new_y = y - 1;
          newIndex = index - SIZE_X;
          if (NOT_VISITED(newIndex) && IS_SOLID(x, new_y) == 0) {
            goto push_node;
          }
        }
        if (can_down) {
          new_y = y + 1;
          newIndex = index + SIZE_X;
          if (NOT_VISITED(newIndex) && IS_SOLID(x, new_y) == 0) {
            goto push_node;
          }
        }
      }
      
      /* Last possible direction */
      if (distX > 0) {
        if (can_left) {
          new_x = x - 1;
          newIndex = index - 1;
          if (NOT_VISITED(newIndex) && IS_SOLID(new_x, y) == 0) {
            goto push_node;
          }
        }
      } else {
        if (can_right) {
          new_x = x + 1;
          newIndex = index + 1;
          if (NOT_VISITED(newIndex) && IS_SOLID(new_x, y) == 0) {
            goto push_node;
          }
        }
      }
    } else {
      /* Vertical first */
      if (distY > 0) { /* low -> up */
        if (can_down) {
          new_y = y + 1;
          newIndex = index + SIZE_X;
          if (NOT_VISITED(newIndex) && IS_SOLID(x, new_y) == 0) {
            goto push_node;
          }
        }
      } else { /* high -> down */
        if (can_up) {
          new_y = y - 1;
          newIndex = index - SIZE_X;
          if (NOT_VISITED(newIndex) && IS_SOLID(x, new_y) == 0) {
            goto push_node;
          }
        }
      }
      
      /* Horizontal axis */
      if (distX > 0) { /* left -> right */
        if (can_right) {
          new_x = x + 1;
          newIndex = index + 1;
          if (NOT_VISITED(newIndex) && IS_SOLID(new_x, y) == 0) {
            goto push_node;
          }
        }
        if (can_left) {
          new_x = x - 1;
          newIndex = index - 1;
          if (NOT_VISITED(newIndex) && IS_SOLID(new_x, y) == 0) {
            goto push_node;
          }
        }
      } else { /* right -> left */
        if (can_left) {
          new_x = x - 1;
          newIndex = index - 1;
          if (NOT_VISITED(newIndex) && IS_SOLID(new_x, y) == 0) {
            goto push_node;
          }
        }
        if (can_right) {
          new_x = x + 1;
          newIndex = index + 1;
          if (NOT_VISITED(newIndex) && IS_SOLID(new_x, y) == 0) {
            goto push_node;
          }
        }
      }
      
      /* Last possible direction */
      if (distY > 0) {
        if (can_up) {
          new_y = y - 1;
          newIndex = index - SIZE_X;
          if (NOT_VISITED(newIndex) && IS_SOLID(x, new_y) == 0) {
            goto push_node;
          }
        }
      } else {
        if (can_down) {
          new_y = y + 1;
          newIndex = index + SIZE_X;
          if (NOT_VISITED(newIndex) && IS_SOLID(x, new_y) == 0) {
            goto push_node;
          }
        }
      }
    }
    
    /* Backtrack: pop current frame + corresponding waypoint entry */
    if (stack_index >= 0) {
      POP(stack);
      --waypoint_index;
    }
    continue;
    
push_node:
    /* Push waypoint and stack together */
    if (waypoint_index < (STACK_SIZE - 1)) {
      ++waypoint_index;
      waypointX[waypoint_index] = x;
      waypointY[waypoint_index] = y;
      ++stack_index;
      stack[stack_index] = newIndex;
      SET_VISITED_AT(newIndex);
    }
  }
  
  /* No solution */
  if (EMPTY(stack)) return 0;
  if (waypoint_index < 0) return 0;
  
  /* Path length is waypoint count */
  num_nodes = (uint16_t)(waypoint_index + 1);
  
  /* Reverse waypoint order on second pass (kept from original behavior) */
  if (pass > 1) {
    end = (uint16_t)(num_nodes - 1);
    for (i = 0; i < (uint16_t)(num_nodes / 2); ++i) {
      tmp = waypointX[i];
      waypointX[i] = waypointX[end];
      waypointX[end] = tmp;
      tmp = waypointY[i];
      waypointY[i] = waypointY[end];
      waypointY[end] = tmp;      
      --end;
    }
  }
  
  /* Optimize path */
  do {
    done = TRUE;
    k = 0;
    i = 0;
    while (i < num_nodes) {
      /* Cache current waypoint */
      sx = waypointX[i];
      sy = waypointY[i];
      /* Compare */
      for (c = i + 2; c < num_nodes; ++c) {
        dx = waypointX[c];
        dy = waypointY[c];
        /* Manhattan distance check */
        if ((uint16_t)(ABS_DIFF(sx, dx) + ABS_DIFF(sy, dy)) == 1) {
          i = c - 1;
          done = FALSE;
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
  } while (!done);
  
  return (int16_t)num_nodes;
}


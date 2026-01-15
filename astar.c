/*
============================================================
A* Pathfinding Algorithm - NES Implementation
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
#include "astar.h"
#include "area.h"
#include <string.h>

#define ONE                       (byte)1
#define BIT_ON(v, n)              (v |= (ONE << (n)))
#define BIT_OFF(v, n)             (v &= (byte)~(ONE << (n)))
#define BIT_VALUE(v, n)           (((v) >> (n)) & ONE)
#define BIT_ARRAY_SET(a, i)       BIT_ON(    (a)[(i) / 8], ((i) % 8) )
#define BIT_ARRAY_UNSET(a, i)     BIT_OFF(   (a)[(i) / 8], ((i) % 8) )
#define BIT_ARRAY_VALUE(a, i)     BIT_VALUE( (a)[(i) / 8], ((i) % 8) )

#define SIZE_X 32
#define SIZE_Y 30

#define CELL_COUNT      (SIZE_X * SIZE_Y)
#define CLOSED_BYTES    ((CELL_COUNT + 7) / 8)
#define MAX_OPEN_SET    256  /* Limit for NES memory constraints */

typedef uint8_t bit8_t;
typedef uint16_t cost_t;

/* Node structure for A* */
typedef struct {
  uint16_t index;      /* Cell index */
  uint16_t parent;     /* Parent cell index */
  cost_t   g;          /* Cost from start */
  cost_t   f;          /* f = g + h */
} Node;

/* Memory layout - using external RAM at 0x6000+ */
#define open_set      (*(Node (*)[MAX_OPEN_SET])(0x6000))
#define g_score       (*(cost_t (*)[CELL_COUNT])(0x7000))
#define parent_map    (*(uint16_t (*)[CELL_COUNT])(0x7800))

/* Static variables */
static uint16_t  open_count;
static bit8_t    closed_set[CLOSED_BYTES];
static uint16_t  index;
static uint16_t  current_index;
static uint16_t  neighbor_index;
static uint16_t  destIndex;
static uint16_t  i, j;
static uint8_t   x, y;
static uint8_t   nx, ny;
static uint8_t   destX, destY;
static cost_t    tentative_g;
static cost_t    h_score;
static cost_t    new_f;
static int16_t   num_nodes;
static uint16_t  trace_index;

/* Direction offsets: right, left, down, up */
static const int8_t dir_dx[4] = {1, -1, 0, 0};
static const int8_t dir_dy[4] = {0, 0, 1, -1};

#define IS_SOLID(x_, y_) ( \
  area[(y_)][(x_)] != ' ' \
)

#define IN_CLOSED(i_) ( \
  BIT_ARRAY_VALUE(closed_set, (i_)) \
)

#define ADD_TO_CLOSED(i_) ( \
  BIT_ARRAY_SET(closed_set, (i_)) \
)

#define IN_BOUNDS_X(x_) ((x_) < SIZE_X)
#define IN_BOUNDS_Y(y_) ((y_) < SIZE_Y)

/* Manhattan distance heuristic */
static cost_t heuristic(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
  return (cost_t)(ABS_DIFF(x1, x2) + ABS_DIFF(y1, y2));
}

/* Find node in open set, returns index or -1 if not found */
static int16_t find_in_open(uint16_t idx) {
  for (i = 0; i < open_count; ++i) {
    if (open_set[i].index == idx) {
      return (int16_t)i;
    }
  }
  return -1;
}

/* Remove node from open set at position */
static void remove_from_open(uint16_t pos) {
  /* Shift remaining elements */
  for (i = pos; i < open_count - 1; ++i) {
    open_set[i] = open_set[i + 1];
  }
  --open_count;
}

/* Find node with lowest f score in open set */
static uint16_t find_lowest_f(void) {
  uint16_t lowest_idx = 0;
  cost_t lowest_f = open_set[0].f;
  
  for (i = 1; i < open_count; ++i) {
    if (open_set[i].f < lowest_f) {
      lowest_f = open_set[i].f;
      lowest_idx = i;
    }
  }
  
  return lowest_idx;
}

/* Add node to open set (or update if exists with better path) */
static bool add_to_open(uint16_t idx, uint16_t parent_idx, cost_t g, cost_t f) {
  int16_t existing;
  
  /* Check if already in open set */
  existing = find_in_open(idx);
  if (existing >= 0) {
    /* Update if we found a better path */
    if (g < open_set[existing].g) {
      open_set[existing].g = g;
      open_set[existing].f = f;
      open_set[existing].parent = parent_idx;
    }
    return TRUE;
  }
  
  /* Add new node if space available */
  if (open_count < MAX_OPEN_SET) {
    open_set[open_count].index = idx;
    open_set[open_count].parent = parent_idx;
    open_set[open_count].g = g;
    open_set[open_count].f = f;
    ++open_count;
    return TRUE;
  }
  
  return FALSE; /* Open set full */
}

/* Reconstruct path from parent map */
static int16_t reconstruct_path(uint16_t start_idx, uint16_t goal_idx) {
  num_nodes = 0;
  trace_index = goal_idx;
  
  /* Trace back from goal to start */
  while (trace_index != start_idx && num_nodes < STACK_SIZE) {
    y = (uint8_t)(trace_index / SIZE_X);
    x = (uint8_t)(trace_index % SIZE_X);
    
    waypointX[num_nodes] = x;
    waypointY[num_nodes] = y;
    ++num_nodes;
    
    trace_index = parent_map[trace_index];
    
    /* Safety check for corrupted parent map */
    if (trace_index >= CELL_COUNT) {
      return 0;
    }
  }
  
  /* Add start point */
  if (num_nodes < STACK_SIZE) {
    y = (uint8_t)(start_idx / SIZE_X);
    x = (uint8_t)(start_idx % SIZE_X);
    waypointX[num_nodes] = x;
    waypointY[num_nodes] = y;
    ++num_nodes;
  }
  
  /* Reverse path (was goal->start, need start->goal) */
  for (i = 0; i < (uint16_t)(num_nodes / 2); ++i) {
    j = num_nodes - 1 - i;
    
    x = waypointX[i];
    waypointX[i] = waypointX[j];
    waypointX[j] = x;
    
    y = waypointY[i];
    waypointY[i] = waypointY[j];
    waypointY[j] = y;
  }
  
  return num_nodes;
}

int16_t __fastcall__ solve_astar(uint8_t sx, uint8_t sy, uint8_t dx, uint8_t dy) {
  uint8_t dir;
  uint16_t current_pos;
  cost_t current_g;
  
  /* Reject invalid / degenerate requests */
  if (sx == dx && sy == dy) return 0;
  if (!IN_BOUNDS_X(sx) || !IN_BOUNDS_X(dx) || !IN_BOUNDS_Y(sy) || !IN_BOUNDS_Y(dy)) return 0;
  if (IS_SOLID(sx, sy) || IS_SOLID(dx, dy)) return 0;
  
  /* Initialize */
  memset(closed_set, 0, CLOSED_BYTES);
  memset(g_score, 0xFF, sizeof(g_score)); /* Initialize to max value */
  
  open_count = 0;
  destX = dx;
  destY = dy;
  
  /* Calculate indices */
  index = (uint16_t)((sy * SIZE_X) + sx);
  destIndex = (uint16_t)((dy * SIZE_X) + dx);
  
  /* Initialize start node */
  g_score[index] = 0;
  parent_map[index] = index;
  h_score = heuristic(sx, sy, dx, dy);
  
  if (!add_to_open(index, index, 0, h_score)) {
    return 0; /* Failed to add start node */
  }
  
  /* Main A* loop */
  while (open_count > 0) {
    /* Find node with lowest f score */
    current_pos = find_lowest_f();
    current_index = open_set[current_pos].index;
    current_g = open_set[current_pos].g;
    
    /* Check if we reached the goal */
    if (current_index == destIndex) {
      return reconstruct_path(index, destIndex);
    }
    
    /* Move current from open to closed */
    remove_from_open(current_pos);
    ADD_TO_CLOSED(current_index);
    
    /* Get current coordinates */
    y = (uint8_t)(current_index / SIZE_X);
    x = (uint8_t)(current_index % SIZE_X);
    
    /* Explore neighbors */
    for (dir = 0; dir < 4; ++dir) {
      /* Calculate neighbor position */
      nx = x + dir_dx[dir];
      ny = y + dir_dy[dir];
      
      /* Check bounds */
      if (!IN_BOUNDS_X(nx) || !IN_BOUNDS_Y(ny)) continue;
      
      /* Calculate neighbor index */
      neighbor_index = (uint16_t)((ny * SIZE_X) + nx);
      
      /* Skip if solid or already in closed set */
      if (IS_SOLID(nx, ny) || IN_CLOSED(neighbor_index)) continue;
      
      /* Calculate tentative g score */
      tentative_g = current_g + 1; /* Cost is always 1 for adjacent cells */
      
      /* Skip if not a better path */
      if (tentative_g >= g_score[neighbor_index]) continue;
      
      /* This is a better path, record it */
      parent_map[neighbor_index] = current_index;
      g_score[neighbor_index] = tentative_g;
      
      /* Calculate f score */
      h_score = heuristic(nx, ny, destX, destY);
      new_f = tentative_g + h_score;
      
      /* Add to open set (or update if already there) */
      if (!add_to_open(neighbor_index, current_index, tentative_g, new_f)) {
        /* Open set full - this shouldn't happen with reasonable mazes */
        /* but we need to handle it gracefully */
        continue;
      }
    }
  }
  
  /* No path found */
  return 0;
}

void __fastcall__ initialize_astar_solver(void) {
  /* Clear all data structures */
  memset(closed_set, 0, CLOSED_BYTES);
  memset(g_score, 0xFF, sizeof(g_score));
  open_count = 0;
}
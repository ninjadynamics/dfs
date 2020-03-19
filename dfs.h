#ifndef DFS_H
#define DFS_H

#include "neslib.h"

#include <inttypes.h>

#define SIZE_OF_ARRAY(array) \
  (sizeof(array) / sizeof(array[0]))

#define LAST_INDEX_OF(array) \
  (sizeof(array) / sizeof(array[0]) - 1)

#define ABS(n) ( \
  n < 0 ? -n : n \
)

#define ABS_DIFF(a, b) ( \
  a < b ? b - a : a - b \
)

#define STACK_SIZE 1024

extern const char area[32][32];

#define waypointX    (*(volatile uint8_t (*)[STACK_SIZE])(0x6800))
#define waypointY    (*(volatile uint8_t (*)[STACK_SIZE])(0x6C00))

void __fastcall__ initialize_solver(void);
bool __fastcall__ solve(uint8_t sx, uint8_t sy, uint8_t dx, uint8_t dy);

#endif // dfs.h
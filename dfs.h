#ifndef DFS_H
#define DFS_H

#include "neslib.h"

#include <inttypes.h>

extern const char area[32][32];

extern uint8_t waypointX [128];
extern uint8_t waypointY [128];

void __fastcall__ initialize_solver(void);
bool __fastcall__ solve(uint8_t sx, uint8_t sy, uint8_t dx, uint8_t dy);

#endif // dfs.h
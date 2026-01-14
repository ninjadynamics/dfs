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
    No additional restrictions — You may not apply legal
    terms or technological measures that legally restrict
    others from doing anything the license permits.
============================================================
*/

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

#define STACK_SIZE 30*32

extern const char area[30][32];

#define waypointX    (*(volatile uint8_t (*)[STACK_SIZE])(0x6800))
#define waypointY    (*(volatile uint8_t (*)[STACK_SIZE])(0x6C00))

void __fastcall__ initialize_solver(void);
int16_t __fastcall__ solve(uint8_t sx, uint8_t sy, uint8_t dx, uint8_t dy);

#endif // dfs.h